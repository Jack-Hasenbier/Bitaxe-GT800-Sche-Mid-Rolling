// ============================================================
//  bm1370.c  –  Bitaxe GT800 Version D (Scheduler Edition)
//  Basierend auf Version C + echter Work-Scheduler
//
//  NEU:
//   - Job Queue + Scheduler Thread
//   - Stale Job Protection (~200ms) mit Memory Leak Fix
//   - Saubere Job-ID Sequenz (kein +24 mehr)
//   - Kontrolliertes ASIC Feeding (Timing)
// ============================================================

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <assert.h>
#include "driver/uart.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Projekt-Header
#include "bm1370.h"
#include "global_state.h"
#include "crc.h"
#include "serial.h"
#include "utils.h"
#include "frequency_transition_bmXX.h"
#include "pll.h"

// ============================
// 🛠 INTERNE DEFINITIONEN & STRUCTS
// ============================

#define BM1370_CHIP_ID 0x1370
#define BM1370_CHIP_ID_RESPONSE_LENGTH 11

#define TYPE_JOB 0x20
#define TYPE_CMD 0x40

#define GROUP_SINGLE 0x00
#define GROUP_ALL 0x10

#define CMD_SETADDRESS 0x00
#define CMD_WRITE 0x01
#define CMD_READ 0x02
#define CMD_INACTIVE 0x03

#define BM_CHIP_ID 0x00
#define MISC_CONTROL 0x18
#define FAST_UART_CONFIGURATION 0x28

static const char * TAG = "bm1370";

// Vorwärtsdeklaration der internen Sende-Funktion
static void _send_BM1370(uint8_t type, uint8_t *data, uint8_t len, bool debug);

// Register-Mapping für die Auswertung
static const uint8_t REGISTER_MAP[] = {
    0x00, 0x01, 0x08, 0x10, 0x14, 0x18, 0x20, 0x28, 0x30, 0x38, 0x40, 0x48, 0x58, 0x60, 0x68, 0x70, 0x78, 0x80, 0x84
};

// Strukturen für die ASIC-Antwort-Verarbeitung
typedef struct __attribute__((__packed__)) {
    uint8_t id;
    uint32_t nonce;
    uint16_t version;
} bm1370_job_response_t;

typedef struct __attribute__((__packed__)) {
    uint8_t register_address;
    uint8_t asic_address;
    uint32_t value;
} bm1370_cmd_response_t;

typedef struct {
    bool is_job_response;
    union {
        bm1370_job_response_t job;
        bm1370_cmd_response_t cmd;
    };
} bm1370_asic_result_t;

// Globale Variablen für dieses Modul
static task_result result;
static int address_interval;
static uint8_t unknown_reg_warned[256] = {0};

// ============================
// 🔁 WORK SCHEDULER LOGIK
// ============================

#define JOB_QUEUE_SIZE 32
#define MAX_JOB_LIFETIME_MS 200
#define SCHEDULER_DELAY_MS 20

typedef struct {
    bm_job* job;
    uint32_t timestamp;
} queued_job_t;

static queued_job_t job_queue[JOB_QUEUE_SIZE];
static int job_queue_head = 0;
static int job_queue_tail = 0;

static SemaphoreHandle_t job_queue_mutex;
static TaskHandle_t scheduler_task_handle = NULL;
static uint8_t sched_id = 0;

static bool enqueue_job(bm_job* job) {
    if (!job_queue_mutex) return false;
    xSemaphoreTake(job_queue_mutex, portMAX_DELAY);

    int next_tail = (job_queue_tail + 1) % JOB_QUEUE_SIZE;

    if (next_tail == job_queue_head) {
        if (job_queue[job_queue_head].job != NULL) {
            free_bm_job(job_queue[job_queue_head].job);
        }
        job_queue_head = (job_queue_head + 1) % JOB_QUEUE_SIZE;
    }

    job_queue[job_queue_tail].job = job;
    job_queue[job_queue_tail].timestamp = xTaskGetTickCount();
    job_queue_tail = next_tail;

    xSemaphoreGive(job_queue_mutex);
    return true;
}

static queued_job_t* peek_job() {
    if (job_queue_head == job_queue_tail) return NULL;
    return &job_queue[job_queue_head];
}

static void pop_job() {
    job_queue_head = (job_queue_head + 1) % JOB_QUEUE_SIZE;
}

// Scheduler Thread
static void bm1370_scheduler_task(void * pvParameters) {
    GlobalState * GLOBAL_STATE = (GlobalState *) pvParameters;

    while (1) {
        xSemaphoreTake(job_queue_mutex, portMAX_DELAY);
        queued_job_t* qjob = peek_job();

        if (qjob == NULL) {
            xSemaphoreGive(job_queue_mutex);
            vTaskDelay(pdMS_TO_TICKS(5)); 
            continue;
        }

        uint32_t now = xTaskGetTickCount();
        if ((now - qjob->timestamp) > pdMS_TO_TICKS(MAX_JOB_LIFETIME_MS)) {
            if (qjob->job != NULL) free_bm_job(qjob->job);
            pop_job();
            xSemaphoreGive(job_queue_mutex);
            continue;
        }

        bm_job* next_bm_job = qjob->job;
        BM1370_job job;

        sched_id = (sched_id + 1) % 128;
        job.job_id = sched_id;
        job.num_midstates = (next_bm_job->num_midstates == 4) ? 4 : 1;

        memcpy(&job.starting_nonce, &next_bm_job->starting_nonce, 4);
        memcpy(&job.nbits, &next_bm_job->target, 4);
        memcpy(&job.ntime, &next_bm_job->ntime, 4);
        memcpy(job.merkle_root, next_bm_job->merkle_root_be, 32);
        memcpy(job.prev_block_hash, next_bm_job->prev_block_hash_be, 32);
        memcpy(&job.version, &next_bm_job->version, 4);

        if (GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[job.job_id] != NULL) {
            free_bm_job(GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[job.job_id]);
        }
        GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[job.job_id] = next_bm_job;

        pthread_mutex_lock(&GLOBAL_STATE->valid_jobs_lock);
        GLOBAL_STATE->valid_jobs[job.job_id] = 1;
        pthread_mutex_unlock(&GLOBAL_STATE->valid_jobs_lock);

        _send_BM1370((TYPE_JOB | GROUP_SINGLE | CMD_WRITE), (uint8_t *)&job, sizeof(BM1370_job), BM1370_DEBUG_WORK);

        pop_job();
        xSemaphoreGive(job_queue_mutex);
        vTaskDelay(pdMS_TO_TICKS(SCHEDULER_DELAY_MS));
    }
}

// ============================
// PUBLIC API IMPLEMENTATION
// ============================

void BM1370_send_work(void * pvParameters, bm_job * next_bm_job) {
    enqueue_job(next_bm_job);
}

uint8_t BM1370_init(void * pvParameters, float frequency, uint16_t asic_count, uint16_t difficulty) {
    GlobalState * GLOBAL_STATE = (GlobalState *) pvParameters;
    BM1370_set_version_mask(STRATUM_DEFAULT_VERSION_MASK);

    int chip_counter = count_asic_chips(asic_count, BM1370_CHIP_ID, BM1370_CHIP_ID_RESPONSE_LENGTH);
    if (chip_counter == 0) return 0;

    address_interval = 256 / chip_counter;
    do_frequency_transition(frequency, BM1370_send_hash_frequency);

    unsigned char set_10_hash_counting[6] = {0x00, 0x10, 0x00, 0x00, 0x1E, 0xB5};
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), set_10_hash_counting, 6, BM1370_SERIALTX_DEBUG);

    if (!job_queue_mutex) job_queue_mutex = xSemaphoreCreateMutex();
    
    if (job_queue_mutex != NULL) {
        xTaskCreate(bm1370_scheduler_task, "bm1370_sched", 4096, GLOBAL_STATE, 5, &scheduler_task_handle);
        ESP_LOGI(TAG, "BM1370 init complete (Scheduler Mode)");
    }

    return chip_counter;
}

int BM1370_set_default_baud(void) {
    unsigned char baudrate[] = {0x00, MISC_CONTROL, 0x00, 0x00, 0b01111010, 0b00110001};
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), baudrate, 6, BM1370_SERIALTX_DEBUG);
    return 115749;
}

int BM1370_set_max_baud(void) {
    unsigned char fast_uart[] = {0x00, FAST_UART_CONFIGURATION, 0x11, 0x30, 0x02, 0x00};
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), fast_uart, 6, BM1370_SERIALTX_DEBUG);
    return 1000000;
}

task_result * BM1370_process_work(void * pvParameters) {
    bm1370_asic_result_t asic_result = {0};
    memset(&result, 0, sizeof(task_result));

    if (receive_work((uint8_t *)&asic_result, sizeof(asic_result)) == ESP_FAIL) return NULL;

    if (!asic_result.is_job_response) {
        result.register_type = REGISTER_MAP[asic_result.cmd.register_address];
        if (result.register_type == REGISTER_INVALID) {
            uint8_t reg = asic_result.cmd.register_address;
            if (!unknown_reg_warned[reg]) {
                ESP_LOGW(TAG, "Unknown register: %02x", reg);
                unknown_reg_warned[reg] = 1;
            }
            return NULL;
        }
        result.asic_nr = asic_result.cmd.asic_address / address_interval;
        result.value = ntohl(asic_result.cmd.value);
        return &result;
    }

    uint8_t job_id = (asic_result.job.id & 0xf0) >> 1;
    uint32_t nonce_h = ntohl(asic_result.job.nonce);
    uint8_t asic_nr = (uint8_t)((nonce_h >> 17) & 0xff) / address_interval;
    uint32_t version_bits = (ntohs(asic_result.job.version) << 13);
    
    GlobalState * GLOBAL_STATE = (GlobalState *) pvParameters;
    if (GLOBAL_STATE->valid_jobs[job_id] == 0) return NULL;

    result.job_id = job_id;
    result.nonce = asic_result.job.nonce;
    result.rolled_version = GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[job_id]->version | version_bits;
    result.asic_nr = asic_nr;

    return &result;
}

void BM1370_read_registers(void) {
    int size = sizeof(REGISTER_MAP) / sizeof(REGISTER_MAP[0]);
    for (int reg = 0; reg < size; reg++) {
        if (REGISTER_MAP[reg] != REGISTER_INVALID) {
            uint8_t data[] = {0x00, reg};
            _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_READ), data, 2, BM1370_SERIALTX_DEBUG);
            vTaskDelay(pdMS_TO_TICKS(1)); 
        }
    }
}

// ============================
// 📨 HILFSFUNKTIONEN
// ============================

static void _send_BM1370(uint8_t type, uint8_t *data, uint8_t len, bool debug) {
    uint8_t packet[len + 4];
    packet[0] = 0x55;
    packet[1] = 0xAA;
    packet[2] = type;
    packet[3] = len;

    if (len > 0) memcpy(&packet[4], data, len);

    uint16_t crc = crc16(packet, len + 4);
    uint8_t crc_buf[2];
    crc_buf[0] = (crc >> 8) & 0xFF;
    crc_buf[1] = crc & 0xFF;

    uart_write_bytes(UART_NUM_1, (const char*)packet, len + 4);
    uart_write_bytes(UART_NUM_1, (const char*)crc_buf, 2);

    if (debug) {
        ESP_LOG_BUFFER_HEXDUMP(TAG, packet, len + 4, ESP_LOG_DEBUG);
        ESP_LOG_BUFFER_HEXDUMP(TAG, crc_buf, 2, ESP_LOG_DEBUG);
    }
}