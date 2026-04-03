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

#include "bm1370.h"

#include "crc.h"
#include "global_state.h"
#include "serial.h"
#include "utils.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "frequency_transition_bmXX.h"
#include "pll.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <assert.h>

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

static task_result result;
static int address_interval;

// Neu: Array für OPT-6 (Einmalige Register-Warnung)
static uint8_t unknown_reg_warned[256] = {0};

// ============================
// 🔁 WORK SCHEDULER
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

// ============================
// Queue
// ============================

static bool enqueue_job(bm_job* job)
{
    xSemaphoreTake(job_queue_mutex, portMAX_DELAY);

    int next_tail = (job_queue_tail + 1) % JOB_QUEUE_SIZE;

    // FIX: Wenn die Queue voll ist, wird der älteste Job verworfen.
    // Dieser muss freigegeben werden, sonst gibt es ein Memory Leak!
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

static queued_job_t* peek_job()
{
    if (job_queue_head == job_queue_tail) return NULL;
    return &job_queue[job_queue_head];
}

static void pop_job()
{
    job_queue_head = (job_queue_head + 1) % JOB_QUEUE_SIZE;
}

// ============================
// Scheduler Thread
// ============================

static void bm1370_scheduler_task(void * pvParameters)
{
    GlobalState * GLOBAL_STATE = (GlobalState *) pvParameters;

    while (1) {
        xSemaphoreTake(job_queue_mutex, portMAX_DELAY);

        queued_job_t* qjob = peek_job();

        if (qjob == NULL) {
            xSemaphoreGive(job_queue_mutex);
            vTaskDelay(pdMS_TO_TICKS(1)); // CPU-Zeit sparen, falls Queue leer
            continue;
        }

        uint32_t now = xTaskGetTickCount();

        // FIX: Auch abgelaufene (stale) Jobs müssen freigegeben werden!
        if ((now - qjob->timestamp) > pdMS_TO_TICKS(MAX_JOB_LIFETIME_MS)) {
            if (qjob->job != NULL) {
                free_bm_job(qjob->job);
            }
            pop_job();
            xSemaphoreGive(job_queue_mutex);
            continue;
        }

        bm_job* next_bm_job = qjob->job;
        BM1370_job job;

        sched_id = (sched_id + 1) % 128;
        job.job_id = sched_id;

        // [OPT-5] Midstate Logik sauber übernommen
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

        #if BM1370_DEBUG_JOBS
        ESP_LOGI(TAG, "Sched Send Job: %02X (Midstates: %d)", job.job_id, job.num_midstates);
        #endif

        _send_BM1370((TYPE_JOB | GROUP_SINGLE | CMD_WRITE),
                     (uint8_t *)&job,
                     sizeof(BM1370_job),
                     BM1370_DEBUG_WORK);

        pop_job();
        xSemaphoreGive(job_queue_mutex);

        vTaskDelay(pdMS_TO_TICKS(SCHEDULER_DELAY_MS));
    }
}

// ============================
// SEND FUNCTION (Einzige und korrekte Version)
// ============================

void BM1370_send_work(void * pvParameters, bm_job * next_bm_job)
{
    // Packt den Job nur in die Queue. Der Scheduler Thread (bm1370_scheduler_task) übernimmt den Rest.
    enqueue_job(next_bm_job);
}

// ============================
// INIT 
// ============================

uint8_t BM1370_init(GlobalState * GLOBAL_STATE, float frequency, uint16_t asic_count, uint16_t difficulty)
{
    BM1370_set_version_mask(STRATUM_DEFAULT_VERSION_MASK);

    int chip_counter = count_asic_chips(asic_count, BM1370_CHIP_ID, BM1370_CHIP_ID_RESPONSE_LENGTH);
    if (chip_counter == 0) return 0;

    address_interval = 256 / chip_counter;

    do_frequency_transition(frequency, BM1370_send_hash_frequency);

    unsigned char set_10_hash_counting[6] = {0x00, 0x10, 0x00, 0x00, 0x1E, 0xB5};
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), set_10_hash_counting, 6, BM1370_SERIALTX_DEBUG);

    // Scheduler starten
    job_queue_mutex = xSemaphoreCreateMutex();
    
    if (job_queue_mutex != NULL) {
        xTaskCreate(
            bm1370_scheduler_task,
            "bm1370_sched",
            4096,
            GLOBAL_STATE,
            5,
            &scheduler_task_handle
        );
        ESP_LOGI(TAG, "BM1370 init complete (Scheduler Mode)");
    } else {
        ESP_LOGE(TAG, "Failed to create job queue mutex!");
    }

    return chip_counter;
}

int BM1370_set_default_baud(void)
{
    unsigned char baudrate[] = {0x00, MISC_CONTROL, 0x00, 0x00, 0b01111010, 0b00110001};
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), baudrate, 6, BM1370_SERIALTX_DEBUG);
    return 115749;
}

int BM1370_set_max_baud(void)
{
    ESP_LOGI(TAG, "Setting max baud of 1000000");
    unsigned char fast_uart[] = {0x00, FAST_UART_CONFIGURATION, 0x11, 0x30, 0x02, 0x00};
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), fast_uart, 6, BM1370_SERIALTX_DEBUG);
    return 1000000;
}

task_result * BM1370_process_work(void * pvParameters)
{
    bm1370_asic_result_t asic_result = {0};

    memset(&result, 0, sizeof(task_result));

    if (receive_work((uint8_t *)&asic_result, sizeof(asic_result)) == ESP_FAIL) {
        return NULL;
    }

    if (!asic_result.is_job_response) {
        result.register_type = REGISTER_MAP[asic_result.cmd.register_address];
        if (result.register_type == REGISTER_INVALID) {
            // [OPT-6] Jede unbekannte Registeradresse nur 1x warnen
            uint8_t reg = asic_result.cmd.register_address;
            if (!unknown_reg_warned[reg]) {
                ESP_LOGW(TAG, "Unknown register read (once): %02x", reg);
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
    uint8_t core_id = (uint8_t)((nonce_h >> 25) & 0x7f);
    uint8_t small_core_id = asic_result.job.id & 0x0f;
    uint32_t version_bits = (ntohs(asic_result.job.version) << 13);
    
    // Log auf Debug-Level (oder reduzierten Aufruf), da es bei hohen Hashraten sehr viel spammen kann.
    // ESP_LOGI(TAG, "Job ID: %02X, Asic nr: %d, Core: %d/%d, Ver: %08" PRIX32,
    //          job_id, asic_nr, core_id, small_core_id, version_bits);

    GlobalState * GLOBAL_STATE = (GlobalState *) pvParameters;

    if (GLOBAL_STATE->valid_jobs[job_id] == 0) {
        ESP_LOGW(TAG, "Invalid job nonce found, 0x%02X", job_id);
        return NULL;
    }

    uint32_t rolled_version = GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[job_id]->version | version_bits;

    result.job_id = job_id;
    result.nonce = asic_result.job.nonce;
    result.rolled_version = rolled_version;
    result.asic_nr = asic_nr;

    return &result;
}

void BM1370_read_registers(void)
{
    int size = sizeof(REGISTER_MAP) / sizeof(REGISTER_MAP[0]);
    for (int reg = 0; reg < size; reg++) {
        if (REGISTER_MAP[reg] != REGISTER_INVALID) {
            _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_READ), (uint8_t[]){0x00, reg}, 2, BM1370_SERIALTX_DEBUG);
            // FIX: Saubere Nutzung der FreeRTOS Tick Rate anstelle von Integer-Division (die 0 ergibt)
            vTaskDelay(pdMS_TO_TICKS(1)); 
        }
    }
}