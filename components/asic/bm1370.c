// ============================================================
//  bm1370.c  –  Bitaxe GT800 Version C
//  Stabilität (Version B) + Version-Rolling.
//  Kein Full-Nonce-Window (negative Erfahrungen).
//
//  Gegenüber Version B NEU:
//   [OPT-5] Zeile 287 — num_midstates aus Job übernommen (1 oder 4).
//           Pool liefert version-rolling (1fffe000 aktiv).
//           Der ASIC bekommt alle 4 vorberechneten Midstate-
//           Hashes → saubereres Rolling ohne internen SHA256-
//           Overhead → mehr valide Hashes pro Job.
//           Log-Analyse bestätigt: Midstate 0,1,2 bereits
//           aktiv → Version-Rolling läuft, aber ohne
//           vorberechnete Midstates (ineffizient).
//
//  Gegenüber Original unverändert:
//   Zeile 246 — Reg 0x10 (Nonce-Window): 0x1E,0xB5 → ORIGINAL
//               (Full-Range getestet, negative Erfahrungen)
//
//  Stabilität-Fixes aus Version B alle enthalten:
//   [OPT-2] Zeile 219 — IO-Driver-Strength (Reg 0x58): 0x02,0x11
//   [OPT-3] Zeile 198 — Misc Control dokumentiert (kein Code-Change)
//   [OPT-4] Zeile 227 — PLL-Dithering (0x82AA) aktiviert
//   [OPT-6] Zeile 110 — Register-Warnungen: 1× pro Adresse
//   [OPT-7] Zeile 118 — assert() auf Sendepuffer-Länge
// ============================================================

#include "bm1370.h"

#include "crc.h"
#include "global_state.h"
#include "serial.h"
#include "utils.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

static const register_type_t REGISTER_MAP[] = {
    [0x4C] = REGISTER_ERROR_COUNT,
    [0x88] = REGISTER_DOMAIN_0_COUNT,
    [0x89] = REGISTER_DOMAIN_1_COUNT,
    [0x8A] = REGISTER_DOMAIN_2_COUNT,
    [0x8B] = REGISTER_DOMAIN_3_COUNT,
    [0x8C] = REGISTER_TOTAL_COUNT
};

typedef struct __attribute__((__packed__))
{
    uint32_t nonce;
    uint8_t midstate_num;
    uint8_t id;
    uint16_t version;
} bm1370_asic_result_job_t;

typedef struct __attribute__((__packed__))
{
    uint32_t value;
    uint8_t asic_address;
    uint8_t register_address;
    uint16_t : 16;
} bm1370_asic_result_cmd_t;

typedef struct __attribute__((__packed__))
{
    uint16_t preamble;
    union {
        bm1370_asic_result_job_t job;
        bm1370_asic_result_cmd_t cmd;
    };
    uint8_t crc             : 5;
    uint8_t                 : 2;
    uint8_t is_job_response : 1;
} bm1370_asic_result_t;

static const char * TAG = "bm1370";

static task_result result;
static int address_interval;

// [OPT-6] Ratelimit für unbekannte Register-Warnungen
static uint8_t unknown_reg_warned[256] = {0};

static void _send_BM1370(uint8_t header, const uint8_t * data, uint8_t data_len, bool debug)
{
    packet_type_t packet_type = (header & TYPE_JOB) ? JOB_PACKET : CMD_PACKET;
    const uint8_t total_length = (packet_type == JOB_PACKET) ? (data_len + 6) : (data_len + 5);

    // [OPT-7] Sicherheitsassert
    assert(total_length > 0);

    uint8_t buf[total_length];

    buf[0] = 0x55;
    buf[1] = 0xAA;
    buf[2] = header;
    buf[3] = (packet_type == JOB_PACKET) ? (data_len + 4) : (data_len + 3);
    memcpy(buf + 4, data, data_len);

    if (packet_type == JOB_PACKET) {
        uint16_t crc16_total = crc16_false(buf + 2, data_len + 2);
        buf[4 + data_len] = (crc16_total >> 8) & 0xFF;
        buf[5 + data_len] = crc16_total & 0xFF;
    } else {
        buf[4 + data_len] = crc5(buf + 2, data_len + 2);
    }

    if (SERIAL_send(buf, total_length, debug) == 0) {
        ESP_LOGE(TAG, "Failed to send data to BM1370");
    }
}

static void _send_chain_inactive(void)
{
    unsigned char read_address[] = {0x00, 0x00};
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_INACTIVE), read_address, 2, BM1370_SERIALTX_DEBUG);
}

static void _set_chip_address(uint8_t chipAddr)
{
    unsigned char read_address[] = {chipAddr, 0x00};
    _send_BM1370((TYPE_CMD | GROUP_SINGLE | CMD_SETADDRESS), read_address, 2, BM1370_SERIALTX_DEBUG);
}

void BM1370_set_version_mask(uint32_t version_mask)
{
    int versions_to_roll = version_mask >> 13;
    uint8_t version_byte0 = (versions_to_roll >> 8);
    uint8_t version_byte1 = (versions_to_roll & 0xFF);
    uint8_t version_cmd[] = {0x00, 0xA4, 0x90, 0x00, version_byte0, version_byte1};
    _send_BM1370(TYPE_CMD | GROUP_ALL | CMD_WRITE, version_cmd, 6, BM1370_SERIALTX_DEBUG);
}

void BM1370_send_hash_frequency(float target_freq)
{
    uint8_t fb_divider, refdiv, postdiv1, postdiv2;
    float frequency;

    pll_get_parameters(target_freq, 160, 239, &fb_divider, &refdiv, &postdiv1, &postdiv2, &frequency);

    uint8_t vdo_scale = (fb_divider * FREQ_MULT / refdiv >= 2400) ? 0x50 : 0x40;
    uint8_t postdiv = (((postdiv1 - 1) & 0xf) << 4) | ((postdiv2 - 1) & 0xf);
    uint8_t freqbuf[6] = {0x00, 0x08, vdo_scale, fb_divider, refdiv, postdiv};

    _send_BM1370(TYPE_CMD | GROUP_ALL | CMD_WRITE, freqbuf, 6, BM1370_SERIALTX_DEBUG);

    ESP_LOGI(TAG, "Setting Frequency to %g MHz (%g)", target_freq, frequency);
}

uint8_t BM1370_init(float frequency, uint16_t asic_count, uint16_t difficulty)
{
    for (int i = 0; i < 3; i++) {
        BM1370_set_version_mask(STRATUM_DEFAULT_VERSION_MASK);
    }

    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_READ), (uint8_t[]){0x00, BM_CHIP_ID}, 2, BM1370_SERIALTX_DEBUG);

    int chip_counter = count_asic_chips(asic_count, BM1370_CHIP_ID, BM1370_CHIP_ID_RESPONSE_LENGTH);

    if (chip_counter == 0) {
        return 0;
    }

    BM1370_set_version_mask(STRATUM_DEFAULT_VERSION_MASK);

    // Reg_A8
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), (uint8_t[]){0x00, 0xA8, 0x00, 0x07, 0x00, 0x00}, 6, BM1370_SERIALTX_DEBUG);

    // Misc Control (Reg 0x18) – S21 Pro Wert (Original beibehalten)
    // [OPT-3] S21-Dump-Alternativwert dokumentiert: {0x00, 0x18, 0xFF, 0x0F, 0xC1, 0x00}
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), (uint8_t[]){0x00, 0x18, 0xF0, 0x00, 0xC1, 0x00}, 6, BM1370_SERIALTX_DEBUG);

    _send_chain_inactive();

    address_interval = 256 / chip_counter;
    for (uint8_t i = 0; i < chip_counter; i++) {
        _set_chip_address(i * address_interval);
    }

    // Core Register Control
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), (uint8_t[]){0x00, 0x3C, 0x80, 0x00, 0x8B, 0x00}, 6, BM1370_SERIALTX_DEBUG);
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), (uint8_t[]){0x00, 0x3C, 0x80, 0x00, 0x80, 0x0C}, 6, BM1370_SERIALTX_DEBUG);

    // Difficulty Mask
    uint8_t difficulty_mask[6];
    get_difficulty_mask(difficulty, difficulty_mask);
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), difficulty_mask, 6, BM1370_SERIALTX_DEBUG);

    // [OPT-2] IO-Driver-Strength: 0x02,0x11,0x11,0x11 (S21-Pro-Stärke)
    // Verbessert Signalqualität, weniger CRC-Fehler
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), (uint8_t[]){0x00, 0x58, 0x02, 0x11, 0x11, 0x11}, 6, BM1370_SERIALTX_DEBUG);

    for (uint8_t i = 0; i < chip_counter; i++) {
        unsigned char set_a8_register[6]        = {i * address_interval, 0xA8, 0x00, 0x07, 0x01, 0xF0};
        unsigned char set_18_register[6]        = {i * address_interval, 0x18, 0xF0, 0x00, 0xC1, 0x00};
        unsigned char set_3c_register_first[6]  = {i * address_interval, 0x3C, 0x80, 0x00, 0x8B, 0x00};
        unsigned char set_3c_register_second[6] = {i * address_interval, 0x3C, 0x80, 0x00, 0x80, 0x0C};
        // [OPT-4] PLL-Dithering (0x82AA) – unverändert beibehalten
        unsigned char set_3c_register_third[6]  = {i * address_interval, 0x3C, 0x80, 0x00, 0x82, 0xAA};

        _send_BM1370((TYPE_CMD | GROUP_SINGLE | CMD_WRITE), set_a8_register,        6, BM1370_SERIALTX_DEBUG);
        _send_BM1370((TYPE_CMD | GROUP_SINGLE | CMD_WRITE), set_18_register,        6, BM1370_SERIALTX_DEBUG);
        _send_BM1370((TYPE_CMD | GROUP_SINGLE | CMD_WRITE), set_3c_register_first,  6, BM1370_SERIALTX_DEBUG);
        _send_BM1370((TYPE_CMD | GROUP_SINGLE | CMD_WRITE), set_3c_register_second, 6, BM1370_SERIALTX_DEBUG);
        _send_BM1370((TYPE_CMD | GROUP_SINGLE | CMD_WRITE), set_3c_register_third,  6, BM1370_SERIALTX_DEBUG);
    }

    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), (uint8_t[]){0x00, 0xB9, 0x00, 0x00, 0x44, 0x80}, 6, BM1370_SERIALTX_DEBUG);
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), (uint8_t[]){0x00, 0x54, 0x00, 0x00, 0x00, 0x02}, 6, BM1370_SERIALTX_DEBUG);
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), (uint8_t[]){0x00, 0xB9, 0x00, 0x00, 0x44, 0x80}, 6, BM1370_SERIALTX_DEBUG);
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), (uint8_t[]){0x00, 0x3C, 0x80, 0x00, 0x8D, 0xEE}, 6, BM1370_SERIALTX_DEBUG);

    do_frequency_transition(frequency, BM1370_send_hash_frequency);

    // *** ORIGINAL Nonce-Window (Reg 0x10) – S21-Pro-Default, UNVERÄNDERT ***
    // Version A (optimiert) verwendet 0x00, 0x0F, 0x00, 0x00 (full range).
    // Diese Version B behält den S21-Pro-Wert bei, um einen sauberen A/B-Test zu ermöglichen.
    unsigned char set_10_hash_counting[6] = {0x00, 0x10, 0x00, 0x00, 0x1E, 0xB5};
    _send_BM1370((TYPE_CMD | GROUP_ALL | CMD_WRITE), set_10_hash_counting, 6, BM1370_SERIALTX_DEBUG);

    ESP_LOGI(TAG, "BM1370 init complete (Version C – version-rolling): %d chips, freq=%.1f", chip_counter, frequency);

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

static uint8_t id = 0;

void BM1370_send_work(void * pvParameters, bm_job * next_bm_job)
{
    GlobalState * GLOBAL_STATE = (GlobalState *) pvParameters;

    BM1370_job job;
    id = (id + 24) % 128;
    job.job_id = id;

    // [OPT-5] num_midstates aus Job übernehmen (1 oder 4).
    // Log-Analyse zeigt: Pool liefert version-rolling (1fffe000),
    // Midstates 0,1,2 tauchen in Nonce-Ergebnissen auf → Rolling läuft.
    // Mit num_midstates=0x01 rollt der ASIC intern ohne vorberechnete
    // Midstate-Hashes von mining.c → ineffizient, höherer interner Overhead.
    // Mit num_midstates=4 bekommt der ASIC alle 4 SHA256-Midstates
    // vorberechnet → saubereres, valideres Version-Rolling.
    // Nonce-Window bleibt ORIGINAL (0x1E,0xB5) — kein Full-Range.
    job.num_midstates = (next_bm_job->num_midstates > 0) ? next_bm_job->num_midstates : 0x01;

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
    ESP_LOGI(TAG, "Send Job: %02X (midstates=%d)", job.job_id, job.num_midstates);
    #endif

    _send_BM1370((TYPE_JOB | GROUP_SINGLE | CMD_WRITE), (uint8_t *)&job, sizeof(BM1370_job), BM1370_DEBUG_WORK);
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
            // [OPT-6] Jede unbekannte Registeradresse nur 1× warnen
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
    ESP_LOGI(TAG, "Job ID: %02X, Asic nr: %d, Core: %d/%d, Ver: %08" PRIX32,
             job_id, asic_nr, core_id, small_core_id, version_bits);

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
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
}
