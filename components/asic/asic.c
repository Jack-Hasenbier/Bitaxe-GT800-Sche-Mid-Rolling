#include <string.h>
#include <inttypes.h>

#include <esp_log.h>

#include "bm1397.h"
#include "bm1366.h"
#include "bm1368.h"
#include "bm1370.h"

#include "asic.h"
#include "device_config.h"
#include "frequency_transition_bmXX.h"

static const double NONCE_SPACE = 4294967296.0; // 2^32
static const char *TAG = "asic";

// -------------------- INTERNAL HELPERS --------------------

static inline bool asic_state_valid(GlobalState *state)
{
    if (state == NULL) {
        ESP_LOGE(TAG, "GLOBAL_STATE is NULL");
        return false;
    }

    if (state->DEVICE_CONFIG.family.asic_count == 0) {
        ESP_LOGE(TAG, "ASIC count is 0");
        return false;
    }

    return true;
}

static inline int get_asic_id(GlobalState *state)
{
    return state->DEVICE_CONFIG.family.asic.id;
}

// -------------------- PUBLIC API --------------------

uint8_t ASIC_init(GlobalState *state)
{
    if (!asic_state_valid(state)) return 0;

    ESP_LOGI(TAG, "Initializing %dx %s",
             state->DEVICE_CONFIG.family.asic_count,
             state->DEVICE_CONFIG.family.asic.name);

    int id = get_asic_id(state);

    switch (id) {
        case BM1397:
            return BM1397_init(state->POWER_MANAGEMENT_MODULE.frequency_value,
                               state->DEVICE_CONFIG.family.asic_count,
                               state->DEVICE_CONFIG.family.asic.difficulty);

        case BM1366:
            return BM1366_init(state->POWER_MANAGEMENT_MODULE.frequency_value,
                               state->DEVICE_CONFIG.family.asic_count,
                               state->DEVICE_CONFIG.family.asic.difficulty);

        case BM1368:
            return BM1368_init(state->POWER_MANAGEMENT_MODULE.frequency_value,
                               state->DEVICE_CONFIG.family.asic_count,
                               state->DEVICE_CONFIG.family.asic.difficulty);

        case BM1370:
            return BM1370_init(state,
                               state->POWER_MANAGEMENT_MODULE.frequency_value,
                               state->DEVICE_CONFIG.family.asic_count,
                               state->DEVICE_CONFIG.family.asic.difficulty);

        default:
            ESP_LOGE(TAG, "Unknown ASIC ID: %d", id);
            return 0;
    }
}

task_result * ASIC_process_work(GlobalState *state)
{
    if (!asic_state_valid(state)) return NULL;

    switch (get_asic_id(state)) {
        case BM1397: return BM1397_process_work(state);
        case BM1366: return BM1366_process_work(state);
        case BM1368: return BM1368_process_work(state);
        case BM1370: return BM1370_process_work(state);
        default:
            ESP_LOGE(TAG, "Invalid ASIC ID in process_work");
            return NULL;
    }
}

int ASIC_set_max_baud(GlobalState *state)
{
    if (!asic_state_valid(state)) return 0;

    switch (get_asic_id(state)) {
        case BM1397: return BM1397_set_max_baud();
        case BM1366: return BM1366_set_max_baud();
        case BM1368: return BM1368_set_max_baud();
        case BM1370: return BM1370_set_max_baud();
        default:
            ESP_LOGE(TAG, "Invalid ASIC ID in set_max_baud");
            return 0;
    }
}

void ASIC_send_work(GlobalState *state, void *job)
{
    if (!asic_state_valid(state)) return;

    if (job == NULL) {
        ESP_LOGE(TAG, "NULL job passed to ASIC_send_work");
        return;
    }

    switch (get_asic_id(state)) {
        case BM1397: BM1397_send_work(state, job); break;
        case BM1366: BM1366_send_work(state, job); break;
        case BM1368: BM1368_send_work(state, job); break;
        case BM1370: BM1370_send_work(state, job); break;
        default:
            ESP_LOGE(TAG, "Invalid ASIC ID in send_work");
            break;
    }
}

void ASIC_set_version_mask(GlobalState *state, uint32_t mask)
{
    if (!asic_state_valid(state)) return;

    switch (get_asic_id(state)) {
        case BM1397: BM1397_set_version_mask(mask); break;
        case BM1366: BM1366_set_version_mask(mask); break;
        case BM1368: BM1368_set_version_mask(mask); break;
        case BM1370: BM1370_set_version_mask(mask); break;
        default:
            ESP_LOGE(TAG, "Invalid ASIC ID in set_version_mask");
            break;
    }
}

bool ASIC_set_frequency(GlobalState *state, float frequency)
{
    if (!asic_state_valid(state)) return false;

    switch (get_asic_id(state)) {
        case BM1397:
            ESP_LOGE(TAG, "Frequency transition not implemented for BM1397");
            return false;

        case BM1366:
            do_frequency_transition(frequency, BM1366_send_hash_frequency);
            return true;

        case BM1368:
            do_frequency_transition(frequency, BM1368_send_hash_frequency);
            return true;

        case BM1370:
            do_frequency_transition(frequency, BM1370_send_hash_frequency);
            return true;

        default:
            ESP_LOGE(TAG, "Invalid ASIC ID in set_frequency");
            return false;
    }
}

double ASIC_get_asic_job_frequency_ms(GlobalState *state)
{
    if (!asic_state_valid(state)) return 500;

    uint32_t count = state->DEVICE_CONFIG.family.asic_count;
    uint32_t cores = state->DEVICE_CONFIG.family.asic.small_core_count;
    float freq = state->POWER_MANAGEMENT_MODULE.frequency_value;

    switch (get_asic_id(state)) {
        case BM1397:
            if (freq == 0 || cores == 0) {
                ESP_LOGE(TAG, "Invalid freq/core config");
                return 500;
            }
            return (NONCE_SPACE / (double)(freq * cores * 1000)) / (double)count;

        case BM1366:
            return 2000.0 / count;

        case BM1368:
        case BM1370:
            return 500.0 / count;

        default:
            ESP_LOGE(TAG, "Invalid ASIC ID in get_job_freq");
            return 500;
    }
}

void ASIC_read_registers(GlobalState *state)
{
    if (!asic_state_valid(state)) return;

    switch (get_asic_id(state)) {
        case BM1397: BM1397_read_registers(); break;
        case BM1366: BM1366_read_registers(); break;
        case BM1368: BM1368_read_registers(); break;
        case BM1370: BM1370_read_registers(); break;
        default:
            ESP_LOGE(TAG, "Invalid ASIC ID in read_registers");
            break;
    }
}
