// ============================================================
//  hashrate_monitor_task.c  –  Bitaxe GT800 optimiert
//
//  Fixes gegenüber Original:
//
//  [FIX-1] Zero-Hashrate-Guard in check_hashrate_anomaly()
//  [FIX-2] NaN/Inf-Schutz für lowerThreshold
//  [FIX-3] Threshold-Logging beim Start
//  [FIX-6] highest_hashrate nach Recovery zurücksetzen
//  [FIX-7] Messwert-Strukturen nach Recovery zurücksetzen
//
//  [FIX-8] Stale-Packet Guard & Hashrate Sanity Check
//          Behebt den "2 EH/s Spike" nach der Recovery. Alte Pakete
//          im UART-Queue überschrieben bisher nach clear_measurements 
//          die Zähler, was beim ersten frischen Poll zu riesigen Deltas 
//          (Unterläufen) führte. 
//          - just_recovered Flag leert die Messungen direkt vor dem Poll.
//          - MAX_VALID_HASHRATE_GHS ignoriert absurde Berechnungen.
// ============================================================

#include <string.h>
#include <esp_heap_caps.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "system.h"
#include "common.h"
#include "asic.h"
#include "utils.h"
#include "asic_init.h"
#include "driver/uart.h"

#define EPSILON 0.0001f
#define POLL_RATE 5000
#define HASHRATE_UNIT 0x100000uLL
#define MAX_VALID_HASHRATE_GHS 10000000.0f // [FIX-8] 10 PH/s Limit für Sanity Check

static const char *TAG = "hashrate_monitor";
static float highest_hashrate = 0.0f;
static uint8_t lowHashrateCount = 0;
static int reinitiateCount = 0;
static float lowerThresholdHashratePercent = 0.50f;
static float upperThresholdHashratePercent = 2.00f;
static bool just_recovered = false; // [FIX-8] Flag für Post-Delay Clear

static float sum_hashrates(measurement_t * measurement, int asic_count)
{
    if (asic_count == 1) return measurement[0].hashrate;

    float total = 0;
    for (int asic_nr = 0; asic_nr < asic_count; asic_nr++) {
        total += measurement[asic_nr].hashrate;
    }
    return total;
}

static void clear_measurements(GlobalState * GLOBAL_STATE)
{
    HashrateMonitorModule * HASHRATE_MONITOR_MODULE = &GLOBAL_STATE->HASHRATE_MONITOR_MODULE;

    int asic_count = GLOBAL_STATE->DEVICE_CONFIG.family.asic_count;
    int hash_domains = GLOBAL_STATE->DEVICE_CONFIG.family.asic.hash_domains;

    memset(HASHRATE_MONITOR_MODULE->total_measurement, 0, asic_count * sizeof(measurement_t));
    if (hash_domains > 0) {
        memset(HASHRATE_MONITOR_MODULE->domain_measurements[0], 0, asic_count * hash_domains * sizeof(measurement_t));
    }
    memset(HASHRATE_MONITOR_MODULE->error_measurement, 0, asic_count * sizeof(measurement_t));
}

static void update_hashrate(uint32_t value, measurement_t * measurement, int asic_nr)
{
    uint8_t flag_long = (value & 0x80000000) >> 31;
    uint32_t hashrate_value = value & 0x7FFFFFFF;

    if (hashrate_value != 0x007FFFFF && !flag_long) {
        float hashrate = (hashrate_value * (float)HASHRATE_UNIT) / 1e9f;
        // [FIX-8] Sanity Check: Ignoriere unmögliche Werte (Verhindert 2 EH/s Glitches)
        if (hashrate < MAX_VALID_HASHRATE_GHS) {
            measurement[asic_nr].hashrate = hashrate;
        }
    }
}

static void update_hash_counter(uint32_t time_ms, uint32_t value, measurement_t * measurement)
{
    uint32_t previous_time_ms = measurement->time_ms;
    
    // [FIX-8] time_ms > previous_time_ms stellt sicher, dass duration_ms niemals negativ wird.
    if (previous_time_ms != 0 && time_ms > previous_time_ms) {
        uint32_t duration_ms = time_ms - previous_time_ms;
        uint32_t counter = value - measurement->value;
        float new_hr = hashCounterToGhs(duration_ms, counter);
        
        // [FIX-8] Sanity Check: Schützt vor Stale-Packet Unterläufen
        if (new_hr < MAX_VALID_HASHRATE_GHS) {
            measurement->hashrate = new_hr;
        } else {
            ESP_LOGW(TAG, "Ignoring impossible hashrate spike: %.3f Gh/s", new_hr);
        }
    }

    measurement->value = value;
    measurement->time_ms = time_ms;
}

void check_hashrate_anomaly(void *pvParameters, float current_hashrate)
{
    GlobalState * GLOBAL_STATE = (GlobalState *)pvParameters;
    HashrateMonitorModule * HASHRATE_MONITOR_MODULE = &GLOBAL_STATE->HASHRATE_MONITOR_MODULE;

    float expected_hashrate = GLOBAL_STATE->POWER_MANAGEMENT_MODULE.expected_hashrate;

    // [FIX-1] Zero-Hashrate-Guard
    if (current_hashrate < EPSILON) {
        lowHashrateCount = 0;
        return;
    }

    if (current_hashrate < highest_hashrate &&
        (current_hashrate < expected_hashrate * lowerThresholdHashratePercent ||
         current_hashrate > expected_hashrate * upperThresholdHashratePercent)) {
        lowHashrateCount++;
        ESP_LOGW(TAG, "Low hashrate detected: %.3f Gh/s (expected: %.3f Gh/s, threshold: %.0f Gh/s). Count: %d",
                 current_hashrate, expected_hashrate,
                 expected_hashrate * lowerThresholdHashratePercent,
                 lowHashrateCount);
    } else {
        lowHashrateCount = 0;
        return;
    }

    if (lowHashrateCount >= 10) {  // Increased threshold to prevent false triggers from temp sensor issues
        reinitiateCount++;
        ESP_LOGW(TAG, "Reinitiating ASICs due to sustained low hashrate. Reinitiate count: %d", reinitiateCount);

        ESP_LOGI(TAG, "Stopping ASIC tasks...");
        GLOBAL_STATE->ASIC_initalized = false;
        vTaskDelay(500 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Flushing UART buffers...");
        uart_flush(UART_NUM_1);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        uint8_t chip_count = asic_initialize(GLOBAL_STATE, ASIC_INIT_RECOVERY, 2000);

        if (chip_count > 0) {
            ESP_LOGI(TAG, "Resuming normal operation.");
        }

        lowHashrateCount = 0;

        // [FIX-6] highest_hashrate nach Recovery zurücksetzen.
        highest_hashrate = 0.0f;
        ESP_LOGI(TAG, "highest_hashrate reset after recovery");

        // [FIX-7] Initiale Leerung
        clear_measurements(GLOBAL_STATE);
        ESP_LOGI(TAG, "measurements cleared after recovery");
        
        // [FIX-8] Flag setzen, um Pakete, die sich während der folgenden Pause 
        // im UART-Buffer verfangen haben, vor dem nächsten Durchlauf endgültig zu löschen.
        just_recovered = true;
    }
}

void hashrate_monitor_task(void *pvParameters)
{
    GlobalState * GLOBAL_STATE = (GlobalState *)pvParameters;
    HashrateMonitorModule * HASHRATE_MONITOR_MODULE = &GLOBAL_STATE->HASHRATE_MONITOR_MODULE;
    SystemModule * SYSTEM_MODULE = &GLOBAL_STATE->SYSTEM_MODULE;

    int asic_count = GLOBAL_STATE->DEVICE_CONFIG.family.asic_count;
    int hash_domains = GLOBAL_STATE->DEVICE_CONFIG.family.asic.hash_domains;

    float expected_hashrate = GLOBAL_STATE->POWER_MANAGEMENT_MODULE.expected_hashrate;

    // Laufzeit-Berechnung des unteren Schwellwerts
    lowerThresholdHashratePercent = 1.0f - ((expected_hashrate / asic_count / hash_domains * 2.0f) / expected_hashrate);

    // [FIX-2] NaN/Inf-Schutz
    if (!isfinite(lowerThresholdHashratePercent) || lowerThresholdHashratePercent <= 0.0f) {
        ESP_LOGW(TAG, "lowerThreshold calculation produced invalid value (expected_hashrate=%.1f), using fallback 0.50", expected_hashrate);
        lowerThresholdHashratePercent = 0.50f;
    }

    // [FIX-3] Threshold explizit loggen
    ESP_LOGI(TAG, "Hashrate thresholds: lower=%.2f (%.0f GH/s), upper=%.2f (%.0f GH/s)",
             lowerThresholdHashratePercent,
             expected_hashrate * lowerThresholdHashratePercent,
             upperThresholdHashratePercent,
             expected_hashrate * upperThresholdHashratePercent);

    HASHRATE_MONITOR_MODULE->total_measurement = heap_caps_malloc(asic_count * sizeof(measurement_t), MALLOC_CAP_SPIRAM);
    if (hash_domains > 0) {
        measurement_t* data = heap_caps_malloc(asic_count * hash_domains * sizeof(measurement_t), MALLOC_CAP_SPIRAM);
        HASHRATE_MONITOR_MODULE->domain_measurements = heap_caps_malloc(asic_count * sizeof(measurement_t*), MALLOC_CAP_SPIRAM);
        for (size_t asic_nr = 0; asic_nr < asic_count; asic_nr++) {
            HASHRATE_MONITOR_MODULE->domain_measurements[asic_nr] = data + (asic_nr * hash_domains);
        }
    }
    HASHRATE_MONITOR_MODULE->error_measurement = heap_caps_malloc(asic_count * sizeof(measurement_t), MALLOC_CAP_SPIRAM);

    clear_measurements(GLOBAL_STATE);

    HASHRATE_MONITOR_MODULE->is_initialized = true;

    TickType_t taskWakeTime = xTaskGetTickCount();
    while (1) {
        // [FIX-8] Zweite Leerung: Entfernt Stale Packets, die während der letzten 
        // Pause (nach einer Anomalie) vom UART-Task verarbeitet wurden.
        if (just_recovered) {
            clear_measurements(GLOBAL_STATE);
            just_recovered = false;
        }

        ASIC_read_registers(GLOBAL_STATE);

        vTaskDelay(100 / portTICK_PERIOD_MS);

        float current_hashrate = sum_hashrates(HASHRATE_MONITOR_MODULE->total_measurement, asic_count);
        if (current_hashrate > highest_hashrate) {
            highest_hashrate = current_hashrate;
            ESP_LOGI(TAG, "New Highest Hashrate: %.3f Gh/s", highest_hashrate);
        }
        float error_hashrate = sum_hashrates(HASHRATE_MONITOR_MODULE->error_measurement, asic_count);

        SYSTEM_MODULE->current_hashrate = current_hashrate;
        SYSTEM_MODULE->error_percentage = current_hashrate > 0 ? error_hashrate / current_hashrate * 100.f : 0;
        check_hashrate_anomaly(pvParameters, current_hashrate);
        vTaskDelayUntil(&taskWakeTime, POLL_RATE / portTICK_PERIOD_MS);
    }
}

void hashrate_monitor_register_read(void *pvParameters, register_type_t register_type, uint8_t asic_nr, uint32_t value)
{
    uint32_t time_ms = esp_timer_get_time() / 1000;

    GlobalState * GLOBAL_STATE = (GlobalState *)pvParameters;
    HashrateMonitorModule * HASHRATE_MONITOR_MODULE = &GLOBAL_STATE->HASHRATE_MONITOR_MODULE;

    int asic_count = GLOBAL_STATE->DEVICE_CONFIG.family.asic_count;

    if (asic_nr >= asic_count) {
        ESP_LOGE(TAG, "Asic nr out of bounds [%d]", asic_nr);
        return;
    }

    switch (register_type) {
        case REGISTER_HASHRATE:
            update_hashrate(value, HASHRATE_MONITOR_MODULE->total_measurement, asic_nr);
            break;
        case REGISTER_TOTAL_COUNT:
            update_hash_counter(time_ms, value, &HASHRATE_MONITOR_MODULE->total_measurement[asic_nr]);
            break;
        case REGISTER_DOMAIN_0_COUNT:
            update_hash_counter(time_ms, value, &HASHRATE_MONITOR_MODULE->domain_measurements[asic_nr][0]);
            break;
        case REGISTER_DOMAIN_1_COUNT:
            update_hash_counter(time_ms, value, &HASHRATE_MONITOR_MODULE->domain_measurements[asic_nr][1]);
            break;
        case REGISTER_DOMAIN_2_COUNT:
            update_hash_counter(time_ms, value, &HASHRATE_MONITOR_MODULE->domain_measurements[asic_nr][2]);
            break;
        case REGISTER_DOMAIN_3_COUNT:
            update_hash_counter(time_ms, value, &HASHRATE_MONITOR_MODULE->domain_measurements[asic_nr][3]);
            break;
        case REGISTER_ERROR_COUNT:
            update_hash_counter(time_ms, value, &HASHRATE_MONITOR_MODULE->error_measurement[asic_nr]);
            break;
        case REGISTER_INVALID:
            ESP_LOGE(TAG, "Invalid register type");
            break;
    }
}