// ============================================================
//  hashrate_monitor_task.c  –  Bitaxe GT800 optimiert
//
//  Fixes gegenüber Original:
//
//  [FIX-1] Zero-Hashrate-Guard in check_hashrate_anomaly()
//          Bei Pool-Reconnect / Stratum-Pause fällt current_hashrate
//          kurz auf 0. Der Original-Code zählte das als Anomalie
//          (0 < highest && 0 < threshold) → nach 3x → Reinitiate.
//          Das war die Hauptursache der "willkürlichen Neustarts".
//          Fix: current_hashrate == 0 → sofort return, kein Zählen.
//
//  [FIX-2] NaN/Inf-Schutz für lowerThreshold
//          Wenn expected_hashrate beim Task-Start noch 0.0 ist,
//          ergibt die Laufzeit-Berechnung in C: 0.0/0.0 = NaN.
//          NaN-Vergleiche sind immer FALSE → Monitor blind.
//          Fix: isfinite()-Check, Fallback auf sicheren Wert 0.50f.
//
//  [FIX-3] Threshold-Logging beim Start
//          Der tatsächlich aktive Threshold war bisher unsichtbar
//          im Log. Jetzt wird er explizit geloggt.
//
//  UNVERÄNDERT:
//  - Alle Messwert-Funktionen (update_hashrate, update_hash_counter)
//  - ASIC-Reinitiate-Logik (lowHashrateCount >= 3)
//  - Poll-Rate (5000ms)
//  - upperThresholdHashratePercent (2.00f)
//  - hashrate_monitor_register_read()
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

static const char *TAG = "hashrate_monitor";
static float highest_hashrate = 0.0f;
static uint8_t lowHashrateCount = 0;
static int reinitiateCount = 0;
static float lowerThresholdHashratePercent = 0.50f;
static float upperThresholdHashratePercent = 2.00f;

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
        float hashrate = hashrate_value * (float)HASHRATE_UNIT;
        measurement[asic_nr].hashrate = hashrate / 1e9f;
    }
}

static void update_hash_counter(uint32_t time_ms, uint32_t value, measurement_t * measurement)
{
    uint32_t previous_time_ms = measurement->time_ms;
    if (previous_time_ms != 0) {
        uint32_t duration_ms = time_ms - previous_time_ms;
        uint32_t counter = value - measurement->value;
        measurement->hashrate = hashCounterToGhs(duration_ms, counter);
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
    // current_hashrate = 0 tritt normal auf bei:
    //   - Pool-Reconnect (Stratum sendet kurz keine Jobs)
    //   - Netzwerkunterbrechung
    //   - Messregister noch nicht bereit nach Boot
    // Das ist KEIN ASIC-Fehler → nicht zählen, sofort zurück.
    // Original-Code zählte das als Anomalie → 3x → Reinitiate.
    if (current_hashrate < EPSILON) {
        // Zähler zurücksetzen damit kein teilweiser Zählstand bleibt
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

    if (lowHashrateCount >= 3) {
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
        // highest_hashrate ist statisch und wird nie verringert.
        // Nach Recovery baut der ASIC die Hashrate schrittweise auf
        // (z.B. 0 → 200 → 800 → 2897 GH/s). Ohne Reset gilt sofort:
        // current(200) < highest(2897) → Anomalie-Zähler läuft hoch
        // → nach 3 Zyklen neues Reinitiate → Reinitiate-Schleife.
        // Mit Reset: highest startet bei 0, kein false-positive während
        // der Aufwärmphase nach Recovery.
        highest_hashrate = 0.0f;
        ESP_LOGI(TAG, "highest_hashrate reset after recovery");
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
    // GT800 (2 ASICs, 4 Domains): ergibt 0.75 → Trigger bei 600 GH/s
    lowerThresholdHashratePercent = 1.0f - ((expected_hashrate / asic_count / hash_domains * 2.0f) / expected_hashrate);

    // [FIX-2] NaN/Inf-Schutz
    // Wenn expected_hashrate beim Task-Start noch 0.0 ist:
    // C berechnet 0.0/0.0 = NaN → alle Anomalie-Checks schlagen still fehl.
    // Fallback auf 0.50f als sicheren Minimalwert.
    if (!isfinite(lowerThresholdHashratePercent) || lowerThresholdHashratePercent <= 0.0f) {
        ESP_LOGW(TAG, "lowerThreshold calculation produced invalid value (expected_hashrate=%.1f), using fallback 0.50", expected_hashrate);
        lowerThresholdHashratePercent = 0.50f;
    }

    // [FIX-3] Threshold explizit loggen — war bisher unsichtbar
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
