#include "esp_event.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "asic_result_task.h"
#include "asic_task.h"
#include "create_jobs_task.h"
#include "hashrate_monitor_task.h"
#include "statistics_task.h"
#include "system.h"
#include "http_server.h"
#include "serial.h"
#include "stratum_task.h"
#include "i2c_bitaxe.h"
#include "adc.h"
#include "nvs_config.h"
#include "self_test.h"
#include "asic.h"
#include "bap/bap.h"
#include "device_config.h"
#include "connect.h"
#include "asic_reset.h"
#include "asic_init.h"
#include "../../main/version_rolling.h"   // Dynamisches Version‑Rolling

static GlobalState GLOBAL_STATE;
static const char * TAG = "bitaxe";

// ========== Optimierte Task‑Prioritäten ==========
// Höhere Zahl = höhere Priorität
#define PRIO_ASIC_RESULT        15   // Nonce‑Ergebnisse sofort verarbeiten
#define PRIO_ASIC               12   // ASIC‑Kommunikation (timing‑kritisch)
#define PRIO_STRATUM_MINER      8    // Erzeugt Jobs aus Stratum‑Shares
#define PRIO_POWER_MANAGEMENT   7    // Stromregelung (wichtig, aber nicht kritisch)
#define PRIO_STRATUM_ADMIN      5    // Verbindungsmanagement (periodisch)
#define PRIO_HASHRATE_MONITOR   4    // Nur Statistik
#define PRIO_STATISTICS         3    // Niedrigste Priorität

// ========== Optimierte Stack‑Größen (in Bytes) ==========
#define STACK_POWER_MANAGEMENT  8192
#define STACK_STRATUM_ADMIN     12288
#define STACK_STRATUM_MINER     10240
#define STACK_ASIC              12288
#define STACK_ASIC_RESULT       12288
#define STACK_HASHRATE_MONITOR  8192
#define STACK_STATISTICS        8192

// ========== Hilfsmakro für kritische Task‑Erstellung ==========
#define CHECK_TASK(expr, task_name) do { \
    if ((expr) != pdPASS) { \
        ESP_LOGE(TAG, "FATAL: Failed to create task '%s'", task_name); \
        esp_restart(); \
    } \
} while(0)

void app_main(void)
{
    ESP_LOGI(TAG, "Welcome to the bitaxe - FOSS || GTFO!");

    // PSRAM‑Prüfung
    if (!esp_psram_is_initialized()) {
        ESP_LOGE(TAG, "No PSRAM available on ESP32 device!");
        GLOBAL_STATE.psram_is_available = false;
    } else {
        GLOBAL_STATE.psram_is_available = true;
    }

    // I2C initialisieren
    ESP_ERROR_CHECK(i2c_bitaxe_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // ASIC‑Reset auf Low halten (stromsparend)
    ESP_ERROR_CHECK(asic_hold_reset_low());
    ESP_LOGI(TAG, "RST pin initialized to low");

    vTaskDelay(100 / portTICK_PERIOD_MS); // I2C‑Stabilisierung

    ADC_init();

    if (nvs_config_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init NVS");
        return;
    }

    if (device_config_init(&GLOBAL_STATE) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init device config");
        return;
    }

    version_rolling_init();

    if (self_test(&GLOBAL_STATE)) return;

    SYSTEM_init_system(&GLOBAL_STATE);
    wifi_init(&GLOBAL_STATE);

    if (SYSTEM_init_peripherals(&GLOBAL_STATE) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init peripherals");
        return;
    }

    // Power‑Management‑Task (Priorität 7, Stack 8k)
    CHECK_TASK(xTaskCreate(POWER_MANAGEMENT_task, "power mgmt",
                           STACK_POWER_MANAGEMENT, &GLOBAL_STATE,
                           PRIO_POWER_MANAGEMENT, NULL), "power mgmt");

    // REST‑API starten (läuft im eigenen Task des HTTP‑Servers)
    start_rest_server(&GLOBAL_STATE);

    // BAP‑Interface (optional)
    esp_err_t bap_ret = BAP_init(&GLOBAL_STATE);
    if (bap_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BAP interface: %d", bap_ret);
        // Nicht kritisch → weiter
    }

    // Warten auf Netzwerkverbindung mit Timeout (30 Sekunden)
    int timeout_ms = 30000;
    while (!GLOBAL_STATE.SYSTEM_MODULE.is_connected && timeout_ms > 0) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        timeout_ms -= 100;
    }
    if (!GLOBAL_STATE.SYSTEM_MODULE.is_connected) {
        ESP_LOGW(TAG, "No network connection after 30s – continuing in AP mode");
    }

    queue_init(&GLOBAL_STATE.stratum_queue);
    queue_init(&GLOBAL_STATE.ASIC_jobs_queue);

    if (asic_initialize(&GLOBAL_STATE, ASIC_INIT_COLD_BOOT, 0) == 0) {
        ESP_LOGE(TAG, "ASIC initialization failed – restarting");
        esp_restart();
    }

    // ========== Task‑Erstellung mit optimierten Parametern ==========
    // stratum_admin (Priorität 5)
    CHECK_TASK(xTaskCreate(stratum_task, "stratum admin",
                           STACK_STRATUM_ADMIN, &GLOBAL_STATE,
                           PRIO_STRATUM_ADMIN, NULL), "stratum admin");

    // stratum_miner (Priorität 8)
    CHECK_TASK(xTaskCreate(create_jobs_task, "stratum miner",
                           STACK_STRATUM_MINER, &GLOBAL_STATE,
                           PRIO_STRATUM_MINER, NULL), "stratum miner");

    // ASIC‑Task (Priorität 12) – **kein** PSRAM, da zeitkritisch
    CHECK_TASK(xTaskCreate(ASIC_task, "asic",
                           STACK_ASIC, &GLOBAL_STATE,
                           PRIO_ASIC, NULL), "asic");

    // ASIC‑Result‑Task (Priorität 15) – höchste Priorität, kein PSRAM
    CHECK_TASK(xTaskCreate(ASIC_result_task, "asic result",
                           STACK_ASIC_RESULT, &GLOBAL_STATE,
                           PRIO_ASIC_RESULT, NULL), "asic result");

    // Hashrate‑Monitor (Priorität 4) – PSRAM erlaubt (langsamer, aber unkritisch)
    CHECK_TASK(xTaskCreateWithCaps(hashrate_monitor_task, "hash mon",
                                   STACK_HASHRATE_MONITOR, &GLOBAL_STATE,
                                   PRIO_HASHRATE_MONITOR, NULL, MALLOC_CAP_SPIRAM),
                                   "hash mon");

    // Statistik‑Task (Priorität 3) – PSRAM erlaubt
    CHECK_TASK(xTaskCreateWithCaps(statistics_task, "stats",
                                   STACK_STATISTICS, &GLOBAL_STATE,
                                   PRIO_STATISTICS, NULL, MALLOC_CAP_SPIRAM),
                                   "stats");

    ESP_LOGI(TAG, "All tasks created successfully – system running");
}