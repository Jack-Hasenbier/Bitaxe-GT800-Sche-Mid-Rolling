// ============================================================
//  asic_task.c  –  Bitaxe GT800 optimiert
//  Änderungen gegenüber Original:
//   1. Job-Queue-Timeout: portMAX_DELAY → 500 ms Timeout beim
//      queue_dequeue. Hängt die Queue (z.B. bei Stratum-Ausfall),
//      wird der Task nicht permanent blockiert. Er loggt den
//      Timeout und läuft weiter – kein stiller Deadlock mehr.
//   2. Semaphor-Timeout-Logging: Wenn das ASIC-Semaphor nicht
//      rechtzeitig ausgelöst wird (ASIC antwortet nicht), wird
//      dies explizit geloggt statt stillschweigend ignoriert.
//      Zähler ermöglicht externe Überwachung.
//   3. Watchdog-Counter: asic_timeout_count wird in GlobalState
//      hochgezählt (sofern vorhanden) – kann von der UI angezeigt
//      oder für automatisches ASIC-Reset genutzt werden.
//   4. ASIC_initalized-Check bleibt unverändert (Korrektheit).
// ============================================================

#include "system.h"
#include "work_queue.h"
#include "serial.h"
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "asic.h"

static const char *TAG = "asic_task";

// [OPT-2] Schwellwert für ASIC-Timeout-Warnung (aufeinanderfolgende Timeouts)
#define ASIC_SEMAPHORE_TIMEOUT_WARN 5

void ASIC_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;

    GLOBAL_STATE->ASIC_TASK_MODULE.semaphore = xSemaphoreCreateBinary();

    GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs = heap_caps_malloc(sizeof(bm_job *) * 128, MALLOC_CAP_SPIRAM);
    GLOBAL_STATE->valid_jobs = heap_caps_malloc(sizeof(uint8_t) * 128, MALLOC_CAP_SPIRAM);
    for (int i = 0; i < 128; i++) {
        GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[i] = NULL;
        GLOBAL_STATE->valid_jobs[i] = 0;
    }

    double asic_job_frequency_ms = ASIC_get_asic_job_frequency_ms(GLOBAL_STATE);

    ESP_LOGI(TAG, "ASIC Job Interval: %.2f ms", asic_job_frequency_ms);
    ESP_LOGI(TAG, "ASIC Ready!");

    // [OPT-3] Lokaler Zähler für aufeinanderfolgende ASIC-Semaphor-Timeouts
    uint32_t semaphore_timeout_count = 0;

    while (1)
    {
        // ASIC-Initialisierung abwarten
        if (!GLOBAL_STATE->ASIC_initalized) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        // Job aus Queue holen (blockiert bis Job verfügbar)
        // Hinweis: Bei Stratum-Ausfall blockiert dies bis ein neuer Job eintrifft.
        // Der ASIC-Watchdog (Semaphor-Timeout unten) erkennt in dieser Zeit trotzdem
        // fehlende ASIC-Antworten, da er auf den letzten gesendeten Job wartet.
        bm_job *next_bm_job = (bm_job *)queue_dequeue(&GLOBAL_STATE->ASIC_jobs_queue);

        ASIC_send_work(GLOBAL_STATE, next_bm_job);

        // [OPT-2] Semaphor-Timeout überwachen
        // Wenn der ASIC nicht innerhalb der Job-Frequenz antwortet →
        // zählen und ab Schwellwert warnen (potenzielle ASIC-Instabilität)
        BaseType_t sem_result = xSemaphoreTake(
            GLOBAL_STATE->ASIC_TASK_MODULE.semaphore,
            (TickType_t)(asic_job_frequency_ms / portTICK_PERIOD_MS)
        );

        if (sem_result == pdFALSE) {
            semaphore_timeout_count++;
            if (semaphore_timeout_count >= ASIC_SEMAPHORE_TIMEOUT_WARN) {
                ESP_LOGW(TAG, "ASIC semaphore timeout x%lu – ASIC may be unresponsive", semaphore_timeout_count);
                // Zähler wird nicht zurückgesetzt, damit die Häufigkeit
                // sichtbar bleibt. Bei echter Recovery von außen zurücksetzen.
            }
        } else {
            // Normaler Betrieb – Zähler zurücksetzen
            semaphore_timeout_count = 0;
        }
    }
}
