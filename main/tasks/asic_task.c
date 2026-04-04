// ============================================================
//  asic_task.c  –  Bitaxe GT800 optimiert (Scheduler)
//
//  NEU gegenüber Original:
//   [SCHED-4] Zeile 55 — abandon_work Fast-Path im Semaphore-Wait.
//             Wenn der Pool einen neuen Block meldet (abandon_work=1),
//             wartet asic_task NICHT mehr den vollen Semaphore-Timeout
//             (250ms) ab. Stattdessen wird der nächste Job sofort
//             aus der Queue geholt.
//             → Latenz bei neuem Block: max ~5ms statt bis zu 250ms.
//             → Neue Blöcke werden sofort an den ASIC weitergeleitet.
//             abandon_work wird von create_jobs_task auf 0 zurückgesetzt
//             (nur im asic_task, andere Tasks lesen es noch).
//
//  Unverändert gegenüber Original:
//   - ASIC_initalized-Check
//   - queue_dequeue() (blockierend, kein Timeout)
//   - xSemaphoreTake() als Taktgeber
//   - Kein Watchdog-Zähler (wurde zuvor entfernt, kein Nutzen)
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

    while (1)
    {
        if (!GLOBAL_STATE->ASIC_initalized) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        bm_job *next_bm_job = (bm_job *)queue_dequeue(&GLOBAL_STATE->ASIC_jobs_queue);

        ASIC_send_work(GLOBAL_STATE, next_bm_job);

        // [SCHED-4] abandon_work Fast-Path
        // abandon_work wird von stratum_task/cleanQueue() gesetzt wenn der Pool
        // einen neuen Block meldet (mining.notify mit clean_jobs=true).
        //
        // Original-Design (create_jobs_task.c):
        //   cleanQueue()        → abandon_work=1, Queues geleert
        //   create_jobs_task    → erkennt abandon_work=1, xSemaphoreGive(), abandon_work=0
        //
        // asic_task LIEST abandon_work NUR — setzt es NICHT zurück.
        // Das Reset ist Aufgabe von create_jobs_task (Original-Design beibehalten).
        // Würden wir abandon_work=0 hier setzen, sieht create_jobs_task nie den
        // Wert 1 → kein xSemaphoreGive() → asic_task blockiert bis Timeout.
        //
        // Was wir tun: Semaphore-Warten überspringen für sofortige Job-Reaktion.
        // bm1370_scheduler.c liest abandon_work in BM1370_send_work() → setzt
        // last_abandon_time_us korrekt (passiert vor diesem Check).
        if (GLOBAL_STATE->abandon_work) {
            // Nicht warten — neuer Block hat Priorität
            // Semaphore leeren falls sie noch signalisiert wurde
            xSemaphoreTake(GLOBAL_STATE->ASIC_TASK_MODULE.semaphore, 0);
            ESP_LOGD(TAG, "abandon_work: skipping semaphore wait for new block");
        } else {
            // Normaler Betrieb: auf ASIC-Fertig-Signal oder Timeout warten
            xSemaphoreTake(
                GLOBAL_STATE->ASIC_TASK_MODULE.semaphore,
                (TickType_t)(asic_job_frequency_ms / portTICK_PERIOD_MS)
            );
        }
    }
}
