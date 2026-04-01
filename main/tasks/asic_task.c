// ============================================================
//  asic_task.c  –  Bitaxe GT800
//  Gegenüber Original: keine Änderungen.
//  Unnötiger Semaphor-Zähler entfernt.
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

        xSemaphoreTake(
            GLOBAL_STATE->ASIC_TASK_MODULE.semaphore,
            (TickType_t)(asic_job_frequency_ms / portTICK_PERIOD_MS)
        );
    }
}
