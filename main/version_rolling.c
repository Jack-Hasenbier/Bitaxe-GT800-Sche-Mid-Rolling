#include "version_rolling.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "version_rolling";

#define ADJUST_INTERVAL_MS   (3 * 60 * 1000)   // Alle 3 Minuten anpassen, abhängig von der Hashrate und der Anzahl der Shares
#define MIN_SHARES_BEFORE_ADJUST 5              // Hashrate abhängig, aber mindestens 5 Shares, bevor Anpassung erfolgt
                                                // in etwas 20 anpassungen in der Stunden, abhängig von shares/h
static struct {
    uint32_t version_mask;
    uint32_t success_count[4];
    uint32_t total_shares;
    uint8_t  order[4];
    int64_t  last_adjust_time_us;
} vr = {
    .version_mask = 0x1FFFE000,
    .success_count = {0,0,0,0},
    .total_shares = 0,
    .order = {0,1,2,3},
    .last_adjust_time_us = 0
};

void version_rolling_init(void) {
    vr.last_adjust_time_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Version Rolling initialized, mask=0x%08lX, order=%d%d%d%d",
             vr.version_mask, vr.order[0], vr.order[1], vr.order[2], vr.order[3]);
}

void version_rolling_record_success(uint8_t midstate_index) {
    if (midstate_index >= 4) return;
    vr.success_count[midstate_index]++;
    vr.total_shares++;
}

const uint8_t* version_rolling_get_order(void) {
    return vr.order;
}

uint32_t version_rolling_get_mask(void) {
    return vr.version_mask;
}

void version_rolling_adjust(void) {
    int64_t now = esp_timer_get_time();
     if (now - vr.last_adjust_time_us < ADJUST_INTERVAL_MS * 1000LL) return;  // Zeit noch nicht erreicht
     if (vr.total_shares < MIN_SHARES_BEFORE_ADJUST) return;                  // Zu wenige Shares
    // Beide Bedingungen erfüllt → optimieren

    uint32_t counts[4];
    memcpy(counts, vr.success_count, sizeof(counts));
    uint8_t new_order[4] = {0,1,2,3};
    for (int i = 0; i < 3; i++) {
        for (int j = i+1; j < 4; j++) {
            if (counts[new_order[j]] > counts[new_order[i]]) {
                uint8_t tmp = new_order[i];
                new_order[i] = new_order[j];
                new_order[j] = tmp;
            }
        }
    }

if (memcmp(vr.order, new_order, 4) != 0) {
    // Magenta Farbe: \033[35m, Reset: \033[0m (Standardfarbe)
    ESP_LOGI(TAG, "\033[33mOptimizing midstate order: %d%d%d%d -> %d%d%d%d (success: %lu/%lu/%lu/%lu)\033[0m",
             vr.order[0], vr.order[1], vr.order[2], vr.order[3],
             new_order[0], new_order[1], new_order[2], new_order[3],
             vr.success_count[0], vr.success_count[1],
             vr.success_count[2], vr.success_count[3]);
    memcpy(vr.order, new_order, 4);
}

    memset(vr.success_count, 0, sizeof(vr.success_count));
    vr.total_shares = 0;
    vr.last_adjust_time_us = now;
}

void version_rolling_reset(void) {
    memset(vr.success_count, 0, sizeof(vr.success_count));
    vr.total_shares = 0;
    ESP_LOGI(TAG, "Version rolling statistics reset");
}