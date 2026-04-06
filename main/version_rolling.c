#include "version_rolling.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "version_rolling";

// Konfiguration
#define ADJUST_INTERVAL_MS   (10 * 60 * 1000)  // 10 Minuten
#define MIN_SHARES_BEFORE_ADJUST 50            // mindestens so viele Shares vor Neuordnung

// Dynamischer Zustand
static struct {
    uint32_t version_mask;          // aktuelle Maske (z.Zt. fest, später erweiterbar)
    uint32_t success_count[4];      // Erfolge pro Midstate (Index 0..3)
    uint32_t total_shares;
    uint8_t  order[4];              // aktuelle Reihenfolge (order[0] = erster Midstate)
    int64_t  last_adjust_time_us;
} vr = {
    .version_mask = 0x1FFFE000,     // Standard BIP320
    .success_count = {0,0,0,0},
    .total_shares = 0,
    .order = {0,1,2,3},             // initiale Reihenfolge
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
    if (now - vr.last_adjust_time_us < ADJUST_INTERVAL_MS * 1000LL) {
        return; // noch nicht Zeit
    }
    if (vr.total_shares < MIN_SHARES_BEFORE_ADJUST) {
        ESP_LOGD(TAG, "Not enough shares (%lu) for adjustment", vr.total_shares);
        return;
    }

    // Kopie der Erfolgszahlen für Sortierung
    uint32_t counts[4];
    memcpy(counts, vr.success_count, sizeof(counts));

    // Indizes nach absteigender Erfolgszahl sortieren (Bubble-Sort, klein genug)
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

    // Nur ändern, wenn sich die Reihenfolge wirklich unterscheidet
    if (memcmp(vr.order, new_order, 4) != 0) {
        ESP_LOGI(TAG, "Optimizing midstate order: %d%d%d%d -> %d%d%d%d (success: %lu/%lu/%lu/%lu)",
                 vr.order[0], vr.order[1], vr.order[2], vr.order[3],
                 new_order[0], new_order[1], new_order[2], new_order[3],
                 vr.success_count[0], vr.success_count[1],
                 vr.success_count[2], vr.success_count[3]);
        memcpy(vr.order, new_order, 4);
    } else {
        ESP_LOGD(TAG, "Order unchanged, still %d%d%d%d", vr.order[0], vr.order[1], vr.order[2], vr.order[3]);
    }

    // Optional: Erfolgszahlen zurücksetzen (gleitendes Fenster)
    memset(vr.success_count, 0, sizeof(vr.success_count));
    vr.total_shares = 0;
    vr.last_adjust_time_us = now;
}

void version_rolling_reset(void) {
    memset(vr.success_count, 0, sizeof(vr.success_count));
    vr.total_shares = 0;
    // Reihenfolge bleibt erhalten, nur Statistik zurückgesetzt
    ESP_LOGI(TAG, "Version rolling statistics reset");
}