#ifndef VERSION_ROLLING_H_
#define VERSION_ROLLING_H_

#include <stdint.h>
#include <stdbool.h>

// Initialisierung (einmal beim Start aufrufen)
void version_rolling_init(void);

// Erfolg einer Midstate-Version registrieren (midstate_index 0..3)
void version_rolling_record_success(uint8_t midstate_index);

// Aktuelle optimierte Reihenfolge abrufen (Array mit 4 Einträgen, z.B. {2,0,3,1})
const uint8_t* version_rolling_get_order(void);

// Aktuelle Version-Maske abrufen (optional, falls dynamisch)
uint32_t version_rolling_get_mask(void);

// Periodischen Adjuster aufrufen (alle 10 Minuten oder nach vielen Shares)
void version_rolling_adjust(void);

// Statistik zurücksetzen (z.B. nach einem neuen Block)
void version_rolling_reset(void);

#endif /* VERSION_ROLLING_H_ */