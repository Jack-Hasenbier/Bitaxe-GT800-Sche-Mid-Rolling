#ifndef VERSION_ROLLING_H_
#define VERSION_ROLLING_H_

#include <stdint.h>
#include <stdbool.h>

void version_rolling_init(void);
void version_rolling_record_success(uint8_t midstate_index);
const uint8_t* version_rolling_get_order(void);
uint32_t version_rolling_get_mask(void);
void version_rolling_adjust(void);
void version_rolling_reset(void);

#endif