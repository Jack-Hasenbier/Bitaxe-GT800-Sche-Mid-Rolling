#ifndef TMP1075_H_
#define TMP1075_H_

#include "esp_err.h"

#define TMP1075_I2CADDR_DEFAULT 0x4A
#define TMP1075_TEMP_REG        0x00
#define TMP1075_CONFIG_REG      0x01
#define TMP1075_LOW_LIMIT       0x02
#define TMP1075_HIGH_LIMIT      0x03
#define TMP1075_DEVICE_ID       0x0F

esp_err_t TMP1075_init(int temp_offset_param);
float TMP1075_read_temperature(int device_index);

#endif /* TMP1075_H_ */