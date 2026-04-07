#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "i2c_bitaxe.h"
#include "TMP1075.h"

static const char *TAG = "TMP1075";
static i2c_master_dev_handle_t tmp1075_dev_handle[2];
static int temp_offset = 0;
static bool initialized = false;

static float convert_raw_to_celsius(uint16_t raw) {
    int16_t val = (int16_t)(raw >> 4);
    if (val & 0x0800) {
        val |= 0xF000;
    }
    return (float)val * 0.0625f;
}

esp_err_t TMP1075_init(int temp_offset_param) {
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    esp_err_t err;
    err = i2c_bitaxe_add_device(TMP1075_I2CADDR_DEFAULT,     &tmp1075_dev_handle[0], "TMP1075-0");
    if (err != ESP_OK) return err;
    err = i2c_bitaxe_add_device(TMP1075_I2CADDR_DEFAULT + 1, &tmp1075_dev_handle[1], "TMP1075-1");
    if (err != ESP_OK) return err;

    temp_offset = temp_offset_param;
    initialized = true;
    ESP_LOGI(TAG, "Initialized with offset %d°C", temp_offset);
    return ESP_OK;
}

float TMP1075_read_temperature(int device_index) {
    if (!initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return -273.15f;
    }
    if (device_index < 0 || device_index >= 2) {
        ESP_LOGE(TAG, "Invalid index %d", device_index);
        return -273.15f;
    }
    if (tmp1075_dev_handle[device_index] == NULL) {
        ESP_LOGE(TAG, "Device handle %d is NULL", device_index);
        return -273.15f;
    }

    uint8_t data[2];
    esp_err_t err = i2c_bitaxe_register_read(tmp1075_dev_handle[device_index], TMP1075_TEMP_REG, data, 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read failed for device %d: %s", device_index, esp_err_to_name(err));
        return -273.15f;
    }

    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
    float temp = convert_raw_to_celsius(raw) + temp_offset;
    ESP_LOGD(TAG, "Device %d: raw=0x%04X, temp=%.2f°C", device_index, raw, temp);
    return temp;
}