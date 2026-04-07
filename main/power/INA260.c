#include <stdio.h>
#include "esp_log.h"
#include "i2c_bitaxe.h"
#include "INA260.h"

static const char *TAG = "INA260";
static i2c_master_dev_handle_t ina260_dev_handle = NULL;
static bool initialized = false;

esp_err_t INA260_init(void)
{
    if (initialized) {
        ESP_LOGW(TAG, "INA260 already initialized");
        return ESP_OK;
    }

    esp_err_t err = i2c_bitaxe_add_device(INA260_I2CADDR_DEFAULT, &ina260_dev_handle, TAG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add INA260 device");
        ina260_dev_handle = NULL;
        return err;
    }

    initialized = true;
    return ESP_OK;
}

float INA260_read_current(void)
{
    if (!initialized || ina260_dev_handle == NULL) {
        ESP_LOGE(TAG, "Device not initialized");
        return 0.0f;
    }

    uint8_t data[2];
    esp_err_t err = i2c_bitaxe_register_read(ina260_dev_handle, INA260_REG_CURRENT, data, 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read current: %s", esp_err_to_name(err));
        return 0.0f;
    }

    int16_t raw = (int16_t)((data[0] << 8) | data[1]);
    return (float)raw * 1.25f;  // 1.25 mA pro LSB
}

float INA260_read_voltage(void)
{
    if (!initialized || ina260_dev_handle == NULL) {
        ESP_LOGE(TAG, "Device not initialized");
        return 0.0f;
    }

    uint8_t data[2];
    esp_err_t err = i2c_bitaxe_register_read(ina260_dev_handle, INA260_REG_BUSVOLTAGE, data, 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read voltage: %s", esp_err_to_name(err));
        return 0.0f;
    }

    uint16_t raw = (data[0] << 8) | data[1];
    return (float)raw * 1.25f;  // 1.25 mV pro LSB
}

float INA260_read_power(void)
{
    if (!initialized || ina260_dev_handle == NULL) {
        ESP_LOGE(TAG, "Device not initialized");
        return 0.0f;
    }

    uint8_t data[2];
    esp_err_t err = i2c_bitaxe_register_read(ina260_dev_handle, INA260_REG_POWER, data, 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read power: %s", esp_err_to_name(err));
        return 0.0f;
    }

    uint16_t raw = (data[0] << 8) | data[1];
    return (float)raw * 10.0f;  // 10 mW pro LSB
}