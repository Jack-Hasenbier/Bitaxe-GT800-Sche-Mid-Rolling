#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "i2c_bitaxe.h"
#include "DS4432U.h"

#define DS4432U_SENSOR_ADDR 0x48
#define DS4432U_OUT0_REG 0xF8
#define DS4432U_OUT1_REG 0xF9

#define BITAXE_IFS 0.000098921
#define BITAXE_RA 4750.0
#define BITAXE_RB 3320.0
#define BITAXE_VNOM 1.451
#define BITAXE_VMAX 2.39
#define BITAXE_VMIN 0.046
#define TPS40305_VFB 0.6

static const char *TAG = "DS4432U";
static i2c_master_dev_handle_t ds4432u_dev_handle = NULL;
static bool initialized = false;

esp_err_t DS4432U_init(void)
{
    if (initialized) {
        ESP_LOGW(TAG, "DS4432U already initialized");
        return ESP_OK;
    }

    esp_err_t err = i2c_bitaxe_add_device(DS4432U_SENSOR_ADDR, &ds4432u_dev_handle, TAG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add DS4432U device");
        ds4432u_dev_handle = NULL;
        return err;
    }

    initialized = true;
    return ESP_OK;
}

static esp_err_t DS4432U_set_current_code(uint8_t output, uint8_t code)
{
    if (!initialized || ds4432u_dev_handle == NULL) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t reg = (output == 0) ? DS4432U_OUT0_REG : DS4432U_OUT1_REG;
    return i2c_bitaxe_register_write_byte(ds4432u_dev_handle, reg, code);
}

static esp_err_t DS4432U_get_current_code(uint8_t output, uint8_t *code)
{
    if (!initialized || ds4432u_dev_handle == NULL) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t reg = (output == 0) ? DS4432U_OUT0_REG : DS4432U_OUT1_REG;
    return i2c_bitaxe_register_read(ds4432u_dev_handle, reg, code, 1);
}

esp_err_t DS4432U_set_voltage(float vout)
{
    if (!initialized || ds4432u_dev_handle == NULL) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (vout > BITAXE_VMAX || vout < BITAXE_VMIN) {
        ESP_LOGE(TAG, "Voltage %.3fV out of range [%.3f, %.3f]", vout, BITAXE_VMIN, BITAXE_VMAX);
        return ESP_ERR_INVALID_ARG;
    }

    float change = fabs((((TPS40305_VFB / BITAXE_RB) - ((vout - TPS40305_VFB) / BITAXE_RA)) / BITAXE_IFS) * 127);
    uint8_t reg = (uint8_t)ceil(change);
    if (vout < BITAXE_VNOM) {
        reg |= 0x80;
    }

    esp_err_t err = DS4432U_set_current_code(0, reg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set current code");
        return err;
    }

    uint8_t verify = 0;
    err = DS4432U_get_current_code(0, &verify);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DAC readback failed, cannot verify code");
    } else if (verify != reg) {
        ESP_LOGE(TAG, "DAC code mismatch: wrote 0x%02X, read 0x%02X", reg, verify);
        return ESP_FAIL;
    } else {
        ESP_LOGD(TAG, "DAC code verified: 0x%02X", verify);
    }
    return ESP_OK;
}

esp_err_t DS4432U_test(void)
{
    if (!initialized || ds4432u_dev_handle == NULL) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data;
    esp_err_t err = i2c_bitaxe_register_read(ds4432u_dev_handle, DS4432U_OUT0_REG, &data, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read OUT0 register");
        return err;
    }
    ESP_LOGI(TAG, "DS4432U+ OUT0 = 0x%02X", data);
    return ESP_OK;
}