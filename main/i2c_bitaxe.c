#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "i2c_bitaxe.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"   // für pdMS_TO_TICKS

#define GPIO_I2C_SDA CONFIG_GPIO_I2C_SDA
#define GPIO_I2C_SCL CONFIG_GPIO_I2C_SCL

#define I2C_MASTER_NUM         0
#define I2C_TIMEOUT_TICKS      pdMS_TO_TICKS(I2C_TIMEOUT_MS)

static const char *TAG = "i2c_bitaxe";
static i2c_master_bus_handle_t i2c_bus_handle = NULL;

#define MAX_DEVICES 10
typedef struct {
    i2c_master_dev_handle_t handle;
    uint16_t device_address;
    char device_tag[32];
} i2c_dev_map_entry_t;

static i2c_dev_map_entry_t i2c_device_map[MAX_DEVICES];
static int i2c_device_count = 0;

static esp_err_t log_on_error(esp_err_t err, i2c_master_dev_handle_t handle) {
    if (err == ESP_OK) return ESP_OK;

    for (int i = 0; i < i2c_device_count; i++) {
        if (i2c_device_map[i].handle == handle) {
            ESP_LOGE(TAG, "I2C error on device '%s' (addr 0x%02x): %s",
                     i2c_device_map[i].device_tag,
                     i2c_device_map[i].device_address,
                     esp_err_to_name(err));
            return err;
        }
    }
    ESP_LOGE(TAG, "I2C error on unknown device handle: %s", esp_err_to_name(err));
    return err;
}

esp_err_t i2c_bitaxe_init(void) {
    if (i2c_bus_handle != NULL) {
        ESP_LOGW(TAG, "I2C bus already initialized");
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = GPIO_I2C_SCL,
        .sda_io_num = GPIO_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(err));
        i2c_bus_handle = NULL;
    }
    return err;
}

esp_err_t i2c_bitaxe_add_device(uint8_t device_address, i2c_master_dev_handle_t *dev_handle, const char *device_tag) {
    if (dev_handle == NULL) {
        ESP_LOGE(TAG, "dev_handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized. Call i2c_bitaxe_init() first.");
        return ESP_ERR_INVALID_STATE;
    }
    if (i2c_device_count >= MAX_DEVICES) {
        ESP_LOGE(TAG, "Device map full");
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = I2C_BUS_SPEED_HZ,
    };

    esp_err_t err = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device 0x%02x: %s", device_address, esp_err_to_name(err));
        return err;
    }

    i2c_device_map[i2c_device_count].handle = *dev_handle;
    i2c_device_map[i2c_device_count].device_address = device_address;
    strncpy(i2c_device_map[i2c_device_count].device_tag, device_tag,
            sizeof(i2c_device_map[i2c_device_count].device_tag) - 1);
    i2c_device_map[i2c_device_count].device_tag[sizeof(i2c_device_map[i2c_device_count].device_tag) - 1] = '\0';
    i2c_device_count++;

    return ESP_OK;
}

esp_err_t i2c_bitaxe_get_master_bus_handle(i2c_master_bus_handle_t *dev_handle) {
    if (dev_handle == NULL) return ESP_ERR_INVALID_ARG;
    *dev_handle = i2c_bus_handle;
    return (i2c_bus_handle == NULL) ? ESP_ERR_INVALID_STATE : ESP_OK;
}

esp_err_t i2c_bitaxe_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *read_buf, size_t len) {
    if (dev_handle == NULL) return ESP_ERR_INVALID_ARG;
    if (read_buf == NULL && len > 0) return ESP_ERR_INVALID_ARG;
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, read_buf, len, I2C_TIMEOUT_TICKS);
    return log_on_error(err, dev_handle);
}

esp_err_t i2c_bitaxe_register_write_addr(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr) {
    if (dev_handle == NULL) return ESP_ERR_INVALID_ARG;
    esp_err_t err = i2c_master_transmit(dev_handle, &reg_addr, 1, I2C_TIMEOUT_TICKS);
    return log_on_error(err, dev_handle);
}

esp_err_t i2c_bitaxe_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data) {
    if (dev_handle == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t write_buf[2] = {reg_addr, data};
    esp_err_t err = i2c_master_transmit(dev_handle, write_buf, 2, I2C_TIMEOUT_TICKS);
    return log_on_error(err, dev_handle);
}

esp_err_t i2c_bitaxe_register_write_bytes(i2c_master_dev_handle_t dev_handle, uint8_t *data, uint8_t len) {
    if (dev_handle == NULL) return ESP_ERR_INVALID_ARG;
    if (data == NULL && len > 0) return ESP_ERR_INVALID_ARG;
    esp_err_t err = i2c_master_transmit(dev_handle, data, len, I2C_TIMEOUT_TICKS);
    return log_on_error(err, dev_handle);
}

esp_err_t i2c_bitaxe_register_write_word(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint16_t data) {
    if (dev_handle == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t write_buf[3] = {reg_addr, (uint8_t)(data & 0xFF), (uint8_t)((data >> 8) & 0xFF)};
    esp_err_t err = i2c_master_transmit(dev_handle, write_buf, 3, I2C_TIMEOUT_TICKS);
    return log_on_error(err, dev_handle);
}