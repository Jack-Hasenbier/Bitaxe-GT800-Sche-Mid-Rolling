#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pmbus_commands.h"
#include "i2c_bitaxe.h"
#include "TPS546.h"

//#define DEBUG_TPS546_MEAS 1
//#define DEBUG_TPS546_STATUS 1

#define I2C_MASTER_NUM     0
#define WRITE_BIT          I2C_MASTER_WRITE
#define READ_BIT           I2C_MASTER_READ
#define ACK_CHECK          true
#define NO_ACK_CHECK       false
#define ACK_VALUE          0x0
#define NACK_VALUE         0x1
#define MAX_BLOCK_LEN      32

static const char *TAG = "TPS546";

// File‑scope fault counter (kann von TPS546_clear_faults zurückgesetzt werden)
static uint8_t tps546_fault_count = 0;

static uint8_t DEVICE_ID_TPS546D24A[] = {0x54, 0x49, 0x54, 0x6D, 0x24, 0x41};
static uint8_t DEVICE_ID_TPS546D24S[] = {0x54, 0x49, 0x54, 0x6D, 0x24, 0x62};

static i2c_master_dev_handle_t tps546_i2c_handle;
static TPS546_CONFIG tps546_config;

// Globaler Fehlerstring für die UI
static char tps_error_message[256] = "Power Fault Detected.";

static esp_err_t TPS546_parse_status(uint16_t);

/* ------------------- Hilfsfunktionen (SMBus) ------------------- */
static esp_err_t smb_read_byte(uint8_t command, uint8_t *data) {
    return i2c_bitaxe_register_read(tps546_i2c_handle, command, data, 1);
}

static esp_err_t smb_write_byte(uint8_t command, uint8_t data) {
    return i2c_bitaxe_register_write_byte(tps546_i2c_handle, command, data);
}

static esp_err_t smb_write_addr(uint8_t command) {
    return i2c_bitaxe_register_write_addr(tps546_i2c_handle, command);
}

static esp_err_t smb_read_word(uint8_t command, uint16_t *result) {
    uint8_t data[2];
    esp_err_t ret = i2c_bitaxe_register_read(tps546_i2c_handle, command, data, 2);
    if (ret != ESP_OK) return ret;
    *result = (data[1] << 8) + data[0];
    return ESP_OK;
}

static esp_err_t smb_write_word(uint8_t command, uint16_t data) {
    return i2c_bitaxe_register_write_word(tps546_i2c_handle, command, data);
}

/**
 * @brief SMBus Block Read mit Prüfung des Längenbytes
 */
static esp_err_t smb_read_block(uint8_t command, uint8_t *data, uint8_t len) {
    uint8_t *buf = malloc(len + 1);
    if (!buf) return ESP_ERR_NO_MEM;
    esp_err_t ret = i2c_bitaxe_register_read(tps546_i2c_handle, command, buf, len + 1);
    if (ret != ESP_OK) {
        free(buf);
        return ret;
    }
    if (buf[0] != len) {
        ESP_LOGW(TAG, "Block length mismatch: expected %d, got %d", len, buf[0]);
        free(buf);
        return ESP_ERR_INVALID_RESPONSE;
    }
    memcpy(data, buf + 1, len);
    free(buf);
    return ESP_OK;
}

/* ------------------- Umrechnungen Linear11 / ULINEAR16 ------------------- */
static int slinear11_2_int(uint16_t value) {
    int exponent, mantissa;
    // Exponent (5 Bit, Zweierkomplement)
    if (value & 0x8000)
        exponent = -1 * (((~value >> 11) & 0x001F) + 1);
    else
        exponent = (value >> 11);
    // Mantisse (11 Bit, Zweierkomplement) – ACHTUNG: Maske 0x07FF, nicht 0x03FF!
    if (value & 0x0400)
        mantissa = -1 * ((~value & 0x07FF) + 1);
    else
        mantissa = (value & 0x07FF);
    return (int)(mantissa * powf(2.0f, exponent));
}

static float slinear11_2_float(uint16_t value) {
    int exponent, mantissa;
    if (value & 0x8000)
        exponent = -1 * (((~value >> 11) & 0x001F) + 1);
    else
        exponent = (value >> 11);
    if (value & 0x0400)
        mantissa = -1 * ((~value & 0x07FF) + 1);
    else
        mantissa = (value & 0x07FF);
    return mantissa * powf(2.0f, exponent);
}

static uint16_t int_2_slinear11(int value) {
    // Nur für positive Werte getestet (Frequenz, Zeit, Temperatur)
    int mantissa;
    int exponent = 0;
    for (int i = 0; i <= 15; i++) {
        mantissa = value / (int)powf(2.0f, i);
        if (mantissa < 1024) {
            exponent = i;
            break;
        }
    }
    return ((exponent << 11) & 0xF800) + mantissa;
}

static uint16_t float_2_slinear11(float value) {
    int mantissa;
    int exponent = 0;
    if (value > 0) {
        for (int i = 0; i <= 15; i++) {
            mantissa = (int)(value * powf(2.0f, i));
            if (mantissa >= 1024) {
                exponent = i - 1;
                mantissa = (int)(value * powf(2.0f, exponent));
                break;
            }
        }
    } else {
        ESP_LOGE(TAG, "Negative values not supported in float_2_slinear11");
        return 0;
    }
    return (((~exponent + 1) << 11) & 0xF800) + mantissa;
}

static float ulinear16_2_float(uint16_t value) {
    uint8_t voutmode;
    int exponent;
    if (smb_read_byte(PMBUS_VOUT_MODE, &voutmode) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read VOUT_MODE");
        return 0;
    }
    if (voutmode & 0x10)
        exponent = -1 * ((~voutmode & 0x1F) + 1);
    else
        exponent = (voutmode & 0x1F);
    return value * powf(2.0f, exponent);
}

static uint16_t float_2_ulinear16(float value) {
    uint8_t voutmode;
    float exponent;
    if (smb_read_byte(PMBUS_VOUT_MODE, &voutmode) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read VOUT_MODE for conversion");
        return 0;
    }
    if (voutmode & 0x10)
        exponent = -1 * ((~voutmode & 0x1F) + 1);
    else
        exponent = (voutmode & 0x1F);
    return (uint16_t)(value / powf(2.0f, exponent));
}

/* ------------------- Öffentliche Funktionen ------------------- */
esp_err_t TPS546_init(TPS546_CONFIG config) {
    uint8_t u8_value = 0;
    uint16_t u16_value = 0;
    uint8_t read_mfr_revision[4];
    uint8_t comp_config[5];
    uint8_t voutmode;

    tps546_config = config;

    ESP_LOGI(TAG, "Initializing core voltage regulator");
    ESP_RETURN_ON_ERROR(i2c_bitaxe_add_device(TPS546_I2CADDR, &tps546_i2c_handle, TAG),
                        TAG, "Failed to add TPS546 I2C");

    vTaskDelay(pdMS_TO_TICKS(15)); // Power‑up Guard

    // Device ID mit Wiederholungen lesen
    uint8_t id[6] = {0};
    bool id_matched = false;
    for (int attempt = 0; attempt < 6; ++attempt) {
        esp_err_t err = smb_read_block(PMBUS_IC_DEVICE_ID, id, 6);
        if (err == ESP_OK) {
            if (memcmp(id, DEVICE_ID_TPS546D24A, 6) == 0 ||
                memcmp(id, DEVICE_ID_TPS546D24S, 6) == 0) {
                id_matched = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(3));
    }

    ESP_LOGI(TAG, "Device ID: %02x %02x %02x %02x %02x %02x",
             id[0], id[1], id[2], id[3], id[4], id[5]);

    if (!id_matched) {
        ESP_LOGE(TAG, "Cannot find TPS546 regulator - Device ID mismatch");
        return ESP_FAIL;
    }

    // Abschalten bis zur vollständigen Konfiguration
    smb_write_byte(PMBUS_OPERATION, OPERATION_OFF);
    smb_write_byte(PMBUS_ON_OFF_CONFIG,
                   ON_OFF_CONFIG_DELAY | ON_OFF_CONFIG_POLARITY |
                   ON_OFF_CONFIG_CP | ON_OFF_CONFIG_CMD | ON_OFF_CONFIG_PU);

    TPS546_read_mfr_info(read_mfr_revision);
    ESP_LOGI(TAG, "Writing new config values");
    smb_read_byte(PMBUS_VOUT_MODE, &voutmode);
    ESP_LOGI(TAG, "VOUT_MODE: %02x", voutmode);
    TPS546_write_entire_config();

    TPS546_show_voltage_settings();

    smb_read_word(PMBUS_STATUS_WORD, &u16_value);
    ESP_LOGI(TAG, "STATUS_WORD after init: %04x", u16_value);

    ESP_LOGI(TAG, "-----------VOLTAGE/CURRENT---------------------");
    smb_read_word(PMBUS_READ_VIN, &u16_value);
    ESP_LOGI(TAG, "READ_VIN: %.2fV", slinear11_2_float(u16_value));
    smb_read_word(PMBUS_READ_IOUT, &u16_value);
    ESP_LOGI(TAG, "READ_IOUT: %.2fA", slinear11_2_float(u16_value));
    smb_read_word(PMBUS_READ_VOUT, &u16_value);
    ESP_LOGI(TAG, "READ_VOUT: %.2fV", ulinear16_2_float(u16_value));

    // Compensation Config auslesen
    if (smb_read_block(PMBUS_COMPENSATION_CONFIG, comp_config, 5) != ESP_OK)
        ESP_LOGE(TAG, "Failed to read COMPENSATION CONFIG");
    else
        ESP_LOGI(TAG, "COMPENSATION CONFIG: %02x %02x %02x %02x %02x",
                 comp_config[0], comp_config[1], comp_config[2], comp_config[3], comp_config[4]);

    ESP_LOGI(TAG, "Clearing faults");
    TPS546_clear_faults();

    smb_read_word(PMBUS_STATUS_WORD, &u16_value);
    ESP_LOGI(TAG, "Final STATUS_WORD: %04x", u16_value);

    return ESP_OK;
}

esp_err_t TPS546_clear_faults(void) {
    esp_err_t ret = smb_write_addr(PMBUS_CLEAR_FAULTS);
    tps546_fault_count = 0;   // Zähler nach manuellem Löschen zurücksetzen
    return ret;
}

void TPS546_read_mfr_info(uint8_t *read_mfr_revision) {
    uint8_t read_mfr_id[4] = {0};
    uint8_t read_mfr_model[4] = {0};
    if (smb_read_block(PMBUS_MFR_ID, read_mfr_id, 3) != ESP_OK)
        ESP_LOGE(TAG, "Failed to read MFR ID");
    if (smb_read_block(PMBUS_MFR_MODEL, read_mfr_model, 3) != ESP_OK)
        ESP_LOGE(TAG, "Failed to read MFR MODEL");
    if (smb_read_block(PMBUS_MFR_REVISION, read_mfr_revision, 3) != ESP_OK)
        ESP_LOGE(TAG, "Failed to read MFR REVISION");
    ESP_LOGI(TAG, "MFR_ID: %02X %02X %02X", read_mfr_id[0], read_mfr_id[1], read_mfr_id[2]);
    ESP_LOGI(TAG, "MFR_MODEL: %02X %02X %02X", read_mfr_model[0], read_mfr_model[1], read_mfr_model[2]);
    ESP_LOGI(TAG, "MFR_REVISION: %02X %02X %02X", read_mfr_revision[0], read_mfr_revision[1], read_mfr_revision[2]);
}

void TPS546_write_entire_config(void) {
    ESP_LOGI(TAG, "--- Writing new config values ---");

    // Phase
    smb_write_byte(PMBUS_PHASE, TPS546_INIT_PHASE);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Frequenz
    smb_write_word(PMBUS_FREQUENCY_SWITCH, int_2_slinear11(TPS546_INIT_FREQUENCY));
    vTaskDelay(pdMS_TO_TICKS(1));

    // VIN Schwellen
    if (tps546_config.TPS546_INIT_VIN_UV_WARN_LIMIT > 0) {
        smb_write_word(PMBUS_VIN_UV_WARN_LIMIT,
                       float_2_slinear11(tps546_config.TPS546_INIT_VIN_UV_WARN_LIMIT));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    smb_write_word(PMBUS_VIN_ON, float_2_slinear11(tps546_config.TPS546_INIT_VIN_ON));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_VIN_OFF, float_2_slinear11(tps546_config.TPS546_INIT_VIN_OFF));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_VIN_OV_FAULT_LIMIT,
                   float_2_slinear11(tps546_config.TPS546_INIT_VIN_OV_FAULT_LIMIT));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_byte(PMBUS_VIN_OV_FAULT_RESPONSE, TPS546_INIT_VIN_OV_FAULT_RESPONSE);
    vTaskDelay(pdMS_TO_TICKS(1));

    // VOUT Konfiguration
    smb_write_word(PMBUS_VOUT_SCALE_LOOP, float_2_slinear11(tps546_config.TPS546_INIT_SCALE_LOOP));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_VOUT_COMMAND, float_2_ulinear16(tps546_config.TPS546_INIT_VOUT_COMMAND));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_VOUT_MAX, float_2_ulinear16(tps546_config.TPS546_INIT_VOUT_MAX));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_VOUT_MIN, float_2_ulinear16(tps546_config.TPS546_INIT_VOUT_MIN));
    vTaskDelay(pdMS_TO_TICKS(1));

    // Relativwerte (Prozent) – Achtung: hier wird der absolute Wert aus der Config verwendet
    float vout_abs = tps546_config.TPS546_INIT_VOUT_COMMAND;
    smb_write_word(PMBUS_VOUT_OV_FAULT_LIMIT, float_2_ulinear16(vout_abs * TPS546_INIT_VOUT_OV_FAULT_LIMIT));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_VOUT_OV_WARN_LIMIT, float_2_ulinear16(vout_abs * TPS546_INIT_VOUT_OV_WARN_LIMIT));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_VOUT_MARGIN_HIGH, float_2_ulinear16(vout_abs * TPS546_INIT_VOUT_MARGIN_HIGH));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_VOUT_MARGIN_LOW, float_2_ulinear16(vout_abs * TPS546_INIT_VOUT_MARGIN_LOW));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_VOUT_UV_WARN_LIMIT, float_2_ulinear16(vout_abs * TPS546_INIT_VOUT_UV_WARN_LIMIT));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_VOUT_UV_FAULT_LIMIT, float_2_ulinear16(vout_abs * TPS546_INIT_VOUT_UV_FAULT_LIMIT));
    vTaskDelay(pdMS_TO_TICKS(1));

    // IOUT Limits
    smb_write_word(PMBUS_IOUT_OC_WARN_LIMIT,
                   float_2_slinear11(tps546_config.TPS546_INIT_IOUT_OC_WARN_LIMIT));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_IOUT_OC_FAULT_LIMIT,
                   float_2_slinear11(tps546_config.TPS546_INIT_IOUT_OC_FAULT_LIMIT));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_byte(PMBUS_IOUT_OC_FAULT_RESPONSE, TPS546_INIT_IOUT_OC_FAULT_RESPONSE);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Temperatur
    smb_write_word(PMBUS_OT_WARN_LIMIT, int_2_slinear11(TPS546_INIT_OT_WARN_LIMIT));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_OT_FAULT_LIMIT, int_2_slinear11(TPS546_INIT_OT_FAULT_LIMIT));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_byte(PMBUS_OT_FAULT_RESPONSE, TPS546_INIT_OT_FAULT_RESPONSE);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Timing
    smb_write_word(PMBUS_TON_DELAY, int_2_slinear11(TPS546_INIT_TON_DELAY));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_TON_RISE, int_2_slinear11(TPS546_INIT_TON_RISE));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_TON_MAX_FAULT_LIMIT, int_2_slinear11(TPS546_INIT_TON_MAX_FAULT_LIMIT));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_byte(PMBUS_TON_MAX_FAULT_RESPONSE, TPS546_INIT_TON_MAX_FAULT_RESPONSE);
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_TOFF_DELAY, int_2_slinear11(TPS546_INIT_TOFF_DELAY));
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_TOFF_FALL, int_2_slinear11(TPS546_INIT_TOFF_FALL));
    vTaskDelay(pdMS_TO_TICKS(1));

    // Weitere Konfiguration
    smb_write_word(PMBUS_STACK_CONFIG, INIT_STACK_CONFIG);
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_byte(PMBUS_SYNC_CONFIG, INIT_SYNC_CONFIG);
    vTaskDelay(pdMS_TO_TICKS(1));
    smb_write_word(PMBUS_PIN_DETECT_OVERRIDE, INIT_PIN_DETECT_OVERRIDE);
    vTaskDelay(pdMS_TO_TICKS(1));

    ESP_LOGI(TAG, "--- Config write finished ---");
}

int TPS546_get_frequency(void) {
    uint16_t value = 0;
    if (smb_read_word(PMBUS_FREQUENCY_SWITCH, &value) != ESP_OK) return 0;
    return slinear11_2_int(value);
}

void TPS546_set_frequency(int newfreq) {
    ESP_LOGI(TAG, "Setting frequency to %d kHz", newfreq);
    smb_write_word(PMBUS_FREQUENCY_SWITCH, int_2_slinear11(newfreq));
}

int TPS546_get_temperature(void) {
    uint16_t value = 0;
    if (smb_read_word(PMBUS_READ_TEMPERATURE_1, &value) != ESP_OK) return 0;
    return slinear11_2_int(value);
}

float TPS546_get_vin(void) {
    uint16_t u16_value = 0;
    if (smb_read_word(PMBUS_READ_VIN, &u16_value) != ESP_OK) {
        ESP_LOGE(TAG, "Could not read VIN");
        return 0;
    }
    float vin = slinear11_2_float(u16_value);
#ifdef DEBUG_TPS546_MEAS
    ESP_LOGI(TAG, "Vin = %.3f V", vin);
#endif
    return vin;
}

float TPS546_get_iout(void) {
    uint16_t u16_value = 0;
    // Phase auf alle Phasen umschalten
    smb_write_byte(PMBUS_PHASE, 0xFF);
    esp_err_t ret = smb_read_word(PMBUS_READ_IOUT, &u16_value);
    // Phase immer zurücksetzen (auch bei Fehler)
    smb_write_byte(PMBUS_PHASE, TPS546_INIT_PHASE);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Could not read Iout");
        return 0;
    }
    float iout = slinear11_2_float(u16_value);
#ifdef DEBUG_TPS546_MEAS
    ESP_LOGI(TAG, "Iout = %.3f A", iout);
#endif
    return iout;
}

float TPS546_get_vout(void) {
    uint16_t u16_value = 0;
    if (smb_read_word(PMBUS_READ_VOUT, &u16_value) != ESP_OK) {
        ESP_LOGE(TAG, "Could not read Vout");
        return 0;
    }
    float vout = ulinear16_2_float(u16_value);
#ifdef DEBUG_TPS546_MEAS
    ESP_LOGI(TAG, "Vout = %.3f V", vout);
#endif
    return vout;
}

esp_err_t TPS546_check_status(GlobalState *GLOBAL_STATE) {
    SystemModule *SYSTEM_MODULE = &GLOBAL_STATE->SYSTEM_MODULE;
    uint16_t status;

    esp_err_t err = smb_read_word(PMBUS_STATUS_WORD, &status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read STATUS_WORD");
        return err;
    }

    // Prüfe auf ernste Fehlerbits (die einen Power-Fault auslösen sollen)
    if (status & (TPS546_STATUS_OFF | TPS546_STATUS_VOUT_OV |
                  TPS546_STATUS_IOUT_OC | TPS546_STATUS_VIN_UV |
                  TPS546_STATUS_TEMP)) {

        tps546_fault_count++;
        if (tps546_fault_count >= 3) {
            if (!SYSTEM_MODULE->power_fault) {
                ESP_LOGW(TAG, "Sustained power fault after %d checks", tps546_fault_count);
                TPS546_parse_status(status);
                SYSTEM_MODULE->power_fault = 1;
            }
        } else {
            ESP_LOGD(TAG, "Transient fault (count=%d/3)", tps546_fault_count);
        }
    } else {
        if (tps546_fault_count > 0) {
            ESP_LOGD(TAG, "Fault cleared after %d counts", tps546_fault_count);
        }
        tps546_fault_count = 0;
        if (SYSTEM_MODULE->power_fault) {
            ESP_LOGI(TAG, "Power fault resolved, clearing TPS546 fault registers");
            TPS546_clear_faults();
            SYSTEM_MODULE->power_fault = 0;
        }
    }
    return ESP_OK;
}

const char* TPS546_get_error_message(void) {
    return tps_error_message;
}

static esp_err_t TPS546_parse_status(uint16_t status) {
    uint8_t u8_value = 0;
    ESP_LOGE(TAG, "STATUS_WORD = 0x%04X", status);

    // Basis‑Fehlerbits aus dem Low‑Byte
    if (status & TPS546_STATUS_BUSY)  ESP_LOGE(TAG, "Device busy");
    if (status & TPS546_STATUS_OFF)   ESP_LOGE(TAG, "Regulator is OFF");
    if (status & TPS546_STATUS_VOUT_OV) ESP_LOGE(TAG, "VOUT overvoltage fault");
    if (status & TPS546_STATUS_IOUT_OC) ESP_LOGE(TAG, "IOUT overcurrent fault");
    if (status & TPS546_STATUS_VIN_UV)  ESP_LOGE(TAG, "VIN undervoltage fault");
    if (status & TPS546_STATUS_TEMP) {
        ESP_LOGE(TAG, "Temperature fault/warning");
        if (smb_read_byte(PMBUS_STATUS_TEMPERATURE, &u8_value) == ESP_OK) {
            ESP_LOGE(TAG, "STATUS_TEMPERATURE = 0x%02X", u8_value);
            if (u8_value & TPS546_STATUS_TEMP_OTF) ESP_LOGE(TAG, "  OT fault");
            if (u8_value & TPS546_STATUS_TEMP_OTW) ESP_LOGE(TAG, "  OT warning");
        }
    }
    if (status & TPS546_STATUS_CML) {
        ESP_LOGE(TAG, "CML fault");
        if (smb_read_byte(PMBUS_STATUS_CML, &u8_value) == ESP_OK) {
            ESP_LOGE(TAG, "STATUS_CML = 0x%02X", u8_value);
            if (u8_value & TPS546_STATUS_CML_IVC)  ESP_LOGE(TAG, "  Invalid command");
            if (u8_value & TPS546_STATUS_CML_IVD)  ESP_LOGE(TAG, "  Invalid data");
            if (u8_value & TPS546_STATUS_CML_PEC)  ESP_LOGE(TAG, "  PEC error");
            if (u8_value & TPS546_STATUS_CML_MEM)  ESP_LOGE(TAG, "  Memory error");
            if (u8_value & TPS546_STATUS_CML_PROC) ESP_LOGE(TAG, "  Logic core error");
            if (u8_value & TPS546_STATUS_CML_COMM) ESP_LOGE(TAG, "  Communication error");
        }
    }

    // Erweiterte Statusregister (High‑Byte Bits)
    if (status & TPS546_STATUS_VOUT) {
        if (smb_read_byte(PMBUS_STATUS_VOUT, &u8_value) == ESP_OK) {
            ESP_LOGE(TAG, "STATUS_VOUT = 0x%02X", u8_value);
            if (u8_value & TPS546_STATUS_VOUT_OVF)     ESP_LOGE(TAG, "  VOUT OV fault");
            if (u8_value & TPS546_STATUS_VOUT_OVW)     ESP_LOGE(TAG, "  VOUT OV warning");
            if (u8_value & TPS546_STATUS_VOUT_UVW)     ESP_LOGE(TAG, "  VOUT UV warning");
            if (u8_value & TPS546_STATUS_VOUT_UVF)     ESP_LOGE(TAG, "  VOUT UV fault");
            if (u8_value & TPS546_STATUS_VOUT_MIN_MAX) ESP_LOGE(TAG, "  VOUT outside MIN/MAX");
            if (u8_value & TPS546_STATUS_VOUT_TON_MAX) ESP_LOGE(TAG, "  TON_MAX fault");
        }
    }
    if (status & TPS546_STATUS_IOUT) {
        if (smb_read_byte(PMBUS_STATUS_IOUT, &u8_value) == ESP_OK) {
            ESP_LOGE(TAG, "STATUS_IOUT = 0x%02X", u8_value);
            if (u8_value & TPS546_STATUS_IOUT_OCF) ESP_LOGE(TAG, "  IOUT OC fault");
            if (u8_value & TPS546_STATUS_IOUT_OCW) ESP_LOGE(TAG, "  IOUT OC warning");
        }
    }
    if (status & TPS546_STATUS_INPUT) {
        if (smb_read_byte(PMBUS_STATUS_INPUT, &u8_value) == ESP_OK) {
            ESP_LOGE(TAG, "STATUS_INPUT = 0x%02X", u8_value);
            if (u8_value & TPS546_STATUS_VIN_OVF)     ESP_LOGE(TAG, "  VIN OV fault");
            if (u8_value & TPS546_STATUS_VIN_UVW)     ESP_LOGE(TAG, "  VIN UV warning");
            if (u8_value & TPS546_STATUS_VIN_LOW_VIN) ESP_LOGE(TAG, "  VIN low (OFF)");
        }
    }
    if (status & TPS546_STATUS_MFR) {
        if (smb_read_byte(PMBUS_STATUS_MFR_SPECIFIC, &u8_value) == ESP_OK) {
            ESP_LOGE(TAG, "STATUS_MFR_SPECIFIC = 0x%02X", u8_value);
            if (u8_value & TPS546_STATUS_MFR_POR)   ESP_LOGE(TAG, "  Power-on reset");
            if (u8_value & TPS546_STATUS_MFR_SELF)  ESP_LOGE(TAG, "  Self-check pending");
            if (u8_value & TPS546_STATUS_MFR_RESET) ESP_LOGE(TAG, "  RESET_VOUT event");
            if (u8_value & TPS546_STATUS_MFR_BCX)   ESP_LOGE(TAG, "  BCX fault");
            if (u8_value & TPS546_STATUS_MFR_SYNC)  ESP_LOGE(TAG, "  SYNC fault");
        }
    }
    if (status & TPS546_STATUS_PGOOD) {
        ESP_LOGE(TAG, "PGOOD is LOW (output not in regulation window)");
    }
    if (status & TPS546_STATUS_OTHER) {
        if (smb_read_byte(PMBUS_STATUS_OTHER, &u8_value) == ESP_OK) {
            ESP_LOGE(TAG, "STATUS_OTHER = 0x%02X", u8_value);
            if (u8_value & TPS546_STATUS_OTHER_FIRST) ESP_LOGE(TAG, "  First to assert SMBALERT");
        }
    }

    // Aktualisiere den globalen Fehlerstring für die UI
    snprintf(tps_error_message, sizeof(tps_error_message),
             "Power fault: STATUS=0x%04X", status);
    return ESP_OK;
}

esp_err_t TPS546_set_vout(float volts) {
    uint16_t value;
    uint8_t value8;

    if (volts == 0) {
        if (smb_write_byte(PMBUS_OPERATION, OPERATION_OFF) != ESP_OK) {
            ESP_LOGE(TAG, "Could not turn off Vout");
            return ESP_FAIL;
        }
    } else {
        if (volts < tps546_config.TPS546_INIT_VOUT_MIN ||
            volts > tps546_config.TPS546_INIT_VOUT_MAX) {
            ESP_LOGE(TAG, "Voltage %.2f V out of range [%.2f, %.2f]",
                     volts, tps546_config.TPS546_INIT_VOUT_MIN,
                     tps546_config.TPS546_INIT_VOUT_MAX);
            return ESP_FAIL;
        }
        value = float_2_ulinear16(volts);
        if (smb_write_word(PMBUS_VOUT_COMMAND, value) != ESP_OK) {
            ESP_LOGE(TAG, "Could not set Vout to %.2f V", volts);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Vout set to %.2f V", volts);
        if (smb_write_byte(PMBUS_OPERATION, OPERATION_ON) != ESP_OK) {
            ESP_LOGE(TAG, "Could not turn on Vout");
            return ESP_FAIL;
        }
        if (smb_read_byte(PMBUS_OPERATION, &value8) == ESP_OK && value8 != OPERATION_ON) {
            ESP_LOGW(TAG, "OPERATION not set to ON (0x%02X)", value8);
        }
    }
    return ESP_OK;
}

void TPS546_show_voltage_settings(void) {
    uint16_t u16_value = 0;
    uint8_t u8_value;
    float f_value;

    ESP_LOGI(TAG, "-----------VOLTAGE SETTINGS-----------------");
    if (smb_read_word(PMBUS_VIN_ON, &u16_value) == ESP_OK)
        ESP_LOGI(TAG, "VIN_ON: %.2fV", slinear11_2_float(u16_value));
    if (smb_read_word(PMBUS_VIN_OFF, &u16_value) == ESP_OK)
        ESP_LOGI(TAG, "VIN_OFF: %.2fV", slinear11_2_float(u16_value));
    if (smb_read_word(PMBUS_VIN_OV_FAULT_LIMIT, &u16_value) == ESP_OK)
        ESP_LOGI(TAG, "VIN_OV_FAULT_LIMIT: %.2fV", slinear11_2_float(u16_value));
    if (smb_read_word(PMBUS_VIN_UV_WARN_LIMIT, &u16_value) == ESP_OK)
        ESP_LOGI(TAG, "VIN_UV_WARN_LIMIT: %.2fV", slinear11_2_float(u16_value));
    if (smb_read_byte(PMBUS_VIN_OV_FAULT_RESPONSE, &u8_value) == ESP_OK)
        ESP_LOGI(TAG, "VIN_OV_FAULT_RESPONSE: 0x%02X", u8_value);

    if (smb_read_word(PMBUS_VOUT_MAX, &u16_value) == ESP_OK)
        ESP_LOGI(TAG, "VOUT_MAX: %.2fV", ulinear16_2_float(u16_value));
    if (smb_read_word(PMBUS_VOUT_OV_FAULT_LIMIT, &u16_value) == ESP_OK)
        ESP_LOGI(TAG, "VOUT_OV_FAULT_LIMIT: %.2fV", ulinear16_2_float(u16_value));
    if (smb_read_word(PMBUS_VOUT_OV_WARN_LIMIT, &u16_value) == ESP_OK)
        ESP_LOGI(TAG, "VOUT_OV_WARN_LIMIT: %.2fV", ulinear16_2_float(u16_value));
    if (smb_read_word(PMBUS_VOUT_MARGIN_HIGH, &u16_value) == ESP_OK)
        ESP_LOGI(TAG, "VOUT_MARGIN_HIGH: %.2fV", ulinear16_2_float(u16_value));
    if (smb_read_word(PMBUS_VOUT_COMMAND, &u16_value) == ESP_OK)
        ESP_LOGI(TAG, "VOUT_COMMAND: %.2fV", ulinear16_2_float(u16_value));
    if (smb_read_word(PMBUS_VOUT_MARGIN_LOW, &u16_value) == ESP_OK)
        ESP_LOGI(TAG, "VOUT_MARGIN_LOW: %.2fV", ulinear16_2_float(u16_value));
    if (smb_read_word(PMBUS_VOUT_UV_WARN_LIMIT, &u16_value) == ESP_OK)
        ESP_LOGI(TAG, "VOUT_UV_WARN_LIMIT: %.2fV", ulinear16_2_float(u16_value));
    if (smb_read_word(PMBUS_VOUT_UV_FAULT_LIMIT, &u16_value) == ESP_OK)
        ESP_LOGI(TAG, "VOUT_UV_FAULT_LIMIT: %.2fV", ulinear16_2_float(u16_value));
    if (smb_read_word(PMBUS_VOUT_MIN, &u16_value) == ESP_OK)
        ESP_LOGI(TAG, "VOUT_MIN: %.2fV", ulinear16_2_float(u16_value));
}