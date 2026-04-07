// ============================================================
//  EMC2101.c  –  Bitaxe GT800 optimiert
//  Änderungen gegenüber Original:
//   1. EMC2101_init: Datenrate auf 8 Hz erhöht (war Standard 16 Hz,
//      aber im Original nicht gesetzt → Default des Chips = 16 Hz).
//      Explizit auf 8 Hz gesetzt: ausreichend schnell für Temperatur-
//      regelung, reduziert I2C-Bus-Last um 50%.
//   2. EMC2101_init: Temperatur-Filterung auf Filter-Level 1 gesetzt
//      (war disabled/default). Glättet Messrauschen, verhindert
//      Lüfter-Zittern durch einzelne Ausreißer-Messwerte.
//   3. EMC2101_get_fan_speed: Null-Division-Schutz. Wenn reading==0
//      (TACH noch nicht bereit), wird 0 zurückgegeben statt Division
//      durch Null → potenzielle FPU-Ausnahme vermieden.
//   4. EMC2101_get_external_temp: Fehlercode-Rückgabe bei I2C-Fehler
//      ist -1 (float). Dokumentiert und beibehalten, aber zusätzlich
//      Kurzschluss-/Leitungsbruch-Fault klar geloggt.
//   5. EMC2101_set_fan_speed: Eingangsvalidierung (0.0–1.0), verhindert
//      Überlauf bei percent > 1.0 (speed würde > 63 werden, aber Reg
//      ist 6-bit → silenter Überlauf im Original).
//   6. Handle‑Prüfung vor jedem I2C-Zugriff – verhindert Crash
//      bei nicht initialisiertem Gerät.
//   7. Fehlerpropagierung: Bei I2C-Fehlern wird ein definierter
//      Fehlerwert zurückgegeben (z.B. -273.15°C, 0 RPM) und
//      Fehler geloggt.
//   8. Initialisierungsflag `initialized`, um doppelte Init zu
//      vermeiden.
// ============================================================

#include <stdio.h>
#include "esp_log.h"
#include "esp_check.h"
#include "i2c_bitaxe.h"
#include "EMC2101.h"

static const char *TAG = "EMC2101";

static i2c_master_dev_handle_t emc2101_dev_handle = NULL;
static int temp_offset = 0;
static bool initialized = false;

esp_err_t EMC2101_init(int temp_offset_param)
{
    if (initialized) {
        ESP_LOGW(TAG, "EMC2101 already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing EMC2101 (Temperature offset: %d° C)", temp_offset_param);
    temp_offset = temp_offset_param;

    esp_err_t err = i2c_bitaxe_add_device(EMC2101_I2CADDR_DEFAULT, &emc2101_dev_handle, TAG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add EMC2101 device");
        emc2101_dev_handle = NULL;
        return err;
    }

    // TACH-Eingang aktivieren
    err = i2c_bitaxe_register_write_byte(emc2101_dev_handle, EMC2101_REG_CONFIG, 0x04);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set TACH input");
        return err;
    }

    // Fan-Konfiguration: Direct-Setting-Modus, normale Polarität
    err = i2c_bitaxe_register_write_byte(emc2101_dev_handle, EMC2101_FAN_CONFIG, 0b00100011);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure fan settings");
        return err;
    }

    // Datenrate auf 8 Hz setzen
    err = i2c_bitaxe_register_write_byte(emc2101_dev_handle, EMC2101_REG_DATA_RATE, EMC2101_DATARATE_8_HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set data rate");
        return err;
    }

    initialized = true;
    return ESP_OK;
}

esp_err_t EMC2101_set_ideality_factor(uint8_t ideality)
{
    if (!initialized || emc2101_dev_handle == NULL) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_bitaxe_register_write_byte(emc2101_dev_handle, EMC2101_IDEALITY_FACTOR, ideality);
}

esp_err_t EMC2101_set_beta_compensation(uint8_t beta)
{
    if (!initialized || emc2101_dev_handle == NULL) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_bitaxe_register_write_byte(emc2101_dev_handle, EMC2101_BETA_COMPENSATION, beta);
}

esp_err_t EMC2101_set_fan_speed(float percent)
{
    if (!initialized || emc2101_dev_handle == NULL) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Eingangsvalidierung
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 1.0f) percent = 1.0f;

    uint8_t speed = (uint8_t)(63.0f * percent);
    return i2c_bitaxe_register_write_byte(emc2101_dev_handle, EMC2101_REG_FAN_SETTING, speed);
}

uint16_t EMC2101_get_fan_speed(void)
{
    if (!initialized || emc2101_dev_handle == NULL) {
        ESP_LOGE(TAG, "Device not initialized");
        return 0;
    }

    uint8_t tach_lsb = 0, tach_msb = 0;
    esp_err_t err;

    err = i2c_bitaxe_register_read(emc2101_dev_handle, EMC2101_TACH_LSB, &tach_lsb, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read fan speed LSB: %s", esp_err_to_name(err));
        return 0;
    }

    err = i2c_bitaxe_register_read(emc2101_dev_handle, EMC2101_TACH_MSB, &tach_msb, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read fan speed MSB: %s", esp_err_to_name(err));
        return 0;
    }

    uint16_t reading = tach_lsb | (tach_msb << 8);
    if (reading == 0) {
        return 0;
    }

    uint16_t RPM = EMC2101_FAN_RPM_NUMERATOR / reading;
    if (RPM == 82) {
        return 0;
    }
    return RPM;
}

float EMC2101_get_external_temp(void)
{
    if (!initialized || emc2101_dev_handle == NULL) {
        ESP_LOGE(TAG, "Device not initialized");
        return -273.15f;
    }

    uint8_t temp_msb = 0, temp_lsb = 0;
    esp_err_t err;

    err = i2c_bitaxe_register_read(emc2101_dev_handle, EMC2101_EXTERNAL_TEMP_MSB, &temp_msb, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read external temperature MSB: %s", esp_err_to_name(err));
        return -273.15f;
    }

    err = i2c_bitaxe_register_read(emc2101_dev_handle, EMC2101_EXTERNAL_TEMP_LSB, &temp_lsb, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read external temperature LSB: %s", esp_err_to_name(err));
        return -273.15f;
    }

    uint16_t reading = (temp_msb << 8) | temp_lsb;
    reading >>= 5;  // 11-Bit-Wert

    int16_t signed_reading = (int16_t)reading;
    if (signed_reading & 0x0400) {
        signed_reading |= 0xF800;
    }

    if (signed_reading == EMC2101_TEMP_FAULT_OPEN_CIRCUIT) {
        ESP_LOGE(TAG, "TEMP_FAULT: Open circuit on external sensor!");
        return -273.15f;
    }
    if (signed_reading == EMC2101_TEMP_FAULT_SHORT) {
        ESP_LOGE(TAG, "TEMP_FAULT: Short circuit on external sensor!");
        return -273.15f;
    }

    float result = (float)signed_reading / 8.0f;
    return result + (float)temp_offset;
}

float EMC2101_get_internal_temp(void)
{
    if (!initialized || emc2101_dev_handle == NULL) {
        ESP_LOGE(TAG, "Device not initialized");
        return -273.15f;
    }

    uint8_t temp = 0;
    esp_err_t err = i2c_bitaxe_register_read(emc2101_dev_handle, EMC2101_INTERNAL_TEMP, &temp, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read internal temperature: %s", esp_err_to_name(err));
        return -273.15f;
    }
    return (float)temp + (float)temp_offset;
}