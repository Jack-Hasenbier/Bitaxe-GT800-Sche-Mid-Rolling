#include <string.h>
#include "INA260.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "global_state.h"
#include "math.h"
#include "mining.h"
#include "nvs_config.h"
#include "serial.h"
#include "TPS546.h"
#include "vcore.h"
#include "thermal.h"
#include "PID.h"
#include "power.h"
#include "asic.h"
#include "bm1370.h"
#include "utils.h"
#include "asic_init.h"
#include "asic_reset.h"
#include "driver/uart.h"

#define EPSILON 0.0001f
#define POLL_RATE 1800
#define MAX_TEMP 90.0
#define THROTTLE_TEMP 75.0
#define SAFE_TEMP 45.0
#define THROTTLE_TEMP_RANGE (MAX_TEMP - THROTTLE_TEMP)

#define VOLTAGE_START_THROTTLE 4900
#define VOLTAGE_MIN_THROTTLE 3500
#define VOLTAGE_RANGE (VOLTAGE_START_THROTTLE - VOLTAGE_MIN_THROTTLE)

#define TPS546_THROTTLE_TEMP 105.0
#define TPS546_MAX_TEMP 145.0

#define ASIC_REDUCTION 100.0

static const char * TAG = "power_management";

static int invalid_temp_count = 0;
static const int INVALID_TEMP_THRESHOLD = 5;

double pid_input = 0.0;
double pid_output = 0.0;
double min_fan_pct;
double pid_setPoint;
double pid_p = 15.0;        
double pid_i = 0.2;
double pid_d = 3.0;
double pid_d_startup = 20.0;

bool pid_startup_phase = true;
int pid_startup_counter = 0;

#define PID_STARTUP_HOLD_DURATION 3
#define PID_STARTUP_RAMP_DURATION 17

PIDController pid;

static float expected_hashrate(GlobalState * GLOBAL_STATE, float frequency)
{
    return frequency * GLOBAL_STATE->DEVICE_CONFIG.family.asic.small_core_count * GLOBAL_STATE->DEVICE_CONFIG.family.asic_count / 1000.0;
}

void POWER_MANAGEMENT_init_frequency(void * pvParameters)
{
    GlobalState * GLOBAL_STATE = (GlobalState *) pvParameters;

    float frequency = nvs_config_get_float(NVS_CONFIG_ASIC_FREQUENCY);

    GLOBAL_STATE->POWER_MANAGEMENT_MODULE.frequency_value = frequency;
    GLOBAL_STATE->POWER_MANAGEMENT_MODULE.expected_hashrate = expected_hashrate(GLOBAL_STATE, frequency);
    
    char expected_hashrate_str[16] = {0};
    suffixString(GLOBAL_STATE->POWER_MANAGEMENT_MODULE.expected_hashrate * 1e6, expected_hashrate_str, sizeof(expected_hashrate_str), 0);
    ESP_LOGI(TAG, "ASIC Frequency: %g MHz, Expected hashrate: %sH/s", frequency, expected_hashrate_str);
}

void POWER_MANAGEMENT_task(void * pvParameters)
{
    ESP_LOGI(TAG, "Starting");

    GlobalState * GLOBAL_STATE = (GlobalState *) pvParameters;

    PowerManagementModule * power_management = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;
    SystemModule * sys_module = &GLOBAL_STATE->SYSTEM_MODULE;

    power_management->last_valid_chip_temp = 25.0;
    power_management->last_valid_chip_temp2 = 25.0;

    POWER_MANAGEMENT_init_frequency(GLOBAL_STATE);
    
    float last_asic_frequency = power_management->frequency_value;

    pid_setPoint = (double)nvs_config_get_u16(NVS_CONFIG_TEMP_TARGET);
    min_fan_pct = (double)nvs_config_get_u16(NVS_CONFIG_MIN_FAN_SPEED);

    pid_init(&pid, &pid_input, &pid_output, &pid_setPoint, pid_p, pid_i, pid_d_startup, PID_P_ON_E, PID_REVERSE);
    pid_set_sample_time(&pid, POLL_RATE - 1);
    pid_set_output_limits(&pid, min_fan_pct, 100);
    pid_set_mode(&pid, AUTOMATIC);

    vTaskDelay(500 / portTICK_PERIOD_MS);
    uint16_t last_core_voltage = 0.0;

    uint16_t last_known_asic_voltage = 0;
    float last_known_asic_frequency = 0.0;

    while (1) {

        pid_setPoint = (double)nvs_config_get_u16(NVS_CONFIG_TEMP_TARGET);

        power_management->voltage = Power_get_input_voltage(GLOBAL_STATE);
        power_management->power = Power_get_power(GLOBAL_STATE);

        power_management->fan_rpm = Thermal_get_fan_speed(&GLOBAL_STATE->DEVICE_CONFIG);
        power_management->fan2_rpm = Thermal_get_fan2_speed(&GLOBAL_STATE->DEVICE_CONFIG);
        
        float temp1 = Thermal_get_chip_temp(GLOBAL_STATE);
        float temp2 = Thermal_get_chip_temp2(GLOBAL_STATE);
        
        if (temp1 >= 0) {
            power_management->chip_temp_avg = temp1;
            power_management->last_valid_chip_temp = temp1;
            invalid_temp_count = 0;
        } else {
            invalid_temp_count++;
            power_management->chip_temp_avg = power_management->last_valid_chip_temp;
            ESP_LOGW(TAG, "Invalid temperature reading (%.1f °C), using last valid: %.1f °C (count: %d)", 
                     temp1, power_management->last_valid_chip_temp, invalid_temp_count);
        }
        
        if (temp2 >= 0) {
            power_management->chip_temp2_avg = temp2;
            power_management->last_valid_chip_temp2 = temp2;
        } else {
            power_management->chip_temp2_avg = power_management->last_valid_chip_temp2;
        }

        power_management->vr_temp = Power_get_vreg_temp(GLOBAL_STATE);
        bool asic_overheat = 
            power_management->chip_temp_avg > THROTTLE_TEMP
            || power_management->chip_temp2_avg > THROTTLE_TEMP;
        
        if ((power_management->vr_temp > TPS546_THROTTLE_TEMP || asic_overheat) && (power_management->frequency_value > 50 || power_management->voltage > 1000)) {
            if (power_management->chip_temp2_avg > 0) {
                ESP_LOGE(TAG, "OVERHEAT! VR: %fC ASIC1: %fC ASIC2: %fC", power_management->vr_temp, power_management->chip_temp_avg, power_management->chip_temp2_avg);
            } else {
                ESP_LOGE(TAG, "OVERHEAT! VR: %fC ASIC: %fC", power_management->vr_temp, power_management->chip_temp_avg);
            }
            power_management->fan_perc = 100;
            // [FIX] Kein exit() bei Fehler, nur loggen
            if (Thermal_set_fan_percent(&GLOBAL_STATE->DEVICE_CONFIG, 1.0f) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set fan to 100%% during overheat – continuing anyway");
            }

            VCORE_set_voltage(GLOBAL_STATE, 0.0f);
            
            ESP_LOGI(TAG, "Setting RST pin to low due to overheat condition");
            ESP_ERROR_CHECK(asic_hold_reset_low());

            last_known_asic_voltage = nvs_config_get_u16(NVS_CONFIG_ASIC_VOLTAGE);
            last_known_asic_frequency = nvs_config_get_float(NVS_CONFIG_ASIC_FREQUENCY);
            nvs_config_set_bool(NVS_CONFIG_AUTO_FAN_SPEED, false);
            nvs_config_set_u16(NVS_CONFIG_MANUAL_FAN_SPEED, 100);
            nvs_config_set_bool(NVS_CONFIG_OVERHEAT_MODE, true);
            ESP_LOGW(TAG, "Entering safe mode due to overheat condition. System operation halted.");
            
            bool asic_temp_valid = GLOBAL_STATE->DEVICE_CONFIG.emc_internal_temp;
            int cooling_cycles = 0;
            const int MIN_COOLING_CYCLES = 6;
            
            while (cooling_cycles < MIN_COOLING_CYCLES || power_management->vr_temp > TPS546_THROTTLE_TEMP - 10) {
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                cooling_cycles++;
                
                power_management->vr_temp = Power_get_vreg_temp(GLOBAL_STATE);
                
                if (asic_temp_valid) {
                    power_management->chip_temp_avg = Thermal_get_chip_temp(GLOBAL_STATE);
                    power_management->chip_temp2_avg = Thermal_get_chip_temp2(GLOBAL_STATE);
                    ESP_LOGW(TAG, "Safe mode active (cycle %d) - VR: %.1fC ASIC1: %.1fC ASIC2: %.1fC",
                             cooling_cycles, power_management->vr_temp, power_management->chip_temp_avg, power_management->chip_temp2_avg);
                    
                    if (power_management->chip_temp_avg > SAFE_TEMP || power_management->chip_temp2_avg > SAFE_TEMP) {
                        cooling_cycles = 0;
                    }
                } else {
                    ESP_LOGW(TAG, "Safe mode active (cycle %d/%d) - VR: %.1fC (ASIC temps unavailable while powered down)",
                             cooling_cycles, MIN_COOLING_CYCLES, power_management->vr_temp);
                }
            }
            ESP_LOGI(TAG, "Temperature normalized after %d cooling cycles. Reinitializing ASIC...", cooling_cycles);
            
            ESP_LOGI(TAG, "Restoring core voltage to %umV = %.3fV (original settings preserved in NVS)...",
                     last_known_asic_voltage, last_known_asic_voltage/1000.0);
            VCORE_set_voltage(GLOBAL_STATE, (double)last_known_asic_voltage / 1000.0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            
            ESP_LOGI(TAG, "Stopping ASIC tasks...");
            GLOBAL_STATE->ASIC_initalized = false;
            vTaskDelay(500 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "Flushing UART buffers...");
            uart_flush(UART_NUM_1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            
            uint8_t chip_count = asic_initialize(GLOBAL_STATE, ASIC_INIT_RECOVERY, 2000);
            
            if (chip_count > 0) {
                nvs_config_set_bool(NVS_CONFIG_OVERHEAT_MODE, false);
                ESP_LOGI(TAG, "Resuming normal operation at %.0f MHz / %umV (original NVS values preserved).",
                         last_known_asic_frequency, last_known_asic_voltage);
            }
        }

        if (nvs_config_get_bool(NVS_CONFIG_AUTO_FAN_SPEED)) {
            if (invalid_temp_count < INVALID_TEMP_THRESHOLD) {
                if (power_management->chip_temp2_avg > power_management->chip_temp_avg) {
                    pid_input = power_management->chip_temp2_avg;
                } else {
                    pid_input = power_management->chip_temp_avg;
                }
                
                if (pid_startup_phase) {
                    pid_startup_counter++;
                    
                    if (pid_startup_counter >= (PID_STARTUP_HOLD_DURATION + PID_STARTUP_RAMP_DURATION)) {
                        pid_set_tunings(&pid, pid_p, pid_i, pid_d);
                        pid_startup_phase = false;
                        ESP_LOGI(TAG, "PID startup phase complete, switching to normal D value: %.1f", pid_d);
                    } else if (pid_startup_counter > PID_STARTUP_HOLD_DURATION) {
                        int ramp_counter = pid_startup_counter - PID_STARTUP_HOLD_DURATION;
                        double current_d = pid_d_startup - ((pid_d_startup - pid_d) * (double)ramp_counter / PID_STARTUP_RAMP_DURATION);
                        pid_set_tunings(&pid, pid_p, pid_i, current_d);
                        ESP_LOGI(TAG, "PID startup ramp phase: %d/%d (Total cycle: %d), current D: %.1f", 
                                 ramp_counter, PID_STARTUP_RAMP_DURATION, pid_startup_counter, current_d);
                    } else {
                        pid_set_tunings(&pid, pid_p, pid_i, pid_d_startup);
                        ESP_LOGI(TAG, "PID startup hold phase: %d/%d, holding D at: %.1f", 
                                 pid_startup_counter, PID_STARTUP_HOLD_DURATION, pid_d_startup);
                    }
                }
                
                pid_compute(&pid);
                power_management->fan_perc = pid_output;
                // [FIX] Kein exit() mehr, nur loggen bei Fehler
                if (Thermal_set_fan_percent(&GLOBAL_STATE->DEVICE_CONFIG, pid_output / 100.0) != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set fan speed to %.1f%% – keeping previous value", pid_output);
                }
            } else {
                if (GLOBAL_STATE->SYSTEM_MODULE.ap_enabled) {
                    ESP_LOGW(TAG, "Too many invalid temperature readings (%d) in AP mode - Setting fan to 70%%", invalid_temp_count);
                    power_management->fan_perc = 70;
                    if (Thermal_set_fan_percent(&GLOBAL_STATE->DEVICE_CONFIG, 0.7f) != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to set fan to 70%% – keeping previous value");
                    }
                } else {
                    ESP_LOGW(TAG, "Too many invalid temperature readings (%d) - Setting fan to 100%%", invalid_temp_count);
                    power_management->fan_perc = 100;
                    if (Thermal_set_fan_percent(&GLOBAL_STATE->DEVICE_CONFIG, 1.0f) != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to set fan to 100%% – keeping previous value");
                    }
                }
            }
        } else {
            uint16_t fan_perc = nvs_config_get_u16(NVS_CONFIG_MANUAL_FAN_SPEED);
            if (fabs(power_management->fan_perc - fan_perc) > EPSILON) {
                ESP_LOGI(TAG, "Setting manual fan speed to %d%%", fan_perc);
                power_management->fan_perc = fan_perc;
                if (Thermal_set_fan_percent(&GLOBAL_STATE->DEVICE_CONFIG, fan_perc / 100.0f) != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set manual fan speed – keeping previous value");
                }
            }
        }

        uint16_t core_voltage = nvs_config_get_u16(NVS_CONFIG_ASIC_VOLTAGE);
        float asic_frequency = nvs_config_get_float(NVS_CONFIG_ASIC_FREQUENCY);

        if (core_voltage != last_core_voltage) {
            ESP_LOGI(TAG, "setting new vcore voltage to %umV", core_voltage);
            VCORE_set_voltage(GLOBAL_STATE, (double) core_voltage / 1000.0);
            last_core_voltage = core_voltage;
        }

        if (asic_frequency != last_asic_frequency) {
            ESP_LOGI(TAG, "New ASIC frequency requested: %g MHz (current: %g MHz)", asic_frequency, last_asic_frequency);
            
            bool success = ASIC_set_frequency(GLOBAL_STATE, asic_frequency);
            
            if (success) {
                power_management->frequency_value = asic_frequency;
                power_management->expected_hashrate = expected_hashrate(GLOBAL_STATE, asic_frequency);
            }
            
            last_asic_frequency = asic_frequency;
        }

        bool new_overheat_mode = nvs_config_get_bool(NVS_CONFIG_OVERHEAT_MODE);
        if (new_overheat_mode != sys_module->overheat_mode) {
            sys_module->overheat_mode = new_overheat_mode;
            ESP_LOGI(TAG, "Overheat mode updated to: %d", sys_module->overheat_mode);
        }

        VCORE_check_fault(GLOBAL_STATE);

        vTaskDelay(POLL_RATE / portTICK_PERIOD_MS);
    }
}