#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "i2c_util.h"
#include "rv8803.h"
#include "tmp117.h"

static const char *TAG = "SENSOR_TEST";

// Global device handles
static i2c_master_dev_handle_t tmp117_dev_handle;
static i2c_master_dev_handle_t rv8803_dev_handle;
static tmp117_dev_t tmp117_device;
static rv8803_dev_t rv8803_device;

// Task handles
static TaskHandle_t sensor_task_handle = NULL;

// Initialize RTC with current time if needed
static esp_err_t setup_rtc_time(void)
{
    rv8803_datetime_t current_time;
    esp_err_t ret = rv8803_get_time(&rv8803_device, &current_time);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read RTC time");
        return ret;
    }

    // Check if time seems valid (year should be reasonable)
    if (current_time.year < 2023 || current_time.year > 2030) {
        ESP_LOGW(TAG, "RTC time seems invalid (%d), setting default time", current_time.year);
        
        // Set a default time: 2024-01-01 12:00:00, Monday
        rv8803_datetime_t default_time = {
            .year = 2024,
            .month = 1,
            .date = 1,
            .hour = 12,
            .minute = 0,
            .second = 0,
            .weekday = 1  // Monday
        };
        
        ret = rv8803_set_time(&rv8803_device, &default_time);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set default RTC time");
            return ret;
        }
        ESP_LOGI(TAG, "Default RTC time set");
    } else {
        ESP_LOGI(TAG, "RTC time is valid: %04d-%02d-%02d %02d:%02d:%02d", 
                 current_time.year, current_time.month, current_time.date,
                 current_time.hour, current_time.minute, current_time.second);
    }

    return ESP_OK;
}

// Sensor reading task
static void sensor_task(void *pvParameters)
{
    rv8803_datetime_t rtc_time;
    float temperature;
    bool voltage_low;
    
    ESP_LOGI(TAG, "Sensor monitoring task started");

    while (1) {
        // Read current time from RV8803
        esp_err_t rtc_ret = rv8803_get_time(&rv8803_device, &rtc_time);
        
        // Read temperature from TMP117
        temperature = tmp117_read_temp_c(&tmp117_device);
        
        // Check RTC voltage status
        esp_err_t voltage_ret = rv8803_check_voltage_low(&rv8803_device, &voltage_low);
        
        // Log the readings
        if (rtc_ret == ESP_OK && !isnan(temperature)) {
            ESP_LOGI(TAG, "[%04d-%02d-%02d %02d:%02d:%02d] Temperature: %.4f°C%s",
                     rtc_time.year, rtc_time.month, rtc_time.date,
                     rtc_time.hour, rtc_time.minute, rtc_time.second,
                     temperature,
                     (voltage_ret == ESP_OK && voltage_low) ? " [RTC LOW VOLTAGE]" : "");
        } else {
            ESP_LOGW(TAG, "Failed to read sensors - RTC: %s, TMP117: %s",
                     esp_err_to_name(rtc_ret),
                     isnan(temperature) ? "FAILED" : "OK");
        }

        // Wait for next reading (10 seconds)
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// Application main function
void app_main(void)
{
    ESP_LOGI(TAG, "Starting RV8803 RTC + TMP117 Temperature Sensor Test");
    
    // Initialize shared I2C bus first
    ESP_LOGI(TAG, "Initializing shared I2C bus...");
    esp_err_t ret = i2c_util_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
        return;
    }

    // Add RV8803 device to the I2C bus
    ESP_LOGI(TAG, "Adding RV8803 RTC to I2C bus...");
    ret = i2c_util_add_device(RV8803_I2C_ADDR, &rv8803_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add RV8803 device to I2C bus");
        return;
    }

    // Initialize RV8803 RTC
    ESP_LOGI(TAG, "Initializing RV8803 RTC...");
    ret = rv8803_init(&rv8803_device, rv8803_dev_handle, RV8803_I2C_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize RV8803: %s", esp_err_to_name(ret));
        return;
    }

    // Set up RTC time if needed
    ret = setup_rtc_time();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup RTC time");
        return;
    }

    // Add TMP117 device to the I2C bus
    ESP_LOGI(TAG, "Adding TMP117 to I2C bus...");
    ret = i2c_util_add_device(TMP117_I2C_ADDR, &tmp117_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add TMP117 device to I2C bus");
        return;
    }

    // Initialize TMP117 sensor
    ESP_LOGI(TAG, "Initializing TMP117 sensor...");
    ret = tmp117_init(&tmp117_device, tmp117_dev_handle, TMP117_I2C_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize TMP117: %s", esp_err_to_name(ret));
        return;
    }

    // Configure TMP117 for optimal performance
    tmp117_config_t tmp117_config = {
        .alert_mode = ALERT_MODE_ALERT,
        .alert_polarity = ALERT_POL_LOW,
        .averaging = AVG_8_SAMPLES,           // 8 samples averaging for better accuracy
        .conversion_cycle = CC_250MS,         // 250ms conversion cycle
        .conversion_mode = MODE_CONTINUOUS,   // Continuous conversion
        .data_ready_alert_enable = false
    };

    ret = tmp117_set_config(&tmp117_device, &tmp117_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to configure TMP117, using defaults: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "TMP117 configured with 8-sample averaging and 250ms cycle");
    }

    // Test initial readings
    ESP_LOGI(TAG, "Testing initial sensor readings...");
    
    rv8803_datetime_t test_time;
    ret = rv8803_get_time(&rv8803_device, &test_time);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RTC Test: %04d-%02d-%02d %02d:%02d:%02d", 
                 test_time.year, test_time.month, test_time.date,
                 test_time.hour, test_time.minute, test_time.second);
    }

    float test_temp = tmp117_read_temp_c(&tmp117_device);
    if (!isnan(test_temp)) {
        ESP_LOGI(TAG, "TMP117 Test: %.4f°C (%.2f°F)", test_temp, tmp117_read_temp_f(&tmp117_device));
    }

    // Create sensor monitoring task
    BaseType_t task_ret = xTaskCreate(
        sensor_task,
        "sensor_task",
        4096,
        NULL,
        5,
        &sensor_task_handle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        return;
    }

    ESP_LOGI(TAG, "System initialized successfully. Starting periodic sensor logging...");
    ESP_LOGI(TAG, "Logging interval: 10 seconds");
}