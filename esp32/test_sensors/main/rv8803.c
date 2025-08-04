#include "rv8803.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "RV8803";
static const TickType_t I2C_TIMEOUT_MS = 1000 / portTICK_PERIOD_MS;

// Initialize RV8803 device
esp_err_t rv8803_init(rv8803_dev_t *dev, i2c_master_dev_handle_t dev_handle, uint8_t device_addr)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    
    dev->dev_handle = dev_handle;
    dev->device_addr = device_addr;
    
    // Test communication by reading seconds register
    uint8_t test_data;
    esp_err_t ret = rv8803_read_register(dev, RV8803_REG_SEC, &test_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RV8803 communication test failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Clear any flags
    rv8803_clear_flags(dev);
    
    ESP_LOGI(TAG, "RV8803 initialized successfully at address 0x%02X", device_addr);
    return ESP_OK;
}

// Write single register
esp_err_t rv8803_write_register(rv8803_dev_t *dev, uint8_t reg_addr, uint8_t data)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t buf[2] = {reg_addr, data};
    esp_err_t ret = i2c_master_transmit(dev->dev_handle, buf, 2, I2C_TIMEOUT_MS);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    }
    
    return ret;
}

// Read single register
esp_err_t rv8803_read_register(rv8803_dev_t *dev, uint8_t reg_addr, uint8_t *data)
{
    if (!dev || !data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Write register address
    esp_err_t ret = i2c_master_transmit(dev->dev_handle, &reg_addr, 1, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register address 0x%02X: %s", reg_addr, esp_err_to_name(ret));
        return ret;
    }
    
    // Read data
    ret = i2c_master_receive(dev->dev_handle, data, 1, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    }
    
    return ret;
}

// Write multiple registers
esp_err_t rv8803_write_registers(rv8803_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (!dev || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create buffer with register address + data
    uint8_t *buf = malloc(len + 1);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate memory for write buffer");
        return ESP_ERR_NO_MEM;
    }
    
    buf[0] = reg_addr;
    memcpy(buf + 1, data, len);
    
    esp_err_t ret = i2c_master_transmit(dev->dev_handle, buf, len + 1, I2C_TIMEOUT_MS);
    
    free(buf);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write registers starting at 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    }
    
    return ret;
}

// Read multiple registers
esp_err_t rv8803_read_registers(rv8803_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (!dev || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Write register address
    esp_err_t ret = i2c_master_transmit(dev->dev_handle, &reg_addr, 1, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register address 0x%02X: %s", reg_addr, esp_err_to_name(ret));
        return ret;
    }
    
    // Read data
    ret = i2c_master_receive(dev->dev_handle, data, len, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read registers starting at 0x%02X: %s", reg_addr, esp_err_to_name(ret));
    }
    
    return ret;
}

// Convert decimal to BCD
uint8_t rv8803_dec_to_bcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

// Convert BCD to decimal
uint8_t rv8803_bcd_to_dec(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// Set date and time
esp_err_t rv8803_set_time(rv8803_dev_t *dev, const rv8803_datetime_t *datetime)
{
    if (!dev || !datetime) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate input ranges
    if (datetime->second > 59 || datetime->minute > 59 || datetime->hour > 23 ||
        datetime->weekday > 6 || datetime->date < 1 || datetime->date > 31 ||
        datetime->month < 1 || datetime->month > 12 || 
        datetime->year < 2000 || datetime->year > 2099) {
        ESP_LOGE(TAG, "Invalid datetime values");
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t time_data[7];
    time_data[0] = rv8803_dec_to_bcd(datetime->second);
    time_data[1] = rv8803_dec_to_bcd(datetime->minute);
    time_data[2] = rv8803_dec_to_bcd(datetime->hour);
    time_data[3] = rv8803_dec_to_bcd(datetime->weekday);
    time_data[4] = rv8803_dec_to_bcd(datetime->date);
    time_data[5] = rv8803_dec_to_bcd(datetime->month);
    time_data[6] = rv8803_dec_to_bcd(datetime->year - 2000);
    
    esp_err_t ret = rv8803_write_registers(dev, RV8803_REG_SEC, time_data, 7);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Time set: %04d-%02d-%02d %02d:%02d:%02d (weekday: %d)",
                 datetime->year, datetime->month, datetime->date,
                 datetime->hour, datetime->minute, datetime->second, datetime->weekday);
    }
    
    return ret;
}

// Get date and time
esp_err_t rv8803_get_time(rv8803_dev_t *dev, rv8803_datetime_t *datetime)
{
    if (!dev || !datetime) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t time_data[7];
    esp_err_t ret = rv8803_read_registers(dev, RV8803_REG_SEC, time_data, 7);
    
    if (ret == ESP_OK) {
        datetime->second = rv8803_bcd_to_dec(time_data[0] & 0x7F);  // Mask VL bit
        datetime->minute = rv8803_bcd_to_dec(time_data[1] & 0x7F);
        datetime->hour = rv8803_bcd_to_dec(time_data[2] & 0x3F);
        datetime->weekday = rv8803_bcd_to_dec(time_data[3] & 0x07);
        datetime->date = rv8803_bcd_to_dec(time_data[4] & 0x3F);
        datetime->month = rv8803_bcd_to_dec(time_data[5] & 0x1F);
        datetime->year = 2000 + rv8803_bcd_to_dec(time_data[6]);
        
        ESP_LOGD(TAG, "Time read: %04d-%02d-%02d %02d:%02d:%02d (weekday: %d)",
                 datetime->year, datetime->month, datetime->date,
                 datetime->hour, datetime->minute, datetime->second, datetime->weekday);
    }
    
    return ret;
}

// Reset RTC
esp_err_t rv8803_reset(rv8803_dev_t *dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = rv8803_write_register(dev, RV8803_REG_CTRL, RV8803_CTRL_RST);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10)); // Wait for reset to complete
        ESP_LOGI(TAG, "RV8803 reset completed");
    }
    return ret;
}

// Check if voltage is low
esp_err_t rv8803_check_voltage_low(rv8803_dev_t *dev, bool *is_low)
{
    if (!dev || !is_low) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t flag_reg;
    esp_err_t ret = rv8803_read_register(dev, RV8803_REG_FLAG, &flag_reg);
    
    if (ret == ESP_OK) {
        *is_low = (flag_reg & RV8803_FLAG_VLF) != 0;
        if (*is_low) {
            ESP_LOGW(TAG, "Low voltage detected on RV8803");
        }
    }
    
    return ret;
}

// Clear all flags
esp_err_t rv8803_clear_flags(rv8803_dev_t *dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = rv8803_write_register(dev, RV8803_REG_FLAG, 0x00);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "RV8803 flags cleared");
    }
    return ret;
}