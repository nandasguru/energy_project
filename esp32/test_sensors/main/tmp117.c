#include "tmp117.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TMP117_DRV";
static const TickType_t I2C_TIMEOUT_MS = 100 / portTICK_PERIOD_MS;

// TMP117 temperature resolution: 0.0078125°C per LSB
#define TMP117_TEMP_RESOLUTION  0.0078125f

// --- Private Helper Functions (static to this file) ---

static esp_err_t _tmp117_read_register16(tmp117_dev_t *dev, uint8_t reg_addr, uint8_t *data) {
    esp_err_t err = i2c_master_transmit(dev->dev_handle, &reg_addr, 1, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write reg addr 0x%02X: %s", reg_addr, esp_err_to_name(err));
        return err;
    }
    err = i2c_master_receive(dev->dev_handle, data, 2, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read reg 0x%02X: %s", reg_addr, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t _tmp117_write_register16(tmp117_dev_t *dev, uint8_t reg_addr, uint8_t data_msb, uint8_t data_lsb) {
    uint8_t buf[3] = {reg_addr, data_msb, data_lsb};
    esp_err_t err = i2c_master_transmit(dev->dev_handle, buf, 3, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write reg 0x%02X: %s", reg_addr, esp_err_to_name(err));
    }
    return err;
}

static float _tmp117_raw_to_celsius(int16_t raw_temp) {
    return (float)raw_temp * TMP117_TEMP_RESOLUTION;
}

static int16_t _tmp117_celsius_to_raw(float temp_celsius) {
    return (int16_t)(temp_celsius / TMP117_TEMP_RESOLUTION);
}

// --- Public Functions ---

esp_err_t tmp117_init(tmp117_dev_t *dev, i2c_master_dev_handle_t dev_handle, uint8_t device_addr) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    dev->dev_handle = dev_handle;
    dev->device_addr = device_addr;
    
    // Verify device ID
    uint16_t device_id;
    esp_err_t err = tmp117_read_device_id(dev, &device_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device ID: %s", esp_err_to_name(err));
        return err;
    }
    
    if (device_id != TMP117_DEVICE_ID) {
        ESP_LOGE(TAG, "Invalid device ID: 0x%04X, expected 0x%04X", device_id, TMP117_DEVICE_ID);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "TMP117 found at 0x%02X, Device ID: 0x%04X", device_addr, device_id);
    return ESP_OK;
}

esp_err_t tmp117_read_device_id(tmp117_dev_t *dev, uint16_t *device_id) {
    if (!dev || !device_id) return ESP_ERR_INVALID_ARG;
    
    uint8_t data[2];
    esp_err_t err = _tmp117_read_register16(dev, DEVICE_ID_REG, data);
    if (err != ESP_OK) return err;
    
    *device_id = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t tmp117_set_config(tmp117_dev_t *dev, const tmp117_config_t *config) {
    if (!dev || !config) return ESP_ERR_INVALID_ARG;
    
    uint16_t config_reg = 0;
    
    // High byte configuration
    if (config->alert_polarity == ALERT_POL_HIGH) {
        config_reg |= (1 << CONFIG_HIGH_POL_BIT);
    }
    
    if (config->alert_mode == ALERT_MODE_THERM) {
        config_reg |= (1 << CONFIG_HIGH_T_NA_BIT);
    }
    
    if (config->data_ready_alert_enable) {
        config_reg |= (1 << CONFIG_HIGH_DR_ALERT_BIT);
    }
    
    config_reg |= ((config->averaging & 0x03) << CONFIG_HIGH_AVG_BIT);
    config_reg |= ((config->conversion_cycle & 0x03) << CONFIG_HIGH_CC_BIT);
    
    // Low byte configuration  
    config_reg |= ((config->conversion_mode & 0x03) << CONFIG_LOW_MOD_BIT);
    
    uint8_t data_msb = (uint8_t)(config_reg >> 8);
    uint8_t data_lsb = (uint8_t)(config_reg & 0xFF);
    
    esp_err_t ret = _tmp117_write_register16(dev, CONFIG_REG, data_msb, data_lsb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set config: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t tmp117_get_config(tmp117_dev_t *dev, tmp117_config_t *config) {
    if (!dev || !config) return ESP_ERR_INVALID_ARG;
    
    uint8_t data[2];
    esp_err_t err = _tmp117_read_register16(dev, CONFIG_REG, data);
    if (err != ESP_OK) return err;
    
    uint16_t config_reg = ((uint16_t)data[0] << 8) | data[1];
    
    // Parse configuration
    config->alert_polarity = (config_reg & (1 << CONFIG_HIGH_POL_BIT)) ? ALERT_POL_HIGH : ALERT_POL_LOW;
    config->alert_mode = (config_reg & (1 << CONFIG_HIGH_T_NA_BIT)) ? ALERT_MODE_THERM : ALERT_MODE_ALERT;
    config->data_ready_alert_enable = (config_reg & (1 << CONFIG_HIGH_DR_ALERT_BIT)) ? true : false;
    config->averaging = (tmp117_averaging_t)((config_reg >> CONFIG_HIGH_AVG_BIT) & 0x03);
    config->conversion_cycle = (tmp117_conversion_cycle_t)((config_reg >> CONFIG_HIGH_CC_BIT) & 0x03);
    config->conversion_mode = (tmp117_conversion_mode_t)((config_reg >> CONFIG_LOW_MOD_BIT) & 0x03);
    
    return ESP_OK;
}

esp_err_t tmp117_set_t_low_limit(tmp117_dev_t *dev, float t_low, bool is_fahrenheit) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    // Convert to Celsius if needed
    if (is_fahrenheit) {
        t_low = (t_low - 32.0f) * 5.0f / 9.0f;
    }
    
    // Clamp to TMP117 range (-256°C to +255.99°C)
    if (t_low > 255.99f) t_low = 255.99f;
    if (t_low < -256.0f) t_low = -256.0f;
    
    int16_t raw_temp = _tmp117_celsius_to_raw(t_low);
    uint8_t data_msb = (uint8_t)(raw_temp >> 8);
    uint8_t data_lsb = (uint8_t)(raw_temp & 0xFF);
    
    esp_err_t ret = _tmp117_write_register16(dev, T_LOW_LIMIT_REG, data_msb, data_lsb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set T_LOW limit: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t tmp117_set_t_high_limit(tmp117_dev_t *dev, float t_high, bool is_fahrenheit) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    // Convert to Celsius if needed
    if (is_fahrenheit) {
        t_high = (t_high - 32.0f) * 5.0f / 9.0f;
    }
    
    // Clamp to TMP117 range (-256°C to +255.99°C)
    if (t_high > 255.99f) t_high = 255.99f;
    if (t_high < -256.0f) t_high = -256.0f;
    
    int16_t raw_temp = _tmp117_celsius_to_raw(t_high);
    uint8_t data_msb = (uint8_t)(raw_temp >> 8);
    uint8_t data_lsb = (uint8_t)(raw_temp & 0xFF);
    
    esp_err_t ret = _tmp117_write_register16(dev, T_HIGH_LIMIT_REG, data_msb, data_lsb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set T_HIGH limit: %s", esp_err_to_name(ret));
    }
    return ret;
}

float tmp117_read_t_low_limit(tmp117_dev_t *dev, bool read_in_fahrenheit) {
    if (!dev) return NAN;
    
    uint8_t data[2];
    if (_tmp117_read_register16(dev, T_LOW_LIMIT_REG, data) != ESP_OK) {
        return NAN;
    }
    
    int16_t raw_temp = ((int16_t)data[0] << 8) | data[1];
    float temp = _tmp117_raw_to_celsius(raw_temp);
    
    if (read_in_fahrenheit) {
        temp = temp * 9.0f / 5.0f + 32.0f;
    }
    
    return temp;
}

float tmp117_read_t_high_limit(tmp117_dev_t *dev, bool read_in_fahrenheit) {
    if (!dev) return NAN;
    
    uint8_t data[2];
    if (_tmp117_read_register16(dev, T_HIGH_LIMIT_REG, data) != ESP_OK) {
        return NAN;
    }
    
    int16_t raw_temp = ((int16_t)data[0] << 8) | data[1];
    float temp = _tmp117_raw_to_celsius(raw_temp);
    
    if (read_in_fahrenheit) {
        temp = temp * 9.0f / 5.0f + 32.0f;
    }
    
    return temp;
}

esp_err_t tmp117_set_temp_offset(tmp117_dev_t *dev, float offset_celsius) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    // Clamp offset to valid range (-256°C to +255.99°C)
    if (offset_celsius > 255.99f) offset_celsius = 255.99f;
    if (offset_celsius < -256.0f) offset_celsius = -256.0f;
    
    int16_t raw_offset = _tmp117_celsius_to_raw(offset_celsius);
    uint8_t data_msb = (uint8_t)(raw_offset >> 8);
    uint8_t data_lsb = (uint8_t)(raw_offset & 0xFF);
    
    esp_err_t ret = _tmp117_write_register16(dev, TEMP_OFFSET_REG, data_msb, data_lsb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set temperature offset: %s", esp_err_to_name(ret));
    }
    return ret;
}

float tmp117_read_temp_offset(tmp117_dev_t *dev) {
    if (!dev) return NAN;
    
    uint8_t data[2];
    if (_tmp117_read_register16(dev, TEMP_OFFSET_REG, data) != ESP_OK) {
        return NAN;
    }
    
    int16_t raw_offset = ((int16_t)data[0] << 8) | data[1];
    return _tmp117_raw_to_celsius(raw_offset);
}

esp_err_t tmp117_sleep(tmp117_dev_t *dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    tmp117_config_t config;
    esp_err_t err = tmp117_get_config(dev, &config);
    if (err != ESP_OK) return err;
    
    config.conversion_mode = MODE_SHUTDOWN;
    return tmp117_set_config(dev, &config);
}

esp_err_t tmp117_wakeup(tmp117_dev_t *dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    tmp117_config_t config;
    esp_err_t err = tmp117_get_config(dev, &config);
    if (err != ESP_OK) return err;
    
    config.conversion_mode = MODE_CONTINUOUS;
    return tmp117_set_config(dev, &config);
}

esp_err_t tmp117_one_shot(tmp117_dev_t *dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    tmp117_config_t config;
    esp_err_t err = tmp117_get_config(dev, &config);
    if (err != ESP_OK) return err;
    
    config.conversion_mode = MODE_ONE_SHOT_CC;
    return tmp117_set_config(dev, &config);
}

esp_err_t tmp117_is_data_ready(tmp117_dev_t *dev, bool *data_ready) {
    if (!dev || !data_ready) return ESP_ERR_INVALID_ARG;
    
    uint8_t data[2];
    esp_err_t err = _tmp117_read_register16(dev, CONFIG_REG, data);
    if (err != ESP_OK) return err;
    
    uint16_t config_reg = ((uint16_t)data[0] << 8) | data[1];
    *data_ready = (config_reg & (1 << CONFIG_LOW_DATA_READY_BIT)) ? true : false;
    
    return ESP_OK;
}

esp_err_t tmp117_soft_reset(tmp117_dev_t *dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    // Read current config to preserve settings
    uint8_t data[2];
    esp_err_t err = _tmp117_read_register16(dev, CONFIG_REG, data);
    if (err != ESP_OK) return err;
    
    // Set reset bit
    data[0] |= (1 << (CONFIG_HIGH_RESET_BIT - 8));
    
    err = _tmp117_write_register16(dev, CONFIG_REG, data[0], data[1]);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to perform soft reset: %s", esp_err_to_name(err));
        return err;
    }
    
    // Wait for reset to complete (typically ~2ms)
    vTaskDelay(pdMS_TO_TICKS(5));
    
    return ESP_OK;
}

float tmp117_read_temp_c(tmp117_dev_t *dev) {
    if (!dev) return NAN;
    
    uint8_t data[2];
    if (_tmp117_read_register16(dev, TEMP_READ_REG, data) != ESP_OK) {
        return NAN;
    }
    
    // TMP117 returns 16-bit signed temperature data, no bit shifting needed
    int16_t raw_temp = ((int16_t)data[0] << 8) | data[1];
    return _tmp117_raw_to_celsius(raw_temp);
}

float tmp117_read_temp_f(tmp117_dev_t *dev) {
    float celsius = tmp117_read_temp_c(dev);
    if (isnan(celsius)) return NAN;
    
    return celsius * 9.0f / 5.0f + 32.0f;
}

esp_err_t tmp117_get_alert_status(tmp117_dev_t *dev, bool *high_alert, bool *low_alert) {
    if (!dev || !high_alert || !low_alert) return ESP_ERR_INVALID_ARG;
    
    uint8_t data[2];
    esp_err_t err = _tmp117_read_register16(dev, CONFIG_REG, data);
    if (err != ESP_OK) return err;
    
    uint16_t config_reg = ((uint16_t)data[0] << 8) | data[1];
    
    *high_alert = (config_reg & (1 << CONFIG_LOW_HIGH_ALERT_BIT)) ? true : false;
    *low_alert = (config_reg & (1 << CONFIG_LOW_LOW_ALERT_BIT)) ? true : false;
    
    return ESP_OK;
}

float TMP117_ReadTemperature(tmp117_dev_t *dev) {
    return tmp117_read_temp_c(dev);
}