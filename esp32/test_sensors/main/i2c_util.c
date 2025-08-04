#include "i2c_util.h"
#include "esp_log.h"

static const char *TAG = "I2C_UTIL";

// Global I2C bus handle
i2c_master_bus_handle_t g_i2c_bus_handle = NULL;
static bool s_i2c_initialized = false;

esp_err_t i2c_util_init(void)
{
    if (s_i2c_initialized) {
        ESP_LOGW(TAG, "I2C bus already initialized");
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &g_i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    s_i2c_initialized = true;
    ESP_LOGI(TAG, "I2C master bus initialized (SCL: %d, SDA: %d, Freq: %d Hz)", 
             I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, I2C_MASTER_FREQ_HZ);
    
    return ESP_OK;
}

esp_err_t i2c_util_deinit(void)
{
    if (!s_i2c_initialized || g_i2c_bus_handle == NULL) {
        ESP_LOGW(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = i2c_del_master_bus(g_i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    g_i2c_bus_handle = NULL;
    s_i2c_initialized = false;
    ESP_LOGI(TAG, "I2C master bus deinitialized");
    
    return ESP_OK;
}

esp_err_t i2c_util_add_device(uint8_t device_addr, i2c_master_dev_handle_t *dev_handle)
{
    if (!s_i2c_initialized || g_i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized. Call i2c_util_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    if (dev_handle == NULL) {
        ESP_LOGE(TAG, "Device handle pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_master_bus_add_device(g_i2c_bus_handle, &dev_config, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device 0x%02X: %s", device_addr, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Device 0x%02X added to I2C bus", device_addr);
    return ESP_OK;
}

esp_err_t i2c_util_remove_device(i2c_master_dev_handle_t dev_handle)
{
    if (!s_i2c_initialized || g_i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (dev_handle == NULL) {
        ESP_LOGE(TAG, "Device handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_bus_rm_device(dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove device: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Device removed from I2C bus");
    return ESP_OK;
}

bool i2c_util_is_initialized(void)
{
    return s_i2c_initialized && (g_i2c_bus_handle != NULL);
}