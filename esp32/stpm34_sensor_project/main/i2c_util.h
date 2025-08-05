#ifndef I2C_UTIL_H
#define I2C_UTIL_H

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// I2C Bus Configuration
#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_FREQ_HZ      100000
#define I2C_MASTER_TIMEOUT_MS   1000

// Known device addresses
#define RV8803_I2C_ADDR         0x32
#define TMP117_I2C_ADDR         0x48

// I2C Bus handle (shared across all devices)
extern i2c_master_bus_handle_t g_i2c_bus_handle;

/**
 * @brief Initialize the shared I2C master bus
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_util_init(void);

/**
 * @brief Deinitialize the I2C master bus
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_util_deinit(void);

/**
 * @brief Add a device to the I2C bus
 * @param device_addr I2C slave address of the device
 * @param dev_handle Pointer to store the device handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_util_add_device(uint8_t device_addr, i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Remove a device from the I2C bus
 * @param dev_handle Device handle to remove
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_util_remove_device(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Check if I2C bus is initialized
 * @return true if initialized, false otherwise
 */
bool i2c_util_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif // I2C_UTIL_H