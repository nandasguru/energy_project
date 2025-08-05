#ifndef TMP117_H
#define TMP117_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h> // For NAN
#include "esp_err.h" // For esp_err_t return types
#include "driver/i2c_master.h" // For i2c_master_dev_handle_t

// --- TMP117 Register Addresses ---
#define TEMP_READ_REG       0x00
#define CONFIG_REG          0x01
#define T_HIGH_LIMIT_REG    0x02
#define T_LOW_LIMIT_REG     0x03
#define EEPROM_UL_REG       0x04
#define EEPROM1_REG         0x05
#define EEPROM2_REG         0x06
#define EEPROM3_REG         0x07
#define TEMP_OFFSET_REG     0x07  // Same as EEPROM3
#define DEVICE_ID_REG       0x0F

// --- TMP117 Device ID ---
#define TMP117_DEVICE_ID    0x0117

// --- Configuration Register Bit Definitions (CONFIG_REG) ---
// High Byte (bits 15-8)
#define CONFIG_HIGH_RESET_BIT       15 // Software Reset (R/W)
#define CONFIG_HIGH_DR_ALERT_BIT    14 // Data Ready Alert (R/W)
#define CONFIG_HIGH_POL_BIT         13 // ALERT Polarity (R/W)
#define CONFIG_HIGH_T_NA_BIT        12 // Therm/Alert mode select (R/W)
#define CONFIG_HIGH_AVG_BIT         10 // Conversion averaging modes (R/W) (2 bits)
#define CONFIG_HIGH_CC_BIT          8  // Conversion cycle time (R/W) (2 bits)

// Low Byte (bits 7-0)
#define CONFIG_LOW_MOD_BIT          4  // Conversion mode (R/W) (2 bits)
#define CONFIG_LOW_EEPROM_BUSY_BIT  3  // EEPROM busy (R)
#define CONFIG_LOW_DATA_READY_BIT   2  // Data ready flag (R)
#define CONFIG_LOW_LOW_ALERT_BIT    1  // Low Alert flag (R)
#define CONFIG_LOW_HIGH_ALERT_BIT   0  // High Alert flag (R)

// --- Enums for TMP117 Configuration ---
typedef enum {
    ALERT_MODE_ALERT = 0,    // Alert mode (default)
    ALERT_MODE_THERM = 1     // Thermostat mode
} tmp117_alert_mode_t;

typedef enum {
    ALERT_POL_LOW = 0,       // Active low (default)
    ALERT_POL_HIGH = 1       // Active high
} tmp117_alert_polarity_t;

typedef enum {
    AVG_NONE = 0,           // No averaging (1 sample)
    AVG_8_SAMPLES = 1,      // 8 averaged samples
    AVG_32_SAMPLES = 2,     // 32 averaged samples  
    AVG_64_SAMPLES = 3      // 64 averaged samples
} tmp117_averaging_t;

typedef enum {
    CC_15_5MS = 0,          // 15.5ms conversion cycle
    CC_125MS = 1,           // 125ms conversion cycle
    CC_250MS = 2,           // 250ms conversion cycle
    CC_500MS = 3            // 500ms conversion cycle
} tmp117_conversion_cycle_t;

typedef enum {
    MODE_CONTINUOUS = 0,    // Continuous conversion (default)
    MODE_SHUTDOWN = 1,      // Shutdown mode
    MODE_ONE_SHOT_CC = 2,   // One-shot with conversion cycle time
    MODE_ONE_SHOT_NO_CC = 3 // One-shot, conversion cycle time ignored
} tmp117_conversion_mode_t;

// --- Configuration Struct ---
typedef struct {
    tmp117_alert_mode_t alert_mode;
    tmp117_alert_polarity_t alert_polarity;
    tmp117_averaging_t averaging;
    tmp117_conversion_cycle_t conversion_cycle;
    tmp117_conversion_mode_t conversion_mode;
    bool data_ready_alert_enable;
} tmp117_config_t;

// --- Device Context Struct ---
typedef struct {
    i2c_master_dev_handle_t dev_handle; // I2C device handle for ESP-IDF v5.x+
    uint8_t device_addr; // The I2C slave address of the TMP117
} tmp117_dev_t;

// --- Function Prototypes ---

/**
 * @brief Initializes the TMP117 sensor and verifies device ID.
 * @param dev Pointer to the TMP117 device context.
 * @param dev_handle I2C device handle.
 * @param device_addr The I2C slave address of the TMP117.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t tmp117_init(tmp117_dev_t *dev, i2c_master_dev_handle_t dev_handle, uint8_t device_addr);

/**
 * @brief Reads and verifies the TMP117 device ID.
 * @param dev Pointer to the TMP117 device context.
 * @param device_id Pointer to store the device ID.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t tmp117_read_device_id(tmp117_dev_t *dev, uint16_t *device_id);

/**
 * @brief Sets the configuration of the TMP117 sensor.
 * @param dev Pointer to the TMP117 device context.
 * @param config Pointer to the configuration struct.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t tmp117_set_config(tmp117_dev_t *dev, const tmp117_config_t *config);

/**
 * @brief Gets the current configuration of the TMP117 sensor.
 * @param dev Pointer to the TMP117 device context.
 * @param config Pointer to store the configuration.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t tmp117_get_config(tmp117_dev_t *dev, tmp117_config_t *config);

/**
 * @brief Sets the T_LOW threshold.
 * @param dev Pointer to the TMP117 device context.
 * @param t_low The desired T_LOW temperature in Celsius.
 * @param is_fahrenheit True if t_low is in Fahrenheit, false for Celsius.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t tmp117_set_t_low_limit(tmp117_dev_t *dev, float t_low, bool is_fahrenheit);

/**
 * @brief Sets the T_HIGH threshold.
 * @param dev Pointer to the TMP117 device context.
 * @param t_high The desired T_HIGH temperature in Celsius.
 * @param is_fahrenheit True if t_high is in Fahrenheit, false for Celsius.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t tmp117_set_t_high_limit(tmp117_dev_t *dev, float t_high, bool is_fahrenheit);

/**
 * @brief Reads the T_LOW threshold from the sensor.
 * @param dev Pointer to the TMP117 device context.
 * @param read_in_fahrenheit True to read in Fahrenheit, false for Celsius.
 * @return The T_LOW temperature, or NAN on error.
 */
float tmp117_read_t_low_limit(tmp117_dev_t *dev, bool read_in_fahrenheit);

/**
 * @brief Reads the T_HIGH threshold from the sensor.
 * @param dev Pointer to the TMP117 device context.
 * @param read_in_fahrenheit True to read in Fahrenheit, false for Celsius.
 * @return The T_HIGH temperature, or NAN on error.
 */
float tmp117_read_t_high_limit(tmp117_dev_t *dev, bool read_in_fahrenheit);

/**
 * @brief Sets the temperature offset for calibration.
 * @param dev Pointer to the TMP117 device context.
 * @param offset_celsius Temperature offset in Celsius (-256°C to +255.99°C).
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t tmp117_set_temp_offset(tmp117_dev_t *dev, float offset_celsius);

/**
 * @brief Reads the temperature offset.
 * @param dev Pointer to the TMP117 device context.
 * @return Temperature offset in Celsius, or NAN on error.
 */
float tmp117_read_temp_offset(tmp117_dev_t *dev);

/**
 * @brief Puts the TMP117 sensor into shutdown mode.
 * @param dev Pointer to the TMP117 device context.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t tmp117_sleep(tmp117_dev_t *dev);

/**
 * @brief Wakes up the TMP117 sensor from shutdown mode.
 * @param dev Pointer to the TMP117 device context.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t tmp117_wakeup(tmp117_dev_t *dev);

/**
 * @brief Triggers a one-shot conversion.
 * @param dev Pointer to the TMP117 device context.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t tmp117_one_shot(tmp117_dev_t *dev);

/**
 * @brief Checks if data is ready for reading.
 * @param dev Pointer to the TMP117 device context.
 * @param data_ready Pointer to store data ready status.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t tmp117_is_data_ready(tmp117_dev_t *dev, bool *data_ready);

/**
 * @brief Performs a software reset of the TMP117.
 * @param dev Pointer to the TMP117 device context.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t tmp117_soft_reset(tmp117_dev_t *dev);

/**
 * @brief Reads the current temperature in Celsius.
 * @param dev Pointer to the TMP117 device context.
 * @return Temperature in Celsius, or NAN on error.
 */
float tmp117_read_temp_c(tmp117_dev_t *dev);

/**
 * @brief Reads the current temperature in Fahrenheit.
 * @param dev Pointer to the TMP117 device context.
 * @return Temperature in Fahrenheit, or NAN on error.
 */
float tmp117_read_temp_f(tmp117_dev_t *dev);

/**
 * @brief Checks alert status flags.
 * @param dev Pointer to the TMP117 device context.
 * @param high_alert Pointer to store high alert status.
 * @param low_alert Pointer to store low alert status.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t tmp117_get_alert_status(tmp117_dev_t *dev, bool *high_alert, bool *low_alert);

// High-level temperature function (compatible with TMP112)
float TMP117_ReadTemperature(tmp117_dev_t *dev);

#endif // TMP117_H