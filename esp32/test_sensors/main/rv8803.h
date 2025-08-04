#ifndef RV8803_H
#define RV8803_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// RV8803 I2C Address
#define RV8803_I2C_ADDR         0x32

// RV8803 Register Addresses
#define RV8803_REG_SEC          0x00
#define RV8803_REG_MIN          0x01
#define RV8803_REG_HOUR         0x02
#define RV8803_REG_WDAY         0x03
#define RV8803_REG_DATE         0x04
#define RV8803_REG_MONTH        0x05
#define RV8803_REG_YEAR         0x06
#define RV8803_REG_RAM          0x07
#define RV8803_REG_ALARM_MIN    0x08
#define RV8803_REG_ALARM_HOUR   0x09
#define RV8803_REG_ALARM_WDAY   0x0A
#define RV8803_REG_TIMER_CNT0   0x0B
#define RV8803_REG_TIMER_CNT1   0x0C
#define RV8803_REG_EXT          0x0D
#define RV8803_REG_FLAG         0x0E
#define RV8803_REG_CTRL         0x0F

// Control Register Bits
#define RV8803_CTRL_RST         (1 << 0)
#define RV8803_CTRL_EIE         (1 << 2)
#define RV8803_CTRL_AIE         (1 << 3)
#define RV8803_CTRL_TIE         (1 << 4)
#define RV8803_CTRL_UIE         (1 << 5)

// Flag Register Bits
#define RV8803_FLAG_VLF         (1 << 1)
#define RV8803_FLAG_EVF         (1 << 2)
#define RV8803_FLAG_AF          (1 << 3)
#define RV8803_FLAG_TF          (1 << 4)
#define RV8803_FLAG_UF          (1 << 5)

// Date and Time Structure
typedef struct {
    uint8_t second;     // 0-59
    uint8_t minute;     // 0-59
    uint8_t hour;       // 0-23
    uint8_t weekday;    // 0-6 (0=Sunday)
    uint8_t date;       // 1-31
    uint8_t month;      // 1-12
    uint16_t year;      // 2000-2099
} rv8803_datetime_t;

// Device Context Struct (similar to TMP117 approach)
typedef struct {
    i2c_master_dev_handle_t dev_handle; // I2C device handle for ESP-IDF v5.x+
    uint8_t device_addr; // The I2C slave address of the RV8803
} rv8803_dev_t;

// Function Prototypes
/**
 * @brief Initializes the RV8803 device.
 * @param dev Pointer to the RV8803 device context.
 * @param dev_handle I2C device handle from i2c_util.
 * @param device_addr The I2C slave address of the RV8803.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t rv8803_init(rv8803_dev_t *dev, i2c_master_dev_handle_t dev_handle, uint8_t device_addr);

/**
 * @brief Writes a single register.
 * @param dev Pointer to the RV8803 device context.
 * @param reg_addr Register address.
 * @param data Data to write.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t rv8803_write_register(rv8803_dev_t *dev, uint8_t reg_addr, uint8_t data);

/**
 * @brief Reads a single register.
 * @param dev Pointer to the RV8803 device context.
 * @param reg_addr Register address.
 * @param data Pointer to store the read data.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t rv8803_read_register(rv8803_dev_t *dev, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Writes multiple registers.
 * @param dev Pointer to the RV8803 device context.
 * @param reg_addr Starting register address.
 * @param data Pointer to data array.
 * @param len Number of bytes to write.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t rv8803_write_registers(rv8803_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Reads multiple registers.
 * @param dev Pointer to the RV8803 device context.
 * @param reg_addr Starting register address.
 * @param data Pointer to store the read data.
 * @param len Number of bytes to read.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t rv8803_read_registers(rv8803_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Sets the date and time.
 * @param dev Pointer to the RV8803 device context.
 * @param datetime Pointer to the datetime structure.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t rv8803_set_time(rv8803_dev_t *dev, const rv8803_datetime_t *datetime);

/**
 * @brief Gets the current date and time.
 * @param dev Pointer to the RV8803 device context.
 * @param datetime Pointer to store the datetime.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t rv8803_get_time(rv8803_dev_t *dev, rv8803_datetime_t *datetime);

/**
 * @brief Resets the RTC.
 * @param dev Pointer to the RV8803 device context.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t rv8803_reset(rv8803_dev_t *dev);

/**
 * @brief Checks if voltage is low.
 * @param dev Pointer to the RV8803 device context.
 * @param is_low Pointer to store the voltage status.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t rv8803_check_voltage_low(rv8803_dev_t *dev, bool *is_low);

/**
 * @brief Clears all flags.
 * @param dev Pointer to the RV8803 device context.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t rv8803_clear_flags(rv8803_dev_t *dev);

/**
 * @brief Convert decimal to BCD.
 * @param dec Decimal value.
 * @return BCD value.
 */
uint8_t rv8803_dec_to_bcd(uint8_t dec);

/**
 * @brief Convert BCD to decimal.
 * @param bcd BCD value.
 * @return Decimal value.
 */
uint8_t rv8803_bcd_to_dec(uint8_t bcd);

#ifdef __cplusplus
}
#endif

#endif // RV8803_H