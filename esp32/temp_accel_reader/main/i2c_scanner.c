#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 6
#define I2C_MASTER_SCL_IO 7
#define I2C_MASTER_FREQ_HZ 100000

static const char *TAG = "I2C_SCANNER";

esp_err_t i2c_master_init(void){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    esp_err_t res = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (res != ESP_OK) return res;

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                              0, 0, 0);
}

void app_main(void){
    ESP_ERROR_CHECK(i2c_master_init());

    ESP_LOGI(TAG, "Starting I2C scanner...");

    int devices_found = 0;

    for (uint8_t addr = 1; addr < 127; addr++){
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Device found at 0x%02X", addr);
            devices_found++;
        }
    }

    if (devices_found == 0){
        ESP_LOGI(TAG, "No I2C devices found.");
    }
}
