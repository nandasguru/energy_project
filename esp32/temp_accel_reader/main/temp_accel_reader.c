#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 6
#define I2C_MASTER_SCL_IO 7
#define I2C_MASTER_FREQUENCY_HZ 10000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define TMP117_ADDR 0x48
#define TMP117_TEMP_REG 0x00

#define KX134_ADDR 0x1F
#define KX134_XOUT_L 0x08
#define KX134_CNTL1 0x1B
#define KX134_ODCNTL 0x21
#define KX134_CNTL1_PC1_BIT 0x80

#define KX134_WHO_AM_I 0x13

#define INTERVAL_S 2.0 // In seconds

esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQUENCY_HZ,
        .clk_flags = 0,
    };
    esp_err_t res = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (res != ESP_OK) return res;

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t tmp117_read_temp(float *temperature) {
    uint8_t data_h, data_l;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TMP117_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data_h, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data_l, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    int16_t temp_raw = (data_h << 8) | data_l;
    *temperature = temp_raw * 0.0078125;

    return ESP_OK;
}

esp_err_t kx134_init(void) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (KX134_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, KX134_CNTL1, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (KX134_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, KX134_ODCNTL, true);
    i2c_master_write_byte(cmd, 0x02, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (KX134_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, KX134_CNTL1, true);
    i2c_master_write_byte(cmd, 0xC1, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t kx134_read_xyz(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (KX134_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, KX134_XOUT_L, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (KX134_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);

    return ESP_OK;
}

void app_main(void) {
    i2c_master_init();
    kx134_init();

    float last_x = 0.0, last_y = 0.0, last_z = 0.0;
    
    while (1) {
        float temperature = 0.0;
        int16_t x, y, z;
        esp_err_t temp_status = tmp117_read_temp(&temperature);
        esp_err_t accel_status = kx134_read_xyz(&x, &y, &z);

        if (temp_status == ESP_OK && accel_status == ESP_OK) {
            float xg = x / 4096.0;
            float yg = y / 4096.0;
            float zg = z / 4096.0;

            float dx = fabs(x - last_x);
            float dy = fabs(y - last_y);
            float dz = fabs(z - last_z);

            // Root Mean Square (RMS) of deltas for vibration value
            float vibration_value = sqrtf(((dx*dx) + (dy*dy) + (dz*dz))  / 3.0 );

            printf(
                "{\"Temperature\": %.2f, "
//                "\"raw_x\": %d, "
                "\"x_g\": %.3f, "
//                "\"raw_y\": %d, "
                "\"y_g\": %.3f, "
//                "\"raw_z\": %d, "
                "\"z_g\": %.3f, "
                "\"Vibration\": %.3f}\n", 
                temperature, /*x,*/ xg, /*y,*/ yg, /*z,*/ zg, vibration_value
            );

            if(vibration_value > 20){
                printf("Vibration Detected\n");
            }
            last_x = x;
            last_y = y;
            last_z = z;

            
        } else {
            printf("Failed to read sensors\n");
        }

        vTaskDelay(INTERVAL_S * 1000.0 / portTICK_PERIOD_MS);
    }
}
