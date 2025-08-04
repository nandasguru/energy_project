#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "i2c_util.h"
#include "tmp117.h"

#define KX134_ADDR 0x1F
#define KX134_XOUT_L 0x08
#define KX134_CNTL1 0x1B
#define KX134_ODCNTL 0x21
#define TAG "SENSOR_TEST"

static i2c_master_dev_handle_t tmp117_dev_handle;
static i2c_master_dev_handle_t kx134_dev_handle;
static tmp117_dev_t tmp117_device;
static TaskHandle_t sensor_task_handle = NULL;

// === KX134 Functions ===
esp_err_t kx134_init(i2c_master_dev_handle_t dev)
{
    esp_err_t ret;
    //uint8_t data;

    // Set CNTL1 to standby (PC1 = 0) to configure
    
    uint8_t tx1[2] = {KX134_CNTL1, 0x00};
    ret = i2c_master_transmit(dev, tx1, sizeof(tx1), I2C_MASTER_TIMEOUT_MS);

    if (ret != ESP_OK) return ret;

    // Set output data rate
    uint8_t tx2[2] = {KX134_ODCNTL, 0x02};
    ret = i2c_master_transmit(dev, tx2, sizeof(tx2), I2C_MASTER_TIMEOUT_MS);

    if (ret != ESP_OK) return ret;

    // Set CNTL1 to operating mode with 2g range, low noise, high resolution
    uint8_t tx3[2] = {KX134_CNTL1, 0xC1};
    ret = i2c_master_transmit(dev, tx3, sizeof(tx3), I2C_MASTER_TIMEOUT_MS);
    return ret;
}

esp_err_t kx134_read_xyz(i2c_master_dev_handle_t dev, int16_t *x, int16_t *y, int16_t *z)
{
    esp_err_t ret;
    uint8_t reg = KX134_XOUT_L;
    uint8_t data[6];

    ret = i2c_master_transmit(dev, &reg, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) return ret;

    ret = i2c_master_receive(dev, data, sizeof(data), I2C_MASTER_TIMEOUT_MS);
    if(ret != ESP_OK) return ret;

    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);
    return ESP_OK;
}

// === Sensor Task ===
static void sensor_task(void *pvParameters)
{
    float temperature;
    int16_t x, y, z;
    int16_t last_x = 0, last_y = 0, last_z = 0;

    ESP_LOGI(TAG, "Sensor monitoring task started");

    while (1) {
        temperature = tmp117_read_temp_c(&tmp117_device);
        esp_err_t accel_status = kx134_read_xyz(kx134_dev_handle, &x, &y, &z);

        if (!isnan(temperature) && accel_status == ESP_OK) {
            float xg = x / 4096.0;
            float yg = y / 4096.0;
            float zg = z / 4096.0;

            float dx = fabsf((float)x - (float)last_x);
            float dy = fabsf((float)y - (float)last_y);
            float dz = fabsf((float)z - (float)last_z);
            float vibration = sqrtf((dx * dx + dy * dy + dz * dz) / 3.0);

            ESP_LOGI(TAG, "Temp: %.2f C | Accel (g): X=%.3f Y=%.3f Z=%.3f | Vibration=%.3f",
                     temperature, xg, yg, zg, vibration);

            if (vibration > 20.0) {
                ESP_LOGW(TAG, "Vibration Detected!");
            }

            last_x = x;
            last_y = y;
            last_z = z;
        } else {
            ESP_LOGW(TAG, "Sensor read failed: TMP117=%s, KX134=%s",
                     isnan(temperature) ? "FAIL" : "OK",
                     accel_status == ESP_OK ? "OK" : "FAIL");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// === Main ===
void app_main(void)
{
    ESP_LOGI(TAG, "Starting TMP117 + KX134 Test");

    if (i2c_util_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return;
    }

    if (i2c_util_add_device(TMP117_I2C_ADDR, &tmp117_dev_handle) != ESP_OK ||
        tmp117_init(&tmp117_device, tmp117_dev_handle, TMP117_I2C_ADDR) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize TMP117");
        return;
    }

    if (i2c_util_add_device(KX134_ADDR, &kx134_dev_handle) != ESP_OK ||
        kx134_init(kx134_dev_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize KX134");
        return;
    }

    tmp117_config_t cfg = {
        .alert_mode = ALERT_MODE_ALERT,
        .alert_polarity = ALERT_POL_LOW,
        .averaging = AVG_8_SAMPLES,
        .conversion_cycle = CC_250MS,
        .conversion_mode = MODE_CONTINUOUS,
        .data_ready_alert_enable = false
    };
    tmp117_set_config(&tmp117_device, &cfg);

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, &sensor_task_handle);

    ESP_LOGI(TAG, "System initialized. Logging every 2s");
}
