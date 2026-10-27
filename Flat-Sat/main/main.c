#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "ina219.h"
#include "bmi270.h"

static const char *TAG = "flat-sat";

#define SDA_PIN        GPIO_NUM_4   // blue
#define SCL_PIN        GPIO_NUM_5   // yellow
#define LOOP_PERIOD_MS 250          // 4 Hz, matches telemetry rate

void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(3000));  // time to attach the monitor

    i2c_master_bus_config_t bus_config = {
        .i2c_port = -1,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));
    i2c_master_bus_reset(bus);   // unstick a sensor left mid-transfer

    i2c_master_dev_handle_t ina, imu;
    esp_err_t ina_err = ina219_init(bus, &ina);
    esp_err_t imu_err = bmi270_init(bus, &imu);

    while (1) {
        // INA219
        if (ina_err != ESP_OK) {
            ESP_LOGE(TAG, "INA219 init failed: %s", esp_err_to_name(ina_err));
        } else {
            ina219_reading_t r;
            if (ina219_read(ina, &r) == ESP_OK) {
                ESP_LOGI(TAG, "bus = %.3f V   shunt = %.2f mV   current = %.1f mA",
                         r.bus_v, r.shunt_mv, r.current_ma);
            } else {
                ESP_LOGW(TAG, "INA219 read failed");
            }
        }

        // BMI270
        if (imu_err != ESP_OK) {
            ESP_LOGE(TAG, "BMI270 init failed: %s", esp_err_to_name(imu_err));
        } else {
            bmi270_reading_t m;
            if (bmi270_read(imu, &m) == ESP_OK) {
                ESP_LOGI(TAG, "acc = %.2f %.2f %.2f m/s^2   gyro = %.2f %.2f %.2f dps",
                         m.accel_x, m.accel_y, m.accel_z,
                         m.gyro_x, m.gyro_y, m.gyro_z);
            } else {
                ESP_LOGW(TAG, "BMI270 read failed");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}