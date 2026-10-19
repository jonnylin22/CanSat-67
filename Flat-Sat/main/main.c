#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "ina219.h"

static const char *TAG = "flat-sat";

#define SDA_PIN GPIO_NUM_8
#define SCL_PIN GPIO_NUM_9
#define LOOP_PERIOD_MS 250  // 4 Hz, matches telemetry rate

void app_main(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = -1,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));

    i2c_master_dev_handle_t ina;
    if (ina219_init(bus, &ina) != ESP_OK) {
        ESP_LOGE(TAG, "INA219 not found at 0x%02X", INA219_ADDR);
        return;
    }
    ESP_LOGI(TAG, "INA219 ready");

    while (1) {
        ina219_reading_t r;
        if (ina219_read(ina, &r) == ESP_OK) {
            ESP_LOGI(TAG, "bus = %.3f V   shunt = %.2f mV   current = %.1f mA",
                     r.bus_v, r.shunt_mv, r.current_ma);
        } else {
            ESP_LOGW(TAG, "INA219 read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}