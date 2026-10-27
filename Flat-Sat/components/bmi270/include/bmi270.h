#pragma once
#include "driver/i2c_master.h"
#include "esp_err.h"

#define BMI270_ADDR 0x68

typedef struct {
    /* measured in m/s^2 and degrees/s */
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} bmi270_reading_t;

esp_err_t bmi270_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev);
esp_err_t bmi270_read(i2c_master_dev_handle_t dev, bmi270_reading_t *out);