#pragma once
#include "driver/i2c_master.h"
#include "esp_err.h"
 
#define INA219_ADDR 0x40
 
typedef struct {
    float v_read;
    float shunt_mv;
    float current_ma; 
} ina219_reading_t;

// instantiates ina219
esp_err_t ina219_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev);
// reads data from ina219
esp_err_t ina219_read(i2c_master_dev_handle_t dev, ina219_reading_t *out);
 