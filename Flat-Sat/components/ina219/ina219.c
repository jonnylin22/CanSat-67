#include "ina219.h"

#define CONFIG_REG 0x00
#define SHUNT_REG 0x01  // measured in 10 uV chunks
#define BUS_REG 0x02  // measured in 4 mv chunks
#define FREQ_HZ 100000
#define TIMEOUT_MS 50
#define SHUNT_R 0.1f // measured in ohms

static esp_err_t read_data(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t *data) {
    /*Reads sensor values from ina219 by transmitting the register to pull data from, then pulls the data,
    then checking if it is good to do so*/
    uint8_t sensor_read[2];
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, sensor_read, 2, TIMEOUT_MS);
    // i2c_dev, *write_buffer, write_size, *read_buffer, read_size, timeout_ms
    if (err == ESP_OK) {
        *data = ((uint16_t)sensor_read[0] << 8) | sensor_read[1];
    }
    return err;
}

esp_err_t ina219_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev) {
    /*Instantiates ina219 i2c device by checking if connected to device and adding the configged device*/
    esp_err_t err = i2c_master_probe(bus, INA219_ADDR, TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INA219_ADDR,
        .scl_speed_hz = FREQ_HZ,
    };
    return i2c_master_bus_add_device(bus, &dev_cfg, dev);
}

esp_err_t ina219_read(i2c_master_dev_handle_t dev, ina219_reading_t *out) {
    /*reads values from sensors, translates them to usable values*/
    uint16_t bus_reading, shunt_reading;
    esp_err_t err;
    err = read_data(dev, BUS_REG, &bus_reading);
    if (err != ESP_OK){
        return err;
    }
    err = read_data(dev, SHUNT_REG, &shunt_reading);
    if (err != ESP_OK){
        return err;
    }
    out->bus_v = (bus_reading >> 3) * 0.004f; // 3 status bits shifted out, reading is raw * 4 / 1000
    out->shunt_mv = (int16_t)shunt_reading * 0.01f; // reading is 10 / 1000 gives mili
    out->current_ma = out->shunt_mv / SHUNT_R;
    return ESP_OK;
}