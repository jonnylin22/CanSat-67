#include "bmi270.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include <string.h>

#define CHIP_ID_ADDR     0x00
#define PWR_CONF    0x7C
#define PWR_CTRL    0x7D
#define INIT_CTRL   0x59
#define INIT_DATA   0x5E
#define INIT_STATUS  0x21
#define ACC_CONF    0x40
#define GYR_CONF    0x42
#define ACC_DATA    0x0C
#define INIT_ADDR0  0x5B 
#define INIT_ADDR1  0x5C 

#define TIMEOUT_MS 50
#define CHUNK_SIZE 32
#define CONFIG_FILE_SIZE 8192
#define FREQ_HZ 100000 
#define POLL_CAP 150
#define POLL_EVERY 15
#define ACC_LSB_PER_G   4096.0f // will have to change this and next depending on state
#define GYR_LSB_PER_DPS 16.384f   
#define GRAVITY 9.80665f
// init flow is as follows according to docs
// read chip_id and check if its 0x24
// then disable power using pwr_conf_reg set to 0x00
// then sleep for 450 us using rtos
// prepare config load using init_Ctrl set to 0x00
// then load config file and save as array
// then write to init_data
// then complete config load setting init_Ctrl to 0x01
// then read init_status and check if its 1

// normal config flow is as follows
// pwr_ctrl to 0x0E
// acc_conf to 0xA8
// gry_conf to 0xA9
// pwr_conf to 0x02
// read data from acc_Data_reg

extern const uint8_t bmi270_config_file[8192];

static esp_err_t read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data) {
    return i2c_master_transmit_receive(dev, &reg, 1, data, 1, TIMEOUT_MS);
}

static esp_err_t read_burst(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, TIMEOUT_MS);
}

static esp_err_t write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value) {
    uint8_t transmit_data[2] = {reg,value};
    return i2c_master_transmit(dev, transmit_data, 2, TIMEOUT_MS);
}

static esp_err_t upload_config(i2c_master_dev_handle_t dev) {
    /*puts the config file on the chip incrementally instead of all at once
    can alter based on tests, or if we go with the better esp32 chip*/
    uint8_t buffer[1+CHUNK_SIZE];
    esp_err_t err;
    for(uint16_t offset = 0; offset < CONFIG_FILE_SIZE; offset += CHUNK_SIZE){
        uint16_t pos = offset/2;
        err = write_reg(dev, INIT_ADDR0, pos & 0b00001111);
        if(err != ESP_OK){
            return err;
        }
        err = write_reg(dev, INIT_ADDR1, pos >> 4);
        if(err != ESP_OK){
            return err;
        }

        buffer[0] = INIT_DATA;
        memcpy(&buffer[1], &bmi270_config_file[offset], CHUNK_SIZE);
        err = i2c_master_transmit(dev, buffer, sizeof(buffer), TIMEOUT_MS);
        if(err != ESP_OK){
            return err;
        }
    }
    return ESP_OK;
}


esp_err_t bmi270_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev) {
    esp_err_t err;
    //probe device
    err = i2c_master_probe(bus, BMI270_ADDR, TIMEOUT_MS);
    if(err != ESP_OK){
        return err;
    }
    //config then add
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMI270_ADDR,
        .scl_speed_hz = FREQ_HZ,
    };
    err = i2c_master_bus_add_device(bus, &dev_cfg, dev);
    if(err != ESP_OK){
        return err;
    }

    //check chip id is right
    uint8_t chip_id;
    err = read_reg(*dev, CHIP_ID_ADDR, &chip_id);
    if(err != ESP_OK){
        return err;
    } else if(chip_id != 0x24){
        return ESP_ERR_NOT_FOUND;
    }
    //disable power_conf
    err = write_reg(*dev, PWR_CONF, 0x00);
    if(err != ESP_OK){
        return err;
    }
    esp_rom_delay_us(450);
    // set up init_ctrl
    err = write_reg(*dev, INIT_CTRL, 0x00);
    if(err != ESP_OK){
        return err;
    }
    //upload file
    err = upload_config(*dev);
    if(err != ESP_OK){
        return err;
    }
    //complete config load
    err = write_reg(*dev, INIT_CTRL, 0x01);
    if(err != ESP_OK){
        return err;
    } 


    // poll for >= 140ms, i picked 150
    uint8_t status = 0;
    for(int time_elapsed = 0; time_elapsed < POLL_CAP; time_elapsed += POLL_EVERY){
        vTaskDelay(pdMS_TO_TICKS(POLL_EVERY));
        err = read_reg(*dev, INIT_STATUS, &status);
        if(err == ESP_OK && ((status & 0b00001111) == 0b1)){
            break;
        }
    }
    if((status & 0b00001111) != 1){
        return ESP_FAIL;
    }
    err = write_reg(*dev, PWR_CTRL, 0x0E);
    if(err != ESP_OK){
        return err;
    }
    err = write_reg(*dev, ACC_CONF, 0xA8);
    if(err != ESP_OK){
        return err;
    }
    err = write_reg(*dev, GYR_CONF, 0xE9); //im electing to try out performance mode
    if(err != ESP_OK){
        return err;
    }
    err = write_reg(*dev, PWR_CONF, 0x02);
    if(err != ESP_OK){
        return err;
    }
    return ESP_OK;    
}

esp_err_t bmi270_read(i2c_master_dev_handle_t dev, bmi270_reading_t *out) {
       uint8_t read_data[12];
       esp_err_t err = read_burst(dev, ACC_DATA, read_data, sizeof(read_data));
       if(err != ESP_OK){
            return err;
        }
        int16_t ax = (int16_t)((read_data[1]  << 8) | read_data[0]);
        int16_t ay = (int16_t)((read_data[3]  << 8) | read_data[2]);
        int16_t az = (int16_t)((read_data[5]  << 8) | read_data[4]);
        int16_t gx = (int16_t)((read_data[7]  << 8) | read_data[6]);
        int16_t gy = (int16_t)((read_data[9]  << 8) | read_data[8]);
        int16_t gz = (int16_t)((read_data[11] << 8) | read_data[10]);

        out->accel_x = ax / ACC_LSB_PER_G * GRAVITY;
        out->accel_y = ay / ACC_LSB_PER_G * GRAVITY;
        out->accel_z = az / ACC_LSB_PER_G * GRAVITY;
        out->gyro_x  = gx / GYR_LSB_PER_DPS;
        out->gyro_y  = gy / GYR_LSB_PER_DPS;
        out->gyro_z  = gz / GYR_LSB_PER_DPS;
        return ESP_OK;
}