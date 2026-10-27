idf_component_register(SRCS "bmi270.c" "bmi270_config.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c)