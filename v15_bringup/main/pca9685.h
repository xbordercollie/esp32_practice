#pragma once
#include <stdint.h>
#include "driver/i2c.h"

#define PCA9685_ADDR_LEFT     0x40
#define PCA9685_ADDR_RIGHT    0x41

#define PCA9685_MODE1         0x00
#define PCA9685_PRE_SCALE     0xFE
#define PCA9685_LED0_ON_L     0x06

#define PCA9685_MODE1_SLEEP   (1 << 4)
#define PCA9685_MODE1_AI      (1 << 5)
#define PCA9685_MODE1_RESTART (1 << 7)

esp_err_t pca9685_write_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val);
esp_err_t pca9685_read_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *val);
esp_err_t pca9685_init(i2c_port_t port, uint8_t addr);
esp_err_t pca9685_set_pulse(i2c_port_t port, uint8_t addr, uint8_t ch,
                            uint16_t on, uint16_t off);
                            