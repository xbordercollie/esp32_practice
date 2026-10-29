#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"

#define PCA9685_ADDR_LEFT     0x40
#define PCA9685_ADDR_RIGHT    0x41

#define PCA9685_MODE1         0x00
#define PCA9685_MODE2         0x01
#define PCA9685_PRE_SCALE     0xFE
#define PCA9685_LED0_ON_L     0x06

#define PCA9685_MODE1_SLEEP   (1 << 4)
#define PCA9685_MODE1_AI      (1 << 5)
#define PCA9685_MODE1_RESTART (1 << 7)

#define PCA9685_PWM_HZ      50
uint16_t servo_pulse_us(int16_t deg_x10);      /* 角度(0.1°) → 脉宽(µs) */
uint16_t pulse_us_to_count(uint16_t us);       /* 脉宽(µs) → 50Hz 计数 */

esp_err_t pca9685_write_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val);
esp_err_t pca9685_read_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *val);
esp_err_t pca9685_init(i2c_port_t port, uint8_t addr);
esp_err_t pca9685_set_pulse(i2c_port_t port, uint8_t addr, uint8_t ch,
                            uint16_t on, uint16_t off);

/** @brief 该地址的 PCA9685 是否初始化成功（0x40/0x41） */
bool pca9685_is_ready(uint8_t addr);

/** @brief 18 路全部回中位（1500µs）；返回失败路数 */
int servo_center_all(void);

/** @brief 单路原始脉宽（500..2500µs），!K 测试用 */
esp_err_t servo_set_pulse_us(i2c_port_t port, uint8_t servo_id, uint16_t us);

esp_err_t servo_set_angle(i2c_port_t port, uint8_t servo_id, int16_t deg_x10);

/* 最近一次成功写出的角度（0.1°，已钳到 ±900），!A 快照用 */
extern int16_t g_servo_deg[18];
