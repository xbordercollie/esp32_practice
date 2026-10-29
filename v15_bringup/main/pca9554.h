/**
 * @file pca9554.h
 * @brief PCA9554 8 位 I2C 扩展器（v1.5 每块板一颗，地址待硬件确认）
 *
 * 寄存器（硬件接口文档 §4.3）：
 *   0x00 输入端口（读）
 *   0x01 输出端口
 *   0x02 极性反转
 *   0x03 配置：1=输入，0=输出（注意是反的）
 *
 * 引脚功能未文档化，启动只探测/读输入口，不改配置。
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"

#define PCA9554_ADDR_SERVO   0x20   /* 预期：舵机板，I2C0 */
#define PCA9554_ADDR_LOGIC   0x21   /* 预期：逻辑板，I2C1 */

#define PCA9554_REG_INPUT    0x00
#define PCA9554_REG_OUTPUT   0x01
#define PCA9554_REG_POLARITY 0x02
#define PCA9554_REG_CONFIG   0x03

/** @brief 读输入端口探测器件；存在返回 true */
bool pca9554_probe(i2c_port_t port, uint8_t addr);

esp_err_t pca9554_read_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *val);
esp_err_t pca9554_write_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val);
