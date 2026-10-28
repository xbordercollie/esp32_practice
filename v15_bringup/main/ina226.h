#pragma once
#include <stdint.h>
#include "driver/i2c.h"

/* INA226 两颗（接口文档 §3.2 / §4.1）
 *   0x44 舵机包：采样电阻 1 mΩ，量程 ±81.92 A，电流 LSB 2.5 mA，放电读正
 *   0x45 逻辑包：采样电阻 10 mΩ，量程 ±8.192 A，电流 LSB 0.25 mA，放电读负
 * 两颗的 CAL 都写 2048（= 0.00512 / (Current_LSB × R_shunt)）。
 */
#define INA226_ADDR_SERVO   0x44
#define INA226_ADDR_LOGIC   0x45

/**
 * @brief 配置 + 写校准寄存器
 * @note 必须在对应 I2C 总线初始化之后调用；0x44 在 I2C0、0x45 在 I2C1
 */
esp_err_t ina226_init(i2c_port_t port, uint8_t addr);

/**
 * @brief 读母线电压与电流
 * @param bus_mV  输出：母线电压 mV（LSB 1.25 mV）
 * @param cur_mA  输出：电流 mA（正负号按各路极性约定）
 */
esp_err_t ina226_read(i2c_port_t port, uint8_t addr, int32_t *bus_mV, int32_t *cur_mA);
