/**
 * @file ina226.c
 * @brief INA226 电流/电压采样（两颗：舵机包 0x44 / 逻辑包 0x45）
 *
 * 关键点：
 *   1) 校准寄存器 CAL = 0.00512 / (Current_LSB × R_shunt)，两颗都取 2048；
 *      对应舵机包(1mΩ) 2.5 mA/LSB、逻辑包(10mΩ) 0.25 mA/LSB。
 *   2) 母线电压 LSB = 1.25 mV（无符号 16 位），分流电压 LSB = 2.5 µV（有符号）。
 *   3) 两颗极性相反：舵机包放电读正、逻辑包放电读负 —— 显示时保留原始符号。
 */
#include "ina226.h"

#define INA226_REG_CONFIG   0x00
#define INA226_REG_BUS_V    0x02
#define INA226_REG_CURRENT  0x04
#define INA226_REG_CAL      0x05

/* 0x4127: AVG=16 次, 总线转换 1.1 ms, 分流转换 1.1 ms, 连续测量 分流+总线 */
#define INA226_CONFIG_VAL   0x4127
#define INA226_CAL_VAL      2048

static esp_err_t write_reg16(i2c_port_t port, uint8_t addr, uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_master_write_to_device(port, addr, buf, 3, pdMS_TO_TICKS(50));
}

static esp_err_t read_reg16(i2c_port_t port, uint8_t addr, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2] = {0, 0};
    esp_err_t err = i2c_master_write_read_device(port, addr, &reg, 1, buf, 2, pdMS_TO_TICKS(50));
    if (err == ESP_OK) {
        *val = (uint16_t)((buf[0] << 8) | buf[1]);
    }
    return err;
}

esp_err_t ina226_init(i2c_port_t port, uint8_t addr)
{
    esp_err_t err = write_reg16(port, addr, INA226_REG_CONFIG, INA226_CONFIG_VAL);
    if (err != ESP_OK) {
        return err;
    }
    err = write_reg16(port, addr, INA226_REG_CAL, INA226_CAL_VAL);
    if (err != ESP_OK) {
        return err;
    }
    /* 读回 CAL 自检（写进去的必须等于读出来的） */
    uint16_t cal = 0;
    err = read_reg16(port, addr, INA226_REG_CAL, &cal);
    if (err != ESP_OK) {
        return err;
    }
    if (cal != INA226_CAL_VAL) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ina226_read(i2c_port_t port, uint8_t addr, int32_t *bus_mV, int32_t *cur_mA)
{
    if (!bus_mV || !cur_mA) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw = 0;
    esp_err_t err = read_reg16(port, addr, INA226_REG_BUS_V, &raw);
    if (err != ESP_OK) {
        return err;
    }
    /* LSB = 1.25 mV → mV = raw × 1.25 = raw × 5 / 4 */
    *bus_mV = (int32_t)raw * 5 / 4;

    err = read_reg16(port, addr, INA226_REG_CURRENT, &raw);
    if (err != ESP_OK) {
        return err;
    }
    int16_t cur_raw = (int16_t)raw;                 /* 电流是有符号 16 位 */
    if (addr == INA226_ADDR_SERVO) {
        *cur_mA = (int32_t)(cur_raw * 5) / 2;       /* 2.5 mA/LSB */
    } else {
        *cur_mA = (int32_t)(cur_raw) / 4;           /* 0.25 mA/LSB */
    }
    return ESP_OK;
}
