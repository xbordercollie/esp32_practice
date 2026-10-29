#include "pca9554.h"
#include "freertos/FreeRTOS.h"

esp_err_t pca9554_read_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *val)
{
    return i2c_master_write_read_device(port, addr, &reg, 1, val, 1, pdMS_TO_TICKS(20));
}

esp_err_t pca9554_write_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(port, addr, buf, 2, pdMS_TO_TICKS(20));
}

/* 只读输入端口：不写 CONFIG，避免把未知引脚改成输出 */
bool pca9554_probe(i2c_port_t port, uint8_t addr)
{
    uint8_t v = 0;
    return pca9554_read_reg(port, addr, PCA9554_REG_INPUT, &v) == ESP_OK;
}
