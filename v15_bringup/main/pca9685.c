#include "pca9685.h"
#include "freertos/FreeRTOS.h"
#include "esp_rom_sys.h"
#include <stdio.h>

esp_err_t pca9685_write_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(port, addr, buf, 2, pdMS_TO_TICKS(20));
}

/*读取地址 */
esp_err_t pca9685_read_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *val)
{
    return i2c_master_write_read_device(port, addr, &reg, 1, val, 1, pdMS_TO_TICKS(20));
}

esp_err_t pca9685_init(i2c_port_t port, uint8_t addr)
{
    uint8_t mode1 = 0, pre_scale = 0;
    esp_err_t err;

    /*1. Read the current MODE1 register value*/
    err = pca9685_read_reg(port, addr, PCA9685_MODE1, &mode1);
    if (err != ESP_OK) {
        return err;
    }

    /*2. Set the SLEEP bit to enter sleep mode*/
    uint8_t sleep_mode = mode1 | PCA9685_MODE1_SLEEP;
    err = pca9685_write_reg(port, addr, PCA9685_MODE1, sleep_mode);
    if (err != ESP_OK) {
        return err;
    }

    /*3.Set the PRE_SCALE = 60*/
    pre_scale = 60;
    err = pca9685_write_reg(port, addr, PCA9685_PRE_SCALE, pre_scale);
    if (err != ESP_OK) {
        return err;
    }

    /*4. Clear the SLEEP bit to exit sleep mode*/
    uint8_t wake_mode = mode1 & ~PCA9685_MODE1_SLEEP;
    err = pca9685_write_reg(port, addr, PCA9685_MODE1, wake_mode);
    if (err != ESP_OK) {
        return err;
    }

    /*5. Wait for at least 500us to allow the oscillator to stabilize*/
    esp_rom_delay_us(500);

        /* 6. 读回并校验 */
    err = pca9685_read_reg(port, addr, PCA9685_MODE1, &mode1);
    if (err != ESP_OK) {
        return err;
    }
    err = pca9685_read_reg(port, addr, PCA9685_PRE_SCALE, &pre_scale);
    if (err != ESP_OK) {
        return err;
    }

    printf("PCA9685 0x%02X MODE1=0x%02X PRE_SCALE=%u\n", addr, mode1, pre_scale);

    if (mode1 & PCA9685_MODE1_SLEEP) {
        return ESP_FAIL;
    }
    if (pre_scale != 60) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t pca9685_set_pulse(i2c_port_t port, uint8_t addr, uint8_t ch,
                            uint16_t on, uint16_t off)
{
    if (ch > 15) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t base = PCA9685_LED0_ON_L + 4 * ch;
    esp_err_t err;

    err = pca9685_write_reg(port, addr, base + 0, (uint8_t)(on & 0xFF));
    if (err != ESP_OK) return err;
    err = pca9685_write_reg(port, addr, base + 1, (uint8_t)((on >> 8) & 0x0F));
    if (err != ESP_OK) return err;
    err = pca9685_write_reg(port, addr, base + 2, (uint8_t)(off & 0xFF));
    if (err != ESP_OK) return err;
    err = pca9685_write_reg(port, addr, base + 3, (uint8_t)((off >> 8) & 0x0F));
    if (err != ESP_OK) return err;

    return ESP_OK;
}