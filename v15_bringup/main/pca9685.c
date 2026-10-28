#include "pca9685.h"
#include "freertos/FreeRTOS.h"
#include "esp_rom_sys.h"
#include <stdio.h>

#define PCA9685_PWM_HZ 50

/* 写地址 */
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

/* 初始化 PCA9685
 * 1. 读 MODE1
 * 2. 设置 SLEEP=1
 * 3. 设置 PRE_SCALE=121
 * 4. 清 SLEEP=0
 * 5. 等待 >500us
 * 6. 读回并校验
 */
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

    /*3.Set the PRE_SCALE = 121*/
    uint8_t prescale_tx = (uint8_t)(25000000.0f / (4096.0f * PCA9685_PWM_HZ) + 0.5f) - 1;
    err = pca9685_write_reg(port, addr, PCA9685_PRE_SCALE, prescale_tx);
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
    if (pre_scale != prescale_tx) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* 设置通道脉宽
 * on/off 范围 0~4095
 */
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

/* 角度(0.1°) → 脉宽(µs)
 * -90° → 500µs → -900
 *  0°  → 1500µs → 0
 * +90° → 2500µs → 900
 */
uint16_t servo_pulse_us(int16_t deg_x10)
{
    /* 限幅 */
    if (deg_x10 < -900) deg_x10 = -900;
    if (deg_x10 > 900) deg_x10 = 900;

    /* 0.1° → µs */
    int32_t pulse = 1500 + (int16_t)deg_x10 * 1000 / 900;
    return (uint16_t)pulse;
}

/* 脉宽(µs) → 50Hz 计数
 * 50Hz → 20ms周期
 */
uint16_t pulse_us_to_count(uint16_t us)
{
    /* 50Hz → 20ms周期 */
    int32_t count = (int32_t)us * 4096 * 50 / 1000000;
    return (uint16_t)count;
}

/* 舵机 id → 板/通道表（以硬件pcb为准）
 *
 * 两块板接线模式相同：D 网接 LED4..LED12 共 9 路（LED0~3 / LED13~15 空着），
 * 所以 通道 = 4 + (id % 9)。注意原理图上的引脚号（10..13、15..19）不是 LED 序号：
 *   pin10=LED4, pin11=LED5, pin12=LED6, pin13=LED7, pin15=LED8 … pin19=LED12
 *
 * 腿序：LF, LM, LR, RF, RM, RR —— 每条腿 “coxa, femur, tibia” ！！！
 * 左板 0x40 = D0~D8 = LF/LM/LR；右板 0x41 = D9~D17 = RF/RM/RR
 */
static const struct { uint8_t addr; uint8_t ch; } g_servo_map[18] = {
    /* LF */ { PCA9685_ADDR_LEFT,   4 }, { PCA9685_ADDR_LEFT,   5 }, { PCA9685_ADDR_LEFT,   6 },
    /* LM */ { PCA9685_ADDR_LEFT,   7 }, { PCA9685_ADDR_LEFT,   8 }, { PCA9685_ADDR_LEFT,   9 },
    /* LR */ { PCA9685_ADDR_LEFT,  10 }, { PCA9685_ADDR_LEFT,  11 }, { PCA9685_ADDR_LEFT,  12 },
    /* RF */ { PCA9685_ADDR_RIGHT,  4 }, { PCA9685_ADDR_RIGHT,  5 }, { PCA9685_ADDR_RIGHT,  6 },
    /* RM */ { PCA9685_ADDR_RIGHT,  7 }, { PCA9685_ADDR_RIGHT,  8 }, { PCA9685_ADDR_RIGHT,  9 },
    /* RR */ { PCA9685_ADDR_RIGHT, 10 }, { PCA9685_ADDR_RIGHT, 11 }, { PCA9685_ADDR_RIGHT, 12 },
};

/* 设置舵机角度
 * servo_id 范围 0~17（见 g_servo_map）
 * deg_x10 范围 -900~900（0.1 度）
 */
/* 最近一次成功写出的角度（0.1°，已钳到 ±900），!A 快照用 */
int16_t g_servo_deg[18] = {0};

esp_err_t servo_set_angle(i2c_port_t port, uint8_t servo_id, int16_t deg_x10)
{
    if (servo_id > 17) {
        return ESP_ERR_INVALID_ARG;
    }
    /* 钳一次并记下来：记录的是真实发出的角度，不是调用者想发的 */
    int16_t d = deg_x10;
    if (d < -900) d = -900;
    if (d > 900) d = 900;

    esp_err_t err = pca9685_set_pulse(port, g_servo_map[servo_id].addr, g_servo_map[servo_id].ch,
                                      0, pulse_us_to_count(servo_pulse_us(d)));
    if (err == ESP_OK) {
        g_servo_deg[servo_id] = d;
    }
    return err;
}