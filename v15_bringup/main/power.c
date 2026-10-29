#include <stdio.h>
#include "power.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* KILL 高、SERVO_PWR_EN 低、OE 高：全部安全默认。
 * 先写电平再切方向：gpio_set_level 会写输出锁存，切成输出时不会产生毛刺。 */

static bool g_rail_on = false;
/* 开轨 100ms 等待期间的急停标志：power_servo_off 置位，power_servo_on 收尾时检查 */
static volatile bool g_rail_abort = false;

void power_init(void)
{
    g_rail_abort = false;
    gpio_set_level(KILL_PIN, 1);              /* 512ms 窗口内第一件事 */
    gpio_set_direction(KILL_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(SERVO_PWR_EN_PIN, 0);
    gpio_set_direction(SERVO_PWR_EN_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(OE_PIN, 1);                /* 高 = 高阻，防上电乱动 */
    gpio_set_direction(OE_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(LED_PIN, 0);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    /* 板上已有 10k 上拉，不开 MCU 内部上拉 */
    gpio_set_direction(PGOOD_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(ALERT_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(PWR_INT_PIN, GPIO_MODE_INPUT);
#if TLV_PIN >= 0
    gpio_set_direction(TLV_PIN, GPIO_MODE_INPUT);
#endif

    g_rail_on = false;
}

/* SERVO_PWR_EN=1 → 等 100ms → OE=0。等待期间若被 power_servo_off 打断，
 * 保持 EN=0/OE=1 并返回 false，避免急停后又被这里重新上电。 */
bool power_servo_on(void)
{
    if (g_rail_abort) {                       /* center 期间已经来急停 */
        gpio_set_level(OE_PIN, 1);
        gpio_set_level(SERVO_PWR_EN_PIN, 0);
        g_rail_on = false;
        printf("[PWR] rail-on refused: emergency stop pending\n");
        return false;
    }

    gpio_set_level(OE_PIN, 1);                /* 先确保输出高阻 */
    gpio_set_level(SERVO_PWR_EN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));           /* 等舵机轨 2200µF 充满 */

    if (g_rail_abort) {
        gpio_set_level(OE_PIN, 1);
        gpio_set_level(SERVO_PWR_EN_PIN, 0);
        g_rail_on = false;
        printf("[PWR] rail-on aborted by emergency stop\n");
        return false;
    }

    gpio_set_level(OE_PIN, 0);
    g_rail_on = true;
    return true;
}

/* OE=1 → SERVO_PWR_EN=0（顺序相反） */
void power_servo_off(void)
{
    g_rail_abort = true;
    gpio_set_level(OE_PIN, 1);
    gpio_set_level(SERVO_PWR_EN_PIN, 0);
    g_rail_on = false;
}

void power_rail_abort_clear(void)
{
    g_rail_abort = false;
}

void power_kill_release(void)
{
    gpio_set_level(KILL_PIN, 0);
}

bool power_pgood_ok(void)
{
    return gpio_get_level(PGOOD_PIN) != 0;
}

bool power_alert_ok(void)
{
    return gpio_get_level(ALERT_PIN) != 0;
}

bool power_tlv_ok(void)
{
#if TLV_PIN >= 0
    return gpio_get_level(TLV_PIN) != 0;
#else
    return true;
#endif
}

bool power_pwr_int_low(void)
{
    return gpio_get_level(PWR_INT_PIN) == 0;
}

bool power_rail_is_on(void)
{
    return g_rail_on;
}
