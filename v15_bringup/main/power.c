#include "power.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* KILL 高、SERVO_PWR_EN 低、OE 高：全部安全默认 */

void power_init(void)
{
    gpio_set_level(KILL_PIN, 1);
    gpio_set_direction(KILL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SERVO_PWR_EN_PIN, 0);
    gpio_set_direction(SERVO_PWR_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(OE_PIN, 1);
    gpio_set_direction(OE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

/* SERVO_PWR_EN=1 → 等 100ms → OE=0 */
void power_servo_on(void)
{ 
    gpio_set_level(SERVO_PWR_EN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));   /* 等舵机轨 2200µF 充满 */
    gpio_set_level(OE_PIN, 0);
}

/* OE=1 → SERVO_PWR_EN=0（顺序相反） */
void power_servo_off(void)
{
    gpio_set_level(OE_PIN, 1);
    gpio_set_level(SERVO_PWR_EN_PIN, 0);
}
