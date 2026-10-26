/* power.h */
#pragma once
#pragma once
#define KILL_PIN          23
#define SERVO_PWR_EN_PIN   5
#define OE_PIN            32
#define LED_PIN            2

void power_init(void);       /* KILL 高、SERVO_PWR_EN 低、OE 高：全部安全默认 */
void power_servo_on(void);   /* SERVO_PWR_EN=1 → 等 100ms → OE=0 */
void power_servo_off(void);  /* OE=1 → SERVO_PWR_EN=0（顺序相反） */
