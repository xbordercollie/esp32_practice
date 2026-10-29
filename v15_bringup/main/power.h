/* power.h */
#pragma once
#include <stdbool.h>

/* 电源/安全引脚（v1.5 硬件接口文档 §2.1） */
#define KILL_PIN          23
#define SERVO_PWR_EN_PIN   5
#define OE_PIN            32
#define LED_PIN            2

#define PGOOD_PIN         39   /* 输入，高 = 5V 母线正常 */
#define ALERT_PIN         26   /* 输入，低 = 舵机包过流 */
#define PWR_INT_PIN       17   /* 输入，低 = LTC2954 按键请求 */
#define TLV_PIN           -1   /* 输入，低 = 硬件欠压；GPIO 待确认，先禁用 */

void power_init(void);       /* KILL 高、SERVO_PWR_EN 低、OE 高：全部安全默认 */
bool power_servo_on(void);   /* OE=1 → EN=1 → 100ms → OE=0；被急停打断返回 false */
void power_servo_off(void);  /* OE=1 → SERVO_PWR_EN=0（顺序不能反），置 abort */
void power_rail_abort_clear(void); /* 开轨前清 abort（safety_rail_on 调用） */
void power_kill_release(void);  /* KILL=0：交还 LTC2954 断电 */

bool power_pgood_ok(void);   /* PGOOD_5V 高 = 正常 */
bool power_alert_ok(void);   /* ALERT_S 高 = 无过流 */
bool power_tlv_ok(void);     /* TLV 高 = 无欠压（TLV_PIN<0 时恒 true） */
bool power_pwr_int_low(void);/* 按键中断低有效 */
bool power_rail_is_on(void); /* 软件记录的舵机轨状态 */
