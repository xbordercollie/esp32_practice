#pragma once
#include <stdint.h>
#include <stdbool.h>

/* 三角步态参数 */
#define GAIT_STEPS 12   /* 步态周期步数（默认步态，实际以 gait.c 的表为准） */
#define GAIT_LIFT  4    /* 抬腿步数 */
#define GAIT_L     40   /* 步长（前后摆幅 mm） */
#define GAIT_H     20   /* 步高（抬腿高度 mm） */
#define GAIT_MS   100   /* 每步时长 ms */
#define GAIT_SUB   10   /* 每步切成几个子步（子步时长 = GAIT_MS / GAIT_SUB = 10 ms） */
#define GAIT_RAMP  2    /* 软启动：走满 GAIT_RAMP 个周期后步幅到 100%（2 → 0/20/40，第 3 圈满速） */

/* 运行时可调（命令 !U/!D 改抬腿、!V 改步频），初值来自上面的宏。
 * 定义在 gait.c；cmd 里直接改这几个变量。 */
extern double   g_gait_h;     /* 抬腿高度 mm（10..60） */
extern uint16_t g_gait_ms;    /* 每步时长 ms（60..200） */
extern uint8_t  g_gait_sub;   /* 每步子步数 = g_gait_ms / 10（保证子步 10 ms） */

/**
 * @brief 走 cycles 个周期（每周期 gd->steps 步 × GAIT_MS 毫秒）
 * @param cycles 1~10，范围由调用者拦
 * @note 调用前必须先 !O（开舵机电源、拉低 OE），否则脉宽不会驱动舵机
 */
void gait_walk(uint8_t cycles);

/** @brief 请求中止：走完当前周期就停（周期边界六脚全着地，停住是稳的） */
void gait_abort(void);

/** @brief 是否正在走（给心跳任务查电压用） */
bool gait_is_walking(void);

/** @brief 当前步幅比例 0~1（给 !A 打印） */
double gait_stride_scale(void);

/** @brief 可选的步态数量 / 当前序号 / 名字（给 !G 列表用） */
uint8_t gait_count(void);
uint8_t gait_current(void);
const char *gait_name(uint8_t idx);

/** @brief 选择步态（下标越界则忽略） */
void gait_select(uint8_t idx);
