#pragma once
#include <stdint.h>
#include <stdbool.h>

/* 步态默认参数 */
#define GAIT_STEPS 12   /* 步态周期步数（默认步态，实际以 gait.c 的表为准） */
#define GAIT_LIFT  4    /* 抬腿步数 */
#define GAIT_L     40   /* 步长（前后摆幅 mm） */
#define GAIT_H     20   /* 步高（抬腿高度 mm） */
#define GAIT_MS   100   /* 每步时长 ms */
#define GAIT_SUB   10   /* 每步切成几个子步（子步固定 10 ms） */
#define GAIT_RAMP   2   /* 软启动：active 每个周期走 1/GAIT_RAMP 到目标 */

/* 连续行走方向矢量（mm/周期）与渐变速率 */
#define GAIT_FWD_MM     40
#define GAIT_LAT_MM     40
#define GAIT_YAW_UNITS  40
#define GAIT_SLEW_MM    (GAIT_FWD_MM / GAIT_RAMP)     /* 20 mm/周期 */
#define GAIT_SLEW_YAW   (GAIT_YAW_UNITS / GAIT_RAMP)  /* 20/周期 */

/* 方向符号：现场核对后只需改这三行 */
#define GAIT_SIGN_FWD   1
#define GAIT_SIGN_LAT   1
#define GAIT_SIGN_YAW   1

typedef enum {
    GAIT_IDLE = 0,
    GAIT_WALKING,
    GAIT_SETTLING,
    GAIT_FAULT
} gait_state_t;

struct GaitTravel {
    int16_t fwd, lat, yaw;
};

/* 运行时可调（!U/!D 抬腿、!V 步频），初值来自上面的宏。
 * 定义在 gait.c；cmd 里通过 gait_set_* 修改。 */
extern volatile int16_t  g_gait_h;     /* 抬腿高度 mm（10..60） */
extern volatile uint16_t g_gait_ms;    /* 每步时长 ms（60..200） */
extern volatile uint8_t  g_gait_sub;   /* 每步子步数 = g_gait_ms / 10（子步 10 ms） */

/** @brief 步态任务（10ms 周期；行走时唯一写舵机的任务） */
void gait_task(void *arg);

/** @brief 启动/更新连续行走；回站中或故障时返回 false */
bool gait_move(int16_t fwd, int16_t lat, int16_t yaw);

/** @brief 停止行走：走完当前周期 → 400ms 插值回站姿 */
void gait_request_stop(void);

/** @brief 立即停止（低压用）：不等当前周期，直接插值回站姿 */
void gait_request_stop_immediate(void);

/** @brief 非阻塞定周期测试（向前），走完自动停 + 回站姿；未启动返回 false */
bool gait_test_walk(uint8_t cycles);

/** @brief 急停：关闭输出并让任务尽快 quiesce（安全层随后回中/断电） */
void gait_abort_immediate(void);

/** @brief 清除故障/急停状态，允许重新输出（!O 成功后调用） */
void gait_clear_fault(void);

bool gait_is_busy(void);        /* WALKING / SETTLING / FAULT */
bool gait_is_walking(void);
bool gait_is_quiescent(void);
gait_state_t gait_state(void);

void gait_get_travel(struct GaitTravel *target, struct GaitTravel *active);
double gait_stride_scale(void);

void gait_set_lift_mm(int16_t mm);
void gait_set_step_ms(uint16_t ms);

uint8_t gait_count(void);
uint8_t gait_current(void);
const char *gait_name(uint8_t idx);
bool gait_select(uint8_t idx);
