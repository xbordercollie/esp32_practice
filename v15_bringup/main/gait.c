/**
 * @file gait.c
 * @brief 步态引擎（表驱动）：相位表 → 足端 x/y → pose_leg 下发
 *
 * 一张表 = 一种步态：周期步数、抬腿步数、六条腿的相位偏移。
 * 任意时刻“几条腿在抬”由偏移决定：
 *   TRIPOD  两组三角交替 → 最多 3 条
 *   RIPPLE  相邻偏移 2    → 最多 2 条
 *   WAVE    相邻偏移 4    → 最多 1 条
 *
 * 足端坐标（腿局部，见 pose.c）：x = 切向前后、y = 向下高度、z = 径向。
 */
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gait.h"
#include "pose.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define STAND_Y  150.0   /* 站姿高度，与 pose_stand 一致 */
#define STAND_Z  160.0   /* 站姿径向距离 */

/* 一张步态表的定义。off[] 的顺序必须和 pose.c 的 g_cal 一致：
 * LF, LM, LR, RF, RM, RR（v1 是 RR,RM,RF,LR,LM,LF，已重排）。 */
struct GaitDef {
    const char *name;
    uint8_t     steps;
    uint8_t     lift;
    uint8_t     off[6];
};

/* 预定义的步态表 */
static const struct GaitDef g_gaits[] = {
    /* name        步数 抬腿   LF  LM  LR  RF  RM  RR */
    { "TRIPOD_6",   12,  4, {  6,  0,  6,  0,  6,  0 } },   /* 最快，默认 */
    { "RIPPLE_12",  12,  4, { 11,  9,  7,  5,  3,  1 } },   /* 最多 2 条腿抬 */
    { "TRIPOD_8",   16,  4, {  8,  0,  8,  0,  8,  0 } },   /* 同三角，节奏慢 */
    { "WAVE_24",    24,  4, { 20, 16, 12,  0,  8,  4 } },   /* 一次只抬 1 条，最慢 */
    { "TRIPOD_4",   12,  4, {  6,  0,  6,  0,  6,  0 } },   /* 与 TRIPOD_6 同结构 */
};

/* 步态表数量 */
#define GAIT_COUNT  ((uint8_t)(sizeof(g_gaits) / sizeof(g_gaits[0])))

static uint8_t g_gait_idx = 0;

uint8_t gait_count(void)   { return GAIT_COUNT; }
uint8_t gait_current(void) { return g_gait_idx; }
const char *gait_name(uint8_t i) { return (i < GAIT_COUNT) ? g_gaits[i].name : "?"; }

/* 选择步态（下标越界则忽略） */
void gait_select(uint8_t idx)
{
    if (idx < GAIT_COUNT) {
        g_gait_idx = idx;
    }
}

void gait_walk(uint8_t cycles)
{
    const struct GaitDef *gd = &g_gaits[g_gait_idx];
    /* 支撑段步数 = 总步数 - 抬腿步数 */
    uint8_t stance = (uint8_t)(gd->steps - gd->lift);

    for (uint8_t c = 0; c < cycles; c++) {
        for (uint8_t step = 0; step < gd->steps; step++) {
            for (uint8_t leg = 0; leg < 6; leg++) {
                /* 计算当前腿的相位 ph = (step + off) % steps */
                uint8_t ph = (uint8_t)((step + gd->off[leg]) % gd->steps);

                struct FootPos p;
                p.z = STAND_Z;

                if (ph < gd->lift) {
                    /* 抬腿段：x 从 -L/2 走到 +L/4（一次函数）；y 用 4u(1-u) 钟形。
                     * u 取 (ph+0.5)/lift：四个采样点对称落在曲线两端，
                     * 峰值 0.9375*H（H=20 时 18.75mm），落地前一帧离地 8.75mm。
                     * 选这版而不是 sin²：sin² 峰值只有 0.75*H（15mm），坡道/杂物上会粘脚；
                     * 落地的 8.75mm 落差靠舵机减速 + 机械柔性吸收。 */
                    
                    /* 原始h(t) = H * t(4 − t) / n*/
                    /* h(t) = H · (t + 0.5)(n − 0.5 − t) / n  其中n是抬腿帧数（可映射）*/
                    double u = ((double)ph + 0.5) / (double)gd->lift;
                    p.x = (double)GAIT_L * ((double)ph / (double)gd->lift - 0.5);
                    p.y = STAND_Y - (double)GAIT_H * 4.0 * u * (1.0 - u);

                    /* ================= 备选版（2026-10-27 晚，留作对照）=================
                     * 差异只有 y：sin² 两端高度为 0、端点竖直速度也为 0，
                     * 落地零冲击，但峰值只有 0.75*H（H=20 时 15mm）——坡道容易粘脚。
                     * 想换回来就注释掉上面两行、放开下面两行。
                     *
                     * double t = (double)ph / (double)(gd->lift - 1);
                     * p.y = STAND_Y - (double)GAIT_H * sin(M_PI * t) * sin(M_PI * t);
                     * ============================================================ */
                } else {
                    /* 支撑段：x 从 +L/2 匀速退到 -L/2（脚相对机身往后走） */
                    double s = (double)(ph - gd->lift);
                    p.x = (double)GAIT_L * (0.5 - s / (double)stance);
                    p.y = STAND_Y;
                }
                /* 发送给 pose_leg 下发舵机 */
                pose_leg(leg, &p);
            }
            /* 等待下一步 */
            vTaskDelay(pdMS_TO_TICKS(GAIT_MS));
        }
    }
    printf("gait: %s x%u done\n", gd->name, cycles);
}
