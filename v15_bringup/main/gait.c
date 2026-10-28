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

/* 走 cycles 个周期。
 *
 * 一个步态步（GAIT_MS 毫秒）再切成 GAIT_SUB 个子步，所以一圈下发的
 * 位置数 = steps × GAIT_SUB（TRIPOD_6 是 12 × 10 = 120 个点，旧版只有 12 个）。
 * 总时长不变：steps × GAIT_MS。
 */
void gait_walk(uint8_t cycles)
{
    const struct GaitDef *gd = &g_gaits[g_gait_idx];
    /* 支撑段步数 = 总步数 - 抬腿步数 */
    uint8_t stance = (uint8_t)(gd->steps - gd->lift);

    for (uint8_t c = 0; c < cycles; c++) {
        for (uint8_t step = 0; step < gd->steps; step++) {
            for (uint8_t sub = 0; sub < GAIT_SUB; sub++) {
                for (uint8_t leg = 0; leg < 6; leg++) {
                    /* 连续相位 = 第 step 步 + 这一步里的第 sub/GAIT_SUB 份 + 该腿偏移，
                     * 再折回 0 ~ steps。phase 是 double，不能用 % 取模，所以用 while 减。 */
                    double phase = (double)step
                                 + (double)sub / (double)GAIT_SUB
                                 + (double)gd->off[leg];
                    while (phase >= (double)gd->steps) {
                        phase -= (double)gd->steps;
                    }

                    struct FootPos p;
                    p.z = STAND_Z;

                    if (phase < (double)gd->lift) {
                        /* 抬腿段：u 从 0 长到 1，x 从 -L/2 走到 +L/2。 */
                        double u = phase / (double)gd->lift;
                        p.x = (double)GAIT_L * (u - 0.5);

                        /* 当前曲线（2026-10-28 学生拍板）：y 走 sin²(πu)。
                         * 两端高度 0、两端竖直速度 0 → 落地无冲击；
                         * 子步加密后峰值也是满高 H=20 mm（12 帧版只有 15 mm 是采样问题）。 */
                        p.y = STAND_Y - (double)GAIT_H * sin(M_PI * u) * sin(M_PI * u);

                        /* ===== 备选：抛物线 4u(1-u)（2026-10-27 版，留作对照）=====
                         * 差异只有 y：
                         *   峰值同样 20 mm，但落地竖直速度 = 200 mm/s（会砸）；
                         *   好处是两端抬得快 —— 离地 5 mm 只要 27 ms，sin² 要 67 ms。
                         *   坡道/杂物上出现粘脚（脚还没抬起来就往前扫）就换回这一行：
                         * p.y = STAND_Y - (double)GAIT_H * 4.0 * u * (1.0 - u);
                         * ============================================================ */
                    } else {
                        /* 支撑段：x 从 +L/2 匀速退到 -L/2（脚相对机身往后走） */
                        double s = phase - (double)gd->lift;
                        p.x = (double)GAIT_L * (0.5 - s / (double)stance);
                        p.y = STAND_Y;
                    }
                    /* 发送给 pose_leg 下发舵机 */
                    pose_leg(leg, &p);
                }
                /* 一个子步 = GAIT_MS / GAIT_SUB 毫秒 */
                vTaskDelay(pdMS_TO_TICKS(GAIT_MS / GAIT_SUB));
            }
        }
    }
    printf("gait: %s x%u done\n", gd->name, cycles);
}

/* ================= 旧版（2026-10-27，12 帧整数相位，留作对照）=================
 * 差异只有两处：
 *   ① 相位：uint8_t ph = (step + off) % steps —— 一圈 12 个点
 *      → 现在 double phase = step + sub/GAIT_SUB + off —— 一圈 steps×GAIT_SUB 个点
 *   ② 抬腿 u：u = (ph + 0.5) / lift —— 4 点采样为了两端对称加的补丁
 *      → 现在 u = phase / lift
 *      （+0.5 必须去掉：子步会走到 phase=3.9，u=1.1 已经出了 u∈[0,1] 的设计区间，
 *        曲线取值不再是抬腿轨迹 —— 抛物线会反过来顶到地里，sin² 会提前回落。）
 * 想回退：把上面那份 gait_walk 注释掉，放开下面这一份。
 *
void gait_walk(uint8_t cycles)
{
    const struct GaitDef *gd = &g_gaits[g_gait_idx];
    uint8_t stance = (uint8_t)(gd->steps - gd->lift);

    for (uint8_t c = 0; c < cycles; c++) {
        for (uint8_t step = 0; step < gd->steps; step++) {
            for (uint8_t leg = 0; leg < 6; leg++) {
                uint8_t ph = (uint8_t)((step + gd->off[leg]) % gd->steps);

                struct FootPos p;
                p.z = STAND_Z;

                if (ph < gd->lift) {
                    // 旧抬腿：u 带 +0.5，采样点落在 0.5/1.5/2.5/3.5
                    double u = ((double)ph + 0.5) / (double)gd->lift;
                    p.x = (double)GAIT_L * ((double)ph / (double)gd->lift - 0.5);
                    p.y = STAND_Y - (double)GAIT_H * 4.0 * u * (1.0 - u);
                } else {
                    double s = (double)(ph - gd->lift);
                    p.x = (double)GAIT_L * (0.5 - s / (double)stance);
                    p.y = STAND_Y;
                }
                pose_leg(leg, &p);
            }
            vTaskDelay(pdMS_TO_TICKS(GAIT_MS));
        }
    }
    printf("gait: %s x%u done\n", gd->name, cycles);
}
 * ========================================================================== */
