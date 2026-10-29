/**
 * @file gait.c
 * @brief 步态引擎（任务式）：相位表 → 足端 x/z/y → pose_leg 下发
 *
 * 一张表 = 一种步态：周期步数、抬腿步数、六条腿的相位偏移。
 * 任意时刻“几条腿在抬”由偏移决定：
 *   TRIPOD  两组三角交替 → 最多 3 条
 *   RIPPLE  相邻偏移 2    → 最多 2 条
 *   WAVE    相邻偏移 4    → 最多 1 条
 *
 * 足端坐标（腿局部，见 pose.c）：x = 切向前后、y = 向下高度、z = 径向。
 * 连续行走用方向矢量 (fwd, lat, yaw)，每周期按 GAIT_SLEW_* 渐变到目标；
 * 停止时插值回站姿，避免半空中的脚直接砸下来。
 */
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gait.h"
#include "pose.h"
#include "ik.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

#define GAIT_COUNT  ((uint8_t)(sizeof(g_gaits) / sizeof(g_gaits[0])))

/* 每腿在机身坐标系中的安装位置（mm）与左右符号。
 * x_base 前后（+ 机头）、z_base 左右（+ 右）；side:-1 左腿 / +1 右腿。
 * 只用于转向的切向分量；数值取旧版机身参数，可现场微调。 */
struct LegBase {
    int16_t x_base;
    int16_t z_base;
    int8_t  side;
};

static const struct LegBase g_leg_base[6] = {
    /* LF */ {  104, -63, -1 },
    /* LM */ {    0, -79, -1 },
    /* LR */ { -104, -63, -1 },
    /* RF */ {  104,  63, +1 },
    /* RM */ {    0,  79, +1 },
    /* RR */ { -104,  63, +1 },
};

static uint8_t g_gait_idx = 0;

/* 状态与跨任务标志 */
static volatile gait_state_t g_state = GAIT_IDLE;
static volatile bool g_continuous = false;
static volatile uint16_t g_cycles_left = 0;
static volatile bool g_stop_req = false;
static volatile bool g_immediate_stop = false;
static volatile bool g_abort_req = false;
static volatile bool g_output_allowed = true;
static volatile bool g_quiescent = true;

/* 方向矢量：target 由命令任务写，active 由步态任务在周期边界渐变 */
static volatile int16_t g_target_fwd = 0;
static volatile int16_t g_target_lat = 0;
static volatile int16_t g_target_yaw = 0;
static int16_t g_active_fwd = 0;
static int16_t g_active_lat = 0;
static int16_t g_active_yaw = 0;

/* 运行时可调参数 */
volatile int16_t  g_gait_h   = GAIT_H;
volatile uint16_t g_gait_ms  = GAIT_MS;
volatile uint8_t  g_gait_sub = GAIT_SUB;

/* 步态相位 */
static uint8_t g_step = 0;
static uint8_t g_sub = 0;
static uint8_t g_sub_count = GAIT_SUB;
static uint8_t g_settle_tick = 0;
static struct FootPos g_foot_last[6];
static int g_pose_fail_frames = 0;
static int16_t g_lift_active = GAIT_H;   /* 周期边界快照：!U/!D 下一周期生效 */

/* 回站姿插值时长：40 × 10ms = 400ms */
#define SETTLE_TICKS 40

static int16_t slew16(int16_t cur, int16_t target, int16_t step)
{
    int32_t d = (int32_t)target - (int32_t)cur;
    if (d > step)  d = step;
    if (d < -step) d = -step;
    return (int16_t)((int32_t)cur + d);
}

static void foot_last_to_stand(void)
{
    for (uint8_t leg = 0; leg < 6; leg++) {
        g_foot_last[leg].x = 0;
        g_foot_last[leg].y = g_stand_y;
        g_foot_last[leg].z = g_stand_z;
    }
}

uint8_t gait_count(void)   { return GAIT_COUNT; }
uint8_t gait_current(void) { return g_gait_idx; }
const char *gait_name(uint8_t i) { return (i < GAIT_COUNT) ? g_gaits[i].name : "?"; }
gait_state_t gait_state(void) { return g_state; }
bool gait_is_quiescent(void) { return g_quiescent; }
bool gait_is_walking(void) { return g_state == GAIT_WALKING; }
bool gait_is_busy(void)
{
    return g_state == GAIT_WALKING || g_state == GAIT_SETTLING || g_state == GAIT_FAULT;
}

void gait_get_travel(struct GaitTravel *target, struct GaitTravel *active)
{
    if (target != NULL) {
        target->fwd = g_target_fwd;
        target->lat = g_target_lat;
        target->yaw = g_target_yaw;
    }
    if (active != NULL) {
        active->fwd = g_active_fwd;
        active->lat = g_active_lat;
        active->yaw = g_active_yaw;
    }
}

double gait_stride_scale(void)
{
    int16_t t[3] = { g_target_fwd, g_target_lat, g_target_yaw };
    int16_t a[3] = { g_active_fwd, g_active_lat, g_active_yaw };
    double s = 0.0;
    for (int i = 0; i < 3; i++) {
        if (t[i] != 0) {
            double r = fabs((double)a[i] / (double)t[i]);
            if (r > s) s = r;
        }
    }
    return (s > 1.0) ? 1.0 : s;
}

/* 启动/换向前检查可达性：抬腿峰值（x=0,z=stand_z,y=stand_y-lift）
 * 和摆/支撑端点（±半个方向矢量，y=stand_y）都必须有 IK 解。
 * 6 腿 × 5 点，只在命令入口算一次，避免走起来才发现不可达。 */
static bool gait_feasible(int16_t fwd, int16_t lat, int16_t yaw)
{
    struct LegGeom  geom = { LEG_COXA, LEG_FEMUR, LEG_TIBIA };
    struct IkAngles a;

    for (uint8_t leg = 0; leg < 6; leg++) {
        double dx = (double)(GAIT_SIGN_FWD * fwd)
                  - (double)(GAIT_SIGN_YAW * yaw)
                    * (double)g_leg_base[leg].z_base / 100.0;
        double dz = (double)g_leg_base[leg].side *
                    ((double)(GAIT_SIGN_LAT * lat)
                     + (double)(GAIT_SIGN_YAW * yaw)
                       * (double)g_leg_base[leg].x_base / 100.0);

        if (!ik_leg(0, g_stand_y - (double)g_gait_h, g_stand_z, &geom, &a)) {
            return false;
        }
        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                double px = dx * 0.5 * (double)sx;
                double pz = g_stand_z + dz * 0.5 * (double)sz;
                if (!ik_leg(px, g_stand_y, pz, &geom, &a)) {
                    return false;
                }
            }
        }
    }
    return true;
}

void gait_set_lift_mm(int16_t mm)
{
    if (mm < 10) mm = 10;
    if (mm > 60) mm = 60;
    g_gait_h = mm;
}

void gait_set_step_ms(uint16_t ms)
{
    if (ms < 60)  ms = 60;
    if (ms > 200) ms = 200;
    g_gait_ms = ms;
    uint8_t sub = (uint8_t)(ms / 10);
    g_gait_sub = (sub < 1) ? 1 : sub;
}

/* 选择步态（下标越界或行走中则忽略） */
bool gait_select(uint8_t idx)
{
    if (g_state != GAIT_IDLE || idx >= GAIT_COUNT) {
        return false;
    }
    if (idx != g_gait_idx) {
        g_gait_idx = idx;
        printf("gait: select %s\n", g_gaits[idx].name);
    }
    return true;
}

static void start_common(bool continuous, uint16_t cycles)
{
    g_active_fwd = g_active_lat = g_active_yaw = 0;
    g_continuous = continuous;
    g_cycles_left = cycles;
    g_stop_req = false;
    g_immediate_stop = false;
    g_abort_req = false;
    g_step = 0;
    g_sub = 0;
    g_sub_count = g_gait_sub;
    g_lift_active = g_gait_h;
    g_pose_fail_frames = 0;
    foot_last_to_stand();
    pose_clear_faults();
    g_state = GAIT_WALKING;
}

/* 启动/更新连续行走。行走中只改目标，方向在周期边界按 GAIT_SLEW 渐变。 */
bool gait_move(int16_t fwd, int16_t lat, int16_t yaw)
{
    if (!g_output_allowed) {
        return false;   /* 急停后必须先 !O 清故障 */
    }
    if (g_state == GAIT_SETTLING || g_state == GAIT_FAULT) {
        return false;
    }
    if (!gait_feasible(fwd, lat, yaw)) {
        printf("gait: pose infeasible at stand y=%.0f z=%.0f lift=%d "
               "(change !Z/!U or direction)\n",
               g_stand_y, g_stand_z, (int)g_gait_h);
        return false;
    }
    if (g_state == GAIT_IDLE) {
        start_common(true, 0);
        printf("gait: start %s fwd=%d lat=%d yaw=%d\n",
               g_gaits[g_gait_idx].name, fwd, lat, yaw);
    } else {
        /* 行走中直接覆盖目标；如果前面请求过停，这里表示继续走 */
        g_stop_req = false;
        g_immediate_stop = false;
        printf("gait: target fwd=%d lat=%d yaw=%d\n", fwd, lat, yaw);
    }
    g_target_fwd = fwd;
    g_target_lat = lat;
    g_target_yaw = yaw;
    return true;
}

bool gait_test_walk(uint8_t cycles)
{
    if (!g_output_allowed || g_state != GAIT_IDLE || cycles == 0) {
        return false;
    }
    if (!gait_feasible((int16_t)(GAIT_SIGN_FWD * GAIT_FWD_MM), 0, 0)) {
        printf("gait: pose infeasible for test walk (change !Z/!U)\n");
        return false;
    }
    g_target_fwd = (int16_t)(GAIT_SIGN_FWD * GAIT_FWD_MM);
    g_target_lat = 0;
    g_target_yaw = 0;
    start_common(false, cycles);
    printf("gait: test %s x%u\n", g_gaits[g_gait_idx].name, cycles);
    return true;
}

void gait_request_stop(void)
{
    if (g_state == GAIT_WALKING) {
        g_stop_req = true;
        printf("gait: stop at cycle end\n");
    } else if (g_state == GAIT_SETTLING) {
        printf("gait: already settling\n");
    } else {
        printf("gait: idle\n");
    }
}

void gait_request_stop_immediate(void)
{
    if (g_state == GAIT_WALKING) {
        g_stop_req = true;
        g_immediate_stop = true;
        printf("gait: immediate stop\n");
    }
}

void gait_abort_immediate(void)
{
    g_output_allowed = false;
    g_abort_req = true;
}

void gait_clear_fault(void)
{
    if (g_state == GAIT_FAULT || g_state == GAIT_IDLE) {
        g_state = GAIT_IDLE;
    }
    g_output_allowed = true;
    g_abort_req = false;
    g_stop_req = false;
    g_immediate_stop = false;
    g_continuous = false;
    g_cycles_left = 0;
    g_pose_fail_frames = 0;
    pose_clear_faults();
    g_quiescent = true;
}

static void start_settle(void)
{
    g_state = GAIT_SETTLING;
    g_settle_tick = 0;
    g_stop_req = false;
    g_immediate_stop = false;
    g_pose_fail_frames = 0;   /* 回站姿重新计数，不继承行走帧的 I2C 失败 */
    printf("gait: settling to stand\n");
}

static void settle_tick(void)
{
    g_settle_tick++;
    double t = (double)g_settle_tick / (double)SETTLE_TICKS;
    if (t > 1.0) t = 1.0;

    g_quiescent = false;
    pose_clear_frame();
    int fail = 0;
    for (uint8_t leg = 0; leg < 6; leg++) {
        struct FootPos p;
        p.x = g_foot_last[leg].x * (1.0 - t);
        p.y = g_foot_last[leg].y * (1.0 - t) + g_stand_y * t;
        p.z = g_foot_last[leg].z * (1.0 - t) + g_stand_z * t;
        if (!pose_leg(leg, &p)) {
            fail++;
        }
    }
    g_quiescent = true;

    if (fail) g_pose_fail_frames++;
    else      g_pose_fail_frames = 0;
    if (g_pose_fail_frames >= 3) {
        g_state = GAIT_FAULT;
        printf("gait: FAULT during settle\n");
        return;
    }

    if (g_settle_tick >= SETTLE_TICKS) {
        g_state = GAIT_IDLE;
        if (fail) printf("gait: settle done with %d leg failures\n", fail);
        else      printf("gait: stand\n");
    }
}

/* 走一个 10ms 子步 */
static void walk_tick(void)
{
    const struct GaitDef *gd = &g_gaits[g_gait_idx];
    uint8_t steps = gd->steps;
    uint8_t lift = gd->lift;
    uint8_t stance = (uint8_t)(steps - lift);
    if (stance == 0) stance = 1;
    if (g_sub_count == 0) g_sub_count = 1;

    g_quiescent = false;
    pose_clear_frame();
    int fail = 0;

    for (uint8_t leg = 0; leg < 6; leg++) {
        /* 连续相位 = 第 step 步 + 这一步里的第 sub/sub_count 份 + 该腿偏移，
         * 再折回 0 ~ steps。phase 是 double，不能用 % 取模，所以用 while 减。 */
        double phase = (double)g_step
                     + (double)g_sub / (double)g_sub_count
                     + (double)gd->off[leg];
        while (phase >= (double)steps) {
            phase -= (double)steps;
        }

        /* 方向矢量 → 该腿局部 dx/dz（x 切向、z 径向，左腿镜像） */
        double dx = (double)(GAIT_SIGN_FWD * g_active_fwd)
                  - (double)(GAIT_SIGN_YAW * g_active_yaw)
                    * (double)g_leg_base[leg].z_base / 100.0;
        double dz = (double)g_leg_base[leg].side *
                    ((double)(GAIT_SIGN_LAT * g_active_lat)
                     + (double)(GAIT_SIGN_YAW * g_active_yaw)
                       * (double)g_leg_base[leg].x_base / 100.0);

        struct FootPos p;
        if (phase < (double)lift) {
            /* 抬腿段：u 从 0 长到 1，沿方向矢量从 -1/2 走到 +1/2。 */
            double u = phase / (double)lift;
            double f = u - 0.5;
            p.x = dx * f;
            p.z = g_stand_z + dz * f;
            /* y 走 sin²(πu)：两端高度 0、竖直速度 0 → 落地无冲击。 */
            p.y = g_stand_y - (double)g_lift_active * sin(M_PI * u) * sin(M_PI * u);
        } else {
            /* 支撑段：沿方向矢量从 +1/2 匀速退到 -1/2。 */
            double s = phase - (double)lift;
            double f = 0.5 - s / (double)stance;
            p.x = dx * f;
            p.z = g_stand_z + dz * f;
            p.y = g_stand_y;
        }

        if (pose_leg(leg, &p)) {
            g_foot_last[leg] = p;   /* 只记录真正解出来并写出的位置 */
        } else {
            fail++;
        }
    }
    g_quiescent = true;

    /* IK 不可达属于参数/姿态问题：受控回站姿，不切舵机轨。
     * 连续 3 帧 I2C 写失败才进 FAULT，由安全层急停断轨。 */
    if (g_pose_frame_ik_fail != 0) {
        printf("gait: IK unreachable (%u joints), stop and settle\n",
               (unsigned)g_pose_frame_ik_fail);
        start_settle();
        return;
    }
    /* 限位饱和说明这一帧没走到目标姿态：同样受控停下，不继续硬走 */
    if (pose_limit_mask() != 0) {
        printf("gait: joint limit saturated (0x%08lx), stop and settle\n",
               (unsigned long)pose_limit_mask());
        start_settle();
        return;
    }
    if (g_pose_frame_io_fail != 0) {
        g_pose_fail_frames++;
    } else {
        g_pose_fail_frames = 0;
    }
    if (g_pose_fail_frames >= 3) {
        g_state = GAIT_FAULT;
        printf("gait: FAULT (I2C write), last fail=%d\n", fail);
        return;
    }

    g_sub++;
    if (g_sub >= g_sub_count) {
        g_sub = 0;
        g_step++;
        if (g_step >= steps) {
            g_step = 0;
            g_sub_count = g_gait_sub;     /* 周期边界更新子步数 */
            g_lift_active = g_gait_h;     /* 抬腿高度下一周期生效 */

            /* 方向渐变：每周期最多 GAIT_SLEW_* */
            g_active_fwd = slew16(g_active_fwd, g_target_fwd, GAIT_SLEW_MM);
            g_active_lat = slew16(g_active_lat, g_target_lat, GAIT_SLEW_MM);
            g_active_yaw = slew16(g_active_yaw, g_target_yaw, GAIT_SLEW_YAW);

            /* 定周期测试：走满 cycles 圈后自动停 + 回站姿 */
            if (!g_continuous && g_cycles_left > 0) {
                g_cycles_left--;
                if (g_cycles_left == 0) {
                    start_settle();
                    return;
                }
            }

            /* 正常停止：走完当前周期再回站姿 */
            if (g_stop_req) {
                start_settle();
                return;
            }
        }
    }
}

void gait_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    g_state = GAIT_IDLE;
    g_quiescent = true;

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));

        /* 急停/低压硬断：立即 quiesce，不写下一帧 */
        if (g_abort_req || !g_output_allowed) {
            g_abort_req = false;
            g_stop_req = false;
            g_immediate_stop = false;
            g_continuous = false;
            g_cycles_left = 0;
            g_state = GAIT_IDLE;
            g_quiescent = true;
            continue;
        }

        switch (g_state) {
        case GAIT_WALKING:
            if (g_stop_req && g_immediate_stop) {
                start_settle();
            } else {
                walk_tick();
            }
            break;
        case GAIT_SETTLING:
            settle_tick();
            break;
        case GAIT_FAULT:
            g_quiescent = true;
            break;
        case GAIT_IDLE:
        default:
            g_quiescent = true;
            break;
        }
    }
}
