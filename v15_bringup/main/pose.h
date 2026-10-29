#pragma once
#include <stdint.h>
#include <stdbool.h>

struct LegCal {
    uint8_t id[3];
    int8_t  inv[3];
    int16_t horn[3];
};

/* 站姿目标（腿基座坐标，mm）。pose_stand 和 gait 都用。
 * g_stand_y = 站高（脚在基座下方多少 mm，值越大机身越高），由 !H/!Z 在运行时改；
 * g_stand_z = 径向距离，由 !Z 改。 */
extern double g_stand_y;   /* 默认 150.0，定义在 pose.c */
extern double g_stand_z;   /* 径向距离，默认 160.0 */
#define STAND_Z  160.0     /* g_stand_z 的初值 */

struct FootPos { double x, y, z; };
bool pose_leg(uint8_t leg, const struct FootPos *foot);
void pose_stand(void);

/* 最近一帧的逐关节限位饱和位（bit = leg*3 + joint）与故障计数 */
extern volatile uint32_t g_pose_limit_mask;
extern volatile uint16_t g_pose_io_fail;       /* 累计 I2C 写失败 */
extern volatile uint16_t g_pose_frame_ik_fail; /* 本帧 IK 不可达关节数 */
extern volatile uint16_t g_pose_frame_io_fail; /* 本帧 I2C 写失败数 */
void pose_clear_limit_mask(void);
void pose_clear_frame(void);
void pose_clear_faults(void);
uint32_t pose_limit_mask(void);
