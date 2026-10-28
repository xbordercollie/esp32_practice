#pragma once
#include <stdint.h>
#include <stdbool.h>

struct LegCal {
    uint8_t id[3];
    int8_t  inv[3];
    int16_t horn[3];
};

/* 站姿目标（腿基座坐标，mm）。pose_stand 和 gait_walk 都用。
 * g_stand_y = 站高（脚在基座下方多少 mm，值越大机身越高），由 !H 在运行时改；
 * STAND_Z 是径向距离，固定不让改。 */
extern double g_stand_y;   /* 默认 150.0，定义在 pose.c */
extern double g_stand_z;   /* 径向距离，默认 160.0，!M 可调 */
#define STAND_Z  160.0   /* g_stand_z 的初值 */


struct FootPos { double x, y, z; };
bool pose_leg(uint8_t leg, const struct FootPos *foot);
void pose_stand(void);