#pragma once
#include <stdint.h>
#include <stdbool.h>

struct LegCal {
    uint8_t id[3];
    int8_t  inv[3];
    int16_t horn[3];
};

/* 站姿目标（腿基座坐标，mm）。pose_stand 和 gait_walk 都用这两个数，
 * 所以放在头文件里；改站高只改这里。 */
#define STAND_Y  150.0   /* 站姿高度（脚在基座下方多少 mm） */
#define STAND_Z  160.0   /* 站姿径向距离 */


struct FootPos { double x, y, z; };
bool pose_leg(uint8_t leg, const struct FootPos *foot);
void pose_stand(void);