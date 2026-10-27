#pragma once
#include <stdint.h>
#include <stdbool.h>

struct LegCal {
    uint8_t id[3];      /* coxa / femur / tibia 对应的 servo_id */
    int8_t  inv[3];     /* 1 = 该关节反向（镜像安装） */
    int16_t horn[3];    /* 舵盘偏移 0.1°，先全 0，D5 现场改 */
};


struct FootPos { double x, y, z; };
bool pose_leg(uint8_t leg, const struct FootPos *foot);
void pose_stand(void);