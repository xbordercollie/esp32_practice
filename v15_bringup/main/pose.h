#pragma once
#include <stdint.h>
#include <stdbool.h>

struct LegCal {
    uint8_t id[3];
    int8_t  inv[3];
    int16_t horn[3];
};


struct FootPos { double x, y, z; };
bool pose_leg(uint8_t leg, const struct FootPos *foot);
void pose_stand(void);