#include <stdio.h>
#include "pose.h"
#include "ik.h"
#include "pca9685.h"

static const struct LegCal g_cal[6] = {
    /* LF */ { { 0,  1,  2}, {0,0,0}, {0,0,0} },
    /* LM */ { { 3,  4,  5}, {0,0,0}, {0,0,0} },
    /* LR */ { { 6,  7,  8}, {0,0,0}, {0,0,0} },
    /* RR */ { {11, 10,  9}, {0,1,1}, {0,0,0} },   /* 注意 id 降序 */
    /* RM */ { {14, 13, 12}, {0,1,1}, {0,0,0} },
    /* RF */ { {15, 16, 17}, {0,1,1}, {0,0,0} },
};

bool pose_leg(uint8_t leg, const struct FootPos *foot)
{
    if(foot == NULL || leg > 5) {
        return false;
    }

    struct LegGeom  g = { LEG_COXA, LEG_FEMUR, LEG_TIBIA};
    struct IkAngles a = {0};
    if (!ik_leg(foot->x, foot->y, foot->z, &g, &a)) {
        printf("pose_leg: leg %d unreachable (%.0f %.0f %.0f)\n", leg, foot->x, foot->y, foot->z);
        return false;
    }

    int16_t v[3];
    v[0] = (int16_t)((a.coxa_deg  - 90.0) * 10.0);
    v[1] = (int16_t)( a.femur_deg         * 10.0);
    v[2] = (int16_t)((90.0 - a.tibia_deg) * 10.0);

    uint8_t fail = 0;
    for (int j = 0; j < 3; j++) {
        if (g_cal[leg].inv[j]) {
            v[j] = (int16_t)(-v[j]);
        }
        v[j] = (int16_t)(v[j] + g_cal[leg].horn[j]);
        if (servo_set_angle(I2C_NUM_0, g_cal[leg].id[j], v[j]) != ESP_OK) {
            fail++;
        }
    }

    printf("leg %u: coxa=%d femur=%d tibia=%d (%d fail)\n",
           leg, v[0], v[1], v[2], fail);
    return fail == 0;
}

/* 常量{ 0,160,180 } */
void pose_stand(void)
{
    struct FootPos p = { 0, 160, 180 };
    uint8_t fail = 0;

    for (uint8_t leg = 0; leg < 6; leg++) {
        if (!pose_leg(leg, &p)) {
            fail++;
        }
    }
    printf("pose_stand: %d/6 legs failed\n", fail);
}