#include <stdio.h>
#include "pose.h"
#include "ik.h"
#include "pca9685.h"

/* 舵机校准参数
 * 1. id[3] = { coxa, femur, tibia } 对应的 servo_id
 * 2. inv[3] = { coxa, femur, tibia } 1 = 该关节反向（镜像安装）
 * 3. horn[3] = { coxa, femur, tibia } 舵盘偏移 0.1°，先全 0，D5 现场改
 */
/* 六条腿的标定表。顺序 = 舵机 id 的顺序（实测接线）：
 * LF, LM, LR 在左侧板（0x40）D0~D8；RF, RM, RR 在右侧板（0x41）D9~D17。
 * 每条腿三个 id 依次是 coxa / femur / tibia，所以 id = leg*3 + j 是连续的。
 * inv：右腿（RF/RM/RR）镜像安装 → femur/tibia 反向；左腿不反。D5 现场用 !T 验。*/
static const struct LegCal g_cal[6] = {
    /* LF */ { { 0,  1,  2}, {0,0,0}, {0,0,0} },
    /* LM */ { { 3,  4,  5}, {0,0,0}, {0,0,0} },
    /* LR */ { { 6,  7,  8}, {0,0,0}, {0,0,0} },
    /* RF */ { { 9, 10, 11}, {0,1,1}, {0,0,0} },
    /* RM */ { {12, 13, 14}, {0,1,1}, {0,0,0} },
    /* RR */ { {15, 16, 17}, {0,1,1}, {0,0,0} },
};

/* 从解算角度推出舵机角度 包含校准过程 */
bool pose_leg(uint8_t leg, const struct FootPos *foot)
{
    if(foot == NULL || leg > 5) {
        return false;
    }

    /* ik解算 判断是否可达 */
    struct LegGeom  g = { LEG_COXA, LEG_FEMUR, LEG_TIBIA };
    struct IkAngles a = {0};
    if (!ik_leg(foot->x, foot->y, foot->z, &g, &a)) {
        printf("pose_leg: leg %d unreachable (%.0f %.0f %.0f)\n", leg, foot->x, foot->y, foot->z);
        return false;
    }

    /* 计算实际舵机角度 （单腿三舵机）*/
    int16_t v[3];
    v[0] = (int16_t)((a.coxa_deg  - 90.0) * 10.0);
    v[1] = (int16_t)( a.femur_deg         * 10.0);
    v[2] = (int16_t)((90.0 - a.tibia_deg) * 10.0);

    uint8_t fail = 0;
    for (int j = 0; j < 3; j++) {
        /* 反向安装的舵机角度取负 */
        if (g_cal[leg].inv[j]) {
            v[j] = (int16_t)(-v[j]);
        }
        /* 加上舵盘偏移，发送给舵机 */
        v[j] = (int16_t)(v[j] + g_cal[leg].horn[j]);
        if (servo_set_angle(I2C_NUM_0, g_cal[leg].id[j], v[j]) != ESP_OK) {
            fail++;
        }
    }

    printf("leg %u: coxa=%d femur=%d tibia=%d (%d fail)\n",
           leg, v[0], v[1], v[2], fail);
    return fail == 0;
}

/* FootPos装的是足端相对基座的位置 */
/* 站立姿势：x=0, y=STAND_Y, z=STAND_Z */
void pose_stand(void)
{
    struct FootPos p = { 0, STAND_Y, STAND_Z };
    uint8_t fail = 0;

    for (uint8_t leg = 0; leg < 6; leg++) {
        if (!pose_leg(leg, &p)) {
            fail++;
        }
    }
    printf("pose_stand: %d/6 legs failed\n", fail);
}