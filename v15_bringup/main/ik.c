/**
 * @file ik.c
 * @brief 单腿逆运动学（浮点）：足端坐标 → 三个几何角
 *
 * 坐标系（腿基座，原点 = coxa 舵机轴）：
 *   +X 指向机头，+Z 指向机身右侧，+Y 向下；单位 mm
 *
 * 输出单位是「度」。转成舵机指令是另一层的事（减安装角 / invert / horn），
 * 本文件只管几何。
 */
#include "ik.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool ik_leg(double x, double y, double z, const struct LegGeom *g, struct IkAngles *out)
{
    if (!g || !out) {
        return false;
    }

    /* 1. coxa 方位角：足端在 XZ 平面上的方向 */
    double coxa_angle = atan2(z, x) * 180.0 / M_PI;

    /* 2. 扣掉 coxa 长度，得到 femur/tibia 在同一竖直平面内要够到的水平距离 */
    double coxa_feet_dist = sqrt(x * x + z * z);
    if (coxa_feet_dist < g->coxa) {
        return false;   /* 足端落进 coxa 圆内，无解 */
    }
    double ik_feet_dist    = coxa_feet_dist - g->coxa;
    double ik_feet_dist_sq = ik_feet_dist * ik_feet_dist;
    double ik_a_sq         = ik_feet_dist_sq + y * y;   /* femur 起点到足端的直线距离² */
    double ik_a            = sqrt(ik_a_sq);

    /* 3. 三角形不成立检查：太远够不到 / 太近折不起来 */
    if (ik_a > g->femur + g->tibia || ik_a < fabs(g->femur - g->tibia)) {
        return false;
    }

    /* 4. tibia：余弦定理，取内角 */
    double cos_tibia_num = g->femur * g->femur + g->tibia * g->tibia - ik_a_sq;
    double cos_tibia_den = 2.0 * g->femur * g->tibia;
    double tibia_angle   = acos(cos_tibia_num / cos_tibia_den) * 180.0 / M_PI;

    /* 5. femur：直线 a 的俯角 a1，减去它与 femur 的夹角 a2 */
    double angle_a1    = atan2(y, ik_feet_dist) * 180.0 / M_PI;
    double cos_a2_num  = g->femur * g->femur + ik_a_sq - g->tibia * g->tibia;
    double cos_a2_den  = 2.0 * g->femur * ik_a;
    double angle_a2    = acos(cos_a2_num / cos_a2_den) * 180.0 / M_PI;
    double femur_angle = angle_a2 - angle_a1;

    out->coxa_deg  = coxa_angle;
    out->femur_deg = femur_angle;
    out->tibia_deg = tibia_angle;
    return true;
}
