#pragma once
#include <stdbool.h>

#define LEG_COXA 60
#define LEG_FEMUR 96
#define LEG_TIBIA 174

struct LegGeom  { double coxa; double femur; double tibia; };   /* mm */
struct IkAngles { double coxa_deg; double femur_deg; double tibia_deg; };

bool ik_leg(double x, double y, double z,
            const struct LegGeom *g, struct IkAngles *out);