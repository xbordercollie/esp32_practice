/**
 * @file cmd.c
 * @brief 串口命令：表驱动分派（命令名 → 处理函数），参数由各 handler 自己解析
 *
 * 命令（UART0 控制台，115200）：
 *   !M          18 路全部回中位（1500us）
 *   !O          舵机电源开（SERVO_PWR_EN=1 → 100ms → OE=0）
 *   !S          舵机电源关（OE=1 → SERVO_PWR_EN=0）
 *   !T <id> <d> 单路测试：第 id 路转到 d（0.1 度，!T 0 300 = +30 度）
 *   !H <120-190> 站高（mm，越大机身越高），改完立刻重站
 *   !B          读两颗 INA226 的电压/电流
 */
#include <stdio.h>
#include <string.h>
#include "power.h"
#include "pca9685.h"
#include "ik.h"
#include "pose.h"
#include "gait.h"
#include "ina226.h"

/* ==================== 各命令处理函数（统一签名） ==================== */

static void cmd_m(const char *arg)
{
    (void)arg;
    int fail = 0;
    for (uint8_t i = 0; i <= 17; i++) {
        if (servo_set_angle(I2C_NUM_0, i, 0) != ESP_OK) {
            fail++;
        }
    }
    printf("mid 0..17: %d failed\n", fail);
}

static void cmd_o(const char *arg)
{
    (void)arg;
    power_servo_on();
    printf("servo rail ON\n");
}

static void cmd_s(const char *arg)
{
    (void)arg;
    power_servo_off();
    printf("servo rail OFF\n");
}

static void cmd_t(const char *arg)
{
    int id = -1, deg = 0;
    if (sscanf(arg, "%d %d", &id, &deg) != 2 || id < 0 || id > 17) {
        printf("usage: !T <0-17> <deg_x10>\n");
        return;
    }
    esp_err_t err = servo_set_angle(I2C_NUM_0, (uint8_t)id, (int16_t)deg);
    printf("!T %d %d -> %s\n", id, deg, esp_err_to_name(err));
}

static void cmd_i(const char *arg)
{
    double x = 0, y = 0, z = 0;
    if (sscanf(arg, "%lf %lf %lf", &x, &y, &z) != 3) {
        printf("usage: !I <x> <y> <z>\n");
        return;
    }
    struct LegGeom  g = { LEG_COXA, LEG_FEMUR, LEG_TIBIA };
    struct IkAngles a = {0};
    if (!ik_leg(x, y, z, &g, &a)) {
        printf("ik %.0f %.0f %.0f -> unreachable\n", x, y, z);
        return;
    }
    printf("ik %.0f %.0f %.0f -> coxa=%.2f femur=%.2f tibia=%.2f\n",
           x, y, z, a.coxa_deg, a.femur_deg, a.tibia_deg);
}

static void cmd_z(const char *arg)
{
    double y = 0, z = 0;
    int n = sscanf(arg, "%lf %lf", &y, &z);

    if (n == 2) {
        if (y < 120 || y > 190 || z < 140 || z > 180) {
            printf("!Z usage: !Z [y 120-190] [z 140-180]\n");
            return;
        }
        g_stand_y = y;
        g_stand_z = z;
        printf("stand y=%.0f z=%.0f\n", y, z);
    } else if (n == 1) {
        printf("!Z usage: !Z [y 120-190] [z 140-180]\n");
        return;
    }
    pose_stand();
}

static void cmd_w(const char *arg)
{
    int n = 1;
    if (sscanf(arg, "%d", &n) != 1) {
        n = 1;
    }
    if (n < 1 || n > 10) {
        printf("usage: !W [1-10]\n");
        return;
    }
    gait_walk((uint8_t)n);
}

static void cmd_g(const char *arg)
{
    int n = -1;
    if (sscanf(arg, "%d", &n) == 1) {
        if (n < 0 || n >= (int)gait_count()) {
            printf("bad gait index (0..%u)\n", (unsigned)(gait_count() - 1));
            return;
        }
        gait_select((uint8_t)n);
    }
    for (uint8_t i = 0; i < gait_count(); i++) {
        printf("  %u: %-10s%s\n", i, gait_name(i), (i == gait_current()) ? "  <= current" : "");
    }
}

static void cmd_b(const char *arg)
{
    (void)arg;
    int32_t mv = 0, ma = 0;

    if (ina226_read(I2C_NUM_0, INA226_ADDR_SERVO, &mv, &ma) == ESP_OK) {
        printf("servo pack: %ld mV  %ld mA\n", (long)mv, (long)ma);
    } else {
        printf("servo pack: read fail\n");
    }
    if (ina226_read(I2C_NUM_1, INA226_ADDR_LOGIC, &mv, &ma) == ESP_OK) {
        /* 逻辑包放电时读数为负，这里取绝对值显示，另标放电方向 */
        printf("logic pack: %ld mV  %ld mA (discharge reads negative)\n",
               (long)mv, (long)ma);
        if (mv < 6900) {
            printf("WARN: logic pack below 6.90 V soft-stop threshold\n");
        }
    } else {
        printf("logic pack: read fail\n");
    }
}

static void cmd_h(const char *arg)
{
    int y = 0;
    if (sscanf(arg, "%d", &y) != 1) {
        printf("usage: !H <120-190>   (stand height mm, larger = taller)\n");
        return;
    }
    if (y < 120 || y > 190) {
        printf("!H range 120..190 mm, current %d\n", (int)g_stand_y);
        return;
    }
    g_stand_y = (double)y;
    printf("stand_y = %d mm\n", y);
    pose_stand();      /* 立刻按新站高重站 */
}

static void cmd_u(const char *arg)
{
    (void)arg;
    g_gait_h += 5.0;
    if (g_gait_h > 60.0) g_gait_h = 60.0;
    printf("lift = %.0f mm (max 60, 80 is the femur limit)\n", g_gait_h);
}

static void cmd_d(const char *arg)
{
    (void)arg;
    g_gait_h -= 5.0;
    if (g_gait_h < 10.0) g_gait_h = 10.0;
    printf("lift = %.0f mm\n", g_gait_h);
}

static void cmd_v(const char *arg)
{
    int ms = 0;
    if (sscanf(arg, "%d", &ms) != 1) {
        printf("usage: !V <60-200>  (ms per gait step, smaller = faster)\n");
        return;
    }
    if (ms < 60 || ms > 200) {
        printf("!V range 60..200 ms, current %u\n", (unsigned)g_gait_ms);
        return;
    }
    g_gait_ms  = (uint16_t)ms;
    g_gait_sub = (uint8_t)(ms / 10);   /* 子步固定 10 ms：I2C 18 路要 5.3 ms */
    if (g_gait_sub < 1) g_gait_sub = 1;
    printf("step = %u ms, sub = %u (sub-step = %u ms)\n",
           (unsigned)g_gait_ms, (unsigned)g_gait_sub,
           (unsigned)(g_gait_ms / g_gait_sub));
}

static void cmd_a(const char *arg)
{
    (void)arg;
    printf("gait  %u %-10s  L=%.0f H=%.0f ms=%u sub=%u scale=%.2f\n",
           (unsigned)gait_current(), gait_name(gait_current()),
           (double)GAIT_L, g_gait_h, (unsigned)g_gait_ms,
           (unsigned)g_gait_sub, gait_stride_scale());
    printf("stand y=%.0f z=%.0f\n", g_stand_y, g_stand_z);
    for (uint8_t i = 0; i <= 8; i++) {
        printf("[%u]%d ", (unsigned)i, g_servo_deg[i]);
    }
    printf("\n");
    for (uint8_t i = 9; i <= 17; i++) {
        printf("[%u]%d ", (unsigned)i, g_servo_deg[i]);
    }
    printf("\n");
}

/* ==================== 命令表：加命令只在这里加一行 ==================== */
struct Cmd {
    const char *name;             /* 不含 '!' */
    void (*fn)(const char *arg);  /* arg = 命令名之后的剩余部分 */
};

static const struct Cmd g_cmds[] = {
    { "M", cmd_m },
    { "O", cmd_o },
    { "S", cmd_s },
    { "T", cmd_t },
    { "I", cmd_i },
    { "Z", cmd_z },
    { "W", cmd_w },
    { "G", cmd_g },
    { "B", cmd_b },
    { "H", cmd_h },
    { "U", cmd_u },
    { "D", cmd_d },
    { "V", cmd_v },
    { "A", cmd_a },
    /* 新命令加在这里，例如： { "I", cmd_i }, */
};

/* ==================== 读一行 → 查表 → 调用 ==================== */
void cmd_task(void *arg)
{
    (void)arg;
    char line[64];

    while (fgets(line, sizeof(line), stdin) != NULL) {
        char *p = line;
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        if (*p++ != '!') {
            printf("need '!'\n");
            fflush(stdout);
            continue;
        }

        /* 命令名 = '!' 之后到第一个空白 */
        size_t n = strcspn(p, " \t\r\n");

        int hit = 0;
        for (size_t i = 0; i < sizeof(g_cmds) / sizeof(g_cmds[0]); i++) {
            /* 长度相等 + 内容相等 = 整名匹配（不是前缀匹配） */
            if (strlen(g_cmds[i].name) == n && strncmp(g_cmds[i].name, p, n) == 0) {
                g_cmds[i].fn(p + n);
                hit = 1;
                break;
            }
        }
        if (!hit) {
            printf("unknown: %s", line);
        }
        fflush(stdout);
    }
}
