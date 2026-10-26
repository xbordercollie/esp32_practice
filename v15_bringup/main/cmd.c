/**
 * @file cmd.c
 * @brief 串口命令：表驱动分派（命令名 → 处理函数），参数由各 handler 自己解析
 *
 * 命令（UART0 控制台，115200）：
 *   !M          18 路全部回中位（1500us）
 *   !O          舵机电源开（SERVO_PWR_EN=1 → 100ms → OE=0）
 *   !S          舵机电源关（OE=1 → SERVO_PWR_EN=0）
 *   !T <id> <d> 单路测试：第 id 路转到 d（0.1 度，!T 0 300 = +30 度）
 */
#include <stdio.h>
#include <string.h>
#include "power.h"
#include "pca9685.h"
#include "ik.h"

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
