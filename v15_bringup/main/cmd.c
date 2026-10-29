/**
 * @file cmd.c
 * @brief 串口命令：表驱动分派（命令名 → 处理函数），参数由各 handler 自己解析
 *
 * 命令（UART0 控制台，115200）：
 *   !F/!B/!L/!R/!Q/!E  连续行走：前/后/左/右/左转/右转
 *   !S                 停止行走（走完当前周期 → 回站姿）
 *   !W <1-10>          非阻塞定周期测试（向前）
 *   !G [0-4]           列步态 / 选步态
 *   !O                 开舵机电源（先自检：INA/PGOOD/ALERT/PCA9685）
 *   !SOFF              断舵机电源（回中 → OE=1 → EN=0）
 *   !X                 急停（中止 → 回中 → 断电）
 *   !OFF               软关机（回中 → OE=1 → EN=0 → KILL=0）
 *   !M                 18 路全部回中位（1500us）
 *   !T <id> <d>        单路角度测试（0.1 度，!T 0 300 = +30 度）
 *   !K <id> <us>       单路原始脉宽测试（500-2500us）
 *   !I <x> <y> <z>     单腿 IK 试算
 *   !Z [y z]           重站 / 设站高站幅（y 120-190，z 140-180）
 *   !H <120-190>       站高（mm，越大机身越高），改完立刻重站
 *   !U / !D            抬腿高度 +/-5mm
 *   !V <60-200>        每步时长 ms（越小越快）
 *   !SPD <1-10>        速度档位：1 最慢(200ms)，5 默认(100ms)，10 最快(60ms)
 *   !A                 状态快照（步态/方向/站姿/舵机/限位/安全）
 *   !BAT               读两颗 INA226 的电压/电流/静置估算
 *   !C [...]           PCA9554 读/写（引脚功能未文档化，调试用）
 *
 * UART0 和经典蓝牙 SPP 共用同一套命令；命令回复同时写回 UART 和蓝牙。
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "power.h"
#include "pca9685.h"
#include "ik.h"
#include "pose.h"
#include "gait.h"
#include "ina226.h"
#include "safety.h"
#include "pca9554.h"
#include "bt_spp.h"

/* 命令互斥：UART 和蓝牙两个任务共用解析/执行路径 */
static SemaphoreHandle_t g_cmd_mux = NULL;

void cmd_init(void)
{
    g_cmd_mux = xSemaphoreCreateMutex();
    if (g_cmd_mux == NULL) {
        printf("cmd: mutex create failed\n");
    }
}

/* 命令回复同时写 UART0 和蓝牙 SPP（未连接时自动忽略） */
static void cmd_printf(const char *fmt, ...)
{
    char buf[192];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    fputs(buf, stdout);
    if (n > (int)sizeof(buf) - 1) {
        n = (int)sizeof(buf) - 1;
    }
    bt_spp_write((const uint8_t *)buf, n);
}
#define printf cmd_printf

/* ==================== 解析/状态辅助 ==================== */

static const char *state_name(void)
{
    switch (gait_state()) {
    case GAIT_WALKING:  return "walking";
    case GAIT_SETTLING: return "settling";
    case GAIT_FAULT:    return "fault";
    default:            return "idle";
    }
}

/* 解析 0~max 个整数；有尾随垃圾返回 -1 */
static int parse_ints(const char *s, long *out, int max)
{
    int n = 0;
    if (s == NULL) {
        return 0;
    }
    while (n < max) {
        char *end = NULL;
        long v = strtol(s, &end, 10);   /* 十进制：避免 0100 被当八进制 */
        if (end == s) {
            break;
        }
        out[n++] = v;
        s = end;
    }
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') {
        s++;
    }
    if (*s != '\0') {
        return -1;
    }
    return n;
}

static bool require_idle(const char *cmd)
{
    if (gait_is_busy()) {
        printf("%s: gait busy (%s), use !S/!X first\n", cmd, state_name());
        return false;
    }
    return true;
}

static bool require_rail(const char *cmd)
{
    if (!power_rail_is_on()) {
        printf("%s: servo rail off, send !O first\n", cmd);
        return false;
    }
    if (safety_estop_latched()) {
        printf("%s: e-stop latched, check faults then !O\n", cmd);
        return false;
    }
    return true;
}

/* 低压锁存时禁止任何会驱动舵机的命令（!M/!T/!K/!Z/!H） */
static bool require_battery(const char *cmd)
{
    if (safety_low_batt_latched()) {
        printf("%s: low battery latched, charge first\n", cmd);
        return false;
    }
    return true;
}

static void start_move(int16_t fwd, int16_t lat, int16_t yaw)
{
    if (!require_rail("move")) {
        return;
    }
    if (safety_low_batt_latched()) {
        printf("move: low battery latched, charge first\n");
        return;
    }
    if (!gait_move(fwd, lat, yaw)) {
        printf("move: rejected (state %s)\n", state_name());
    } else {
        printf("move: fwd=%d lat=%d yaw=%d\n", (int)fwd, (int)lat, (int)yaw);
    }
}

static int32_t rest_mv(int32_t mv, int32_t ma, int sign, int r_mohm)
{
    return mv + sign * (int32_t)((int64_t)ma * r_mohm / 1000);
}

/* ==================== 各命令处理函数（统一签名） ==================== */

static void cmd_f(const char *arg) { (void)arg; start_move((int16_t)( GAIT_SIGN_FWD * GAIT_FWD_MM), 0, 0); }
static void cmd_b(const char *arg) { (void)arg; start_move((int16_t)(-GAIT_SIGN_FWD * GAIT_FWD_MM), 0, 0); }
static void cmd_l(const char *arg) { (void)arg; start_move(0, (int16_t)(-GAIT_SIGN_LAT * GAIT_LAT_MM), 0); }
static void cmd_r(const char *arg) { (void)arg; start_move(0, (int16_t)( GAIT_SIGN_LAT * GAIT_LAT_MM), 0); }
static void cmd_q(const char *arg) { (void)arg; start_move(0, 0, (int16_t)( GAIT_SIGN_YAW * GAIT_YAW_UNITS)); }
static void cmd_e(const char *arg) { (void)arg; start_move(0, 0, (int16_t)(-GAIT_SIGN_YAW * GAIT_YAW_UNITS)); }

static void cmd_s(const char *arg)
{
    (void)arg;
    gait_request_stop();
    printf("stop: state=%s\n", state_name());
}

static void cmd_w(const char *arg)
{
    long v[1];
    int n = parse_ints(arg, v, 1);
    if (n != 1 || v[0] < 1 || v[0] > 10) {
        printf("usage: !W <1-10>\n");
        return;
    }
    if (!require_idle("!W")) {
        return;
    }
    if (!require_rail("!W")) {
        return;
    }
    if (safety_low_batt_latched()) {
        printf("!W: low battery latched, charge first\n");
        return;
    }
    if (!gait_test_walk((uint8_t)v[0])) {
        printf("!W: not started (state %s)\n", state_name());
        return;
    }
    printf("test walk %ld cycles (non-blocking; !S stops)\n", v[0]);
}

static void cmd_g(const char *arg)
{
    long v[1];
    int n = parse_ints(arg, v, 1);
    if (n < 0) {
        printf("usage: !G [0-4]\n");
        return;
    }
    if (n == 1) {
        if (!require_idle("!G")) {
            return;
        }
        if (v[0] < 0 || v[0] >= (long)gait_count() || !gait_select((uint8_t)v[0])) {
            printf("bad gait index (0..%u)\n", (unsigned)(gait_count() - 1));
            return;
        }
    }
    for (uint8_t i = 0; i < gait_count(); i++) {
        printf("  %u: %-10s%s\n", i, gait_name(i), (i == gait_current()) ? "  <= current" : "");
    }
}

static void cmd_o(const char *arg)
{
    (void)arg;
    /* FAULT 也允许 !O：先急停清故障，再走正常开轨自检 */
    gait_state_t st = gait_state();
    if (st == GAIT_WALKING || st == GAIT_SETTLING) {
        printf("!O: gait busy (%s), use !S/!X first\n", state_name());
        return;
    }
    if (st == GAIT_FAULT) {
        safety_emergency_stop("!O: clear gait fault", true);
    }
    safety_rail_on();
    printf("!O: servo rail %s\n", power_rail_is_on() ? "ON" : "OFF");
}

static void cmd_soff(const char *arg)
{
    (void)arg;
    /* FAULT 时步态任务已经不写舵机，允许回中后断轨 */
    gait_state_t st = gait_state();
    if (st == GAIT_WALKING || st == GAIT_SETTLING) {
        printf("!SOFF: gait busy (%s), use !S/!X first\n", state_name());
        return;
    }
    int fail = servo_center_all();
    vTaskDelay(pdMS_TO_TICKS(100));   /* 给舵机约 5 帧走到中位 */
    power_servo_off();
    printf("servo rail OFF (center fail %d)\n", fail);
}

static void cmd_x(const char *arg)
{
    (void)arg;
    safety_emergency_stop("!X", false);   /* 手动急停：回中 100ms 再断轨 */
    printf("e-stop latched; check faults then !O to recover\n");
}

static void cmd_off(const char *arg)
{
    (void)arg;
    safety_graceful_shutdown("!OFF");
}

static void cmd_m(const char *arg)
{
    (void)arg;
    if (!require_idle("!M") || !require_battery("!M")) {
        return;
    }
    int fail = servo_center_all();
    printf("mid 0..17: %d failed\n", fail);
}

static void cmd_t(const char *arg)
{
    long v[2];
    int n = parse_ints(arg, v, 2);
    if (n != 2 || v[0] < 0 || v[0] > 17 || v[1] < -900 || v[1] > 900) {
        printf("usage: !T <0-17> <-900..900>\n");
        return;
    }
    if (!require_idle("!T") || !require_battery("!T")) {
        return;
    }
    if (!power_rail_is_on()) {
        printf("!T: warning: servo rail is off, pulse has no effect\n");
    }
    esp_err_t err = servo_set_angle(I2C_NUM_0, (uint8_t)v[0], (int16_t)v[1]);
    printf("!T %ld %ld -> %s\n", v[0], v[1], esp_err_to_name(err));
}

static void cmd_k(const char *arg)
{
    long v[2];
    int n = parse_ints(arg, v, 2);
    if (n != 2 || v[0] < 0 || v[0] > 17 || v[1] < 500 || v[1] > 2500) {
        printf("usage: !K <0-17> <500..2500 us>\n");
        return;
    }
    if (!require_idle("!K") || !require_battery("!K")) {
        return;
    }
    if (!power_rail_is_on()) {
        printf("!K: warning: servo rail is off, pulse has no effect\n");
    }
    esp_err_t err = servo_set_pulse_us(I2C_NUM_0, (uint8_t)v[0], (uint16_t)v[1]);
    printf("!K %ld %ld us -> %s\n", v[0], v[1], esp_err_to_name(err));
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
    long v[3];
    int n = parse_ints(arg, v, 3);

    if (n == 0) {
        if (!require_idle("!Z") || !require_battery("!Z")) {
            return;
        }
        pose_stand();
        return;
    }
    if (n != 2) {
        printf("usage: !Z [y 120-190] [z 140-180]\n");
        return;
    }
    if (v[0] < 120 || v[0] > 190 || v[1] < 140 || v[1] > 180) {
        printf("!Z usage: !Z [y 120-190] [z 140-180]\n");
        return;
    }
    if (!require_idle("!Z") || !require_battery("!Z")) {
        return;
    }
    g_stand_y = (double)v[0];
    g_stand_z = (double)v[1];
    printf("stand y=%ld z=%ld\n", v[0], v[1]);
    pose_stand();
}

static void cmd_h(const char *arg)
{
    long v[1];
    int n = parse_ints(arg, v, 1);
    if (n != 1) {
        printf("usage: !H <120-190>   (stand height mm, larger = taller)\n");
        return;
    }
    if (v[0] < 120 || v[0] > 190) {
        printf("!H range 120..190 mm, current %d\n", (int)g_stand_y);
        return;
    }
    if (!require_idle("!H") || !require_battery("!H")) {
        return;
    }
    g_stand_y = (double)v[0];
    printf("stand_y = %ld mm\n", v[0]);
    pose_stand();
}

static void cmd_u(const char *arg)
{
    (void)arg;
    gait_set_lift_mm((int16_t)(g_gait_h + 5));
    printf("lift = %d mm (max 60, 80 is the femur limit)\n", (int)g_gait_h);
}

static void cmd_d(const char *arg)
{
    (void)arg;
    gait_set_lift_mm((int16_t)(g_gait_h - 5));
    printf("lift = %d mm\n", (int)g_gait_h);
}

static void cmd_v(const char *arg)
{
    long v[1];
    int n = parse_ints(arg, v, 1);
    if (n != 1) {
        printf("usage: !V <60-200>  (ms per gait step, smaller = faster)\n");
        return;
    }
    if (v[0] < 60 || v[0] > 200) {
        printf("!V range 60..200 ms, current %u\n", (unsigned)g_gait_ms);
        return;
    }
    gait_set_step_ms((uint16_t)v[0]);
    printf("step = %u ms, sub = %u, actual = %u ms\n",
           (unsigned)g_gait_ms, (unsigned)g_gait_sub,
           (unsigned)(g_gait_sub * 10));
}

/* 速度档位：直接改步态下发坐标的间隔时间（g_gait_ms）。
 * 5 档 = 默认 100ms；档位越高步频越快。 */
static const uint16_t g_spd_ms[10] = {
    200, 175, 150, 125, 100, 90, 80, 72, 65, 60
};

static void cmd_spd(const char *arg)
{
    long v[1];
    int n = parse_ints(arg, v, 1);

    if (n == 0) {
        printf("speed: step=%u ms, sub=%u, actual=%u ms "
               "(use !SPD 1-10; 5=default 100ms)\n",
               (unsigned)g_gait_ms, (unsigned)g_gait_sub,
               (unsigned)(g_gait_sub * 10));
        return;
    }
    if (n != 1 || v[0] < 1 || v[0] > 10) {
        printf("usage: !SPD <1-10>  (1 slowest, 5 default, 10 fastest)\n");
        return;
    }

    gait_set_step_ms(g_spd_ms[v[0] - 1]);
    printf("speed level %ld -> step=%u ms, sub=%u, actual=%u ms\n",
           v[0], (unsigned)g_gait_ms, (unsigned)g_gait_sub,
           (unsigned)(g_gait_sub * 10));
}

static void cmd_a(const char *arg)
{
    (void)arg;
    struct GaitTravel t, a;
    struct SafetyStatus s;
    gait_get_travel(&t, &a);
    safety_get_status(&s);

    printf("gait %u %-10s state=%s target=(%d,%d,%d) active=(%d,%d,%d) scale=%.2f\n",
           (unsigned)gait_current(), gait_name(gait_current()), state_name(),
           (int)t.fwd, (int)t.lat, (int)t.yaw,
           (int)a.fwd, (int)a.lat, (int)a.yaw,
           gait_stride_scale());
    printf("stand y=%.0f z=%.0f lift=%d step=%u sub=%u actual=%u ms\n",
           g_stand_y, g_stand_z, (int)g_gait_h,
           (unsigned)g_gait_ms, (unsigned)g_gait_sub,
           (unsigned)(g_gait_sub * 10));
    printf("safety rail=%d estop=%d low=%d/%d pgood=%d alert=%d tlv=%d ina=%d/%d fault=0x%08lx\n",
           (int)s.rail_on, (int)s.estop, (int)s.low_servo, (int)s.low_logic,
           (int)s.pgood_ok, (int)s.alert_ok, (int)s.tlv_ok,
           (int)s.ina_servo_ok, (int)s.ina_logic_ok,
           (unsigned long)s.fault_mask);
    printf("battery servo=%ldmV/%ldmA rest=%ldmV logic=%ldmV/%ldmA rest=%ldmV\n",
           (long)s.servo_mv, (long)s.servo_ma, (long)s.servo_rest_mv,
           (long)s.logic_mv, (long)s.logic_ma, (long)s.logic_rest_mv);
    printf("limits mask=0x%08lx io_fail=%u\n",
           (unsigned long)pose_limit_mask(), (unsigned)g_pose_io_fail);

    for (uint8_t i = 0; i <= 8; i++) {
        printf("[%u]%d ", (unsigned)i, g_servo_deg[i]);
    }
    printf("\n");
    for (uint8_t i = 9; i <= 17; i++) {
        printf("[%u]%d ", (unsigned)i, g_servo_deg[i]);
    }
    printf("\n");
}

static void cmd_bat(const char *arg)
{
    (void)arg;
    int32_t mv = 0, ma = 0;

    if (ina226_read(I2C_NUM_0, INA226_ADDR_SERVO, &mv, &ma) == ESP_OK) {
        printf("servo pack: %ld mV  %ld mA  rest=%ld mV  (low %d)\n",
               (long)mv, (long)ma,
               (long)rest_mv(mv, ma, +1, BATT_R_SERVO_MOHM), SERVO_LOW_MV);
    } else {
        printf("servo pack: read fail\n");
    }
    if (ina226_read(I2C_NUM_1, INA226_ADDR_LOGIC, &mv, &ma) == ESP_OK) {
        /* 逻辑包放电时读数为负；rest 已按极性和内阻修正 */
        printf("logic pack: %ld mV  %ld mA  rest=%ld mV  (low %d)\n",
               (long)mv, (long)ma,
               (long)rest_mv(mv, ma, -1, BATT_R_LOGIC_MOHM), LOGIC_LOW_MV);
    } else {
        printf("logic pack: read fail\n");
    }
}

static void c_read_defaults(i2c_port_t port, uint8_t addr)
{
    uint8_t in = 0, out = 0, cfg = 0;
    esp_err_t e = pca9554_read_reg(port, addr, PCA9554_REG_INPUT, &in);
    if (e != ESP_OK) {
        printf("PCA9554 0x%02X bus%d: not found (%s)\n", addr, port, esp_err_to_name(e));
        return;
    }
    pca9554_read_reg(port, addr, PCA9554_REG_OUTPUT, &out);
    pca9554_read_reg(port, addr, PCA9554_REG_CONFIG, &cfg);
    printf("PCA9554 0x%02X bus%d: in=0x%02X out=0x%02X cfg=0x%02X\n",
           addr, port, in, out, cfg);
}

static void cmd_c(const char *arg)
{
    int bus = 0, reg = 0, val = 0, used = 0;
    unsigned int addr = 0;
    char op = 0;

    int n = sscanf(arg, "%d %x %c %i %i%n", &bus, &addr, &op, &reg, &val, &used);
    if (n == EOF) {
        /* 无参数：读两颗默认地址 */
        c_read_defaults(I2C_NUM_0, PCA9554_ADDR_SERVO);
        c_read_defaults(I2C_NUM_1, PCA9554_ADDR_LOGIC);
        return;
    }
    if (n < 4 || bus < 0 || bus > 1 || addr < 0x20 || addr > 0x27 || reg < 0 || reg > 3) {
        printf("usage: !C  |  !C <bus 0|1> <addr 0x20-0x27> r <reg 0-3>  |  !C <bus> <addr> w <reg> <val>\n");
        return;
    }
    if (n == 5) {
        const char *rest = arg + used;
        while (*rest == ' ' || *rest == '\t' || *rest == '\r' || *rest == '\n') {
            rest++;
        }
        if (*rest != '\0') {
            printf("!C: trailing garbage after value\n");
            return;
        }
    }

    i2c_port_t port = (bus == 0) ? I2C_NUM_0 : I2C_NUM_1;
    if (n == 4 && op == 'r') {
        uint8_t v = 0;
        esp_err_t e = pca9554_read_reg(port, (uint8_t)addr, (uint8_t)reg, &v);
        printf("!C bus%d 0x%02X reg%d = 0x%02X (%s)\n",
               bus, addr, reg, v, esp_err_to_name(e));
    } else if (n == 5 && op == 'w') {
        if (val < 0 || val > 255) {
            printf("!C write value 0..255\n");
            return;
        }
        if (!require_idle("!C w")) {
            return;
        }
        esp_err_t e = pca9554_write_reg(port, (uint8_t)addr, (uint8_t)reg, (uint8_t)val);
        printf("!C bus%d 0x%02X reg%d <= 0x%02X (%s)\n",
               bus, addr, reg, val, esp_err_to_name(e));
        printf("WARN: CONFIG reg bit=1 input, 0=output; writing may change pin direction\n");
    } else {
        printf("usage: !C  |  !C <bus 0|1> <addr 0x20-0x27> r <reg 0-3>  |  !C <bus> <addr> w <reg> <val>\n");
    }
}

/* ==================== 命令表：加命令只在这里加一行 ==================== */
struct Cmd {
    const char *name;             /* 不含 '!' */
    void (*fn)(const char *arg);  /* arg = 命令名之后的剩余部分 */
};

static const struct Cmd g_cmds[] = {
    { "F",   cmd_f },
    { "B",   cmd_b },
    { "L",   cmd_l },
    { "R",   cmd_r },
    { "Q",   cmd_q },
    { "E",   cmd_e },
    { "S",   cmd_s },
    { "W",   cmd_w },
    { "G",   cmd_g },
    { "BAT", cmd_bat },
    { "O",   cmd_o },
    { "SOFF", cmd_soff },
    { "X",   cmd_x },
    { "OFF", cmd_off },
    { "M",   cmd_m },
    { "T",   cmd_t },
    { "K",   cmd_k },
    { "I",   cmd_i },
    { "Z",   cmd_z },
    { "H",   cmd_h },
    { "U",   cmd_u },
    { "D",   cmd_d },
    { "V",   cmd_v },
    { "SPD", cmd_spd },
    { "A",   cmd_a },
    { "C",   cmd_c },
};

/* ==================== 执行一行 → 查表 → 调用（UART/蓝牙共用） ==================== */
void cmd_execute_line(const char *line)
{
    if (line == NULL) {
        return;
    }

    char buf[96];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    if (g_cmd_mux) {
        xSemaphoreTake(g_cmd_mux, portMAX_DELAY);
    }

    char *p = buf;
    while (*p == ' ' || *p == '\t') {
        p++;
    }
    if (!(*p == '\0' || *p == '\r' || *p == '\n')) {
        if (*p++ != '!') {
            printf("need '!'\n");
        } else {
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
                printf("unknown: %s", buf);
            }
        }
    }

    if (g_cmd_mux) {
        xSemaphoreGive(g_cmd_mux);
    }
    fflush(stdout);
}

/* UART0 控制台读行任务 */
void cmd_task(void *arg)
{
    (void)arg;
    char line[96];

    for (;;) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            /* UART 读端异常时不让任务返回（IDF 会 abort），清错误后重试 */
            printf("console EOF, retrying...\n");
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        /* 超长行：丢弃剩余部分 */
        if (strchr(line, '\n') == NULL) {
            int c;
            while ((c = getchar()) != '\n' && c != EOF) {
            }
        }
        cmd_execute_line(line);
    }
}
