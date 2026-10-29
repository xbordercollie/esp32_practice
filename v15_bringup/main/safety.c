/**
 * @file safety.c
 * @brief 安全策略实现（见 safety.h）
 *
 * 设计取舍：
 *   - 低压不切舵机轨：六腿站着断电会直接瘫倒；用受控回站姿替代。
 *   - 硬故障（PGOOD/ALERT/TLV/INA 读失败/步态 I2C 故障）先断轨再回中，
 *     并且锁存，需要人工 !O 重新自检后才能恢复。
 *   - 手动 !X 回中后等 100ms 再断轨，让舵机真的走到中位。
 *   - 所有电压判断用静置估算：V_rest = V_meas + sign × I × R_int。
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "safety.h"
#include "power.h"
#include "pca9685.h"
#include "ina226.h"
#include "gait.h"

static bool g_i2c0_ok, g_i2c1_ok;
static bool g_pca_l_ok, g_pca_r_ok;
static bool g_ina_s_init_ok, g_ina_l_init_ok;

static bool g_estop;
static bool g_low_servo, g_low_logic;
static bool g_pwr_shutdown;
static bool g_poll_started;
static uint32_t g_fault_mask;
static volatile uint32_t g_estop_seq;
static TickType_t g_boot_tick;

static int g_pgood_low_cnt, g_alert_low_cnt, g_tlv_low_cnt;
static int g_pwr_int_low_cnt;
static int g_ina_s_fail, g_ina_l_fail;
static int g_low_servo_cnt, g_low_logic_cnt;
static int g_clear_servo_cnt, g_clear_logic_cnt;

static struct SafetyStatus g_status;
static portMUX_TYPE g_status_mux = portMUX_INITIALIZER_UNLOCKED;

static int32_t rest_mv(int32_t mv, int32_t ma, int sign, int r_mohm)
{
    return mv + sign * (int32_t)((int64_t)ma * r_mohm / 1000);
}

static void add_reason(char *why, size_t n, const char *msg)
{
    if (why == NULL || n == 0) {
        return;
    }
    size_t len = strlen(why);
    if (len >= n - 1) {
        return;
    }
    snprintf(why + len, n - len, "%s%s", len ? "; " : "", msg);
}

void safety_init(void)
{
    g_i2c0_ok = g_i2c1_ok = false;
    g_pca_l_ok = g_pca_r_ok = false;
    g_ina_s_init_ok = g_ina_l_init_ok = false;
    g_estop = g_low_servo = g_low_logic = false;
    g_pwr_shutdown = false;
    g_poll_started = false;
    g_fault_mask = 0;
    g_estop_seq = 0;
    g_pgood_low_cnt = g_alert_low_cnt = g_tlv_low_cnt = 0;
    g_pwr_int_low_cnt = 0;
    g_ina_s_fail = g_ina_l_fail = 0;
    g_low_servo_cnt = g_low_logic_cnt = 0;
    g_clear_servo_cnt = g_clear_logic_cnt = 0;
    g_boot_tick = xTaskGetTickCount();
    memset(&g_status, 0, sizeof(g_status));
}

void safety_set_ready(bool i2c0_ok, bool i2c1_ok,
                      bool pca_left_ok, bool pca_right_ok,
                      bool ina_servo_ok, bool ina_logic_ok)
{
    g_i2c0_ok = i2c0_ok;
    g_i2c1_ok = i2c1_ok;
    g_pca_l_ok = pca_left_ok;
    g_pca_r_ok = pca_right_ok;
    g_ina_s_init_ok = ina_servo_ok;
    g_ina_l_init_ok = ina_logic_ok;

    g_status.pca_left_ok = pca_left_ok;
    g_status.pca_right_ok = pca_right_ok;
}

bool safety_can_enable_rail(char *why, size_t why_len)
{
    if (why != NULL && why_len > 0) {
        why[0] = '\0';
    }

    if (!g_i2c0_ok) add_reason(why, why_len, "I2C0 init failed");
    if (!g_pca_l_ok) add_reason(why, why_len, "PCA9685 0x40 not ready");
    if (!g_pca_r_ok) add_reason(why, why_len, "PCA9685 0x41 not ready");

#if SAFETY_BOOT_STRICT
    /* 严格模式才要求 I2C1 / INA226 / 电池 / 故障脚；台架排查可置 0 旁路 */
    if (!g_i2c1_ok) add_reason(why, why_len, "I2C1 init failed");
    if (!g_ina_s_init_ok) add_reason(why, why_len, "INA226 servo init failed");
    if (!g_ina_l_init_ok) add_reason(why, why_len, "INA226 logic init failed");
    if (!power_pgood_ok()) add_reason(why, why_len, "PGOOD_5V low");
    if (!power_alert_ok()) add_reason(why, why_len, "ALERT_S low (overcurrent)");
    if (!power_tlv_ok()) add_reason(why, why_len, "TLV3011 undervoltage");

    int32_t mv = 0, ma = 0;
    if (g_ina_s_init_ok) {
        if (ina226_read(I2C_NUM_0, INA226_ADDR_SERVO, &mv, &ma) != ESP_OK) {
            add_reason(why, why_len, "INA226 servo read failed");
        } else {
            /* 已锁存低压时要求回到 CLEAR 阈值，避免跳过回差 */
            int32_t limit = g_low_servo ? SERVO_CLEAR_MV : SERVO_LOW_MV;
            if (rest_mv(mv, ma, +1, BATT_R_SERVO_MOHM) < limit) {
                add_reason(why, why_len, "servo pack low");
            }
        }
    }
    if (g_ina_l_init_ok) {
        if (ina226_read(I2C_NUM_1, INA226_ADDR_LOGIC, &mv, &ma) != ESP_OK) {
            add_reason(why, why_len, "INA226 logic read failed");
        } else {
            int32_t limit = g_low_logic ? LOGIC_CLEAR_MV : LOGIC_LOW_MV;
            if (rest_mv(mv, ma, -1, BATT_R_LOGIC_MOHM) < limit) {
                add_reason(why, why_len, "logic pack low");
            }
        }
    }
#endif

    return (why == NULL || why_len == 0 || why[0] == '\0');
}

void safety_rail_on(void)
{
    if (g_pwr_shutdown) {
        printf("[SAFE] rail ON refused: shutdown in progress\n");
        return;
    }
    if (power_rail_is_on()) {
        printf("[SAFE] servo rail already ON\n");
        return;
    }
    gait_state_t st = gait_state();
    if (st == GAIT_WALKING || st == GAIT_SETTLING) {
        printf("[SAFE] rail ON refused: gait busy (%d)\n", (int)st);
        return;
    }

    char why[192];
    if (!safety_can_enable_rail(why, sizeof(why))) {
        printf("[SAFE] rail ON refused: %s\n", why);
        return;
    }

    /* 开轨前清 abort；center/power-on 期间若来急停会重新置位 */
    uint32_t seq = g_estop_seq;
    power_rail_abort_clear();

    int fail = servo_center_all();
    if (fail != 0) {
        printf("[SAFE] rail ON refused: center %d failed\n", fail);
        return;
    }

    if (!power_servo_on()) {
        return;   /* 被急停打断，锁存保持 */
    }

    if (g_estop_seq != seq) {
        /* 开轨过程中又来了急停：立即关轨，不覆盖锁存 */
        power_servo_off();
        printf("[SAFE] rail ON aborted: new e-stop during sequence\n");
        return;
    }

    gait_clear_fault();
    g_estop = false;
    g_fault_mask &= ~SAFETY_FAULT_ESTOP;
#if SAFETY_BOOT_STRICT
    /* can_enable 已按 CLEAR 阈值检查过，可以安全清低压锁存 */
    g_low_servo = g_low_logic = false;
    g_fault_mask &= ~(SAFETY_FAULT_LOW_SERVO | SAFETY_FAULT_LOW_LOGIC);
    g_low_servo_cnt = g_low_logic_cnt = 0;
    g_clear_servo_cnt = g_clear_logic_cnt = 0;
#endif
    printf("[SAFE] servo rail ON\n");
}

void safety_emergency_stop(const char *reason, bool cut_first)
{
    /* 每次请求都推进序号：正在开轨的 !O 看到序号变化会放弃 */
    g_estop_seq++;

    if (g_estop) {
        /* 已锁存：若轨还意外开着，补一刀断电 */
        if (power_rail_is_on()) {
            power_servo_off();
            servo_center_all();
        }
        return;
    }
    g_estop = true;
    g_fault_mask |= SAFETY_FAULT_ESTOP;
    printf("[SAFE] EMERGENCY STOP: %s\n", reason ? reason : "?");

    gait_abort_immediate();

    if (cut_first) {
        /* 硬故障：立即断轨，再尽力回中（寄存器/输出已禁用） */
        power_servo_off();
        servo_center_all();
        return;
    }

    bool quiescent = false;
    for (int i = 0; i < 5; i++) {          /* 5 × 1 tick = 50ms @100Hz */
        if (gait_is_quiescent()) {
            quiescent = true;
            break;
        }
        vTaskDelay(1);                     /* pdMS_TO_TICKS(5) 在 100Hz 下是 0 */
    }

    if (!quiescent) {
        /* 步态没停下来：fail-safe 先断轨 */
        power_servo_off();
        servo_center_all();
        printf("[SAFE] e-stop: gait not quiescent in 50ms, rail cut first\n");
    } else {
        /* 手动 !X：先回中，给舵机约 5 帧时间，再断轨 */
        servo_center_all();
        vTaskDelay(pdMS_TO_TICKS(100));
        power_servo_off();
    }
}

void safety_graceful_shutdown(const char *reason)
{
    if (g_pwr_shutdown) {
        vTaskDelay(portMAX_DELAY);
    }
    g_pwr_shutdown = true;
    g_estop = true;
    g_fault_mask |= SAFETY_FAULT_ESTOP;
    printf("[SAFE] SHUTDOWN: %s\n", reason ? reason : "?");

    gait_abort_immediate();
    for (int i = 0; i < 5; i++) {          /* 5 × 1 tick = 50ms */
        if (gait_is_quiescent()) {
            break;
        }
        vTaskDelay(1);
    }

    servo_center_all();
    vTaskDelay(pdMS_TO_TICKS(100));        /* 让舵机走到中位 */
    power_servo_off();
    vTaskDelay(pdMS_TO_TICKS(20));
    power_kill_release();
    printf("[SAFE] KILL released, power off\n");
    vTaskDelay(portMAX_DELAY);
}

void safety_poll(void)
{
    if (g_pwr_shutdown) {
        return;
    }

    /* 第一次真正开始轮询时再起算 2s 忽略窗（启动扫描可能已经过了 2s） */
    if (!g_poll_started) {
        g_poll_started = true;
        g_boot_tick = xTaskGetTickCount();
        g_pwr_int_low_cnt = 0;
    }

    /* PWR_INT：开机 2s 内忽略；连续 2 次 100ms（约 200ms）低 → 软关机 */
    if ((xTaskGetTickCount() - g_boot_tick) > pdMS_TO_TICKS(2000)) {
        if (power_pwr_int_low()) {
            if (++g_pwr_int_low_cnt >= 2) {
                safety_graceful_shutdown("PWR_INT pressed");
                return;
            }
        } else {
            g_pwr_int_low_cnt = 0;
        }
    }

    /* 先读硬故障脚：I2C 卡死时也能尽快动作 */
    bool pgood = power_pgood_ok();
    bool alert = power_alert_ok();
    bool tlv   = power_tlv_ok();

    if (!pgood) {
        if (++g_pgood_low_cnt >= 2) g_fault_mask |= SAFETY_FAULT_PGOOD;
    } else {
        g_pgood_low_cnt = 0;
        g_fault_mask &= ~SAFETY_FAULT_PGOOD;
    }
    if (!alert) {
        if (++g_alert_low_cnt >= 2) g_fault_mask |= SAFETY_FAULT_ALERT;
    } else {
        g_alert_low_cnt = 0;
        g_fault_mask &= ~SAFETY_FAULT_ALERT;
    }
    if (!tlv) {
        if (++g_tlv_low_cnt >= 2) g_fault_mask |= SAFETY_FAULT_TLV;
    } else {
        g_tlv_low_cnt = 0;
        g_fault_mask &= ~SAFETY_FAULT_TLV;
    }

    if (!pgood && g_pgood_low_cnt >= 2) {
        safety_emergency_stop("PGOOD_5V low", true);
    } else if (!alert && g_alert_low_cnt >= 2) {
        safety_emergency_stop("ALERT_S overcurrent", true);
    } else if (!tlv && g_tlv_low_cnt >= 2) {
#if TLV_ACTION_SHUTDOWN
        safety_graceful_shutdown("TLV3011 undervoltage");
#else
        safety_emergency_stop("TLV3011 undervoltage", true);
#endif
    }

    /* 再采样两颗 INA226（连续模式，随时可读）；初始化失败的就不读 */
    int32_t smv = 0, sma = 0, lmv = 0, lma = 0;
    bool sok = false, lok = false;

    if (g_ina_s_init_ok) {
        sok = (ina226_read(I2C_NUM_0, INA226_ADDR_SERVO, &smv, &sma) == ESP_OK);
        if (sok) {
            g_ina_s_fail = 0;
            g_fault_mask &= ~SAFETY_FAULT_INA_S;
        } else if (++g_ina_s_fail == 3) {
            g_fault_mask |= SAFETY_FAULT_INA_S;
            printf("[SAFE] INA226 servo read failed 3x\n");
        }
    } else {
        g_ina_s_fail = 0;
    }
    if (g_ina_l_init_ok) {
        lok = (ina226_read(I2C_NUM_1, INA226_ADDR_LOGIC, &lmv, &lma) == ESP_OK);
        if (lok) {
            g_ina_l_fail = 0;
            g_fault_mask &= ~SAFETY_FAULT_INA_L;
        } else if (++g_ina_l_fail == 3) {
            g_fault_mask |= SAFETY_FAULT_INA_L;
            printf("[SAFE] INA226 logic read failed 3x\n");
        }
    } else {
        g_ina_l_fail = 0;
    }

    if (g_ina_s_fail >= 3) {
        safety_emergency_stop("INA226 servo read failure", true);
    } else if (g_ina_l_fail >= 3) {
        safety_emergency_stop("INA226 logic read failure", true);
    }

    /* 步态故障（I2C 写失败）→ 急停；IK 不可达已在 gait 内受控回站姿 */
    if (gait_state() == GAIT_FAULT) {
        g_fault_mask |= SAFETY_FAULT_GAIT;
        safety_emergency_stop("gait fault (I2C)", true);
    } else {
        g_fault_mask &= ~SAFETY_FAULT_GAIT;
    }

    /* 低压：停走 + 回站姿，保持舵机轨；带回差恢复 */
    int32_t rest_s = 0, rest_l = 0;
    if (sok) {
        rest_s = rest_mv(smv, sma, +1, BATT_R_SERVO_MOHM);
    }
    if (lok) {
        rest_l = rest_mv(lmv, lma, -1, BATT_R_LOGIC_MOHM);
    }

    if (sok) {
        int32_t rest = rest_s;
        if (rest < SERVO_LOW_MV) {
            g_clear_servo_cnt = 0;
            if (++g_low_servo_cnt >= 3 && !g_low_servo) {
                g_low_servo = true;
                g_fault_mask |= SAFETY_FAULT_LOW_SERVO;
                printf("[SAFE] servo pack low: rest %ld mV, stop and hold\n", (long)rest);
                gait_request_stop_immediate();
            }
        } else if (rest > SERVO_CLEAR_MV) {
            g_low_servo_cnt = 0;
            if (++g_clear_servo_cnt >= 3 && g_low_servo) {
                g_low_servo = false;
                g_fault_mask &= ~SAFETY_FAULT_LOW_SERVO;
                printf("[SAFE] servo pack recovered: rest %ld mV\n", (long)rest);
            }
        } else {
            g_low_servo_cnt = 0;
            g_clear_servo_cnt = 0;
        }
    }

    if (lok) {
        int32_t rest = rest_l;
        if (rest < LOGIC_LOW_MV) {
            g_clear_logic_cnt = 0;
            if (++g_low_logic_cnt >= 3 && !g_low_logic) {
                g_low_logic = true;
                g_fault_mask |= SAFETY_FAULT_LOW_LOGIC;
                printf("[SAFE] logic pack low: rest %ld mV, stop and hold\n", (long)rest);
                gait_request_stop_immediate();
            }
        } else if (rest > LOGIC_CLEAR_MV) {
            g_low_logic_cnt = 0;
            if (++g_clear_logic_cnt >= 3 && g_low_logic) {
                g_low_logic = false;
                g_fault_mask &= ~SAFETY_FAULT_LOW_LOGIC;
                printf("[SAFE] logic pack recovered: rest %ld mV\n", (long)rest);
            }
        } else {
            g_low_logic_cnt = 0;
            g_clear_logic_cnt = 0;
        }
    }

    /* 更新状态快照（短临界区，避免 !A 拷到一半被高优先级任务打断） */
    taskENTER_CRITICAL(&g_status_mux);
    g_status.rail_on = power_rail_is_on();
    g_status.estop = g_estop;
    g_status.low_servo = g_low_servo;
    g_status.low_logic = g_low_logic;
    g_status.pgood_ok = pgood;
    g_status.alert_ok = alert;
    g_status.tlv_ok = tlv;
    g_status.ina_servo_ok = sok;
    g_status.ina_logic_ok = lok;
    g_status.pca_left_ok = g_pca_l_ok;
    g_status.pca_right_ok = g_pca_r_ok;
    g_status.servo_mv = smv;
    g_status.servo_ma = sma;
    g_status.servo_rest_mv = rest_s;
    g_status.logic_mv = lmv;
    g_status.logic_ma = lma;
    g_status.logic_rest_mv = rest_l;
    g_status.fault_mask = g_fault_mask;
    taskEXIT_CRITICAL(&g_status_mux);
}

void safety_get_status(struct SafetyStatus *out)
{
    if (out == NULL) {
        return;
    }
    taskENTER_CRITICAL(&g_status_mux);
    /* 锁存/轨状态取实时值，传感器值取最近一次 poll */
    g_status.rail_on = power_rail_is_on();
    g_status.estop = g_estop;
    g_status.low_servo = g_low_servo;
    g_status.low_logic = g_low_logic;
    g_status.fault_mask = g_fault_mask;
    *out = g_status;
    taskEXIT_CRITICAL(&g_status_mux);
}

bool safety_estop_latched(void)
{
    return g_estop;
}

bool safety_low_batt_latched(void)
{
    return g_low_servo || g_low_logic;
}
