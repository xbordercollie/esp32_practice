/**
 * @file safety.h
 * @brief 安全层：启动自检、硬件故障输入、双电池低压、急停/软关机
 *
 * 策略见硬件接口文档 §7、§8、§9：
 *   低压（静置口径）→ 停走 + 回站姿，保持舵机轨（断电会瘫倒）
 *   PGOOD/ALERT/TLV/INA 读失败 → 急停 + 断舵机轨 + 锁存
 *   PWR_INT 短按 → 软关机
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* 编译开关 */
#define SAFETY_BOOT_STRICT   1   /* 1=启动/!O 严格检查电池/PGOOD/ALERT/INA；0=台架排查旁路
                                  * 只影响开轨门控；运行中的故障监控始终保留 */
#define TLV_ACTION_SHUTDOWN  0   /* 1=TLV 欠压直接软关机；0=急停断轨 */

/* 电池软停阈值（静置口径 mV）与内阻估计 */
#define SERVO_LOW_MV          6400
#define SERVO_CLEAR_MV        6500
#define LOGIC_LOW_MV          6900
#define LOGIC_CLEAR_MV        7000
#define BATT_R_SERVO_MOHM       20
#define BATT_R_LOGIC_MOHM       25

/* 故障位（!A 打印用） */
#define SAFETY_FAULT_PGOOD      (1u << 0)
#define SAFETY_FAULT_ALERT      (1u << 1)
#define SAFETY_FAULT_TLV        (1u << 2)
#define SAFETY_FAULT_INA_S      (1u << 3)
#define SAFETY_FAULT_INA_L      (1u << 4)
#define SAFETY_FAULT_LOW_SERVO  (1u << 5)
#define SAFETY_FAULT_LOW_LOGIC  (1u << 6)
#define SAFETY_FAULT_GAIT       (1u << 7)
#define SAFETY_FAULT_ESTOP      (1u << 8)

struct SafetyStatus {
    bool rail_on;
    bool estop;
    bool low_servo;
    bool low_logic;
    bool pgood_ok;
    bool alert_ok;
    bool tlv_ok;
    bool ina_servo_ok;
    bool ina_logic_ok;
    bool pca_left_ok;
    bool pca_right_ok;
    int32_t servo_mv, servo_ma, servo_rest_mv;
    int32_t logic_mv, logic_ma, logic_rest_mv;
    uint32_t fault_mask;
};

void safety_init(void);

/** @brief 记录启动初始化结果（I2C/PCA9685/INA226），供开轨门控使用 */
void safety_set_ready(bool i2c0_ok, bool i2c1_ok,
                      bool pca_left_ok, bool pca_right_ok,
                      bool ina_servo_ok, bool ina_logic_ok);

/** @brief 现场重新采样并判断能否开轨；why 输出失败原因（可为空串） */
bool safety_can_enable_rail(char *why, size_t why_len);

/** @brief 门控开轨：回中 → SERVO_PWR_EN=1 → 100ms → OE=0；成功后清急停锁存 */
void safety_rail_on(void);

/** @brief 急停：中止步态 → 等 quiescent ≤50ms → 回中/断电；锁存
 *  @param cut_first true=先断轨再回中（硬件故障）；false=回中+100ms 再断轨（手动 !X） */
void safety_emergency_stop(const char *reason, bool cut_first);

/** @brief 软关机：停 → 回中 → OE=1 → EN=0 → 20ms → KILL=0；不返回 */
void safety_graceful_shutdown(const char *reason);

/** @brief 100ms 周期调用：采样电池/故障脚/PWR_INT，执行策略 */
void safety_poll(void);

void safety_get_status(struct SafetyStatus *out);
bool safety_estop_latched(void);
bool safety_low_batt_latched(void);
