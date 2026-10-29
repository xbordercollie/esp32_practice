#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "pca9685.h"
#include "power.h"
#include "cmd.h"
#include "ina226.h"
#include "gait.h"
#include "safety.h"
#include "pca9554.h"
#include "bt_spp.h"

static bool i2c_master_init(i2c_port_t port, int sda, int scl, uint32_t hz);
static void i2c_scan(i2c_port_t port);
static void heartbeat_task(void *arg);

void app_main(void)
{
    /* 初始化电源管理模块：KILL 必须是第一件事（LTC2954 只给 512ms） */
    power_init();
    printf("[boot] KILL high\n");
    safety_init();

    /* 两路 I2C 先都起来（板上有 2.2k 上拉，不开内部上拉） */
    bool i2c0_ok = i2c_master_init(I2C_NUM_0, 21, 22, 400000);
    bool i2c1_ok = i2c_master_init(I2C_NUM_1, 14, 27, 100000);
    printf("[boot] I2C init: 0=%d 1=%d\n", (int)i2c0_ok, (int)i2c1_ok);
    if (i2c0_ok) i2c_scan(I2C_NUM_0);
    if (i2c1_ok) i2c_scan(I2C_NUM_1);

    /* 两颗 INA226：0x44 舵机包在 I2C0、0x45 逻辑包在 I2C1 */
    bool ina_s_init_ok = false, ina_l_init_ok = false;
    bool ina_s_read_ok = false, ina_l_read_ok = false;
    int32_t smv = 0, sma = 0, lmv = 0, lma = 0;
    if (i2c0_ok) {
        ina_s_init_ok = (ina226_init(I2C_NUM_0, INA226_ADDR_SERVO) == ESP_OK);
    }
    if (i2c1_ok) {
        ina_l_init_ok = (ina226_init(I2C_NUM_1, INA226_ADDR_LOGIC) == ESP_OK);
    }
    /* AVG=16 首次转换约 35ms，等它出来再读 */
    vTaskDelay(pdMS_TO_TICKS(50));
    if (ina_s_init_ok) {
        ina_s_read_ok = (ina226_read(I2C_NUM_0, INA226_ADDR_SERVO, &smv, &sma) == ESP_OK);
    }
    if (ina_l_init_ok) {
        ina_l_read_ok = (ina226_read(I2C_NUM_1, INA226_ADDR_LOGIC, &lmv, &lma) == ESP_OK);
    }
    printf("[boot] INA226: servo init=%d read=%d (%ldmV/%ldmA) "
           "logic init=%d read=%d (%ldmV/%ldmA)\n",
           (int)ina_s_init_ok, (int)ina_s_read_ok, (long)smv, (long)sma,
           (int)ina_l_init_ok, (int)ina_l_read_ok, (long)lmv, (long)lma);

    bool pgood = power_pgood_ok();

    /* PCA9685：两颗都成功才允许开轨；开轨前 18 路先写 1500us */
    bool pca_l_ok = false, pca_r_ok = false;
    if (i2c0_ok) {
        pca_l_ok = (pca9685_init(I2C_NUM_0, PCA9685_ADDR_LEFT) == ESP_OK);
        pca_r_ok = (pca9685_init(I2C_NUM_0, PCA9685_ADDR_RIGHT) == ESP_OK);
    }
    int center_fail = 18;
    if (pca_l_ok && pca_r_ok) {
        center_fail = servo_center_all();
    }
    printf("[boot] PCA9685: left=%d right=%d center_fail=%d\n",
           (int)pca_l_ok, (int)pca_r_ok, center_fail);

    /* PCA9554 只探测，不改配置（引脚功能未文档化，地址待确认） */
    bool chip_s = i2c0_ok && pca9554_probe(I2C_NUM_0, PCA9554_ADDR_SERVO);
    bool chip_l = i2c1_ok && pca9554_probe(I2C_NUM_1, PCA9554_ADDR_LOGIC);
    printf("[boot] PCA9554: 0x%02X on I2C0=%d, 0x%02X on I2C1=%d\n",
           PCA9554_ADDR_SERVO, (int)chip_s, PCA9554_ADDR_LOGIC, (int)chip_l);

    bool alert_ok = power_alert_ok();
    bool tlv_ok = power_tlv_ok();

    /* 只把 init 结果传给安全层；单次读失败由 safety_can_enable_rail 现场重试 */
    safety_set_ready(i2c0_ok, i2c1_ok, pca_l_ok, pca_r_ok, ina_s_init_ok, ina_l_init_ok);

    char why[192];
    bool can_rail = safety_can_enable_rail(why, sizeof(why));
    printf("[boot] self-check: pgood=%d alert=%d tlv=%d -> %s\n",
           (int)pgood, (int)alert_ok, (int)tlv_ok,
           can_rail ? "OK" : why);

    gpio_set_level(OE_PIN, 1);       /* 再显式拉高一次，确保开轨前高阻 */
    if (can_rail) {
        safety_rail_on();
        if (power_rail_is_on()) {
            printf("[boot] servo rail ON\n");
        } else {
            printf("[boot] servo rail NOT on (see reason above)\n");
        }
    } else {
        gpio_set_level(SERVO_PWR_EN_PIN, 0);
        gpio_set_level(OE_PIN, 1);
        printf("[boot] servo rail INHIBITED: %s\n", why);
    }

    /* 步态任务（6）> 控制台（5）；心跳/安全任务最高（7） */
    cmd_init();
    if (xTaskCreate(gait_task, "gait_task", 6144, NULL, 6, NULL) != pdPASS ||
        xTaskCreate(heartbeat_task, "heartbeat_task", 4096, NULL, 7, NULL) != pdPASS ||
        xTaskCreate(cmd_task, "cmd_task", 8192, NULL, 5, NULL) != pdPASS) {
        printf("[boot] task creation failed, safe off and restart\n");
        power_servo_off();
        power_kill_release();
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    printf("Commands: !F/!B/!L/!R/!Q/!E move, !S stop, !W N test\n");
    printf("          !O rail on, !SOFF rail off, !X e-stop, !OFF shutdown\n");
    printf("          !B=back, !BAT=battery; !SPD 1-10 speed, !V ms, !A status\n");
    printf("          !T/!K servo test, !C PCA9554; Bluetooth SPP name=HexaSpider\n");

    /* NVS + 经典蓝牙 SPP（失败不致命，UART 控制台仍然可用） */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_err = nvs_flash_init();
    }
    if (nvs_err != ESP_OK) {
        printf("[boot] NVS init failed: %s (BT may not work)\n", esp_err_to_name(nvs_err));
    }

    if (bt_spp_init()) {
        if (xTaskCreate(bt_spp_task, "bt_spp", 8192, NULL, 4, NULL) != pdPASS) {
            printf("[boot] bt task create failed\n");
        }
    } else {
        printf("[boot] Bluetooth SPP init failed, UART console only\n");
    }
}

/* 心跳/安全任务：100ms 采样；LED 500ms 翻转；2s 打印一次 */
static void heartbeat_task(void *arg)
{
    (void)arg;
    int tick = 0;

    for (;;) {
        safety_poll();

        if (++tick % 5 == 0) {
            gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
        }
        if (tick % 20 == 0) {
            printf("Heartbeat count: %d\n", tick);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* 初始化 I2C 主机模式；返回是否可用 */
static bool i2c_master_init(i2c_port_t port, int sda, int scl, uint32_t hz)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        /* 板上已有 2.2k 上拉：不开 MCU 内部上拉（硬件文档 §8 规则 7） */
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = hz,
    };
    esp_err_t err = i2c_param_config(port, &conf);
    if (err != ESP_OK) {
        printf("[boot] I2C%d param config failed: %s\n", port, esp_err_to_name(err));
        return false;
    }
    err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        printf("[boot] I2C%d driver install failed: %s\n", port, esp_err_to_name(err));
        return false;
    }
    return true;
}

/* 扫描 I2C 总线上的设备 */
static void i2c_scan(i2c_port_t port)
{
    printf("I2C Scan on port %d:\n", port);
    int found = 0;
    for (int addr = 0x08; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(20));
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            printf("Found device at 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0) {
        printf("No I2C devices found.\n");
    }
}
