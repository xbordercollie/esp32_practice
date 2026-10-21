#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

/* 按 09-16 表；你改过就以原理图为准 */
#define KILL_PIN      23
#define OE_LEFT_PIN   32
#define OE_RIGHT_PIN  33
#define LED_PIN        2

static void i2c_master_init(i2c_port_t port, int sda, int scl);
static void i2c_scan(i2c_port_t port);
static void heartbeat_task(void *arg);

void app_main(void)
{
    /* TODO 2：KILL 最先。LTC2954 的 512 ms 确认窗。
     * 两行：先写电平，再切输出。 */
    gpio_set_level(KILL_PIN, 1);
    gpio_set_direction(KILL_PIN, GPIO_MODE_OUTPUT);
    /* TODO 3：OE 两脚上电默认高（禁用）。
     * 四行：每个脚先写高、再切输出。 */
    gpio_set_level(OE_LEFT_PIN, 1);
    gpio_set_direction(OE_LEFT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(OE_RIGHT_PIN, 1);
    gpio_set_direction(OE_RIGHT_PIN, GPIO_MODE_OUTPUT);
    /* TODO 4：LED 配成输出。一行。 */
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    /* 初始化 I2C 并扫描设备 */
    i2c_master_init(I2C_NUM_0, 21, 22);
    i2c_scan(I2C_NUM_0);
    i2c_master_init(I2C_NUM_1, 18, 19);
    i2c_scan(I2C_NUM_1);

    /* TODO 5：建心跳任务。一行。 */
    xTaskCreate(heartbeat_task, "heartbeat_task", 2048, NULL, 5, NULL);
}








/* TODO 1：心跳任务 —— 每 500 ms 翻转 LED；每 4 次（2 s）打印一次计数 */
static void heartbeat_task(void *arg)
{
    (void)arg;
    /* 你的代码 */
    for (int i = 0; ; i++) {
        if (i % 4 == 0) {
        printf("Heartbeat count: %d\n", i);     
    }
        gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void i2c_master_init(i2c_port_t port, int sda, int scl)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000, // 100 kHz
    };
    esp_err_t err = i2c_param_config(port, &conf);
    if (err != ESP_OK) {
        printf("I2C param config failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if(err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        printf("I2C%d driver install failed: %s\n", port, esp_err_to_name(err));
        return;
    }
}

static void i2c_scan(i2c_port_t port)
{
    printf("I2C Scan on port %d:\n", port);
    int found = 0;
    for(int addr = 0x08; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(20));
        i2c_cmd_link_delete(cmd);
        if(err == ESP_OK) {
            printf("Found device at 0x%02X\n", addr);
            found++;
        }
    }
    if(found == 0) {
        printf("No I2C devices found.\n");
    }
}