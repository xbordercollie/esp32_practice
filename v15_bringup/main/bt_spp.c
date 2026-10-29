/**
 * @file bt_spp.c
 * @brief 经典蓝牙 SPP 服务（改自 spider_v2 BT_SPP，已验证可用）
 *
 * 输入字节写环形缓冲，bt_spp_task 拼行后交给 cmd_execute_line()。
 * 命令回复由 cmd.c 的 cmd_printf 镜像到 SPP。
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "bt_spp.h"
#include "cmd.h"
#include "gait.h"

#define BT_DEVICE_NAME   "HexaSpider"
#define BT_RX_BUF_SIZE   256

static uint8_t  g_rx_buf[BT_RX_BUF_SIZE];
static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail = 0;
static volatile bool g_connected = false;
static volatile uint32_t g_spp_handle = 0;
static volatile bool g_banner_pending = false;

static const char g_banner[] =
    "\r\nHexaSpider v1.5 BT ready\r\n"
    "!F/!B/!L/!R/!Q/!E move, !S stop, !A status, !BAT battery\r\n"
    "!SPD 1-10 speed, !V 60-200 ms, !O rail on, !X e-stop\r\n";

static void bt_spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        /* bluedroid 使能后默认不可连接/不可发现，必须设扫描模式 */
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        g_connected = true;
        g_spp_handle = param->srv_open.handle;
        g_banner_pending = true;
        printf("[BT] client connected (handle 0x%lx)\n",
               (unsigned long)param->srv_open.handle);
        break;
    case ESP_SPP_CLOSE_EVT:
        g_connected = false;
        g_spp_handle = 0;
        printf("[BT] client disconnected\n");
        break;
    case ESP_SPP_DATA_IND_EVT: {
        uint16_t len = param->data_ind.len;
        uint16_t used = (uint16_t)((g_rx_head - g_rx_tail) & (BT_RX_BUF_SIZE - 1));
        uint16_t space = (uint16_t)(BT_RX_BUF_SIZE - 1 - used);
        if (len > space) {
            len = space;   /* 缓冲满：丢弃多余字节 */
        }
        for (uint16_t i = 0; i < len; i++) {
            g_rx_buf[g_rx_head] = param->data_ind.data[i];
            g_rx_head = (uint16_t)((g_rx_head + 1) & (BT_RX_BUF_SIZE - 1));
        }
        break;
    }
    default:
        break;
    }
}

bool bt_spp_init(void)
{
    esp_err_t ret;

    /* 只启用经典蓝牙，释放 BLE 内存 */
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        printf("[BT] mem_release failed: %s\n", esp_err_to_name(ret));
        return false;
    }

    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&cfg);
    if (ret != ESP_OK) {
        printf("[BT] controller init: %s\n", esp_err_to_name(ret));
        return false;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        printf("[BT] controller enable: %s\n", esp_err_to_name(ret));
        return false;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        printf("[BT] bluedroid init: %s\n", esp_err_to_name(ret));
        return false;
    }
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        printf("[BT] bluedroid enable: %s\n", esp_err_to_name(ret));
        return false;
    }

    ret = esp_bt_dev_set_device_name(BT_DEVICE_NAME);
    if (ret != ESP_OK) {
        printf("[BT] set device name: %s\n", esp_err_to_name(ret));
        return false;
    }

    ret = esp_spp_register_callback(bt_spp_callback);
    if (ret != ESP_OK) {
        printf("[BT] spp register: %s\n", esp_err_to_name(ret));
        return false;
    }
    ret = esp_spp_init(ESP_SPP_MODE_CB);
    if (ret != ESP_OK) {
        printf("[BT] spp init: %s\n", esp_err_to_name(ret));
        return false;
    }
    ret = esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, BT_DEVICE_NAME);
    if (ret != ESP_OK) {
        printf("[BT] spp start srv: %s\n", esp_err_to_name(ret));
        return false;
    }

    printf("[BT] SPP service started, name=%s\n", BT_DEVICE_NAME);
    return true;
}

int bt_spp_read_bytes(uint8_t *buf, int max)
{
    int n = 0;
    while (n < max) {
        uint16_t used = (uint16_t)((g_rx_head - g_rx_tail) & (BT_RX_BUF_SIZE - 1));
        if (used == 0) {
            break;
        }
        buf[n++] = g_rx_buf[g_rx_tail];
        g_rx_tail = (uint16_t)((g_rx_tail + 1) & (BT_RX_BUF_SIZE - 1));
    }
    return n;
}

bool bt_spp_is_connected(void)
{
    return g_connected;
}

bool bt_spp_write(const uint8_t *data, int len)
{
    if (!g_connected || data == NULL || len <= 0) {
        return false;
    }
    uint32_t h = g_spp_handle;
    if (h == 0) {
        return false;
    }
    return (esp_spp_write(h, len, (uint8_t *)data) == ESP_OK);
}

void bt_spp_task(void *arg)
{
    (void)arg;
    uint8_t buf[32];
    char line[96];
    size_t n = 0;
    bool was_connected = false;

    for (;;) {
        bool now_connected = bt_spp_is_connected();

        if (was_connected && !now_connected) {
            n = 0;
            printf("[BT] link lost, request stop\n");
            gait_request_stop();     /* 断链 fail-safe：走完当前周期回站姿 */
        }
        was_connected = now_connected;

        /* 连接后补发一次欢迎/命令提示 */
        if (now_connected && g_banner_pending) {
            g_banner_pending = false;
            bt_spp_write((const uint8_t *)g_banner, (int)strlen(g_banner));
        }

        int r = bt_spp_read_bytes(buf, sizeof(buf));
        for (int i = 0; i < r; i++) {
            char c = (char)buf[i];
            if (c == '\r' || c == '\n') {
                if (n > 0) {
                    line[n] = '\0';
                    cmd_execute_line(line);
                    n = 0;
                }
            } else if (n < sizeof(line) - 1) {
                line[n++] = c;
            } else {
                n = 0;   /* 超长行丢弃 */
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
