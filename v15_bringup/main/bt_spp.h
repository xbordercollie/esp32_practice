/**
 * @file bt_spp.h
 * @brief 经典蓝牙 SPP（手机「蓝牙串口助手」直接连）
 *
 * 设备名/服务名：HexaSpider。收到的字节由 bt_spp_task 拼成行后交给
 * cmd_execute_line()，和 UART0 共用同一套 ! 命令。
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

bool bt_spp_init(void);
int  bt_spp_read_bytes(uint8_t *buf, int max);   /* 返回读取字节数 */
bool bt_spp_is_connected(void);
bool bt_spp_write(const uint8_t *data, int len);

/** @brief 蓝牙命令任务：拼行 → cmd_execute_line；断连时请求停走 */
void bt_spp_task(void *arg);
