#pragma once

/** @brief 创建命令互斥锁；必须在 cmd_task/bt_spp_task 之前调用 */
void cmd_init(void);

/** @brief UART0 控制台任务（读行 → cmd_execute_line） */
void cmd_task(void *arg);

/** @brief 执行一行 ! 命令（UART 与蓝牙共用；内部有互斥锁） */
void cmd_execute_line(const char *line);
