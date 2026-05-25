#ifndef __APP_TASKS_H__ // 防止头文件重复包含
#define __APP_TASKS_H__

#include <stdio.h> // 提供 snprintf
#include <string.h> // 提供 memset
#include "FreeRTOS.h" // 提供 FreeRTOS 基础类型
#include "task.h" // 提供任务创建和延时接口
#include "app_config.h" // 提供任务栈、优先级和周期配置
#include "app_status.h" // 提供系统状态位接口
#include "bsp_a7670e_uart.h" // 提供 A7670E USART1 初始化接口
#include "debug_service.h" // 提供调试服务初始化接口
#include "debug_log.h" // 提供调试日志接口
#include "freertos_objects.h" // 提供队列、互斥锁和事件组对象
#include "imu_service.h" // 提供 IMU 服务接口
#include "led_service.h" // 提供 LED 服务接口
#include "modem_service.h" // 提供通信服务接口
#include "telemetry_service.h" // 提供遥测组包接口

/**
 * @brief 创建应用层所有 FreeRTOS 任务。
 * @retval None
 */
void APP_TasksCreate(void); // 创建应用任务

#endif // __APP_TASKS_H__
