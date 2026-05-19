#ifndef __APP_TASKS_H__ // 检查 __APP_TASKS_H__ 是否未定义，防止头文件重复包含
#define __APP_TASKS_H__ // 定义 __APP_TASKS_H__ 变量

#include <stdio.h> // 引入 stdio.h 提供的接口、宏和类型定义
#include <string.h> // 引入 string.h 提供的接口、宏和类型定义
#include "FreeRTOS.h" // 引入 FreeRTOS.h 提供的接口、宏和类型定义
#include "task.h" // 引入 task.h 提供的接口、宏和类型定义
#include "app_config.h" // 引入 app_config.h 提供的接口、宏和类型定义
#include "app_status.h" // 引入 app_status.h 提供的接口、宏和类型定义
#include "debug_service.h" // 引入 debug_service.h 提供的接口、宏和类型定义
#include "debug_log.h" // 引入 debug_log.h 提供的接口、宏和类型定义
#include "freertos_objects.h" // 引入 freertos_objects.h 提供的接口、宏和类型定义
#include "imu_service.h" // 引入 imu_service.h 提供的接口、宏和类型定义
#include "led_service.h" // 引入 led_service.h 提供的接口、宏和类型定义
#include "modem_service.h" // 引入 modem_service.h 提供的接口、宏和类型定义
#include "telemetry_service.h" // 引入 telemetry_service.h 提供的接口、宏和类型定义

/**
 * @brief 创建应用层所有 FreeRTOS 任务。
 * @retval None
 */
void APP_TasksCreate(void); // 声明APP_TasksCreate 函数签名：创建应用层所有 FreeRTOS 任务

#endif // 结束当前条件编译或头文件保护范围
