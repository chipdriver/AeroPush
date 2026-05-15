#ifndef __APP_TASKS_H__ /* 防止 app_tasks.h 被重复包含 */
#define __APP_TASKS_H__ /* 定义应用任务头文件保护宏 */

#include <stdio.h>             /* 引入 snprintf 等标准输出接口 */
#include <string.h>            /* 引入 memset 等字符串处理接口 */
#include "FreeRTOS.h"          /* 引入 FreeRTOS 基础定义 */
#include "task.h"              /* 引入 FreeRTOS 任务接口 */
#include "app_config.h"        /* 引入应用配置参数 */
#include "app_status.h"        /* 引入系统状态管理接口 */
#include "debug_service.h"     /* 引入调试服务接口 */
#include "debug_log.h"         /* 引入调试日志接口 */
#include "freertos_objects.h"  /* 引入 FreeRTOS 对象和队列声明 */
#include "imu_service.h"       /* 引入 IMU 服务接口 */
#include "led_service.h"       /* 引入 LED 服务接口 */
#include "modem_service.h"     /* 引入通信服务接口 */
#include "telemetry_service.h" /* 引入遥测服务接口 */

void APP_TasksCreate(void); /* 声明应用层任务创建函数 */

#endif /* 结束应用任务头文件保护宏 */
