#ifndef __FREERTOS_OBJECTS_H__                   /* 防止 freertos_objects.h 被重复包含 */
#define __FREERTOS_OBJECTS_H__                   /* 定义 FreeRTOS 对象头文件保护宏 */

#include "FreeRTOS.h"                            /* 引入 FreeRTOS 基础定义 */
#include "app_types.h"                           /* 引入项目公共数据类型 */
#include "event_groups.h"                        /* 引入 FreeRTOS 事件组接口 */
#include "queue.h"                               /* 引入 FreeRTOS 队列接口 */
#include "semphr.h"                              /* 引入 FreeRTOS 信号量接口 */

extern QueueHandle_t qAttitude;                  /* 声明姿态数据队列句柄 */
extern QueueHandle_t qGnss;                      /* 声明 GNSS 数据队列句柄 */
extern QueueHandle_t qMqttPublish;               /* 声明 MQTT 发布队列句柄 */
extern SemaphoreHandle_t mutexUart6log;          /* 声明调试串口日志互斥锁句柄 */
extern EventGroupHandle_t gSystemEventGroup;     /* 声明系统事件组句柄 */

void FreeRTOS_ObjectsCreate(void);               /* 声明 FreeRTOS 对象创建函数 */

#endif                                           /* 结束 FreeRTOS 对象头文件保护宏 */
