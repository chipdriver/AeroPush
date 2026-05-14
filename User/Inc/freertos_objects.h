#ifndef __FREERTOS_OBJECTS_H__                          // 防止 freertos_objects.h 被重复包含
#define __FREERTOS_OBJECTS_H__                          // 定义 FreeRTOS 对象头文件保护宏
/* headler file incution */                             // 头文件包含区域
#include "FreeRTOS.h"                                   // 引入 FreeRTOS 基础定义
#include "queue.h"                                      // 引入 FreeRTOS 队列接口
#include "semphr.h"                                     // 引入 FreeRTOS 信号量和互斥锁接口
#include "app_types.h"                                  // 引入项目数据类型定义
#include "event_groups.h"                               // 引入 FreeRTOS 事件组接口

/* macro definition */                                  // 全局 FreeRTOS 对象声明区域
extern QueueHandle_t qAttitude;                         // 声明姿态数据队列句柄
extern QueueHandle_t qGnss;                             // 声明 GNSS 数据队列句柄
extern QueueHandle_t qMqttPublish;                      // 声明 MQTT 发布队列句柄
extern SemaphoreHandle_t mutexUart6log;                 // 声明串口日志递归互斥锁句柄
extern EventGroupHandle_t gSystemEventGroup;            // 声明系统事件组句柄
/* function declaration */                              // 函数声明区域

/**
 * @brief  创建 FreeRTOS 队列、互斥锁和事件组等对象。
 * @param  None
 * @retval None
 */
 void FreeRTOS_ObjectsCreate(void);                     // 声明 FreeRTOS 对象创建函数
#endif /* __FREERTOS_OBJECTS_H__ */                     // 结束 FreeRTOS 对象头文件保护宏
