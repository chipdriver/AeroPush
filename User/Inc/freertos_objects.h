#ifndef __FREERTOS_OBJECTS_H__ // 防止头文件重复包含
#define __FREERTOS_OBJECTS_H__

#include "FreeRTOS.h" // 提供 FreeRTOS 基础类型
#include "app_types.h" // 提供姿态、GNSS 和 MQTT 数据结构
#include "event_groups.h" // 提供事件组类型
#include "queue.h" // 提供队列类型
#include "semphr.h" // 提供信号量和互斥锁类型

extern QueueHandle_t qAttitude; // 姿态数据队列
extern QueueHandle_t qGnss; // GNSS 数据队列
extern QueueHandle_t qMqttPublish; // MQTT 发布消息队列
extern SemaphoreHandle_t mutexUart6log; // 调试串口日志递归互斥锁
extern EventGroupHandle_t gSystemEventGroup; // 系统状态事件组

/**
 * @brief 创建系统使用的 FreeRTOS 队列、互斥量和事件组。
 * @retval None
 */
void FreeRTOS_ObjectsCreate(void); // 创建系统 FreeRTOS 对象

#endif // __FREERTOS_OBJECTS_H__
