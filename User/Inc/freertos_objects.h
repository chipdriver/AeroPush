#ifndef __FREERTOS_OBJECTS_H__ // 检查 __FREERTOS_OBJECTS_H__ 是否未定义，防止头文件重复包含
#define __FREERTOS_OBJECTS_H__ // 定义 __FREERTOS_OBJECTS_H__ 变量

#include "FreeRTOS.h" // 引入 FreeRTOS.h 提供的接口、宏和类型定义
#include "app_types.h" // 引入 app_types.h 提供的接口、宏和类型定义
#include "event_groups.h" // 引入 event_groups.h 提供的接口、宏和类型定义
#include "queue.h" // 引入 queue.h 提供的接口、宏和类型定义
#include "semphr.h" // 引入 semphr.h 提供的接口、宏和类型定义

extern QueueHandle_t qAttitude; // 声明 姿态数据队列，供后续计算、状态保存或模块间传递使用
extern QueueHandle_t qGnss; // 声明 GNSS 数据队列，供后续计算、状态保存或模块间传递使用
extern QueueHandle_t qMqttPublish; // 声明 MQTT 发布消息队列，供后续计算、状态保存或模块间传递使用
extern SemaphoreHandle_t mutexUart6log; // 声明 mutexUart6log 变量，供后续计算、状态保存或模块间传递使用
extern EventGroupHandle_t gSystemEventGroup; // 声明 gSystemEventGroup 变量，供后续计算、状态保存或模块间传递使用

/**
 * @brief 创建系统使用的 FreeRTOS 队列、互斥量和事件组。
 * @retval None
 */
void FreeRTOS_ObjectsCreate(void); // 声明FreeRTOS_ObjectsCreate 函数签名：创建系统使用的 FreeRTOS 队列、互斥量和事件组

#endif // 结束当前条件编译或头文件保护范围
