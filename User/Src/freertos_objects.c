#include "freertos_objects.h" // 提供全局 FreeRTOS 对象声明

QueueHandle_t qAttitude = NULL; // 姿态数据队列
QueueHandle_t qGnss = NULL; // GNSS 数据队列
QueueHandle_t qMqttPublish = NULL; // MQTT 发布消息队列

SemaphoreHandle_t mutexUart6log = NULL; // 调试串口日志递归互斥锁

EventGroupHandle_t gSystemEventGroup = NULL; // 系统状态事件组

/**
 * @brief 创建系统使用的 FreeRTOS 队列、互斥量和事件组。
 * @retval None
 */
void FreeRTOS_ObjectsCreate(void) // 创建系统 FreeRTOS 对象
{
    qAttitude = xQueueCreate(1, sizeof(AttitudeData_t)); // 创建只保存最新姿态的队列

    qGnss = xQueueCreate(1, sizeof(GnssData_t)); // 创建只保存最新 GNSS 的队列

    qMqttPublish = xQueueCreate(4, sizeof(MqttPublishMsg_t)); // 创建遥测发布消息队列

    mutexUart6log = xSemaphoreCreateRecursiveMutex(); // 创建调试串口日志互斥锁

    gSystemEventGroup = xEventGroupCreate(); // 创建系统状态事件组
}
