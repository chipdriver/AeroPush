#include "freertos_objects.h" // 引入 freertos_objects.h 提供的接口、宏和类型定义

QueueHandle_t qAttitude = NULL; // 定义 姿态数据队列，初始值设置为 空指针
QueueHandle_t qGnss = NULL; // 定义 GNSS 数据队列，初始值设置为 空指针
QueueHandle_t qMqttPublish = NULL; // 定义 MQTT 发布消息队列，初始值设置为 空指针

SemaphoreHandle_t mutexUart6log = NULL; // 定义 mutexUart6log 变量，初始值设置为 空指针

EventGroupHandle_t gSystemEventGroup = NULL; // 定义 gSystemEventGroup 变量，初始值设置为 空指针

/**
 * @brief 创建系统使用的 FreeRTOS 队列、互斥量和事件组。
 * @retval None
 */
void FreeRTOS_ObjectsCreate(void) // 定义FreeRTOS_ObjectsCreate 函数签名：创建系统使用的 FreeRTOS 队列、互斥量和事件组
{ // 进入当前代码块
    /*
     *   qAttitude 长度为 1;
     *   只保存最新姿态数据,旧数据可以被覆盖
     */
    qAttitude = xQueueCreate(1, sizeof(AttitudeData_t)); // 把 xQueueCreate(1, sizeof(AttitudeData_t)) 写入 姿态数据队列

    /*
     *    qGnss 长度为1;
     *    只保存最新定位数据,旧数据可以被覆盖
     */
    qGnss = xQueueCreate(1, sizeof(GnssData_t)); // 把 xQueueCreate(1, sizeof(GnssData_t)) 写入 GNSS 数据队列

    /*
     *     qMqttPublish 长度为4;
     *     TelemetryTask 生成待发布数据，ModemTask 负责取出并发布
     */
    qMqttPublish = xQueueCreate(4, sizeof(MqttPublishMsg_t)); // 把 xQueueCreate(4, sizeof(MqttPublishMsg_t)) 写入 MQTT 发布消息队列

    /*
     *     调试串口互斥锁;
     *     后续用于避免多个任务同时打印导致串口内容混乱
     */
    mutexUart6log = xSemaphoreCreateRecursiveMutex(); // 把 xSemaphoreCreateRecursiveMutex() 写入 mutexUart6log 变量

    /*  统一配置中心
     *   统一记录 IMU、GNSS、MQTT、错误状态
     */
    gSystemEventGroup = xEventGroupCreate(); // 把 xEventGroupCreate() 写入 gSystemEventGroup 变量
} // 结束当前代码块
