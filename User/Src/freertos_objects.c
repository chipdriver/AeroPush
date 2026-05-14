#include "freertos_objects.h" // 引入 FreeRTOS 对象模块接口

QueueHandle_t qAttitude = NULL;    // 定义姿态队列句柄，初始化为空
QueueHandle_t qGnss = NULL;        // 定义 GNSS 队列句柄，初始化为空
QueueHandle_t qMqttPublish = NULL; // 定义 MQTT 发布队列句柄，初始化为空

SemaphoreHandle_t mutexUart6log = NULL; // 定义串口日志互斥锁句柄，初始化为空

EventGroupHandle_t gSystemEventGroup = NULL; // 定义系统事件组句柄，初始化为空

/**
 * @brief  创建 FreeRTOS 队列、互斥锁和事件组等对象。
 * @param  None
 * @retval None
 */
void FreeRTOS_ObjectsCreate(void) // 定义 FreeRTOS 对象创建函数
{                                 // FreeRTOS_ObjectsCreate 函数体开始
    /*
     *   qAttitude 长度为 1;
     *   只保存最新姿态数据,旧数据可以被覆盖
     */
    qAttitude = xQueueCreate(1, sizeof(AttitudeData_t)); // 创建姿态队列，队列长度为 1，每个元素大小为 AttitudeData_t

    /*
     *    qGnss 长度为1;
     *    只保存最新定位数据,旧数据可以被覆盖
     */
    qGnss = xQueueCreate(1, sizeof(GnssData_t)); // 创建 GNSS 队列，队列长度为 1，每个元素大小为 GnssData_t

    /*
     *     qMqttPublish 长度为4;
     *     TelemetryTask 生成待发布数据，ModemTask 负责取出并发布
     */
    qMqttPublish = xQueueCreate(4, sizeof(MqttPublishMsg_t)); // 创建 MQTT 发布队列，队列长度为 4，每个元素大小为 MqttPublishMsg_t

    /*
     *     调试串口互斥锁;
     *     后续用于避免多个任务同时打印导致串口内容混乱
     */
    mutexUart6log = xSemaphoreCreateRecursiveMutex(); // 创建串口日志互斥锁，用于保护多任务串口打印

    /*  统一配置中心
     *   统一记录 IMU、GNSS、MQTT、错误状态
     */
    gSystemEventGroup = xEventGroupCreate(); // 创建系统事件组，用来保存系统状态位
} // FreeRTOS_ObjectsCreate 函数体结束
