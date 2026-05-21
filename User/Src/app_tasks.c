#include "app_tasks.h" // 提供应用任务创建接口和任务依赖头文件

/* 任务句柄只在本文件内使用，便于后续调试或扩展任务控制。 */
static TaskHandle_t InitTaskHandle = NULL; // 初始化任务句柄
static TaskHandle_t IMUTaskHandle = NULL; // IMU 任务句柄
static TaskHandle_t ModemTaskHandle = NULL; // 通信任务句柄
static TaskHandle_t TelemetryTaskHandle = NULL; // 遥测任务句柄
static TaskHandle_t LedTaskHandle = NULL; // LED 任务句柄

/**
 * @brief 完成 LED、调试串口、IMU 等系统初始化。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void InitTask(void *argument); // 初始化任务入口

/**
 * @brief 周期读取 IMU 数据、执行姿态融合并更新姿态队列。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void ImuTask(void *argument); // IMU 采集和姿态融合任务入口

/**
 * @brief 周期构造 GNSS 数据并处理 MQTT 发布队列。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void ModemTask(void *argument); // 通信任务入口

/**
 * @brief 读取姿态和 GNSS 队列并组装遥测消息。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void TelemetryTask(void *argument); // 遥测组包任务入口

/**
 * @brief 根据系统状态周期翻转 LED 指示运行状态。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void LedTask(void *argument); // LED 状态指示任务入口

/**
 * @brief 创建应用层所有 FreeRTOS 任务。
 * @retval None
 */
void APP_TasksCreate(void) // 创建应用层任务
{
    xTaskCreate(InitTask, // 创建初始化任务
                "InitTask", // 任务名称用于调试识别
                APP_TASK_INIT_STACK_SIZE, // 初始化任务栈大小
                NULL, // 初始化任务不需要入口参数
                APP_TASK_INIT_PRIORITY, // 初始化任务优先级
                &InitTaskHandle); // 保存初始化任务句柄

    xTaskCreate(ImuTask, // 创建 IMU 任务
                "ImuTask", // 任务名称用于调试识别
                APP_TASK_IMU_STACK_SIZE, // IMU 任务栈大小
                NULL, // IMU 任务不需要入口参数
                APP_TASK_IMU_PRIORITY, // IMU 任务优先级
                &IMUTaskHandle); // 保存 IMU 任务句柄

    xTaskCreate(ModemTask, // 创建通信任务
                "ModemTask", // 任务名称用于调试识别
                APP_TASK_MODEM_STACK_SIZE, // 通信任务栈大小
                NULL, // 通信任务不需要入口参数
                APP_TASK_MODEM_PRIORITY, // 通信任务优先级
                &ModemTaskHandle); // 保存通信任务句柄

    xTaskCreate(TelemetryTask, // 创建遥测任务
                "TelemetryTask", // 任务名称用于调试识别
                APP_TASK_TELEMETRY_STACK_SIZE, // 遥测任务栈大小
                NULL, // 遥测任务不需要入口参数
                APP_TASK_TELEMETRY_PRIORITY, // 遥测任务优先级
                &TelemetryTaskHandle); // 保存遥测任务句柄

    xTaskCreate(LedTask, // 创建 LED 任务
                "LedTask", // 任务名称用于调试识别
                APP_TASK_LED_STACK_SIZE, // LED 任务栈大小
                NULL, // LED 任务不需要入口参数
                APP_TASK_LED_PRIORITY, // LED 任务优先级
                &LedTaskHandle); // 保存 LED 任务句柄
}

/**
 * @brief 一次性完成系统启动初始化。
 *
 * 主要做四件事：
 * 1. 初始化 LED 和调试服务；
 * 2. 初始化 IMU，并根据结果更新系统状态；
 * 3. 预置 GNSS、MQTT、网络状态；
 * 4. 删除自身，释放初始化任务资源。
 *
 * @param argument FreeRTOS 任务入口参数，当前未使用。
 * @retval None
 */
static void InitTask(void *argument)
{
    uint8_t imu_ret; // IMU 初始化结果

    (void)argument; // 当前不使用任务参数

    /* 1. 基础服务初始化 */
    LedService_Init(); // 初始化 LED 指示灯服务
    DebugService_Init(); // 初始化调试串口服务

    /* 2. IMU 初始化和姿态融合初始化 */
    imu_ret = ImuService_Init(); // 初始化 IMU 驱动和校准流程
    if (imu_ret == 1) // IMU 初始化成功
    {
        AppStatus_Set(APP_STATUS_IMU_READY); // 标记 IMU 可用
        MPU9250_MahonyInit(0.3f, 0.0f); // 初始化 Mahony 姿态融合参数
    }
    else // IMU 初始化失败
    {
        AppStatus_Set(APP_STATUS_IMU_ERROR); // 标记 IMU 异常
    }

    /* 3. 当前阶段先置位其他业务状态 */
    AppStatus_Set(APP_STATUS_GNSS_READY); // 当前阶段默认 GNSS 服务可用
    AppStatus_Set(APP_STATUS_MQTT_READY); // 当前阶段默认 MQTT 服务可用
    AppStatus_Set(APP_STATUS_NET_READY); // 当前阶段默认网络服务可用

    /* 4. InitTask 只运行一次，初始化完成后删除自身 */
    vTaskDelete(NULL); // 删除当前初始化任务
}

/**
 * @brief IMU 周期任务：采样传感器、更新 Mahony 融合，并发布最新姿态。
 *
 * 这段任务主要看五件事：
 * 1. 先等 InitTask 把 APP_STATUS_IMU_READY 置位；
 * 2. ImuService_ReadPhys() 读取六轴物理量，并尽量读取 AK8963 磁场；
 * 3. 磁力计有效时走 9 轴 Mahony，磁力计无效时退回 6 轴 Mahony；
 * 4. qAttitude 只写入 MPU9250_GetEulerFusedDeg() 取出的融合姿态；
 * 5. vTaskDelayUntil() 用固定唤醒点维持 IMU 任务周期。
 *
 * @param argument FreeRTOS 任务入口参数，当前未使用。
 * @retval None
 */
static void ImuTask(void *argument) // IMU 采样、融合和姿态队列更新任务
{
    TickType_t lastWakeTime; // vTaskDelayUntil 使用的周期基准 tick
    AttitudeData_t attitude; // 即将覆盖写入 qAttitude 的最新姿态
    MPU9250_Physical_Data phys = {0}; // MPU9250 加速度计、陀螺仪物理量
    AK8963_Physical_Data mag_physical = {0}; // AK8963 磁场物理量，9 轴融合时使用
    uint8_t imu_read_ok = 0; // 本轮读取结果：失败、6 轴有效或 9 轴有效
    const float imu_dt = (float)APP_IMU_TASK_PERIOD_MS * 0.001f; // Mahony 融合步长，单位秒
    float roll_deg = 0.0f; // 融合输出横滚角
    float pitch_deg = 0.0f; // 融合输出俯仰角
    float yaw_deg = 0.0f; // 融合输出航向角

    (void)argument; // 当前不使用任务参数

    lastWakeTime = xTaskGetTickCount(); // 记录周期任务初始唤醒点

    while (1) // IMU 任务常驻运行
    {
        /* 1. 等待初始化阶段确认 IMU 可用 */
        if (AppStatus_IsSet(APP_STATUS_IMU_READY) == 0) // IMU 尚未就绪
        {
            vTaskDelay(pdMS_TO_TICKS(100)); // 降低异常状态下的轮询频率
            continue; // 等待下一轮检查
        }

        /* 2. 读取本轮传感器物理量，失败时不刷新姿态队列 */
        imu_read_ok = ImuService_ReadPhys(&phys, &mag_physical); // 读取六轴和可选磁力计数据
        if (imu_read_ok == IMU_SERVICE_READ_FAIL) // 本轮 IMU 读取失败
        {
            vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(APP_IMU_TASK_PERIOD_MS)); // 保持 IMU 任务周期
            continue; // 跳过本轮姿态更新
        }

        /* 3. 根据磁力计是否有效选择 9 轴或 6 轴融合路径 */
        if (imu_read_ok == IMU_SERVICE_READ_9AXIS_OK) // 六轴和磁力计均有效
        {
            MPU9250_MahonyUpdate(&phys, &mag_physical, imu_dt); // 使用九轴数据约束姿态和航向
        }
        else // 只有六轴数据有效
        {
            MPU9250_MahonyUpdateIMU(&phys, imu_dt); // 退回六轴融合，航向不由磁力计修正
        }

        /* 4. 从融合结果取欧拉角，并覆盖发布到姿态队列 */
        MPU9250_GetEulerFusedDeg(&roll_deg, &pitch_deg, &yaw_deg); // 读取融合后的欧拉角
            attitude.roll_deg = roll_deg; // 写入横滚角
            attitude.pitch_deg = pitch_deg; // 写入俯仰角  
            attitude.yaw_deg = yaw_deg; // 写入航向角
            attitude.timestamp_ms = xTaskGetTickCount(); // 写入当前 tick 时间戳
            attitude.valid = 1U; // 标记姿态数据有效

        xQueueOverwrite(qAttitude, &attitude); // 覆盖姿态队列中的旧数据

        /* 5. 使用固定唤醒点维持配置的 IMU 周期 */
        vTaskDelayUntil(&lastWakeTime,pdMS_TO_TICKS(APP_IMU_TASK_PERIOD_MS)); // 周期由 APP_IMU_TASK_PERIOD_MS 统一配置
    }
}

/**
 * @brief 周期构造 GNSS 数据并处理 MQTT 发布队列。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void ModemTask(void *argument) // 通信任务，当前使用模拟 GNSS 和占位发布接口
{
    GnssData_t gnss; // GNSS 定位数据
    MqttPublishMsg_t mqtt_msg; // 待发布 MQTT 消息

    (void)argument; // 当前不使用任务参数

    memset(&gnss, 0, sizeof(gnss)); // 清空 GNSS 数据缓存
    memset(&mqtt_msg, 0, sizeof(mqtt_msg)); // 清空 MQTT 消息缓存

    while (1) // 通信任务常驻运行
    {
        ModemService_BuildSimGnss(&gnss); // 当前阶段生成模拟 GNSS 数据

        if (gnss.fix_valid) // GNSS 当前有有效定位
        {
            AppStatus_Set(APP_STATUS_GNSS_FIX); // 置位定位有效状态
        }
        else // GNSS 当前无有效定位
        {
            AppStatus_Clear(APP_STATUS_GNSS_FIX); // 清除定位有效状态
        }

        xQueueOverwrite(qGnss, &gnss); // 覆盖 GNSS 队列中的旧数据

        if (xQueueReceive(qMqttPublish, &mqtt_msg, 0) == pdPASS) // 检查是否有待发布遥测消息
        {
            ModemService_Publish(&mqtt_msg); // 处理一条 MQTT 发布消息
        }

        vTaskDelay(pdMS_TO_TICKS(APP_MODEM_TASK_PERIOD_MS)); // 按通信任务周期休眠
    }
}

/**
 * @brief 读取姿态和 GNSS 队列并组装遥测消息。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void TelemetryTask(void *argument) // 遥测组包任务
{
    AttitudeData_t attitude; // 最新姿态数据
    GnssData_t gnss; // 最新 GNSS 数据
    MqttPublishMsg_t mqtt_msg; // 组装后的 MQTT 消息
    char log_buf[256]; // 遥测调试日志缓存

    (void)argument; // 当前不使用任务参数

    memset(&attitude, 0, sizeof(attitude)); // 清空姿态缓存
    memset(&gnss, 0, sizeof(gnss)); // 清空 GNSS 缓存
    memset(&mqtt_msg, 0, sizeof(mqtt_msg)); // 清空 MQTT 消息缓存

    while (1) // 遥测任务常驻运行
    {
        xQueuePeek(qAttitude, &attitude, 0); // 读取最新姿态但不移除队列内容
        xQueuePeek(qGnss, &gnss, 0); // 读取最新 GNSS 但不移除队列内容

        Telemetry_BuildMqttMsg(&attitude, &gnss, &mqtt_msg); // 按当前姿态和定位组装遥测消息

        xQueueSend(qMqttPublish, &mqtt_msg, 0); // 将遥测消息发送给通信任务

        snprintf(log_buf, // 写入遥测调试字符串
                 sizeof(log_buf), // 限制日志缓冲区长度
                 "[TelemetryTask] roll=%.1f pitch=%.1f yaw=%.1f lat=%.6f lon=%.6f\r\n", // 遥测日志格式
                 attitude.roll_deg, // 输出横滚角
                 attitude.pitch_deg, // 输出俯仰角
                 attitude.yaw_deg, // 输出航向角
                 gnss.latitude, // 输出纬度
                 gnss.longitude); // 输出经度

        Debug_Print(log_buf); // 需要观察遥测状态时打开
        (void)log_buf; // 当前默认不输出遥测日志

        vTaskDelay(pdMS_TO_TICKS(APP_TELEMETRY_TASK_PERIOD_MS)); // 按遥测任务周期休眠
    }
}

/**
 * @brief 根据系统状态周期翻转 LED 指示运行状态。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void LedTask(void *argument) // LED 状态指示任务
{
    (void)argument; // 当前不使用任务参数

    while (1) // LED 任务常驻运行
    {
        LedService_Toggle(); // 翻转 LED 输出状态

        if (AppStatus_IsSet(APP_STATUS_GNSS_FIX) && AppStatus_IsSet(APP_STATUS_MQTT_READY)) // 定位和 MQTT 都正常
        {
            vTaskDelay(pdMS_TO_TICKS(APP_LED_TASK_PERIOD_MS)); // 使用正常状态闪烁周期
        }
        else // 关键业务状态未满足
        {
            vTaskDelay(pdMS_TO_TICKS(100)); // 使用快速闪烁提示异常或未就绪
        }
    }
}
