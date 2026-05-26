#include "app_tasks.h" // 提供应用任务创建接口和任务依赖头文件

/* 任务句柄只在本文件内使用，便于后续调试或扩展任务控制。 */
static TaskHandle_t InitTaskHandle = NULL; // 初始化任务句柄
#if APP_ENABLE_IMU
static TaskHandle_t IMUTaskHandle = NULL; // IMU 任务句柄
#endif
static TaskHandle_t ModemTaskHandle = NULL; // 通信任务句柄
static TaskHandle_t TelemetryTaskHandle = NULL; // 遥测任务句柄
static TaskHandle_t LedTaskHandle = NULL; // LED 任务句柄

/**
 * @brief 完成 LED、调试串口、A7670E 串口、IMU 等系统初始化。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void InitTask(void *argument); // 初始化任务入口

/**
 * @brief 周期读取 IMU 数据、执行姿态融合并更新姿态队列。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
#if APP_ENABLE_IMU
static void ImuTask(void *argument); // IMU 采集和姿态融合任务入口
#endif

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

#if APP_ENABLE_IMU
    xTaskCreate(ImuTask, // 创建 IMU 任务
                "ImuTask", // 任务名称用于调试识别
                APP_TASK_IMU_STACK_SIZE, // IMU 任务栈大小
                NULL, // IMU 任务不需要入口参数
                APP_TASK_IMU_PRIORITY, // IMU 任务优先级
                &IMUTaskHandle); // 保存 IMU 任务句柄
#endif

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
 * 1. 初始化 LED、调试服务和 A7670E 串口；
 * 2. 初始化 IMU，并根据结果更新系统状态；
 * 3. 预置 GNSS、MQTT、网络状态；
 * 4. 删除自身，释放初始化任务资源。
 *
 * @param argument FreeRTOS 任务入口参数，当前未使用。
 * @retval None
 */
static void InitTask(void *argument)
{
#if APP_ENABLE_IMU
    uint8_t imu_ret; // IMU 初始化结果
#endif

    (void)argument; // 当前不使用任务参数

    /* 1. 基础服务初始化 */
    LedService_Init(); // 初始化 LED 指示灯服务
    DebugService_Init(); // 初始化调试串口服务
    BSP_A7670E_Uart_Init(); // 初始化 A7670E 使用的 USART1 PA9/PA10

#if APP_ENABLE_IMU
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
#else
    /* 2. IMU 已通过 APP_ENABLE_IMU 关闭 */
    AppStatus_Clear(APP_STATUS_IMU_READY); // 保持 IMU 未就绪，遥测只使用 GNSS 数据
#endif

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
#if APP_ENABLE_IMU
static void ImuTask(void *argument) // IMU 采样、融合和姿态队列更新任务
{
    TickType_t lastWakeTime; // vTaskDelayUntil 使用的周期基准 tick
    AttitudeData_t attitude; // 即将覆盖写入 qAttitude 的最新姿态
    MPU9250_Physical_Data phys = {0}; // MPU9250 加速度计、陀螺仪物理量
    AK8963_Physical_Data mag_physical = {0}; // AK8963 磁场物理量，9 轴融合时使用
    uint8_t imu_read_ok = 0; // 本轮读取结果：失败、6 轴有效或 9 轴有效
    uint8_t imu_started = 0U; // 标记 IMU 周期任务是否已经正式开始
    BaseType_t period_blocked = pdFALSE; // 记录周期延时是否真正阻塞
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
            imu_started = 0U; // IMU 未正式开始周期运行
            vTaskDelay(pdMS_TO_TICKS(100)); // 降低异常状态下的轮询频率
            continue; // 等待下一轮检查
        }

        if (imu_started == 0U) // 第一次检测到 IMU ready
        {
            lastWakeTime = xTaskGetTickCount(); // 重置周期基准为当前时间
            imu_started = 1U; // 标记 IMU 已经正式开始
        }

        /* 2. 读取本轮传感器物理量，失败时不刷新姿态队列 */
        imu_read_ok = ImuService_ReadPhys(&phys, &mag_physical); // 读取六轴和可选磁力计数据
        if (imu_read_ok == IMU_SERVICE_READ_FAIL) // 本轮 IMU 读取失败
        {
            period_blocked = xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(APP_IMU_TASK_PERIOD_MS)); // 保持 IMU 任务周期
            if (period_blocked == pdFALSE) // 本轮已经超过配置周期
            {
                lastWakeTime = xTaskGetTickCount(); // 重新对齐下一轮周期基准
                vTaskDelay(pdMS_TO_TICKS(1)); // 主动让低优先级任务获得运行机会
            }
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
        period_blocked = xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(APP_IMU_TASK_PERIOD_MS)); // 周期由 APP_IMU_TASK_PERIOD_MS 统一配置
        if (period_blocked == pdFALSE) // 软件 I2C 读数耗时超过 20 ms
        {
            lastWakeTime = xTaskGetTickCount(); // 避免后续一直追赶旧唤醒点
            vTaskDelay(pdMS_TO_TICKS(1)); // 防止高优先级 IMU 任务饿死串口和 LED 任务
        }
    }
}
#endif

/**
 * @brief 周期构造 GNSS 数据并处理 MQTT 发布队列。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void ModemTask(void *argument) // 通信任务，当前使用模拟 GNSS 和占位发布接口
{
    GnssData_t gnss; // GNSS 定位数据
    MqttPublishMsg_t mqtt_msg; // 待发布 MQTT 消息
    TickType_t last_at_tick; // 上一次发送 AT 测试命令的 tick
    TickType_t now_tick; // 当前任务循环的 tick
    TickType_t at_rx_start_tick; // AT 测试快速接收窗口起点 tick
    uint8_t ch; // A7670E 返回的单个字符

    (void)argument; // 当前不使用任务参数

    memset(&gnss, 0, sizeof(gnss)); // 清空 GNSS 数据缓存
    memset(&mqtt_msg, 0, sizeof(mqtt_msg)); // 清空 MQTT 消息缓存
    last_at_tick = xTaskGetTickCount(); // 初始化 AT 测试命令发送节拍

    while (1) // 通信任务常驻运行
    {
        now_tick = xTaskGetTickCount(); // 读取当前 FreeRTOS tick
        if ((now_tick - last_at_tick) >= pdMS_TO_TICKS(3000)) // 每 3000 ms 发送一次 AT 测试命令
        {
            Debug_Print("[A7670E] TX: AT\\r\\n\r\n"); // 通过 USART6 提示本轮已发送 AT
            BSP_A7670E_Uart_SendString("AT\r\n"); // 通过 USART1 发送 AT 指令到 A7670E
            last_at_tick = now_tick; // 更新最近一次 AT 发送时间
            at_rx_start_tick = xTaskGetTickCount(); // 记录 AT 发送后的快速接收窗口起点
            while ((xTaskGetTickCount() - at_rx_start_tick) < pdMS_TO_TICKS(300)) // 在 300 ms 窗口内快速读取返回
            {
                while (BSP_A7670E_Uart_ReceiveByte(&ch) == 1U) // 读取当前已到达的 A7670E 返回字节
                {
                    BSP_DebugUart_SendChar((char)ch); // 将 A7670E 返回字符转发到 USART6 调试串口
                }
            }
        }

        while (BSP_A7670E_Uart_ReceiveByte(&ch) == 1U) // 非阻塞读取当前已到达的 A7670E 返回字节
        {
            BSP_DebugUart_SendChar((char)ch); // 将 A7670E 返回字符转发到 USART6 调试串口
        }

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
    uint8_t att_valid; // 本轮姿态数据是否有效
    uint8_t gnss_valid; // 本轮 GNSS 数据是否有效

    (void)argument; // 当前不使用任务参数

    memset(&attitude, 0, sizeof(attitude)); // 清空姿态缓存
    memset(&gnss, 0, sizeof(gnss)); // 清空 GNSS 缓存
    memset(&mqtt_msg, 0, sizeof(mqtt_msg)); // 清空 MQTT 消息缓存

    while (1) // 遥测任务常驻运行
    {
        att_valid = 0U; // 默认本轮没有有效姿态
        gnss_valid = 0U; // 默认本轮没有有效 GNSS

#if APP_ENABLE_IMU
        if ((AppStatus_IsSet(APP_STATUS_IMU_READY) != 0) &&
            (xQueuePeek(qAttitude, &attitude, 0) == pdPASS) &&
            (attitude.valid != 0U)) // IMU 就绪且队列中有有效姿态
        {
            att_valid = 1U; // 标记姿态数据可用于遥测
        }
        else // 姿态不可用
        {
            memset(&attitude, 0, sizeof(attitude)); // 清空姿态，避免沿用旧值
        }
#else
        memset(&attitude, 0, sizeof(attitude)); // IMU 关闭时只保留 GNSS 遥测
#endif

        if ((xQueuePeek(qGnss, &gnss, 0) == pdPASS) &&
            (gnss.fix_valid != 0U)) // 队列中有有效 GNSS
        {
            gnss_valid = 1U; // 标记 GNSS 数据可用于遥测
        }
        else // GNSS 不可用
        {
            memset(&gnss, 0, sizeof(gnss)); // 清空 GNSS，避免沿用旧值
        }

        if ((att_valid == 0U) && (gnss_valid == 0U)) // 两类遥测源都无效
        {
            vTaskDelay(pdMS_TO_TICKS(APP_TELEMETRY_TASK_PERIOD_MS)); // 等待下一轮遥测周期
            continue; // 本轮不组装空遥测数据
        }

        Telemetry_BuildMqttMsg(&attitude, &gnss, &mqtt_msg); // 按当前姿态和定位组装遥测消息

        xQueueSend(qMqttPublish, &mqtt_msg, 0); // 将遥测消息发送给通信任务

        snprintf(log_buf, // 写入遥测调试字符串
                 sizeof(log_buf), // 限制日志缓冲区长度
                 "[TelemetryTask] att_valid=%u gnss_valid=%u roll=%.1f pitch=%.1f yaw=%.1f lat=%.6f lon=%.6f\r\n", // 遥测日志格式
                 att_valid, // 输出姿态有效标志
                 gnss_valid, // 输出 GNSS 有效标志
                 attitude.roll_deg, // 输出横滚角
                 attitude.pitch_deg, // 输出俯仰角
                 attitude.yaw_deg, // 输出航向角
                 gnss.latitude, // 输出纬度
                 gnss.longitude); // 输出经度

        // Debug_Print(log_buf); // 需要观察遥测状态时打开
        (void)log_buf; // 当前默认不输出遥测日志

        vTaskDelay(pdMS_TO_TICKS(APP_TELEMETRY_TASK_PERIOD_MS)); // 按遥测任务周期休眠
    }
}

/**
 * @brief 根据校准和运行状态显示红绿 LED 灯语。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void LedTask(void *argument) // LED 状态指示任务
{
    AppCalState_t cal_state; // 当前校准灯语状态
    AppCalState_t last_cal_state = APP_CAL_STATE_IDLE; // 上一轮校准灯语状态
    uint8_t self_check_done = 0U; // 上电自检闪烁是否已执行

    (void)argument; // 当前不使用任务参数

    while (1) // LED 任务常驻运行
    {
        cal_state = AppStatus_GetCalState(); // 读取当前校准状态
        if (cal_state != last_cal_state) // 状态发生切换
        {
            self_check_done = 0U; // 允许新状态重新执行一次性提示
            last_cal_state = cal_state; // 记录最新状态
        }

        switch (cal_state) // 根据校准阶段选择灯语
        {
            case APP_CAL_STATE_SELF_CHECK: // 上电自检
                if (self_check_done == 0U) // 只执行一次自检闪烁
                {
                    LedService_Set(1U, 1U); // 红绿同时点亮
                    vTaskDelay(pdMS_TO_TICKS(100)); // 保持短亮
                    LedService_Set(0U, 0U); // 红绿同时熄灭
                    self_check_done = 1U; // 标记自检闪烁已完成
                }
                vTaskDelay(pdMS_TO_TICKS(100)); // 自检状态保持短周期轮询
                break;

            case APP_CAL_STATE_STATIC: // 陀螺仪和加速度计静止校准
                LedService_Set(0U, 1U); // 绿灯点亮
                vTaskDelay(pdMS_TO_TICKS(500)); // 慢闪亮半周期
                LedService_Set(0U, 0U); // 绿灯熄灭
                vTaskDelay(pdMS_TO_TICKS(500)); // 慢闪灭半周期
                break;

            case APP_CAL_STATE_MAG_ROTATE: // 磁力计旋转校准
                LedService_Set(0U, 1U); // 绿灯点亮
                vTaskDelay(pdMS_TO_TICKS(150)); // 快闪亮半周期
                LedService_Set(0U, 0U); // 绿灯熄灭
                vTaskDelay(pdMS_TO_TICKS(150)); // 快闪灭半周期
                break;

            case APP_CAL_STATE_SUCCESS: // 标定成功
                LedService_Set(0U, 1U); // 绿灯常亮
                vTaskDelay(pdMS_TO_TICKS(3000)); // 成功提示保持 3 秒
                AppStatus_SetCalState(APP_CAL_STATE_IDLE); // 成功提示结束后回到正常运行灯语
                break;

            case APP_CAL_STATE_FAIL: // 标定失败
                LedService_Set(1U, 0U); // 红灯点亮
                vTaskDelay(pdMS_TO_TICKS(150)); // 快闪亮半周期
                LedService_Set(0U, 0U); // 红灯熄灭
                vTaskDelay(pdMS_TO_TICKS(150)); // 快闪灭半周期
                break;

            case APP_CAL_STATE_IDLE: // 正常运行
            default: // 未知状态按正常运行处理
                LedService_Set(0U, 1U); // 绿灯短亮
                vTaskDelay(pdMS_TO_TICKS(100)); // 正常运行闪烁亮宽
                LedService_Set(0U, 0U); // 绿灯熄灭
                vTaskDelay(pdMS_TO_TICKS(1900)); // 正常运行每 2 秒闪一下
                break;
        }
    }
}
