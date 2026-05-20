#include "app_tasks.h" // 引入 app_tasks.h 提供的接口、宏和类型定义

/*任务句柄*/
/*
 *  ①TaskHandle_t 是FreeRTos中的任务句柄类型
 *  任务句柄可以理解为“任务的管理编号”，后续如果要挂起、恢复、删除某个任务，可以通过任务句柄来操作。
 *  ②static表示这些变量只能在这个文件内可见
 */
static TaskHandle_t InitTaskHandle = NULL; // 定义 InitTaskHandle 任务句柄，初始值设置为 空指针
static TaskHandle_t IMUTaskHandle = NULL; // 定义 IMUTaskHandle 任务句柄，初始值设置为 空指针
static TaskHandle_t ModemTaskHandle = NULL; // 定义 ModemTaskHandle 任务句柄，初始值设置为 空指针
static TaskHandle_t TelemetryTaskHandle = NULL; // 定义 TelemetryTaskHandle 任务句柄，初始值设置为 空指针
static TaskHandle_t LedTaskHandle = NULL; // 定义 LedTaskHandle 任务句柄，初始值设置为 空指针

/*任务函数声明*/
/*
 *  ①static表示这个函数只能在这个文件内可见
 *  ②void *argument 是FreeRTOS 任务函数的标准参数形式；即使现在不用这个参数，也要保留这个格式。
 */
/**
 * @brief 完成 LED、调试串口、IMU 等系统初始化。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void InitTask(void *argument); // 声明InitTask 函数签名：完成 LED、调试串口、IMU 等系统初始化

/**
 * @brief 周期读取 IMU 数据、执行姿态融合并更新姿态队列。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void ImuTask(void *argument); // 声明ImuTask 函数签名：周期读取 IMU 数据、执行姿态融合并更新姿态队列

/**
 * @brief 周期构造 GNSS 数据并处理 MQTT 发布队列。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void ModemTask(void *argument); // 声明ModemTask 函数签名：周期构造 GNSS 数据并处理 MQTT 发布队列

/**
 * @brief 读取姿态和 GNSS 队列并组装遥测消息。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void TelemetryTask(void *argument); // 声明TelemetryTask 函数签名：读取姿态和 GNSS 队列并组装遥测消息

/**
 * @brief 根据系统状态周期翻转 LED 指示运行状态。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void LedTask(void *argument); // 声明LedTask 函数签名：根据系统状态周期翻转 LED 指示运行状态

/**
 * @brief 创建应用层所有 FreeRTOS 任务。
 * @retval None
 */
void APP_TasksCreate(void) // 定义APP_TasksCreate 函数签名：创建应用层所有 FreeRTOS 任务
{ // 进入当前代码块
    /*
     *  xTaskCreate()   用于创建一个 FreeRTOS 任务
     *
     *  参数1：任务函数名
     *  参数2：任务名称，主要用于调试查看
     *  参数3：任务栈大小，单位是word，不是byte
     *  参数4：传递给任务函数的参数，这里暂时不用，所以传NULL
     *  参数5：任务优先级，数字越大，优先级越高
     *  参数6：任务句柄地址，用于保存任务句柄
     */
    xTaskCreate(InitTask, // 创建 FreeRTOS 任务并保存任务句柄，参数为 InitTask,
                "InitTask", // 继续传入 "InitTask"，作为当前多行调用或初始化列表的一项
                APP_TASK_INIT_STACK_SIZE, // 继续传入 APP_TASK_INIT_STACK_SIZE 应用配置项，作为当前多行调用或初始化列表的一项
                NULL, // 继续传入 空指针，作为当前多行调用或初始化列表的一项
                APP_TASK_INIT_PRIORITY, // 继续传入 APP_TASK_INIT_PRIORITY 应用配置项，作为当前多行调用或初始化列表的一项
                &InitTaskHandle); // 执行 &InitTaskHandle);，完成当前上下文中的具体处理

    xTaskCreate(ImuTask, // 创建 FreeRTOS 任务并保存任务句柄，参数为 ImuTask,
                "ImuTask", // 继续传入 "ImuTask"，作为当前多行调用或初始化列表的一项
                APP_TASK_IMU_STACK_SIZE, // 继续传入 APP_TASK_IMU_STACK_SIZE 应用配置项，作为当前多行调用或初始化列表的一项
                NULL, // 继续传入 空指针，作为当前多行调用或初始化列表的一项
                APP_TASK_IMU_PRIORITY, // 继续传入 APP_TASK_IMU_PRIORITY 应用配置项，作为当前多行调用或初始化列表的一项
                &IMUTaskHandle); // 执行 &IMUTaskHandle);，完成当前上下文中的具体处理

    xTaskCreate(ModemTask, // 创建 FreeRTOS 任务并保存任务句柄，参数为 ModemTask,
                "ModemTask", // 继续传入 "ModemTask"，作为当前多行调用或初始化列表的一项
                APP_TASK_MODEM_STACK_SIZE, // 继续传入 APP_TASK_MODEM_STACK_SIZE 应用配置项，作为当前多行调用或初始化列表的一项
                NULL, // 继续传入 空指针，作为当前多行调用或初始化列表的一项
                APP_TASK_MODEM_PRIORITY, // 继续传入 APP_TASK_MODEM_PRIORITY 应用配置项，作为当前多行调用或初始化列表的一项
                &ModemTaskHandle); // 执行 &ModemTaskHandle);，完成当前上下文中的具体处理
    xTaskCreate(TelemetryTask, "TelemetryTask", APP_TASK_TELEMETRY_STACK_SIZE, NULL, APP_TASK_TELEMETRY_PRIORITY, &TelemetryTaskHandle); // 创建 FreeRTOS 任务并保存任务句柄，参数为 TelemetryTask, "TelemetryTask", APP_TASK_TELEMETRY_STACK_SIZE, NULL, APP_TASK_TELEMETRY_PRIORITY, &TelemetryTaskHandle
    xTaskCreate(LedTask, "LedTask", APP_TASK_LED_STACK_SIZE, NULL, APP_TASK_LED_PRIORITY, &LedTaskHandle); // 创建 FreeRTOS 任务并保存任务句柄，参数为 LedTask, "LedTask", APP_TASK_LED_STACK_SIZE, NULL, APP_TASK_LED_PRIORITY, &LedTaskHandle
} // 结束当前代码块

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
    uint8_t imu_ret;

    (void)argument;

    /* 1. 基础服务初始化 */
    LedService_Init();      //LED 初始化函数
    DebugService_Init();    //调试串口初始化函数

    /* 2. IMU 初始化和姿态融合初始化 */
    imu_ret = ImuService_Init();
    if (imu_ret == 1)
    {
        AppStatus_Set(APP_STATUS_IMU_READY);
        MPU9250_MahonyInit(0.3f, 0.0f);
    }
    else
    {
        AppStatus_Set(APP_STATUS_IMU_ERROR);
    }

    /* 3. 当前阶段先置位其他业务状态 */
    AppStatus_Set(APP_STATUS_GNSS_READY);
    AppStatus_Set(APP_STATUS_MQTT_READY);
    AppStatus_Set(APP_STATUS_NET_READY);

    /* 4. InitTask 只运行一次，初始化完成后删除自身 */
    vTaskDelete(NULL);
}

/**
 * @brief 周期读取 IMU 数据、执行姿态融合并更新姿态队列。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void ImuTask(void *argument) // 定义ImuTask 函数签名：周期读取 IMU 数据、执行姿态融合并更新姿态队列
{ // 进入当前代码块
    TickType_t lastWakeTime; // 声明 周期任务上一次唤醒的 tick 基准时间，供后续计算、状态保存或模块间传递使用
    (void)argument; // 标记 FreeRTOS 任务入口参数 当前未使用，避免编译器告警
    AttitudeData_t attitude; // 声明 姿态数据结构体，供后续计算、状态保存或模块间传递使用
    //uint32_t print_count = 0; // 定义 串口打印分频计数器，初始值设置为 0
    MPU9250_Physical_Data phys = {0}; // 定义 MPU9250 六轴物理量数据，初始值设置为 全零初始化值
    AK8963_Physical_Data mag_physical = {0}; // 定义 AK8963 磁力计物理量数据，初始值设置为 全零初始化值
    uint8_t imu_read_ok = 0; // 定义 IMU 读取结果标志，初始值设置为 0
    const float imu_dt = (float)APP_IMU_TASK_PERIOD_MS * 0.001f; // 定义 IMU 融合更新周期秒数，初始值设置为 (float)APP_IMU_TASK_PERIOD_MS * 0.001f 字段值
    float roll_deg = 0.0f; // 定义 横滚角角度值，初始值设置为 0.0f 字段值
    float pitch_deg = 0.0f; // 定义 俯仰角角度值，初始值设置为 0.0f 字段值
    float yaw_deg = 0.0f; // 定义 航向角角度值，初始值设置为 0.0f 字段值
    // float mag_roll_deg = 0.0f; // 定义 磁力计直接解算的横滚角角度值，初始值设置为 0.0f 字段值
    // float mag_pitch_deg = 0.0f; // 定义 磁力计直接解算的俯仰角角度值，初始值设置为 0.0f 字段值
    // float mag_yaw_deg = 0.0f; // 定义 磁力计直接解算的航向角角度值，初始值设置为 0.0f 字段值
    /*
     *   xTaskGetTickCount() 用于获取当前 FreeRTOS 系统 tick 计数值。
     *   这里把当前时间保存下来，作为vTaskDelayUntil()   的基准时间
     */
    lastWakeTime = xTaskGetTickCount(); // 把 当前 FreeRTOS tick 计数 写入 周期任务上一次唤醒的 tick 基准时间

    while (1) // 当 1 成立时持续执行循环体
    { // 进入当前代码块

        if (AppStatus_IsSet(APP_STATUS_IMU_READY) == 0) // 判断 AppStatus_IsSet(APP_STATUS_IMU_READY) == 0 是否成立，以选择后续执行路径
        { // 进入当前代码块
            vTaskDelay(pdMS_TO_TICKS(100)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(100)
            continue; // 跳过本轮剩余逻辑，等待下一次循环处理
        } // 结束当前代码块

        imu_read_ok = ImuService_ReadPhys(&phys, &mag_physical); // 把 ImuService_ReadPhys(&phys, &mag_physical) 写入    IMU 读取结果标志
        if (imu_read_ok == IMU_SERVICE_READ_FAIL) // 判断 imu_read_ok == IMU_SERVICE_READ_FAIL 是否成立，以选择后续执行路径
        { // 进入当前代码块
            vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(APP_IMU_TASK_PERIOD_MS)); // 按固定周期延时到下一次任务唤醒点，参数为 &lastWakeTime, pdMS_TO_TICKS(APP_IMU_TASK_PERIOD_MS)
            continue; // 跳过本轮剩余逻辑，等待下一次循环处理
        } // 结束当前代码块

        if (imu_read_ok == IMU_SERVICE_READ_9AXIS_OK) // 判断 imu_read_ok == IMU_SERVICE_READ_9AXIS_OK 是否成立，以选择后续执行路径
        { // 进入当前代码块
            // MPU9250_ComputeEuler_FromAccMag(&phys, &mag_physical); // 调用使用加速度计和磁力计直接计算欧拉角，参数为 &phys, &mag_physical
            // MPU9250_GetEulerDeg(&mag_roll_deg, &mag_pitch_deg, &mag_yaw_deg); // 调用读取加速度计磁力计解算的欧拉角角度值，参数为 &mag_roll_deg, &mag_pitch_deg, &mag_yaw_deg
            MPU9250_MahonyUpdate(&phys, &mag_physical, imu_dt); // 调用使用九轴数据执行 Mahony 姿态融合更新，参数为 &phys, &mag_physical, imu_dt
        } // 结束当前代码块
        else // 处理前面判断条件不成立时的备用逻辑
        { // 进入当前代码块
            MPU9250_MahonyUpdateIMU(&phys, imu_dt); // 调用使用六轴 IMU 数据执行 Mahony 姿态融合更新，参数为 &phys, imu_dt
        } // 结束当前代码块

        MPU9250_GetEulerFusedDeg(&roll_deg, &pitch_deg, &yaw_deg); // 调用读取 Mahony 融合后的欧拉角角度值，参数为 &roll_deg, &pitch_deg, &yaw_deg
            attitude.roll_deg = roll_deg; // 把 横滚角角度值 写入 attitude.roll_deg 字段值
            attitude.pitch_deg = pitch_deg; // 把 俯仰角角度值 写入 attitude.pitch_deg 字段值
            attitude.yaw_deg = yaw_deg; // 把 航向角角度值 写入 attitude.yaw_deg 字段值
            attitude.timestamp_ms = xTaskGetTickCount(); // 把 当前 FreeRTOS tick 计数 写入 attitude.timestamp_ms 字段值
            attitude.valid = 1U; // 把 1U 写入 attitude.valid 字段值
        xQueueOverwrite(qAttitude, &attitude); // 把最新数据写入队列，队列满时覆盖旧数据，参数为 qAttitude, &attitude
        //print_count++; // 将 串口打印分频计数器 自增 1，用于推进计数或索引

        // if (print_count >= 200) // 判断 print_count >= 200 是否成立，以选择后续执行路径
        // { // 进入当前代码块
        //     print_count = 0; // 把 0 写入 串口打印分频计数器

        //     // Debug_Printf("[ImuTask] fused roll=%.1f pitch=%.1f yaw=%.1f mag_yaw=%.1f mag_ok=%u\r\n", // 调用格式化并通过调试串口输出调试信息，参数为 "[ImuTask] fused roll=%.1f pitch=%.1f yaw=%.1f mag_yaw=%.1f mag_ok=%u\r\n",
        //     //              attitude.roll_deg, // 继续传入 attitude.roll_deg 字段值，作为当前多行调用或初始化列表的一项
        //     //              attitude.pitch_deg, // 继续传入 attitude.pitch_deg 字段值，作为当前多行调用或初始化列表的一项
        //     //              attitude.yaw_deg, // 继续传入 attitude.yaw_deg 字段值，作为当前多行调用或初始化列表的一项
        //     //              mag_yaw_deg, // 继续传入 磁力计直接解算的航向角角度值，作为当前多行调用或初始化列表的一项
        //     //              (imu_read_ok == IMU_SERVICE_READ_9AXIS_OK) ? 1U : 0U); // 执行 (imu_read_ok == IMU_SERVICE_READ_9AXIS_OK) ? 1U : 0U);，完成当前上下文中的具体处理
        // } // 结束当前代码块

        /*
         *   vTaskDelayUntil()   用于实现严格周期延时，它与 vTaskDelay() 不一样；
         *   vTaskDelay() 是从“当前时刻”开始延时
         *   vTaskDelayUntil() 是按照“固定时间点”周期执行
         *
         *   APP_IMU_TASK_PERIOD_MS 在 app_config.h 中统一配置
         *   所以这里表示 ImuTask 按配置周期执行一次*/
        vTaskDelayUntil(&lastWakeTime, // 按固定周期延时到下一次任务唤醒点，参数为 &lastWakeTime,
                        pdMS_TO_TICKS(APP_IMU_TASK_PERIOD_MS)); // 调用pdMS_TO_TICKS 函数，参数为 APP_IMU_TASK_PERIOD_MS)
    } // 结束当前代码块
} // 结束当前代码块

/**
 * @brief 周期构造 GNSS 数据并处理 MQTT 发布队列。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void ModemTask(void *argument) // 定义ModemTask 函数签名：周期构造 GNSS 数据并处理 MQTT 发布队列
{ // 进入当前代码块
    GnssData_t gnss; // 声明 GNSS 定位数据结构体，供后续计算、状态保存或模块间传递使用
    MqttPublishMsg_t mqtt_msg; // 声明 MQTT 发布消息结构体，供后续计算、状态保存或模块间传递使用
    (void)argument; // 标记 FreeRTOS 任务入口参数 当前未使用，避免编译器告警

    memset(&gnss, 0, sizeof(gnss)); // 按指定字节值填充目标内存区域，参数为 &gnss, 0, sizeof(gnss)
    memset(&mqtt_msg, 0, sizeof(mqtt_msg)); // 按指定字节值填充目标内存区域，参数为 &mqtt_msg, 0, sizeof(mqtt_msg)

    while (1) // 当 1 成立时持续执行循环体
    { // 进入当前代码块

        /*模拟GNSS数据*/
        ModemService_BuildSimGnss(&gnss); // 调用构造模拟 GNSS 定位数据，参数为 &gnss

        if (gnss.fix_valid) // 判断 gnss.fix_valid 是否成立，以选择后续执行路径
            AppStatus_Set(APP_STATUS_GNSS_FIX); // 调用置位指定系统状态标志，参数为 APP_STATUS_GNSS_FIX
        else // 处理前面判断条件不成立时的备用逻辑
            AppStatus_Clear(APP_STATUS_GNSS_FIX); // 调用清除指定系统状态标志，参数为 APP_STATUS_GNSS_FIX

        xQueueOverwrite(qGnss, &gnss); // 把最新数据写入队列，队列满时覆盖旧数据，参数为 qGnss, &gnss

        /*
         * 2. 处理 MQTT 发布队列
         * 后续这里会替换成真正的 MQTT_Publish。
         */
        if (xQueueReceive(qMqttPublish, &mqtt_msg, 0) == pdPASS) // 判断 xQueueReceive(qMqttPublish, &mqtt_msg, 0) == pdPASS 是否成立，以选择后续执行路径
        { // 进入当前代码块
            ModemService_Publish(&mqtt_msg); // 调用处理一条待发布 MQTT 消息，参数为 &mqtt_msg
        } // 结束当前代码块
        /*
         *   vTaskDelay() 用于让当前任务主动阻塞一段时间
         *   周期在 app_config.h 中统一配置，避免通信任务一直占用CPU
         */
        vTaskDelay(pdMS_TO_TICKS(APP_MODEM_TASK_PERIOD_MS)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(APP_MODEM_TASK_PERIOD_MS)
    } // 结束当前代码块
} // 结束当前代码块


/**
 * @brief 读取姿态和 GNSS 队列并组装遥测消息。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void TelemetryTask(void *argument) // 定义TelemetryTask 函数签名：读取姿态和 GNSS 队列并组装遥测消息
{ // 进入当前代码块
    AttitudeData_t attitude; // 声明 姿态数据结构体，供后续计算、状态保存或模块间传递使用
    GnssData_t gnss; // 声明 GNSS 定位数据结构体，供后续计算、状态保存或模块间传递使用
    MqttPublishMsg_t mqtt_msg; // 声明 MQTT 发布消息结构体，供后续计算、状态保存或模块间传递使用
    char log_buf[256]; // 声明 log_buf 变量，供后续计算、状态保存或模块间传递使用

    (void)argument; // 标记 FreeRTOS 任务入口参数 当前未使用，避免编译器告警

    memset(&attitude, 0, sizeof(attitude)); // 按指定字节值填充目标内存区域，参数为 &attitude, 0, sizeof(attitude)
    memset(&gnss, 0, sizeof(gnss)); // 按指定字节值填充目标内存区域，参数为 &gnss, 0, sizeof(gnss)
    memset(&mqtt_msg, 0, sizeof(mqtt_msg)); // 按指定字节值填充目标内存区域，参数为 &mqtt_msg, 0, sizeof(mqtt_msg)

    while (1) // 当 1 成立时持续执行循环体
    { // 进入当前代码块

        /* 1.获取最新姿态数据 */
        xQueuePeek(qAttitude, &attitude, 0); // 从队列读取最新数据但不移除内容，参数为 qAttitude, &attitude, 0

        /* 2. 获取最新 GNSS 数据 */
        xQueuePeek(qGnss, &gnss, 0); // 从队列读取最新数据但不移除内容，参数为 qGnss, &gnss, 0

        /* 3. 组装JSON数据*/
        Telemetry_BuildMqttMsg(&attitude, &gnss, &mqtt_msg); // 调用根据姿态和 GNSS 数据构造 MQTT 遥测消息，参数为 &attitude, &gnss, &mqtt_msg

        /* 4.发送给ModemTask */
        xQueueSend(qMqttPublish, &mqtt_msg, 0); // 向队列发送一条消息，参数为 qMqttPublish, &mqtt_msg, 0

        /* 5.打印当前遥测状态 */
        snprintf(log_buf, // 把格式化后的文本写入字符缓冲区，参数为 log_buf,
                 sizeof(log_buf), // 调用sizeof 函数，参数为 log_buf),
                 "[TelemetryTask] roll=%.1f pitch=%.1f yaw=%.1f lat=%.6f lon=%.6f\r\n", // 继续传入 "[TelemetryTask] roll=%.1f pitch=%.1f yaw=%.1f lat=%.6f lon=%.6f\r\n" 字段值，作为当前多行调用或初始化列表的一项
                 attitude.roll_deg, // 继续传入 attitude.roll_deg 字段值，作为当前多行调用或初始化列表的一项
                 attitude.pitch_deg, // 继续传入 attitude.pitch_deg 字段值，作为当前多行调用或初始化列表的一项
                 attitude.yaw_deg, // 继续传入 attitude.yaw_deg 字段值，作为当前多行调用或初始化列表的一项
                 gnss.latitude, // 继续传入 gnss.latitude 字段值，作为当前多行调用或初始化列表的一项
                 gnss.longitude); // 执行 gnss.longitude);，完成当前上下文中的具体处理

        // Debug_Print(log_buf);  //通过调试串口打印遥测状态日志

        /*
         *   遥测任务不需要像 IMU 那样高频运行。
         *   周期在 app_config.h 中统一配置
         */
        vTaskDelay(pdMS_TO_TICKS(APP_TELEMETRY_TASK_PERIOD_MS)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(APP_TELEMETRY_TASK_PERIOD_MS)
    } // 结束当前代码块
} // 结束当前代码块

/**
 * @brief 根据系统状态周期翻转 LED 指示运行状态。
 * @param argument FreeRTOS 任务入口参数。
 * @retval None
 */
static void LedTask(void *argument) // 定义LedTask 函数签名：根据系统状态周期翻转 LED 指示运行状态
{ // 进入当前代码块
    (void)argument; // 标记 FreeRTOS 任务入口参数 当前未使用，避免编译器告警
    while (1) // 当 1 成立时持续执行循环体
    { // 进入当前代码块
        LedService_Toggle(); // 调用翻转 LED 当前亮灭状态

        if (AppStatus_IsSet(APP_STATUS_GNSS_FIX) && AppStatus_IsSet(APP_STATUS_MQTT_READY)) // 判断 AppStatus_IsSet(APP_STATUS_GNSS_FIX) && AppStatus_IsSet(APP_STATUS_MQTT_READY) 是否成立，以选择后续执行路径
        { // 进入当前代码块
            vTaskDelay(pdMS_TO_TICKS(APP_LED_TASK_PERIOD_MS)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(APP_LED_TASK_PERIOD_MS)
        } // 结束当前代码块
        else // 处理前面判断条件不成立时的备用逻辑
        { // 进入当前代码块
            vTaskDelay(pdMS_TO_TICKS(100)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(100)
        } // 结束当前代码块
    } // 结束当前代码块
} // 结束当前代码块
