#include "app_tasks.h" // 引入应用任务模块接口

/*任务句柄*/
/*
 *  ①TaskHandle_t 是FreeRTos中的任务句柄类型
 *  任务句柄可以理解为“任务的管理编号”，后续如果要挂起、恢复、删除某个任务，可以通过任务句柄来操作。
 *  ②static表示这些变量只能在这个文件内可见
 */
static TaskHandle_t InitTaskHandle = NULL;      // 初始化任务的句柄，后续用于保存InitTask的任务控制句柄
static TaskHandle_t IMUTaskHandle = NULL;       // IMU任务的句柄，后续用于保存IMUTask的任务控制句柄
static TaskHandle_t ModemTaskHandle = NULL;     // 通信任务的句柄，后续用于保存ModemTask的任务控制句柄
static TaskHandle_t TelemetryTaskHandle = NULL; // 遥测任务的句柄，后续用于保存TelemetryTask的任务控制句柄
static TaskHandle_t LedTaskHandle = NULL;       // LED任务的句柄，后续用于保存LEDTask的任务控制句柄

/*任务函数声明*/
/*
 *  ①static表示这个函数只能在这个文件内可见
 *  ②void *argument 是FreeRTOS 任务函数的标准参数形式；即使现在不用这个参数，也要保留这个格式。
 */
/**
 * @brief  初始化任务函数声明。
 * @param  argument FreeRTOS 任务参数，当前未使用。
 * @retval None
 */
static void InitTask(void *argument); // 声明初始化任务函数

/**
 * @brief  IMU 任务函数声明。
 * @param  argument FreeRTOS 任务参数，当前未使用。
 * @retval None
 */
static void ImuTask(void *argument); // 声明 IMU 任务函数

/**
 * @brief  通信任务函数声明。
 * @param  argument FreeRTOS 任务参数，当前未使用。
 * @retval None
 */
static void ModemTask(void *argument); // 声明通信任务函数

/**
 * @brief  遥测任务函数声明。
 * @param  argument FreeRTOS 任务参数，当前未使用。
 * @retval None
 */
static void TelemetryTask(void *argument); // 声明遥测任务函数

/**
 * @brief  LED 任务函数声明。
 * @param  argument FreeRTOS 任务参数，当前未使用。
 * @retval None
 */
static void LedTask(void *argument); // 声明 LED 任务函数

/**
 * @brief  创建应用层所有 FreeRTOS 任务。
 * @param  None
 * @retval None
 */
void APP_TasksCreate(void) // 定义应用任务创建函数
{                          // APP_TasksCreate 函数体开始
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
    xTaskCreate(InitTask,                 // 创建 InitTask 初始化任务
                "InitTask",               // 任务名称
                APP_TASK_INIT_STACK_SIZE, // 任务栈大小，单位是word
                NULL,                     // 不向任务函数传递参数
                APP_TASK_INIT_PRIORITY,   // 任务优先级为4，当前最高，启动阶段优先执行。
                &InitTaskHandle);         // 保存任务句柄到 InitTaskHandle

    xTaskCreate(ImuTask,                 // 创建 ImuTask 姿态任务
                "ImuTask",               // 任务名称
                APP_TASK_IMU_STACK_SIZE, // 栈大小，单位是word
                NULL,                    // 不向任务函数传递参数
                APP_TASK_IMU_PRIORITY,   // 任务优先级为3，IMU需要高频执行，所以优先级较高。
                &IMUTaskHandle);         // 保存任务句柄到 IMUTaskHandle

    xTaskCreate(ModemTask,                                                                                                               // 创建 ModemTask 通信任务
                "ModemTask",                                                                                                             // 任务名称
                APP_TASK_MODEM_STACK_SIZE,                                                                                               // 任务栈大小，单位是word
                NULL,                                                                                                                    // 不向任务函数传递参数
                APP_TASK_MODEM_PRIORITY,                                                                                                 // 任务优先级为2，负责A7670E、GNSS、MQTT
                &ModemTaskHandle);                                                                                                       // 保存任务句柄到 ModemTask
    xTaskCreate(TelemetryTask, "TelemetryTask", APP_TASK_TELEMETRY_STACK_SIZE, NULL, APP_TASK_TELEMETRY_PRIORITY, &TelemetryTaskHandle); // 创建 TelemetryTask 任务
    xTaskCreate(LedTask, "LedTask", APP_TASK_LED_STACK_SIZE, NULL, APP_TASK_LED_PRIORITY, &LedTaskHandle);                               // 创建 LedTask 任务
} // APP_TasksCreate 函数体结束

/**
 * @brief  初始化任务。
 * @param  argument FreeRTOS 任务参数，当前未使用。
 * @retval None
 * @note   当前负责初始化 LED、调试串口、IMU，并设置系统状态位。
 */
static void InitTask(void *argument) // 定义初始化任务函数
{                                    // InitTask 函数体开始
    uint8_t imu_ret;                 // 保存 IMU 服务初始化结果，1 表示成功，0 表示失败
    (void)argument;                  // 显示表示argument参数暂时不用，避免编译器警告

    LedService_Init();   // 调用 LED 服务完成板载 LED 初始化
    DebugService_Init(); // 调用调试服务完成调试串口初始化

    /*打印调试任务*/
    // Debug_Print("\r\n[AeroPush] RTOS start\r\n"); // 打印系统启动信息
    // Debug_Print("[InitTask] start\r\n");       // 打印 InitTask 开始运行信息

    imu_ret = ImuService_Init();             // 调用 IMU 服务初始化，为后续接入真实 MPU9250 预留入口
    if (imu_ret == 1)                        // 判断条件是否成立
    {                                        // 进入代码块
        AppStatus_Set(APP_STATUS_IMU_READY); // 初始化成功后设置 IMU 就绪状态位
    } // 结束代码块
    else                                     // 处理条件不成立的分支
    {                                        // 进入代码块
        AppStatus_Set(APP_STATUS_IMU_ERROR); // 初始化失败时设置 IMU 错误状态位
    } // 结束代码块

    AppStatus_Set(APP_STATUS_GNSS_READY); // 设置 GNSS 就绪状态位，表示 GNSS 功能已打开
    AppStatus_Set(APP_STATUS_MQTT_READY); // 设置 MQTT 就绪状态位，表示 MQTT 功能已打开
    AppStatus_Set(APP_STATUS_NET_READY);  // 当前阶段先模拟 4G 网络已就绪

    // Debug_Print("[InitTask] status ready\r\n");  // 打印系统状态初始化完成信息
    // Debug_Print("[InitTask] done\r\n");        // 打印 InitTask 初始化完成信息
    /*
     *   初始化任务通常只需要运行一次。
     *   初始化完成后，调用vTaskDelete(NULL) 删除自己。
     *   NULL表示删除当前正在运行的任务
     */
    vTaskDelete(NULL); // 删除当前 InitTask 任务，释放任务资源
} // InitTask 函数体结束

/**
 * @brief  IMU 周期读取任务。
 * @param  argument FreeRTOS 任务参数，当前未使用。
 * @retval None
 * @note   当前读取 MPU9250 六轴和 AK8963 磁力计物理量，后续姿态融合也建议接在这里。
 */
static void ImuTask(void *argument)          // 定义 IMU 任务函数
{                                            // ImuTask 函数体开始
    TickType_t lastWakeTime;                 // 定义一个变量，用于保存上一次任务唤醒的系统 tick 时间
    (void)argument;                          // 显示表示 argument 参数暂时不用，避免编译器警告
    AttitudeData_t attitude;                 // 定义姿态数据变量,用于保存模拟姿态数据
    uint32_t print_count = 0;                // 定义一个计数器变量，用于控制串口打印频率
    MPU9250_Physical_Data phys = {0};        // 定义局部变量
    AK8963_Physical_Data mag_physical = {0}; // 定义局部变量
    uint8_t imu_read_ok = 0;                 // 定义局部变量
    /*
     *   xTaskGetTickCount() 用于获取当前 FreeRTOS 系统 tick 计数值。
     *   这里把当前时间保存下来，作为vTaskDelayUntil()   的基准时间
     */
    lastWakeTime = xTaskGetTickCount(); // 获取当前系统tick，作为周期任务的起始时间

    while (1) // 任务主循环，FreeRTOS 任务一般都是while(1) 死循环结构
    {         // 进入代码块

        if (AppStatus_IsSet(APP_STATUS_IMU_READY) == 0) // 如果 IMU 就绪状态位未设置，说明 IMU 初始化未完成
        {                                               // 进入代码块
            vTaskDelay(pdMS_TO_TICKS(100));             // 每100ms检查一次状态位，等待 IMU 初始化完成
            continue;                                   // 跳过本次循环，继续等待
        } // 结束代码块

        imu_read_ok = ImuService_ReadPhys(&phys, &mag_physical);                   // 调用 IMU 服务读取物理量数据函数，获取最新姿态数据
        if (imu_read_ok == 0U)                                                     // 判断条件是否成立
        {                                                                          // 进入代码块
            vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(APP_IMU_TASK_PERIOD_MS)); // 按固定周期延时到下一次执行
            continue;                                                              // 跳过本次循环后续逻辑
        } // 结束代码块

        print_count++; // 读取成功，增加计数器

        if (print_count >= 200) // ImuTask 周期为 20ms，200 次约等于 4 秒
        {                       // 进入代码块
            print_count = 0;    // 清零打印计数器

            Debug_Printf("[ImuTask] phys ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f mag_x=%.2f mag_y=%.2f mag_z=%.2f\r\n", // 打印 MPU9250 物理量六轴数据
                         phys.accel_x_g,                                                                                        // 打印加速度计 X 轴物理量
                         phys.accel_y_g,                                                                                        // 打印加速度计 Y 轴物理量
                         phys.accel_z_g,                                                                                        // 打印加速度计 Z 轴物理量
                         phys.gyro_x_dps,                                                                                       // 打印陀螺仪 X 轴物理量
                         phys.gyro_y_dps,                                                                                       // 打印陀螺仪 Y 轴物理量
                         phys.gyro_z_dps,                                                                                       // 打印陀螺仪 Z 轴物理量
                         mag_physical.mag_x_ut,                                                                                 // 打印磁力计 X 轴物理量
                         mag_physical.mag_y_ut,                                                                                 // 打印磁力计 Y 轴物理量
                         mag_physical.mag_z_ut);                                                                                // 打印磁力计 Z 轴物理量
        } // 结束代码块

        /*
         *   vTaskDelayUntil()   用于实现严格周期延时，它与 vTaskDelay() 不一样；
         *   vTaskDelay() 是从“当前时刻”开始延时
         *   vTaskDelayUntil() 是按照“固定时间点”周期执行
         *
         *   APP_IMU_TASK_PERIOD_MS 在 app_config.h 中统一配置
         *   所以这里表示 ImuTask 按配置周期执行一次*/
        vTaskDelayUntil(&lastWakeTime,                          // 传入上一次唤醒时间的地址，函数内部会自动更新它
                        pdMS_TO_TICKS(APP_IMU_TASK_PERIOD_MS)); // 延时到下一个 5ms 周期点，实现 200HZ 周期任务
    } // 结束代码块
} // ImuTask 函数体结束

/**
 * @brief  通信任务。
 * @param  argument FreeRTOS 任务参数，当前未使用。
 * @retval None
 * @note   当前负责模拟 GNSS 数据，并从 MQTT 发布队列取出待发送消息。
 */
static void ModemTask(void *argument) // 定义通信任务函数
{                                     // ModemTask 函数体开始
    GnssData_t gnss;                  // 定义 GNSS 数据变量,用于保存模拟数据
    MqttPublishMsg_t mqtt_msg;        // 定义 MQTT 消息变量,用于接收待发布消息
    (void)argument;                   // 显示表示argument参数暂时不用，避免编译器警告

    memset(&gnss, 0, sizeof(gnss));         // 将 GNSS 数据变量清零，避免初始值随机
    memset(&mqtt_msg, 0, sizeof(mqtt_msg)); // 将 MQTT 消息变量清零，避免字符串缓冲区残留脏数据

    while (1) // 通信任务主循环
    {         // 进入代码块

        /*模拟GNSS数据*/
        ModemService_BuildSimGnss(&gnss); // 调用通信服务层接口

        if (gnss.fix_valid)                       // 如果模拟的 GNSS 定位有效
            AppStatus_Set(APP_STATUS_GNSS_FIX);   // 设置 GNSS 已定位状态位
        else                                      // 处理条件不成立的分支
            AppStatus_Clear(APP_STATUS_GNSS_FIX); // 清除 GNSS 已定位状态位

        xQueueOverwrite(qGnss, &gnss); // 将最新 GNSS 数据写入 qGnss 队列，如果已有旧数据则覆盖

        /*
         * 2. 处理 MQTT 发布队列
         * 后续这里会替换成真正的 MQTT_Publish。
         */
        if (xQueueReceive(qMqttPublish, &mqtt_msg, 0) == pdPASS) // 尝试从 MQTT 队列中取出一条发布消息，不等待
        {                                                        // 进入代码块
            ModemService_Publish(&mqtt_msg);                     // 调用通信服务层接口
        } // 结束代码块
        /*
         *   vTaskDelay() 用于让当前任务主动阻塞一段时间
         *   周期在 app_config.h 中统一配置，避免通信任务一直占用CPU
         */
        vTaskDelay(pdMS_TO_TICKS(APP_MODEM_TASK_PERIOD_MS)); // 当前任务按配置周期延时，让出 CPU 给其他任务执行
    } // 结束代码块
} // ModemTask 函数体结束

/**
 * @brief  遥测组包任务。
 * @param  argument FreeRTOS 任务参数，当前未使用。
 * @retval None
 * @note   当前从姿态队列和 GNSS 队列读取数据，组装 MQTT 消息后放入发布队列。
 */

static void TelemetryTask(void *argument) // 定义遥测任务函数
{                                         // TelemetryTask 函数体开始
    AttitudeData_t attitude;              // 定义姿态数据变量，用于保存从 qAttitude 读取到的数据
    GnssData_t gnss;                      // 定义 GNSS 数据变量，用于保存从 qGnss 读取到的数据
    MqttPublishMsg_t mqtt_msg;            // 定义 MQTT 发布消息变量，用于组装 topic 和 payload
    char log_buf[256];                    // 定义日志缓冲区，用于格式化串口打印内容

    (void)argument; // 显示表示 argument 参数暂时不用，避免编译器警告

    memset(&attitude, 0, sizeof(attitude)); // 将姿态数据变量清零,避免初始值随机
    memset(&gnss, 0, sizeof(gnss));         // 将 GNSS 数据变量清零，避免初始值随机
    memset(&mqtt_msg, 0, sizeof(mqtt_msg)); // 将 MQTT 消息变量清零，避免字符串缓冲区残留脏数据

    while (1) // 开始循环执行
    {         // 进入代码块

        /* 1.获取最新姿态数据 */
        xQueuePeek(qAttitude, &attitude, 0); // 从姿态队列读取最新数据，但不把数据从队列中删除

        /* 2. 获取最新 GNSS 数据 */
        xQueuePeek(qGnss, &gnss, 0); // 从 GNSS 队列读取最新数据，但不把数据从队列中删除

        /* 3. 组装JSON数据*/
        Telemetry_BuildMqttMsg(&attitude, &gnss, &mqtt_msg); // 调用遥测组包接口

        /* 4.发送给ModemTask */
        xQueueSend(qMqttPublish, &mqtt_msg, 0); // 将 MQTT 发布消息发送到 qMqttPublish 队列，不等待

        /* 5.打印当前遥测状态 */
        snprintf(log_buf,                                                               // 将格式化后的日志字符串写入 log_buf
                 sizeof(log_buf),                                                       // 限制最大写入长度，防止 log_buf 溢出
                 "[TelemetryTask] roll=%.1f pitch=%.1f yaw=%.1f lat=%.6f lon=%.6f\r\n", // 日志输出格式
                 attitude.roll_deg,                                                     // 写入 roll 角数据
                 attitude.pitch_deg,                                                    // 写入 pitch 角数据
                 attitude.yaw_deg,                                                      // 写入 yaw 角数据
                 gnss.latitude,                                                         // 写入纬度数据
                 gnss.longitude);                                                       // 写入经度数据

        // Debug_Print(log_buf);  //通过调试串口打印遥测状态日志

        /*
         *   遥测任务不需要像 IMU 那样高频运行。
         *   周期在 app_config.h 中统一配置
         */
        vTaskDelay(pdMS_TO_TICKS(APP_TELEMETRY_TASK_PERIOD_MS)); // 当前任务按配置周期延时，控制遥测组包频率
    } // 结束代码块
} // TelemetryTask 函数体结束

/**
 * @brief  LED 状态指示任务。
 * @param  argument FreeRTOS 任务参数，当前未使用。
 * @retval None
 * @note   根据系统事件组中的状态位切换 LED 闪烁节奏。
 */
static void LedTask(void *argument) // 定义 LED 任务函数
{                                   // LedTask 函数体开始
    (void)argument;                 // 显示表示 argument 参数暂时不用，避免编译器警告
    while (1)                       // 开始循环执行
    {                               // 进入代码块
        LedService_Toggle();        // 调用 LED 服务翻转板载 LED 状态

        if (AppStatus_IsSet(APP_STATUS_GNSS_FIX) && AppStatus_IsSet(APP_STATUS_MQTT_READY)) // 如果 GNSS 已定位且 MQTT 已就绪，说明系统状态良好，LED 正常
        {                                                                                   // 进入代码块
            vTaskDelay(pdMS_TO_TICKS(APP_LED_TASK_PERIOD_MS));                              // LED 正常
        } // 结束代码块
        else                                // 处理条件不成立的分支
        {                                   // 进入代码块
            vTaskDelay(pdMS_TO_TICKS(100)); // LED 快闪，提示用户系统状态异常
        } // 结束代码块
    } // 结束代码块
} // LedTask 函数体结束
