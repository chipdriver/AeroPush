#include "app_tasks.h"


/*任务句柄*/
/*
 *  ①TaskHandle_t 是FreeRTos中的任务句柄类型
 *  任务句柄可以理解为“任务的管理编号”，后续如果要挂起、恢复、删除某个任务，可以通过任务句柄来操作。 
 *  ②static表示这些变量只能在这个文件内可见
 */
static TaskHandle_t InitTaskHandle      =   NULL;   //初始化任务的句柄，后续用于保存InitTask的任务控制句柄
static TaskHandle_t IMUTaskHandle       =   NULL;   //IMU任务的句柄，后续用于保存IMUTask的任务控制句柄
static TaskHandle_t ModemTaskHandle     =   NULL;   //通信任务的句柄，后续用于保存ModemTask的任务控制句柄
static TaskHandle_t TelemetryTaskHandle =   NULL;   //遥测任务的句柄，后续用于保存TelemetryTask的任务控制句柄
static TaskHandle_t LedTaskHandle       =   NULL;   //LED任务的句柄，后续用于保存LEDTask的任务控制句柄

/*任务函数声明*/
/*
 *  ①static表示这个函数只能在这个文件内可见
 *  ②void *argument 是FreeRTOS 任务函数的标准参数形式；即使现在不用这个参数，也要保留这个格式。
 */
static void InitTask(void *argument);       //声明初始化任务函数
static void ImuTask(void *argument);        //声明IMU 任务函数
static void ModemTask(void *argument);      //声明通信任务函数
static void TelemetryTask(void *argument);  //声明遥测任务函数
static void LedTask(void *argument);        //声明LED任务函数

/**
 * @brief 创建应用任务
 * @param 无
 * @return 无
 */
void APP_TasksCreate(void)
{
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
    xTaskCreate(InitTask,           //创建 InitTask 初始化任务
                "InitTask",         //任务名称
                APP_TASK_INIT_STACK_SIZE, //任务栈大小，单位是word
                NULL,               //不向任务函数传递参数
                APP_TASK_INIT_PRIORITY, //任务优先级为4，当前最高，启动阶段优先执行。
                &InitTaskHandle);   //保存任务句柄到 InitTaskHandle

    xTaskCreate(ImuTask,           //创建 ImuTask 姿态任务
                "ImuTask",          //任务名称
                APP_TASK_IMU_STACK_SIZE, //栈大小，单位是word
                NULL,               //不向任务函数传递参数
                APP_TASK_IMU_PRIORITY, //任务优先级为3，IMU需要高频执行，所以优先级较高。
                &IMUTaskHandle);    //保存任务句柄到 IMUTaskHandle

    xTaskCreate(ModemTask,      //创建 ModemTask 通信任务
                "ModemTask",    //任务名称
                APP_TASK_MODEM_STACK_SIZE, //任务栈大小，单位是word
                NULL,           //不向任务函数传递参数
                APP_TASK_MODEM_PRIORITY, //任务优先级为2，负责A7670E、GNSS、MQTT
                &ModemTaskHandle);    //保存任务句柄到 ModemTask
    xTaskCreate(TelemetryTask,"TelemetryTask",APP_TASK_TELEMETRY_STACK_SIZE,NULL,APP_TASK_TELEMETRY_PRIORITY,&TelemetryTaskHandle); //创建 TelemetryTask 任务
    xTaskCreate(LedTask,"LedTask",APP_TASK_LED_STACK_SIZE,NULL,APP_TASK_LED_PRIORITY,&LedTaskHandle); //创建 LedTask 任务
}

/**
 * @brief 初始化任务
 * @param argument:任务参数
 * @note 后续用于初始化 MPU9250、A7670E、MQTT 等模块
 * @return 无
 */
static void InitTask(void * argument)
{
    uint8_t imu_ret; //保存 IMU 服务初始化结果，1 表示成功，0 表示失败
    (void)argument; //显示表示argument参数暂时不用，避免编译器警告

    BSP_LED_Init(); // LED 初始化
    BSP_DebugUart_Init();   //串口初始化


    /*打印调试任务*/
    Debug_Print("\r\n[AeroPush] RTOS start\r\n"); // 打印系统启动信息
    Debug_Print("[InitTask] start\r\n");       // 打印 InitTask 开始运行信息

    imu_ret = ImuService_Init(); //调用 IMU 服务初始化，为后续接入真实 MPU9250 预留入口
    if(imu_ret == 1)
    {
        AppStatus_Set(APP_STATUS_IMU_READY); //初始化成功后设置 IMU 就绪状态位
    }
    else
    {
        AppStatus_Set(APP_STATUS_IMU_ERROR); //初始化失败时设置 IMU 错误状态位
    }

    AppStatus_Set(APP_STATUS_GNSS_READY); // 设置 GNSS 就绪状态位，表示 GNSS 功能已打开
    AppStatus_Set(APP_STATUS_MQTT_READY); // 设置 MQTT 就绪状态位，表示 MQTT 功能已打开
    AppStatus_Set(APP_STATUS_NET_READY);            // 当前阶段先模拟 4G 网络已就绪

    Debug_Print("[InitTask] status ready\r\n");  // 打印系统状态初始化完成信息
    Debug_Print("[InitTask] done\r\n");        // 打印 InitTask 初始化完成信息
    /*
    *   初始化任务通常只需要运行一次。
    *   初始化完成后，调用vTaskDelete(NULL) 删除自己。
    *   NULL表示删除当前正在运行的任务
    */
    vTaskDelete(NULL);  //删除当前 InitTask 任务，释放任务资源
}

/**
 * @brief IMU 任务
 * @param argument:任务参数
 * @retval 无
 * @note ①后续用于 MPU9250 读取和 Mahony 姿态融合
 *       ②关于详细的解释在：“知识点”知识库 -> FreeRTOS ->FreeRTOS 周期任务与 Tick 时间控制
 */
static void ImuTask(void *argument)
{
    TickType_t lastWakeTime;    //定义一个变量，用于保存上一次任务唤醒的系统 tick 时间
    (void)argument;             //显示表示 argument 参数暂时不用，避免编译器警告
    AttitudeData_t attitude;    //定义姿态数据变量,用于保存模拟姿态数据

    /*
    *   xTaskGetTickCount() 用于获取当前 FreeRTOS 系统 tick 计数值。
    *   这里把当前时间保存下来，作为vTaskDelayUntil()   的基准时间
    */
   lastWakeTime = xTaskGetTickCount();  //获取当前系统tick，作为周期任务的起始时间

   while(1)         //任务主循环，FreeRTOS 任务一般都是while(1) 死循环结构
   {

    if(AppStatus_IsSet(APP_STATUS_IMU_READY) == 0) //如果 IMU 就绪状态位未设置，说明 IMU 初始化未完成
    {
        vTaskDelay(pdMS_TO_TICKS(100));  //每100ms检查一次状态位，等待 IMU 初始化完成
        continue;   //跳过本次循环，继续等待
    }

    /*模拟姿态数据*/
    ImuService_BuildSimAttitude(&attitude);

    /*队列长度为 1,使用 xQueueOverwrite 保存最新姿态*/
    xQueueOverwrite(qAttitude,&attitude);   //将最新姿态数据写入 qAttitude 队列,如果已有旧数据则覆盖

    /*
    *   vTaskDelayUntil()   用于实现严格周期延时，它与 vTaskDelay() 不一样；
    *   vTaskDelay() 是从“当前时刻”开始延时
    *   vTaskDelayUntil() 是按照“固定时间点”周期执行
    *   
    *   APP_IMU_TASK_PERIOD_MS 在 app_config.h 中统一配置
    *   所以这里表示 ImuTask 按配置周期执行一次*/
    vTaskDelayUntil(&lastWakeTime,//传入上一次唤醒时间的地址，函数内部会自动更新它
                    pdMS_TO_TICKS(APP_IMU_TASK_PERIOD_MS));//延时到下一个 5ms 周期点，实现 200HZ 周期任务
   }
}

/**
 * @brief   通信任务
 * @param   argument:任务参数
 * @retval  无
 * @note    后续用于统一管理A7670E、GNSS、MQTT
 */
static void ModemTask(void *argument)
{
    GnssData_t gnss;        //定义 GNSS 数据变量,用于保存模拟数据
    MqttPublishMsg_t mqtt_msg;  //定义 MQTT 消息变量,用于接收待发布消息
    (void)argument;     //显示表示argument参数暂时不用，避免编译器警告

    memset(&gnss,0,sizeof(gnss));   //将 GNSS 数据变量清零，避免初始值随机
    memset(&mqtt_msg,0,sizeof(mqtt_msg));   //将 MQTT 消息变量清零，避免字符串缓冲区残留脏数据

    while(1)            //通信任务主循环
    {

        /*模拟GNSS数据*/
        ModemService_BuildSimGnss(&gnss);

        if(gnss.fix_valid) //如果模拟的 GNSS 定位有效
            AppStatus_Set(APP_STATUS_GNSS_FIX); //设置 GNSS 已定位状态位
        else
            AppStatus_Clear(APP_STATUS_GNSS_FIX); //清除 GNSS 已定位状态位

        

        xQueueOverwrite(qGnss, &gnss);           // 将最新 GNSS 数据写入 qGnss 队列，如果已有旧数据则覆盖

        /*
         * 2. 处理 MQTT 发布队列
         * 后续这里会替换成真正的 MQTT_Publish。
         */
        if(xQueueReceive(qMqttPublish,&mqtt_msg,0) == pdPASS) // 尝试从 MQTT 队列中取出一条发布消息，不等待
        {
            ModemService_Publish(&mqtt_msg);
        }
        /*
        *   vTaskDelay() 用于让当前任务主动阻塞一段时间
        *   周期在 app_config.h 中统一配置，避免通信任务一直占用CPU
        */
        vTaskDelay(pdMS_TO_TICKS(APP_MODEM_TASK_PERIOD_MS));  //当前任务按配置周期延时，让出 CPU 给其他任务执行
    }
}

/**
 * @brief   遥测任务
 * @param   argument:任务参数
 * @retval  无
 * @note    后续用于姿态数据、定位数据整合和 JSON 组包
 */

static void TelemetryTask(void *argument)
{
    AttitudeData_t attitude;         // 定义姿态数据变量，用于保存从 qAttitude 读取到的数据
    GnssData_t     gnss;             // 定义 GNSS 数据变量，用于保存从 qGnss 读取到的数据
    MqttPublishMsg_t mqtt_msg;       // 定义 MQTT 发布消息变量，用于组装 topic 和 payload
    char log_buf[256];               // 定义日志缓冲区，用于格式化串口打印内容

    (void)argument; //显示表示 argument 参数暂时不用，避免编译器警告

    memset(&attitude,0,sizeof(attitude));   //将姿态数据变量清零,避免初始值随机
    memset(&gnss,0,sizeof(gnss));           // 将 GNSS 数据变量清零，避免初始值随机
    memset(&mqtt_msg,0,sizeof(mqtt_msg));   // 将 MQTT 消息变量清零，避免字符串缓冲区残留脏数据

    while(1)
    {

        /* 1.获取最新姿态数据 */
        xQueuePeek(qAttitude,&attitude,0);  // 从姿态队列读取最新数据，但不把数据从队列中删除

        /* 2. 获取最新 GNSS 数据 */
        xQueuePeek(qGnss,&gnss,0);          // 从 GNSS 队列读取最新数据，但不把数据从队列中删除
        
        /* 3. 组装JSON数据*/
       Telemetry_BuildMqttMsg(&attitude,&gnss,&mqtt_msg);

        /* 4.发送给ModemTask */
        xQueueSend(qMqttPublish,&mqtt_msg,0);   // 将 MQTT 发布消息发送到 qMqttPublish 队列，不等待

        /* 5.打印当前遥测状态 */
        snprintf(log_buf,                        // 将格式化后的日志字符串写入 log_buf
            sizeof(log_buf),                // 限制最大写入长度，防止 log_buf 溢出
            "[TelemetryTask] roll=%.1f pitch=%.1f yaw=%.1f lat=%.6f lon=%.6f\r\n", // 日志输出格式
            attitude.roll_deg,              // 写入 roll 角数据
            attitude.pitch_deg,             // 写入 pitch 角数据
            attitude.yaw_deg,               // 写入 yaw 角数据
            gnss.latitude,                  // 写入纬度数据
            gnss.longitude);                // 写入经度数据

        Debug_Print(log_buf);  //通过调试串口打印遥测状态日志
        
        /*
        *   遥测任务不需要像 IMU 那样高频运行。
        *   周期在 app_config.h 中统一配置
        */
       vTaskDelay(pdMS_TO_TICKS(APP_TELEMETRY_TASK_PERIOD_MS));  //当前任务按配置周期延时，控制遥测组包频率
    }
}

/**
 * @brief   LED 任务
 * @param   argument：任务参数
 * @retval  无
 * @note    后续用于系统状态指示
 */
static void LedTask(void *argument)
{
    (void)argument; //显示表示 argument 参数暂时不用，避免编译器警告
    while(1)
    {
        BSP_LED_Toggle();   //LED 翻转

       if( AppStatus_IsSet(APP_STATUS_GNSS_FIX) && AppStatus_IsSet(APP_STATUS_MQTT_READY) ) //如果 GNSS 已定位且 MQTT 已就绪，说明系统状态良好，LED 正常
        {
            vTaskDelay(pdMS_TO_TICKS(APP_LED_TASK_PERIOD_MS)); // LED 正常
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(100)); //LED 快闪，提示用户系统状态异常
        }
    }
}
