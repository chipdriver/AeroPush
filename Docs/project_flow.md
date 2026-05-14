# AeroPush 工程流程图

之前那版第一张图太宽，Typora 会自动缩小到整页宽度，所以几乎看不见。现在改成“窄图 + 分段图”，每张图只讲一件事，适合直接在 Typora 里看。

## 0. 怎么看

如果图还是偏小，可以在 Typora 里按：

```text
Ctrl + 鼠标滚轮向上
```

或者点菜单：

```text
视图 -> 放大
```

## 1. 总体分层

```mermaid
flowchart TD
    A["main.c<br/>程序入口"] --> B["app_main.c<br/>应用总入口"]
    B --> C["app_tasks.c<br/>创建和运行任务"]

    C --> D["服务层"]
    D --> D1["imu_service.c<br/>IMU 服务"]
    D --> D2["modem_service.c<br/>通信/GNSS 模拟服务"]
    D --> D3["telemetry_service.c<br/>遥测组包服务"]

    D1 --> E["驱动/BSP 层"]
    E --> E1["mpu9250_driver.c<br/>MPU9250 + AK8963 驱动"]
    E --> E2["bsp_i2c_soft.c<br/>软件 I2C"]
    E --> E3["bsp_debug_uart.c<br/>调试串口"]
    E --> E4["bsp_led.c<br/>LED"]

    C --> F["系统支撑"]
    F --> F1["freertos_objects.c<br/>队列/互斥锁/事件组"]
    F --> F2["app_status.c<br/>系统状态位"]
    F --> F3["debug_log.c<br/>日志打印"]
```

## 2. 上电启动流程

```mermaid
flowchart TD
    A["芯片上电/复位"] --> B["main()"]
    B --> C["APP_Main()"]
    C --> D["FreeRTOS_ObjectsCreate()"]
    D --> D1["创建 qAttitude"]
    D --> D2["创建 qGnss"]
    D --> D3["创建 qMqttPublish"]
    D --> D4["创建 mutexUart6log"]
    D --> D5["创建 gSystemEventGroup"]
    C --> E["APP_TasksCreate()"]
    E --> F["创建 InitTask"]
    E --> G["创建 ImuTask"]
    E --> H["创建 ModemTask"]
    E --> I["创建 TelemetryTask"]
    E --> J["创建 LedTask"]
    C --> K["vTaskStartScheduler()"]
    K --> L["FreeRTOS 开始调度任务"]
```

## 3. InitTask 做了什么

```mermaid
flowchart TD
    A["InitTask()"] --> B["BSP_LED_Init()"]
    B --> C["BSP_DebugUart_Init()"]
    C --> D["ImuService_Init()"]
    D --> E["MPU9250_Driver_Init()"]
    E --> F{"IMU 初始化成功?"}
    F -->|成功| G["AppStatus_Set<br/>(APP_STATUS_IMU_READY)"]
    F -->|失败| H["AppStatus_Set<br/>(APP_STATUS_IMU_ERROR)"]
    G --> I["设置 GNSS/MQTT/NET ready<br/>目前是模拟状态"]
    H --> I
    I --> J["vTaskDelete(NULL)<br/>InitTask 删除自己"]
```

## 4. MPU9250_Driver_Init 内部流程

```mermaid
flowchart TD
    A["MPU9250_Driver_Init()"] --> B["BSP_I2C_Soft_Init()"]
    B --> C["mpu9250_check_device()"]
    C --> C1["读 WHO_AM_I<br/>期望 0x71"]
    C1 --> D["mpu9250_config_six_axis()"]
    D --> D1["MPU9250_SoftReset()"]
    D --> D2["mpu_set_clock_to_auto()"]
    D --> D3["mpu_enable_six_axis()"]
    D --> D4["配置 DLPF / 采样率 / 量程"]
    D4 --> E["ak8963_init()"]
    E --> E1["打开 BYPASS"]
    E --> E2["检查 AK8963 ID<br/>期望 0x48"]
    E --> E3["进入 Fuse ROM"]
    E --> E4["读取 ASA 灵敏度系数"]
    E --> E5["进入 16-bit 100Hz 连续测量"]
    E5 --> F["MPU9250_CalibrateGyro()"]
    F --> G["MPU9250_CalibrateAccel()"]
    G --> H["AK8963_CalibrateMag()"]
    H --> I["driver init ok"]
```

## 5. IMU 周期读取流程

```mermaid
flowchart TD
    A["ImuTask<br/>20ms 周期"] --> B{"APP_STATUS_IMU_READY?"}
    B -->|否| C["等待 100ms"]
    B -->|是| D["ImuService_ReadPhys(&phys, &mag)"]
    D --> E["MPU9250_ReadAxis(&raw)"]
    E --> F["MPU9250_ConvertToPhysical()<br/>raw -> g / dps / degC"]
    D --> G["AK8963_Read_Mag_UT(&mag)"]
    G --> H["AK8963_Read_Axis(&raw)"]
    H --> I["AK8963_Calibrate()<br/>ASA + offset + scale"]
    F --> J{"读取成功?"}
    I --> J
    J -->|否| K["跳过本周期"]
    J -->|是| L["每约 1 秒打印一次<br/>accel + gyro + mag"]
    L --> M["下一步接姿态融合"]
```

## 6. 磁力计读取和校准关系

```mermaid
flowchart TD
    A["AK8963_Read_Mag_UT()"] --> B["AK8963_Read_Axis()"]
    B --> B1["检查 ST1<br/>数据是否 ready"]
    B1 --> B2["读取 HXL/HXH"]
    B2 --> B3["读取 HYL/HYH"]
    B3 --> B4["读取 HZL/HZH"]
    B4 --> B5["检查 ST2<br/>是否溢出"]
    B5 --> C["AK8963_Calibrate()"]
    C --> D["ak8963_convert_raw_to_ut()<br/>原始值 * ASA * 0.15"]
    D --> E["ak8963_apply_mag_calibration()<br/>减 offset，再乘 scale"]
    E --> F["输出 mag_x/y/z_uT"]
```

## 7. 三个校准值放在哪里

```mermaid
flowchart TD
    A["校准相关静态变量<br/>只在 mpu9250_driver.c 内部使用"] --> B["g_gyro_bias_dps[3]<br/>陀螺仪零偏"]
    A --> C["g_accel_bias_g[3]<br/>加速度计静态偏置"]
    A --> D["g_ak8963_sensitivity[3]<br/>ASA 灵敏度补偿"]
    A --> E["g_mag_offset_ut[3]<br/>磁力计硬铁偏移"]
    A --> F["g_mag_scale[3]<br/>磁力计软铁缩放"]

    B --> G["MPU9250_ConvertToPhysical() 使用"]
    C --> G
    D --> H["ak8963_convert_raw_to_ut() 使用"]
    E --> I["ak8963_apply_mag_calibration() 使用"]
    F --> I
```

## 8. FreeRTOS 任务之间的数据关系

```mermaid
flowchart TD
    A["InitTask"] --> S["gSystemEventGroup<br/>系统状态位"]
    B["ImuTask"] --> S
    C["ModemTask"] --> S
    D["LedTask"] --> S

    B --> Q1["qAttitude<br/>后续姿态融合结果应写入这里"]
    C --> Q2["qGnss<br/>GNSS 数据"]
    D2["TelemetryTask"] --> Q1
    D2 --> Q2
    D2 --> Q3["qMqttPublish<br/>MQTT 消息"]
    C --> Q3
```

## 9. 姿态融合应该接在哪里

```mermaid
flowchart TD
    A["ImuTask"] --> B["ImuService_ReadPhys(&phys, &mag)"]
    B --> C["拿到融合输入"]
    C --> C1["accel_x/y/z_g"]
    C --> C2["gyro_x/y/z_dps"]
    C --> C3["mag_x/y/z_uT"]
    C --> D["AttitudeFusion_Update()"]
    D --> E["输出 AttitudeData_t"]
    E --> F["写入 qAttitude"]
    F --> G["TelemetryTask 读取姿态并组包"]
```

建议后面新增：

```text
User/Inc/attitude_fusion.h
User/Src/attitude_fusion.c
```

这样 `ImuTask` 只负责周期调度，融合算法单独放在 `attitude_fusion.c`，工程会更清楚。
