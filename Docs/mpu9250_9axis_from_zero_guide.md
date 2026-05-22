# MPU9250 九轴驱动从 0 到 1 编程笔记

## 1. MPU9250 九轴驱动整体目标

这份笔记不是数据手册翻译，而是按“新手现在要写驱动代码”的顺序，把当前 AeroPush 工程里的 MPU9250 + AK8963 九轴链路拆开。最终目标不是单纯读到几个寄存器，而是让 MPU9250 在 FreeRTOS 里周期输出可用于遥测的 `roll / pitch / yaw`。

完整目标包括：

1. 读取加速度计三轴 raw 数据；
2. 读取陀螺仪三轴 raw 数据；
3. 读取 AK8963 磁力计三轴 raw 数据；
4. 将 raw 转换为 `g / dps / uT / degC` 物理量；
5. 完成陀螺仪零偏校准；
6. 完成加速度计零偏校准；
7. 完成磁力计硬铁和软铁校准；
8. 完成磁力计 offset / scale / 3x3 矩阵补偿；
9. 得到稳定的九轴输入；
10. 理解 A+M 初始姿态解算应该怎么写；
11. 完成 Mahony 九轴姿态融合；
12. 输出 `roll_deg / pitch_deg / yaw_deg`；
13. 在 FreeRTOS 的 `ImuTask` 中周期运行；
14. 将姿态数据送入 `qAttitude`，再由 `TelemetryTask` 组包。

MPU9250 实际上由两块功能组成：

- `MPU6500`：三轴加速度计 + 三轴陀螺仪，寄存器在 MPU9250 主地址 `0x68` 下；
- `AK8963`：三轴磁力计，是内部辅助 I2C 总线上的独立芯片，旁路访问时地址是 `0x0C`。

所以九轴驱动不能只初始化 MPU9250 主芯片。六轴配置完成后，还必须打开 AK8963 的访问通道，单独检查 AK8963 的 ID、读取 ASA、设置连续测量模式，并处理 `ST1 / ST2` 状态。

当前工程主链路是：

```text
main()
  -> APP_Main()
  -> APP_TasksCreate()
  -> FreeRTOS_ObjectsCreate()
  -> vTaskStartScheduler()

InitTask()
  -> ImuService_Init()
  -> MPU9250_Driver_Init()
  -> MPU9250_MahonyInit(0.3f, 0.0f)
  -> AppStatus_Set(APP_STATUS_IMU_READY)

ImuTask()
  -> ImuService_ReadPhys(&phys, &mag_physical)
  -> MPU9250_MahonyUpdate() 或 MPU9250_MahonyUpdateIMU()
  -> MPU9250_GetEulerFusedDeg()
  -> xQueueOverwrite(qAttitude, &attitude)

TelemetryTask()
  -> xQueuePeek(qAttitude, &attitude, 0)
  -> Telemetry_BuildMqttMsg()
```

当前工程对应文件：

| 文件 | 作用 |
|---|---|
| `User/Src/bsp_i2c_soft.c` | 软件 I2C START / STOP / ACK / 单寄存器读写 / 连续读写 |
| `User/Inc/bsp_i2c_soft.h` | 软件 I2C 引脚和接口声明 |
| `User/Src/mpu9250_driver.c` | MPU9250、AK8963 初始化、读取、转换、校准、Mahony 融合 |
| `User/Inc/mpu9250_driver.h` | 驱动公开结构体、地址宏和函数声明 |
| `User/Src/imu_service.c` | 任务层访问 IMU 的薄封装 |
| `User/Src/app_tasks.c` | `InitTask / ImuTask / TelemetryTask / LedTask` |
| `User/Src/app_status.c` | 系统状态位和校准灯语状态 |
| `User/Inc/app_config.h` | 任务周期、优先级、磁力计调试输出开关 |
| `User/Src/telemetry_service.c` | 把姿态角写入 MQTT JSON |

## 1.1 这一步要解决什么问题

这一步先把“九轴驱动”的边界讲清楚：新手不能只会读 `ACCEL_XOUT_H`，还要知道数据最后进入哪里、由谁消费、失败时用什么状态表示。

- function：建立从传感器到姿态队列的全局目标。
- why：后面写每个寄存器时，才能知道它服务的是通信、六轴数据、磁力计、校准还是融合。
- if not：容易一上来写 Mahony，结果 I2C、WHO_AM_I、ST2、ASA、单位转换任何一处错了都定位不到。
- before：只需要知道项目当前使用 STM32F4、FreeRTOS、软件 I2C。
- operate：先画出调用链，再逐层写代码。
- register：这一节不直接操作寄存器。
- code：当前入口是 `MPU9250_Driver_Init()` 和 `ImuTask()`。
- check：能说清楚 `MPU9250 -> ImuService -> ImuTask -> qAttitude -> TelemetryTask`。
- debug：如果姿态没有输出，按链路逐段检查，不直接怀疑融合算法。
- next：先写软件 I2C。

## 2. 新手写九轴驱动的总顺序

不要一上来写姿态融合。融合算法只吃已经可靠的 `gyro / accel / mag` 物理量。正确顺序应该是：

1. **先让 I2C 通信可靠**  
   先写 `I2C_Start / I2C_Stop / I2C_SendByte / I2C_ReadByte / I2C_WaitAck / I2C_ReadRegs / I2C_WriteReg`。  
   当前工程在 [bsp_i2c_soft.c](E:/Stm32Project/AeroPush/User/Src/bsp_i2c_soft.c:29)。

2. **再确认 MPU9250 主芯片在线**  
   读取 `WHO_AM_I = 0x75`，正常值应为 `0x71`。  
   当前工程用 `MPU9250_Driver_ReadWhoAmI()` 和 `mpu9250_check_device()`。

3. **再软复位、唤醒、设置时钟源**  
   写 `PWR_MGMT_1 = 0x6B`：  
   `bit7` 软复位，`bit6` 清 0 唤醒，`CLKSEL=1` 选择陀螺仪 PLL。  
   当前工程在 [mpu9250_driver.c](E:/Stm32Project/AeroPush/User/Src/mpu9250_driver.c:698)。

4. **再配置 MPU9250 六轴部分**  
   包括：使能六轴、配置 DLPF、采样率、陀螺仪量程、加速度计量程。  
   当前工程集中在 `mpu9250_config_six_axis()`，位置是 [mpu9250_driver.c](E:/Stm32Project/AeroPush/User/Src/mpu9250_driver.c:628)。

5. **再确认能读到六轴 raw 数据**  
   从 `ACCEL_XOUT_H = 0x3B` 连续读 14 字节。  
   当前工程是 `MPU9250_ReadAxis()`。

6. **再把六轴 raw 转成物理量**  
   当前工程配置是：  
   加速度计 `+-8g`，所以 `raw / 4096.0f`；  
   陀螺仪 `+-1000dps`，所以 `raw / 32.8f`。  
   当前工程是 `MPU9250_ConvertToPhysical()`。

7. **再打开 AK8963 访问通道**  
   AK8963 不是 MPU9250 主寄存器，它是内部磁力计，需要打开 bypass。  
   当前工程用 `mpu_set_ak8963_by_mcu()`。

8. **再确认 AK8963 在线**  
   读取 AK8963 `WIA = 0x00`，正常值是 `0x48`。  
   当前工程是 `AK8963_CheckDeviceID()`。

9. **再配置 AK8963**  
   正确顺序是：  
   `Power Down -> Fuse ROM -> 读取 ASA -> Power Down -> 16bit 连续测量模式`。  
   当前工程在 `ak8963_init()`，位置是 [mpu9250_driver.c](E:/Stm32Project/AeroPush/User/Src/mpu9250_driver.c:648)。

10. **再确认能读到磁力计 raw 数据**  
    先看 `ST1.DRDY`，再连续读 `HXL..ST2`。  
    注意 AK8963 是**低字节在前**。  
    当前工程是 `AK8963_Read_Axis()`。

11. **再把磁力计 raw 转成物理量**  
    当前工程公式是：  
    `raw * ASA修正系数 * 0.15f`，单位是 `uT`。  
    当前工程是 `ak8963_convert_raw_to_ut()`。

12. **再做陀螺仪、加速度计、磁力计校准**  
    当前工程确实做了：  
    `MPU9250_CalibrateGyro(1000U, 10U)`  
    `MPU9250_CalibrateAccel(1000U, 10U)`  
    `AK8963_CalibrateMag(500U, 20U)`  
    调用位置在 [mpu9250_driver.c](E:/Stm32Project/AeroPush/User/Src/mpu9250_driver.c:1373)。

13. **A+M 初始姿态：当前工程没有真正做**  
    这一点要改口。  
    当前工程没有现役的 `MPU9250_ComputeEuler_FromAccMag()`，也没有用 accel + mag 先算初始 roll/pitch/yaw。  
    现在只是保留了旧注释，实际没有参与编译。  
    所以它只能写成：  
    **“A+M 初始姿态是推荐扩展，当前工程未实现。”**

14. **最后才把数据送入 Mahony 姿态融合算法**  
    当前工程实际做的是：  
    `ImuService_ReadPhys()` 读物理量；  
    磁力计有效就调用 `MPU9250_MahonyUpdate()`；  
    磁力计无效就调用 `MPU9250_MahonyUpdateIMU()`；  
    然后 `MPU9250_GetEulerFusedDeg()` 输出 roll/pitch/yaw。  
    这个逻辑在 `ImuTask()`，位置是 [app_tasks.c](E:/Stm32Project/AeroPush/User/Src/app_tasks.c:143)。

当前工程已经把这个顺序集中在 `User/Src/mpu9250_driver.c` 的 `MPU9250_Driver_Init()`：

```c
uint8_t MPU9250_Driver_Init(void)
{
    AppStatus_SetCalState(APP_CAL_STATE_SELF_CHECK);        // 进入上电自检灯语

    BSP_I2C_Soft_Init();                                    // 第一步：初始化软件 I2C

    if (mpu9250_check_device() != 0)                         // 第二步：读取 WHO_AM_I
    {
        Debug_Print("[MPU9250] driver init failed\r\n");     // 打印主芯片检查失败
        AppStatus_SetCalState(APP_CAL_STATE_FAIL);           // 设置失败灯语
        return 0U;                                           // 中断初始化
    }

    mpu9250_config_six_axis();                               // 第三步：配置六轴

    if (ak8963_init() == 0U)                                  // 第四步：初始化 AK8963
    {
        AppStatus_SetCalState(APP_CAL_STATE_FAIL);            // 磁力计失败也算 IMU 初始化失败
        return 0U;
    }

    AppStatus_SetCalState(APP_CAL_STATE_STATIC);              // 第五步：静止校准灯语
    MPU9250_CalibrateGyro(1000U, 10U);                        // 陀螺仪零偏校准
    MPU9250_CalibrateAccel(1000U, 10U);                       // 加速度计零偏校准
    AK8963_CalibrateMag(500U, 20U);                           // 磁力计椭球校准

    Debug_Print("[MPU9250] driver init ok\r\n");              // 初始化完成日志
    return 1U;                                                // 驱动初始化成功
}
```

注意：当前工程把 AK8963 初始化失败视为整个 IMU 初始化失败。这样做适合九轴项目，因为没有磁力计时 yaw 长期会漂；如果产品允许六轴模式启动，可以把 AK8963 失败降级为“只允许 6 轴融合”，但要在状态位里明确表示。

## 3. 软件 I2C 通信准备

软件 I2C 是用普通 GPIO 模拟 I2C 时序：SCL 是时钟线，SDA 是数据线。当前工程使用 PB6 / PB7：

| 项目 | 当前工程定义 | 位置 |
|---|---|---|
| I2C 端口 | `GPIOB` | `User/Inc/bsp_i2c_soft.h` |
| SCL | `GPIO_Pin_6` | `User/Inc/bsp_i2c_soft.h` |
| SDA | `GPIO_Pin_7` | `User/Inc/bsp_i2c_soft.h` |
| MPU9250 地址 | `0x68` | `User/Inc/mpu9250_driver.h` |
| AK8963 地址 | `0x0C` | `User/Inc/mpu9250_driver.h` |

所有初始化之前必须先保证 I2C 读写可靠，因为后面的 WHO_AM_I、复位、量程配置、六轴读取、AK8963 读取全部都是 I2C 寄存器事务。

## 3.1 软件 I2C 的起始条件和停止条件

- function：写出 `I2C_Start()` 和 `I2C_Stop()`。
- why：I2C 从机靠 START 判断一次事务开始，靠 STOP 回到空闲状态。
- if not：没有 START，从机不会解析地址；没有 STOP，下一次事务可能卡在错误状态。
- before：GPIO 已经配置成开漏输出，并且 SCL/SDA 释放为高。
- operate：START 是 SCL 高时 SDA 从高变低；STOP 是 SCL 高时 SDA 从低变高。
- register：软件 I2C 不操作外设 I2C 寄存器，只操作 GPIO BSRR/IDR。
- code：当前工程是 `User/Src/bsp_i2c_soft.c` 的 `I2C_Start()`、`I2C_Stop()`。
- check：示波器或逻辑分析仪能看到 START / STOP 波形。
- debug：如果总线卡低，先跑 `BSP_I2C_Soft_RecoverBus()`。
- next：实现字节发送、字节读取和 ACK。

当前工程代码：

```c
void I2C_Start(void)
{
    SDA_H();                 // 确保 SDA 先处于高电平
    SCL_H();                 // 确保 SCL 先处于高电平
    I2C_Delay();             // 保持总线空闲时间
    SDA_L();                 // SCL 高电平期间拉低 SDA，产生 START
    I2C_Delay();             // 保持起始条件
    SCL_L();                 // 拉低 SCL，准备发送数据
    I2C_Delay();             // 保持低电平时间
}

void I2C_Stop(void)
{
    SCL_L();                 // 先保证 SCL 为低
    SDA_L();                 // 先保证 SDA 为低
    I2C_Delay();             // 保持低电平时间
    SCL_H();                 // 拉高 SCL
    I2C_Delay();             // 等待 SCL 稳定
    SDA_H();                 // SCL 高电平期间释放 SDA，产生 STOP
    I2C_Delay();             // 保持停止条件
}
```

## 3.2 ACK、NACK、发送字节和读取字节

I2C 每发送 8 位数据后，都要有第 9 个时钟给接收方回 ACK/NACK。

- ACK：SDA 为低，表示接收方收到了这个字节；
- NACK：SDA 为高，表示接收方不继续接收，或主机读取最后一个字节时主动结束。

当前工程关键函数：

| 函数 | 文件 | 作用 |
|---|---|---|
| `I2C_SendByte()` | `User/Src/bsp_i2c_soft.c` | 从 bit7 到 bit0 发送一个字节 |
| `I2C_ReadByte()` | `User/Src/bsp_i2c_soft.c` | 从 bit7 到 bit0 读取一个字节 |
| `I2C_WaitAck()` | `User/Src/bsp_i2c_soft.c` | 主机释放 SDA 后读取从机 ACK |
| `I2C_SendACK()` | `User/Src/bsp_i2c_soft.c` | 主机读完一个字节后回复 ACK |
| `I2C_NACK()` / `I2C_SendNACK()` | `User/Src/bsp_i2c_soft.c` | 主机读完最后一个字节后回复 NACK |

推荐理解为：

```text
写方向：主机发地址 -> 从机 ACK -> 主机发寄存器 -> 从机 ACK -> 主机发数据 -> 从机 ACK
读方向：主机发地址写 -> 发寄存器 -> 重复 START -> 主机发地址读 -> 主机读数据 -> 最后 NACK
```

## 3.3 写一个寄存器的流程

当前工程用 `I2C_WriteReg()` 写单字节寄存器，它内部复用 `I2C_WriteRegs()`。

```c
int I2C_WriteReg(uint8_t dev7, uint8_t reg, uint8_t val)
{
    return I2C_WriteRegs(dev7, reg, &val, 1U);  // 单字节写复用连续写接口
}
```

`I2C_WriteRegs()` 的事务顺序是：

| 步骤 | 操作 | 失败返回 | 新手要检查 |
|---|---|---|---|
| 1 | START | 无 | 起始条件波形 |
| 2 | 发送 `(dev7 << 1) | 0` | `-1` | 设备地址、AD0、接线 |
| 3 | 发送寄存器地址 | `-2` | 寄存器地址是否正确 |
| 4 | 逐字节发送数据 | `-3` | 从机是否 ACK 每个字节 |
| 5 | STOP | 无 | 总线是否释放 |

## 3.4 读一个寄存器和连续读取多个寄存器的流程

当前工程提供三个读接口：

| 函数 | 返回方式 | 适合场景 |
|---|---|---|
| `I2C_ReadReg()` | 直接返回寄存器值，失败默认 `0xFF` | 简单调试读取 |
| `I2C_ReadRegData()` | 输出参数 + 错误码 | 需要判断失败原因 |
| `I2C_ReadRegs()` | 连续读取多个字节 | 六轴 14 字节、AK8963 HXL..ST2 |

当前工程读取 MPU9250 WHO_AM_I 使用 `I2C_ReadRegData()`，这是更适合新手驱动的写法，因为能区分“读到了错误 ID”和“I2C 事务失败”：

```c
uint8_t MPU9250_Driver_ReadWhoAmI(uint8_t *id)
{
    int ret;                                                                  // I2C 读取返回值

    if (id == 0)                                                              // 检查输出指针
    {
        return 0U;                                                            // 参数无效
    }

    ret = I2C_ReadRegData(MPU9250_I2C_ADDR7, MPU9250_REG_WHO_AM_I, id);        // 读取 WHO_AM_I
    return (ret == 0) ? 1U : 0U;                                               // 成功返回 1
}
```

注意：`User/Inc/mpu9250_driver.h` 里这个函数的 `@retval` 注释写成了“0 读取成功”，但 `User/Src/mpu9250_driver.c` 的实际实现是“1 成功，0 失败”。写新代码时以实现为准，建议后续修正文档注释。

## 3.5 为什么六轴和磁力计都建议连续读

六轴数据从 `ACCEL_XOUT_H` 到 `GYRO_ZOUT_L` 连续排列。当前工程一次读 14 字节，避免分多次读造成同一帧数据被拆散：

```c
I2C_ReadRegs(MPU9250_I2C_ADDR7,
             MPU9250_ACCEL_XOUT_H_REG,
             frame,
             (uint16_t)sizeof(frame));  // 一次读取加速度、温度和陀螺仪
```

AK8963 也要连续读，而且要读到 `ST2`。当前工程一次读取 `HXL..ST2` 共 7 字节：

```c
I2C_ReadRegs(AK8963_I2C_ADDR7,
             AK8963_REG_HXL,
             frame,
             (uint16_t)sizeof(frame));  // 连续读取 HXL 到 ST2
```

磁力计读取后必须读 `ST2`，否则 AK8963 可能不释放下一帧锁存数据，后续 `DRDY` 或数据更新会异常。

## 4. MPU9250 主芯片通信检测

配置六轴之前必须先读 `WHO_AM_I`。它是芯片固定 ID 寄存器，最适合判断 I2C 地址、接线、供电、时序是否基本正确。

- function：读取 `MPU9250_REG_WHO_AM_I` 并判断是否等于 `MPU9250_WHO_AM_I_VALUE`。
- why：如果主芯片都不在线，后续写复位、唤醒、量程寄存器没有意义。
- if not：可能在错误设备上写寄存器，或者初始化假成功但后续全是零值。
- before：软件 I2C GPIO 已初始化，总线空闲。
- operate：用 `I2C_ReadRegData(0x68, 0x75, &id)` 读取。
- register：见下表。
- code：当前工程 `MPU9250_Driver_ReadWhoAmI()` + `mpu9250_check_device()`。
- check：日志出现 `[MPU9250] WHO_AM_I = 0x71` 和 `WHO_AM_I check ok`。
- debug：如果不是 `0x71`，检查地址、AD0、SDA/SCL、上拉、电源和芯片型号。
- next：通信确认后执行软复位。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| WHO_AM_I | `0x75` | `bit[7:0]` | 设备 ID | 只读，正常 `0x71` | 标识 MPU9250 主芯片 | `mpu9250_check_device()` 判断是否继续初始化 |

当前工程真实代码：

```c
static int mpu9250_check_device(void)
{
    uint8_t id = 0U;                                                   // 保存 WHO_AM_I 读取结果

    if (MPU9250_Driver_ReadWhoAmI(&id) == 0U)                           // 读取 MPU9250 设备 ID
    {
        Debug_Print("[MPU9250] WHO_AM_I read failed\r\n");              // I2C 事务失败
        return -1;
    }

    Debug_Printf("[MPU9250] WHO_AM_I = 0x%02X\r\n", id);                // 打印实际 ID

    if (id != MPU9250_WHO_AM_I_VALUE)                                   // 判断是否等于 0x71
    {
        Debug_Print("[MPU9250] WHO_AM_I value error\r\n");              // ID 不匹配
        return -2;
    }

    Debug_Print("[MPU9250] WHO_AM_I check ok\r\n");                     // 主芯片在线
    return 0;
}
```

如果当前工程没有单独对外的 `MPU9250_CheckWhoAmI()`，推荐新增一个更直观的包装函数。下面是推荐整理后的写法，不一定完全等同于当前工程原代码：

```c
/**
 * @brief  检查 MPU9250 主芯片 WHO_AM_I。
 * @retval 0 检查成功；负数表示 I2C 失败或 ID 不匹配。
 */
int MPU9250_CheckWhoAmI(void)
{
    uint8_t id = 0U;                                                    // 保存设备 ID

    if (I2C_ReadRegData(MPU9250_I2C_ADDR7, MPU9250_REG_WHO_AM_I, &id) != 0)
    {
        return -1;                                                      // I2C 读寄存器失败
    }

    if (id != MPU9250_WHO_AM_I_VALUE)
    {
        return -2;                                                      // 读到 ID，但不是 MPU9250
    }

    return 0;                                                           // 通信和 ID 都正确
}
```

## 5. 配置六轴初始化

六轴初始化指 MPU9250 内部 MPU6500 部分：

- 三轴加速度计；
- 三轴陀螺仪。

当前工程由 `mpu9250_config_six_axis()` 串起来：

```c
static void mpu9250_config_six_axis(void)
{
    MPU9250_SoftReset();         // 软件复位 MPU9250
    mpu_set_clock_to_auto();     // 唤醒芯片并选择时钟源
    mpu_enable_six_axis();       // 使能三轴加速度计和三轴陀螺仪
    mpu_set_dlpf_cfg_3();        // 配置陀螺仪低通滤波
    mpu_set_sample_rate_200hz(); // 配置六轴采样率为 200Hz
    mpu_set_accel_dlpf();        // 配置加速度计低通滤波
    mpu_set_gyro_config();       // 配置陀螺仪量程
    mpu_set_accel_range();       // 配置加速度计量程
    MPU9250_Read_PowerMgmt();    // 读取电源管理寄存器用于调试确认
}
```

## 5.1 软复位

软复位是通过写 `PWR_MGMT_1` 的 bit7，让 MPU9250 内部寄存器恢复默认状态。初始化第一步通常要做软复位，因为芯片可能经历过上一次运行、调试器复位、异常中断或总线错误，寄存器状态不一定干净。

- function：触发 MPU9250 内部寄存器复位。
- why：避免沿用旧配置，比如睡眠位、量程、滤波、I2C master 状态。
- if not：后续写入看似成功，但芯片可能仍处在睡眠、standby 或旧滤波配置中。
- before：WHO_AM_I 已经通过，确认主芯片在线。
- operate：读 `PWR_MGMT_1`，置位 bit7，再写回。
- register：见下表。
- code：当前工程函数 `MPU9250_SoftReset()`。
- check：复位后延时 100ms，再继续唤醒配置。
- debug：复位后 WHO_AM_I 仍应能读到 `0x71`，否则检查供电和 I2C。
- next：清除 SLEEP 位，选择时钟源。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| PWR_MGMT_1 | `0x6B` | bit7 | H_RESET / DEVICE_RESET | `1` | 触发软复位 | 初始化开始时恢复默认状态 |

当前工程真实代码：

```c
void MPU9250_SoftReset(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG); // 读取 PWR_MGMT_1 当前值

    val &= (uint8_t)~(1U << 7U);                                          // 先清除复位位，保证写入动作明确
    val |= (uint8_t)(1U << 7U);                                           // 设置 bit7，触发内部寄存器软复位
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG, val);          // 写回 PWR_MGMT_1，开始复位

    vTaskDelay(pdMS_TO_TICKS(100));                                       // 等待复位完成，避免后续配置过早执行
}
```

bit7 写 1 后会由芯片内部复位流程自动清零。代码不需要手动清零，但要延时等待内部复位完成。

## 5.2 唤醒 MPU9250

软复位后，`PWR_MGMT_1` 的 `SLEEP` 位可能处于默认睡眠状态。睡眠模式下陀螺仪和加速度计不会正常输出，所以必须清除 bit6。

- function：清除 `PWR_MGMT_1.SLEEP`。
- why：让 MPU6500 六轴部分开始工作。
- if not：后续读 raw 可能一直为 0、固定值或不更新。
- before：软复位已完成，至少等待 100ms。
- operate：读改写 `PWR_MGMT_1`，清 bit6。
- register：见下表。
- code：当前工程在 `mpu_set_clock_to_auto()` 中同时完成唤醒和时钟选择。
- check：`MPU9250_Read_PowerMgmt()` 打印的 `PWR_MGMT_1` 中 bit6 应为 0。
- debug：如果仍是睡眠，检查写寄存器 ACK 和写后延时。
- next：设置 `CLKSEL`。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| PWR_MGMT_1 | `0x6B` | bit6 | SLEEP | `0` | 退出睡眠模式 | `mpu_set_clock_to_auto()` 清除该位 |

当前工程不是直接写整个 `PWR_MGMT_1`，而是读改写：

```c
void mpu_set_clock_to_auto(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG); // 读取电源管理寄存器

    val &= (uint8_t)~((1U << 6U) | 0x07U);                                // 清除 SLEEP 和 CLKSEL
    val |= 0x01U;                                                         // 选择 X 轴陀螺仪 PLL
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG, val);          // 写回唤醒和时钟配置
}
```

## 5.3 设置时钟源

`PWR_MGMT_1` 的 bit[2:0] 是 `CLKSEL`。内部振荡器可以工作，但姿态解算依赖陀螺积分，时钟抖动和稳定性会影响积分一致性。通常推荐选择陀螺仪 PLL 时钟源。

- function：设置 `CLKSEL=1`。
- why：使用 X 轴陀螺仪 PLL，稳定性通常好于内部振荡器。
- if not：短时间能读数，但长期姿态积分稳定性更差。
- before：芯片已经唤醒。
- operate：清 `CLKSEL` 三位，再写入 `0x01`。
- register：见下表。
- code：当前工程 `mpu_set_clock_to_auto()`。
- check：读回 `PWR_MGMT_1`，低三位应为 `001`。
- debug：如果低三位不对，检查写寄存器返回值。
- next：确保六轴未 standby。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| PWR_MGMT_1 | `0x6B` | bit[2:0] | CLKSEL | `001b` | 选择 X 轴陀螺仪 PLL | 姿态融合更适合稳定时钟 |

## 5.4 使能六个轴，确保没有 standby

`PWR_MGMT_2` 控制加速度计 X/Y/Z 和陀螺仪 X/Y/Z 是否进入 standby。六轴初始化时要确保六个轴都开启。

- function：清除 `PWR_MGMT_2` 的六个 standby 位。
- why：后续要读取完整 6 轴数据。
- if not：某个轴 raw 可能固定为 0 或不更新，融合会被单轴错误拖偏。
- before：芯片已唤醒。
- operate：读 `PWR_MGMT_2`，清低 6 位。
- register：见下表。
- code：当前工程 `mpu_enable_six_axis()`。
- check：读回 `PWR_MGMT_2`，低 6 位应为 0。
- debug：如果某轴长期为 0，先查是否 standby。
- next：配置 DLPF 和采样率。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| PWR_MGMT_2 | `0x6C` | bit5 | STBY_XA | `0` | 使能 X 加速度 | 清除低 6 位 |
| PWR_MGMT_2 | `0x6C` | bit4 | STBY_YA | `0` | 使能 Y 加速度 | 清除低 6 位 |
| PWR_MGMT_2 | `0x6C` | bit3 | STBY_ZA | `0` | 使能 Z 加速度 | 清除低 6 位 |
| PWR_MGMT_2 | `0x6C` | bit2 | STBY_XG | `0` | 使能 X 陀螺仪 | 清除低 6 位 |
| PWR_MGMT_2 | `0x6C` | bit1 | STBY_YG | `0` | 使能 Y 陀螺仪 | 清除低 6 位 |
| PWR_MGMT_2 | `0x6C` | bit0 | STBY_ZG | `0` | 使能 Z 陀螺仪 | 清除低 6 位 |

当前工程代码：

```c
void mpu_enable_six_axis(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_2_REG); // 读取六轴待机控制寄存器

    val &= (uint8_t)~0x3FU;                                               // 清除六个轴的 standby 位
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_2_REG, val);          // 写回六轴使能配置
}
```

## 5.5 配置陀螺仪 DLPF 和采样率

DLPF 是低通滤波。陀螺仪原始数据会有高频噪声，DLPF 可以降低噪声，但滤波越强响应越慢。`CONFIG` 寄存器控制陀螺仪 DLPF，`SMPLRT_DIV` 控制采样率分频。

- function：设置陀螺仪 DLPF 和采样率。
- why：给姿态融合提供噪声和响应折中的角速度。
- if not：不滤波会抖，采样率不合适会浪费 CPU 或响应慢。
- before：六轴已使能。
- operate：`CONFIG.DLPF_CFG=3`，`SMPLRT_DIV=4`。
- register：见下表。
- code：当前工程 `mpu_set_dlpf_cfg_3()` 和 `mpu_set_sample_rate_200hz()`。
- check：静止时陀螺仪 dps 应接近 0，轻微抖动可接受。
- debug：如果噪声大，降低带宽；如果响应慢，提高带宽或采样率。
- next：配置加速度计 DLPF。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| CONFIG | `0x1A` | bit[2:0] | DLPF_CFG | `3` | 设置陀螺仪低通滤波档位 | `mpu_set_dlpf_cfg_3()` |
| SMPLRT_DIV | `0x19` | bit[7:0] | SMPLRT_DIV | `0x04` | 1kHz / (1 + 4) = 200Hz | `mpu_set_sample_rate_200hz()` |

当前工程代码：

```c
void mpu_set_dlpf_cfg_3(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_CONFIG_REG);     // 读取陀螺仪滤波配置

    val = (uint8_t)((val & (uint8_t)~0x07U) | 0x03U);                     // 设置 DLPF_CFG=3
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_CONFIG_REG, val);             // 写回陀螺仪滤波配置
}

void mpu_set_sample_rate_200hz(void)
{
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_SMPLRT_DIV_REG, 0x04U);        // 1kHz 基准下分频为 200Hz
}
```

当前 FreeRTOS `ImuTask` 周期是 `APP_IMU_TASK_PERIOD_MS = 20ms`，也就是 50Hz。传感器内部 200Hz，任务 50Hz，是合理的入门配置：传感器数据比任务快，任务每次读到的是近期更新值，同时软件 I2C 和 CPU 压力不至于太高。如果把 `ImuTask` 改成 5ms，就要重新评估软件 I2C 读 14 字节 + 7 字节是否会超时。

## 5.6 配置加速度计 DLPF 和采样率

加速度计很容易受振动影响。无人机、风扇、电机、桌面敲击都会让加速度计出现高频扰动。加速度计 DLPF 太弱，roll/pitch 会抖；太强，姿态收敛会变慢。

- function：设置 `ACCEL_CONFIG2`。
- why：给融合提供更平滑的重力方向估计。
- if not：静止时加速度模长抖动大，Mahony 修正项不稳定。
- before：采样率和六轴使能已配置。
- operate：`A_DLPFCFG=3`，清除 `ACCEL_FCHOICE_B`。
- register：见下表。
- code：当前工程 `mpu_set_accel_dlpf()`。
- check：静止时 `sqrt(ax^2 + ay^2 + az^2)` 接近 1g。
- debug：如果振动明显，先看机械安装，再调整 DLPF。
- next：配置陀螺仪量程。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| ACCEL_CONFIG2 | `0x1D` | bit3 | ACCEL_FCHOICE_B | `0` | 使用加速度计 DLPF | `mpu_set_accel_dlpf()` 清除 |
| ACCEL_CONFIG2 | `0x1D` | bit[2:0] | A_DLPFCFG | `3` | 加速度计低通滤波档位 | 当前与陀螺仪一致用 3 |

当前工程代码：

```c
void mpu_set_accel_dlpf(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG2_REG); // 读取加速度计滤波配置

    val &= (uint8_t)~0x07U;                                                  // 清除 A_DLPFCFG 位
    val |= 0x03U;                                                            // 设置加速度计 DLPF_CFG=3
    val &= (uint8_t)~(1U << 3U);                                             // 关闭加速度计高带宽旁路
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG2_REG, val);          // 写回加速度计滤波配置
}
```

## 5.7 配置陀螺仪量程

`GYRO_CONFIG` 的 `FS_SEL` 决定 raw 转 dps 的比例。

| 量程 | FS_SEL | LSB/dps | 适合场景 |
|---|---|---|---|
| `+-250 dps` | `0` | `131.0` | 低动态、静态姿态、精度优先 |
| `+-500 dps` | `1` | `65.5` | 一般运动 |
| `+-1000 dps` | `2` | `32.8` | 当前工程，兼顾动态和分辨率 |
| `+-2000 dps` | `3` | `16.4` | 快速旋转，分辨率最低 |

- function：设置陀螺仪量程为 `+-1000dps`。
- why：AeroPush 当前更像飞行器姿态场景，保留较大角速度余量。
- if not：量程太小会饱和，量程太大静止分辨率变差。
- before：DLPF 已配置。
- operate：`GYRO_CONFIG.FS_SEL=2`。
- register：见下表。
- code：当前工程 `mpu_set_gyro_config()`。
- check：快速转动时 raw 不应频繁打满 `32767/-32768`。
- debug：如果角速度饱和，提高量程；如果静止噪声换算后太大，考虑降低量程。
- next：配置加速度计量程。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| GYRO_CONFIG | `0x1B` | bit[4:3] | FS_SEL | `2` | `+-1000dps` | `MPU9250_GYRO_LSB_PER_DPS = 32.8f` |

当前工程代码：

```c
void mpu_set_gyro_config(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_GYRO_CONFIG_REG); // 读取陀螺仪量程配置

    val &= (uint8_t)~0x03U;                                                // 清除自检相关低位
    val &= (uint8_t)~(3U << 3U);                                           // 清除 FS_SEL 量程位
    val |= (uint8_t)(2U << 3U);                                            // 设置 FS_SEL=2，对应 +-1000dps
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_GYRO_CONFIG_REG, val);          // 写回陀螺仪量程配置
}
```

## 5.8 配置加速度计量程

`ACCEL_CONFIG` 的 `AFS_SEL` 决定 raw 转 g 的比例。

| 量程 | AFS_SEL | LSB/g | 适合场景 |
|---|---|---|---|
| `+-2g` | `0` | `16384` | 静态倾角，精度最高，动态余量小 |
| `+-4g` | `1` | `8192` | 中低动态 |
| `+-8g` | `2` | `4096` | 当前工程，适合有明显运动和振动的场景 |
| `+-16g` | `3` | `2048` | 高冲击场景，分辨率最低 |

- function：设置加速度计量程为 `+-8g`。
- why：给动态运动留余量，避免震动或机动时饱和。
- if not：量程太小会饱和，融合会把错误加速度当成重力。
- before：加速度计 DLPF 已配置。
- operate：`ACCEL_CONFIG.AFS_SEL=2`。
- register：见下表。
- code：当前工程 `mpu_set_accel_range()`。
- check：静止水平时某一轴接近 `1g`，raw 约 `4096`。
- debug：如果静止时模长远离 1g，先看量程常量是否匹配。
- next：检查六轴初始化是否成功。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| ACCEL_CONFIG | `0x1C` | bit[4:3] | AFS_SEL | `2` | `+-8g` | `MPU9250_ACCEL_LSB_PER_G = 4096.0f` |

当前工程代码：

```c
void mpu_set_accel_range(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG_REG); // 读取加速度计量程配置

    val &= (uint8_t)~(3U << 3U);                                           // 清除 AFS_SEL 量程位
    val |= (uint8_t)(2U << 3U);                                            // 设置 AFS_SEL=2，对应 +-8g
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG_REG, val);         // 写回加速度计量程配置
}
```

## 5.9 检查六轴初始化是否成功

六轴初始化完成后，不要马上相信它。按下面顺序检查：

| 检查项 | 成功标志 | 当前工程位置 |
|---|---|---|
| WHO_AM_I | `0x71` | `mpu9250_check_device()` |
| PWR_MGMT_1 | `SLEEP=0`，`CLKSEL=1` | `MPU9250_Read_PowerMgmt()` |
| PWR_MGMT_2 | 低 6 位为 0 | `mpu_enable_six_axis()` |
| 六轴 raw | 14 字节能读到且不是全 0 | `MPU9250_ReadAxis()` |
| 加速度物理量 | 静止时模长约 1g | `MPU9250_ConvertToPhysical()` |
| 陀螺仪物理量 | 静止时接近 0dps | `MPU9250_CalibrateGyro()` 后 |

当前工程会打印：

```text
[MPU9250] WHO_AM_I = 0x71
[MPU9250] WHO_AM_I check ok
[MPU9250] PWR_MGMT_1 = 0x..
```

如果要更适合新手调试，推荐新增一次性六轴检查函数。下面是推荐整理后的写法，不一定完全等同于当前工程原代码：

```c
/**
 * @brief  读取一帧六轴物理量并做最小合理性检查。
 * @retval 0 检查通过；负数表示读数异常。
 */
int MPU9250_CheckSixAxisFrame(void)
{
    MPU9250_raw_Data raw;                                                 // 保存六轴 raw
    MPU9250_Physical_Data phys;                                           // 保存物理量
    float acc_norm;                                                       // 加速度模长

    MPU9250_ReadAxis(&raw);                                               // 当前工程函数没有错误返回
    MPU9250_ConvertToPhysical(&raw, &phys);                               // 转换为 g 和 dps

    acc_norm = sqrtf(phys.accel_x_g * phys.accel_x_g +
                     phys.accel_y_g * phys.accel_y_g +
                     phys.accel_z_g * phys.accel_z_g);                   // 计算静止重力模长

    if ((acc_norm < 0.6f) || (acc_norm > 1.4f))
    {
        return -1;                                                        // 静止时明显不像 1g
    }

    return 0;                                                             // 六轴读数基本可信
}
```

## 6. 读取六轴原始数据

读取六轴 raw 是驱动的第一个运行时核心动作。只有 raw 读对，后面的单位转换、零偏校准、Mahony 融合才有意义。

## 6.1 六轴数据寄存器排列

MPU9250 六轴数据从 `ACCEL_XOUT_H` 开始排列：

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| ACCEL_XOUT_H | `0x3B` | bit[7:0] | AX_H | 只读 | X 加速度高字节 | `frame[0]` |
| ACCEL_XOUT_L | `0x3C` | bit[7:0] | AX_L | 只读 | X 加速度低字节 | `frame[1]` |
| ACCEL_YOUT_H | `0x3D` | bit[7:0] | AY_H | 只读 | Y 加速度高字节 | `frame[2]` |
| ACCEL_YOUT_L | `0x3E` | bit[7:0] | AY_L | 只读 | Y 加速度低字节 | `frame[3]` |
| ACCEL_ZOUT_H | `0x3F` | bit[7:0] | AZ_H | 只读 | Z 加速度高字节 | `frame[4]` |
| ACCEL_ZOUT_L | `0x40` | bit[7:0] | AZ_L | 只读 | Z 加速度低字节 | `frame[5]` |
| TEMP_OUT_H | `0x41` | bit[7:0] | TEMP_H | 只读 | 温度高字节 | `frame[6]` |
| TEMP_OUT_L | `0x42` | bit[7:0] | TEMP_L | 只读 | 温度低字节 | `frame[7]` |
| GYRO_XOUT_H | `0x43` | bit[7:0] | GX_H | 只读 | X 陀螺仪高字节 | `frame[8]` |
| GYRO_XOUT_L | `0x44` | bit[7:0] | GX_L | 只读 | X 陀螺仪低字节 | `frame[9]` |
| GYRO_YOUT_H | `0x45` | bit[7:0] | GY_H | 只读 | Y 陀螺仪高字节 | `frame[10]` |
| GYRO_YOUT_L | `0x46` | bit[7:0] | GY_L | 只读 | Y 陀螺仪低字节 | `frame[11]` |
| GYRO_ZOUT_H | `0x47` | bit[7:0] | GZ_H | 只读 | Z 陀螺仪高字节 | `frame[12]` |
| GYRO_ZOUT_L | `0x48` | bit[7:0] | GZ_L | 只读 | Z 陀螺仪低字节 | `frame[13]` |

一次连续读取 14 字节更合适，因为：

- 六轴和温度来自同一帧寄存器窗口；
- 分开读可能导致加速度是上一帧、陀螺仪是下一帧；
- 软件 I2C 分多次 START/STOP 更慢；
- 代码更容易统一处理读取失败。

## 6.2 高低字节组合

MPU9250 六轴数据是高字节在前。每个轴是有符号 16 位补码，必须合成 `int16_t`。如果错误地用 `uint16_t`，负方向的加速度或角速度会被解释成很大的正数，姿态融合会直接错误。

当前工程代码：

```c
raw->accel_x = (int16_t)(((uint16_t)frame[0] << 8U) | (uint16_t)frame[1]);  // 高字节在前，合成 X 加速度
raw->accel_y = (int16_t)(((uint16_t)frame[2] << 8U) | (uint16_t)frame[3]);  // 合成 Y 加速度
raw->accel_z = (int16_t)(((uint16_t)frame[4] << 8U) | (uint16_t)frame[5]);  // 合成 Z 加速度
raw->temp    = (int16_t)(((uint16_t)frame[6] << 8U) | (uint16_t)frame[7]);  // 合成温度
raw->gyro_x  = (int16_t)(((uint16_t)frame[8] << 8U) | (uint16_t)frame[9]);  // 合成 X 陀螺仪
raw->gyro_y  = (int16_t)(((uint16_t)frame[10] << 8U) | (uint16_t)frame[11]); // 合成 Y 陀螺仪
raw->gyro_z  = (int16_t)(((uint16_t)frame[12] << 8U) | (uint16_t)frame[13]); // 合成 Z 陀螺仪
```

## 6.3 六轴原始数据读取函数

当前工程函数是 `MPU9250_ReadAxis(MPU9250_raw_Data *raw)`。

| 项目 | 当前工程实现 |
|---|---|
| 函数名 | `MPU9250_ReadAxis` |
| 输入参数 | `MPU9250_raw_Data *raw`，输出缓存 |
| 输出参数 | `raw->accel_x/y/z`、`raw->gyro_x/y/z`、`raw->temp` |
| 返回值 | `void` |
| 失败处理 | I2C 读失败时把本帧 raw 全部清 0 |
| 是否适合 ImuTask 高频调用 | 可以，当前 `ImuTask` 20ms 调一次 |
| 超时保护 | 底层软件 I2C 没有显式超时，只靠固定时序 |
| 错误返回 | 当前没有，推荐新手项目改成 `int` 返回错误码 |

当前工程真实代码：

```c
void MPU9250_ReadAxis(MPU9250_raw_Data *raw)
{
    uint8_t frame[14];                                                    // ACCEL_XOUT_H 到 GYRO_ZOUT_L

    if (raw == 0)                                                         // 检查输出结构体指针
    {
        return;                                                           // 参数无效
    }

    if (I2C_ReadRegs(MPU9250_I2C_ADDR7, MPU9250_ACCEL_XOUT_H_REG, frame, (uint16_t)sizeof(frame)) != 0)
    {
        raw->accel_x = 0;                                                 // 读取失败时清空本帧
        raw->accel_y = 0;
        raw->accel_z = 0;
        raw->gyro_x = 0;
        raw->gyro_y = 0;
        raw->gyro_z = 0;
        raw->temp = 0;
        return;
    }

    raw->accel_x = (int16_t)(((uint16_t)frame[0] << 8U) | (uint16_t)frame[1]);
    raw->accel_y = (int16_t)(((uint16_t)frame[2] << 8U) | (uint16_t)frame[3]);
    raw->accel_z = (int16_t)(((uint16_t)frame[4] << 8U) | (uint16_t)frame[5]);
    raw->temp = (int16_t)(((uint16_t)frame[6] << 8U) | (uint16_t)frame[7]);
    raw->gyro_x = (int16_t)(((uint16_t)frame[8] << 8U) | (uint16_t)frame[9]);
    raw->gyro_y = (int16_t)(((uint16_t)frame[10] << 8U) | (uint16_t)frame[11]);
    raw->gyro_z = (int16_t)(((uint16_t)frame[12] << 8U) | (uint16_t)frame[13]);
}
```

推荐整理后的写法，不一定完全等同于当前工程原代码：

```c
/**
 * @brief  连续读取 MPU9250 六轴和温度 raw 数据。
 * @param  raw 输出 raw 数据。
 * @retval 0 成功；负数表示参数无效或 I2C 读取失败。
 */
int MPU9250_ReadAxisChecked(MPU9250_raw_Data *raw)
{
    uint8_t frame[14];                                                    // 六轴 + 温度连续帧

    if (raw == 0)
    {
        return -1;                                                        // 输出指针无效
    }

    if (I2C_ReadRegs(MPU9250_I2C_ADDR7, MPU9250_ACCEL_XOUT_H_REG, frame, sizeof(frame)) != 0)
    {
        return -2;                                                        // I2C 连续读取失败
    }

    raw->accel_x = (int16_t)(((uint16_t)frame[0] << 8U) | frame[1]);       // X 加速度 raw
    raw->accel_y = (int16_t)(((uint16_t)frame[2] << 8U) | frame[3]);       // Y 加速度 raw
    raw->accel_z = (int16_t)(((uint16_t)frame[4] << 8U) | frame[5]);       // Z 加速度 raw
    raw->temp    = (int16_t)(((uint16_t)frame[6] << 8U) | frame[7]);       // 温度 raw
    raw->gyro_x  = (int16_t)(((uint16_t)frame[8] << 8U) | frame[9]);       // X 陀螺仪 raw
    raw->gyro_y  = (int16_t)(((uint16_t)frame[10] << 8U) | frame[11]);     // Y 陀螺仪 raw
    raw->gyro_z  = (int16_t)(((uint16_t)frame[12] << 8U) | frame[13]);     // Z 陀螺仪 raw

    return 0;                                                             // 读取成功
}
```

## 7. 六轴原始数据转换为物理量

raw 是寄存器里的补码计数值，不带单位。姿态融合不能直接使用 raw，必须根据量程转换为物理量。

## 7.1 加速度计 raw 转 g

- function：把 `accel_x/y/z` raw 转成 `accel_x/y/z_g`。
- why：Mahony 使用归一化加速度方向，但零偏校准和调试都需要 g 单位。
- if not：量纲错误会让 1g 变成 4096 或 0.00024，融合判断全错。
- before：加速度计量程已设置为 `+-8g`。
- operate：`accel_g = raw / 4096.0f - accel_bias`。
- register：量程由 `ACCEL_CONFIG.AFS_SEL=2` 决定。
- code：当前工程 `MPU9250_ConvertToPhysical()`。
- check：静止水平时某一轴约 `+1g` 或 `-1g`，模长约 1。
- debug：如果模长明显不是 1，检查量程和 `MPU9250_ACCEL_LSB_PER_G`。
- next：转换陀螺仪。

当前工程比例系数：

```c
#define MPU9250_ACCEL_LSB_PER_G 4096.0f // 加速度计 +-8g 量程下每 g 对应 LSB
```

转换代码：

```c
physical->accel_x_g = (float)raw->accel_x / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[0]; // X 轴 g 值
physical->accel_y_g = (float)raw->accel_y / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[1]; // Y 轴 g 值
physical->accel_z_g = (float)raw->accel_z / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[2]; // Z 轴 g 值
```

## 7.2 陀螺仪 raw 转 dps

- function：把 `gyro_x/y/z` raw 转成 `gyro_x/y/z_dps`。
- why：陀螺仪输出是角速度，Mahony 内部再从 dps 转 rad/s 积分四元数。
- if not：dps 和 rad/s 混用会导致姿态积分速度错误约 57.3 倍。
- before：陀螺仪量程已设置为 `+-1000dps`。
- operate：`gyro_dps = raw / 32.8f - gyro_bias`。
- register：量程由 `GYRO_CONFIG.FS_SEL=2` 决定。
- code：当前工程 `MPU9250_ConvertToPhysical()`。
- check：静止时三轴应接近 0dps。
- debug：如果静止仍有固定偏差，检查 `MPU9250_CalibrateGyro()` 是否在静止状态执行。
- next：转换温度。

当前工程比例系数：

```c
#define MPU9250_GYRO_LSB_PER_DPS 32.8f // 陀螺仪 +-1000dps 量程下每 dps 对应 LSB
```

转换代码：

```c
physical->gyro_x_dps = (float)raw->gyro_x / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[0]; // X 轴 dps
physical->gyro_y_dps = (float)raw->gyro_y / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[1]; // Y 轴 dps
physical->gyro_z_dps = (float)raw->gyro_z / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[2]; // Z 轴 dps
```

## 7.3 温度 raw 转摄氏度

温度寄存器夹在加速度计和陀螺仪之间，所以连续读 14 字节时会自然读到它。当前工程转换公式是：

```c
physical->temp_c = ((float)raw->temp / 333.87f) + 21.0f; // 转换芯片温度，单位摄氏度
```

当前工程没有把温度用于动态补偿，只保存到 `MPU9250_Physical_Data.temp_c`。温度变化会影响陀螺仪零偏，如果产品要长期户外运行，可以后续建立“温度 -> gyro bias”的补偿表；当前阶段先靠上电静止校准处理。

## 7.4 六轴物理量结构体

当前结构体在 `User/Inc/mpu9250_driver.h`：

```c
typedef struct
{
    float accel_x_g;   // 加速度 X 轴，单位 g
    float accel_y_g;   // 加速度 Y 轴，单位 g
    float accel_z_g;   // 加速度 Z 轴，单位 g
    float gyro_x_dps;  // 角速度 X 轴，单位 dps
    float gyro_y_dps;  // 角速度 Y 轴，单位 dps
    float gyro_z_dps;  // 角速度 Z 轴，单位 dps
    float temp_c;      // 温度，单位摄氏度
} MPU9250_Physical_Data;
```

| 字段名 | 含义 | 单位 | 来源 raw | 转换公式 | 备注 |
|---|---|---|---|---|---|
| `accel_x_g` | X 轴加速度 | g | `raw->accel_x` | `raw / 4096 - bias` | 当前量程 `+-8g` |
| `accel_y_g` | Y 轴加速度 | g | `raw->accel_y` | `raw / 4096 - bias` | Mahony 中会取反统一方向 |
| `accel_z_g` | Z 轴加速度 | g | `raw->accel_z` | `raw / 4096 - bias` | 静止水平通常接近 1g |
| `gyro_x_dps` | X 轴角速度 | dps | `raw->gyro_x` | `raw / 32.8 - bias` | Mahony 中转 rad/s |
| `gyro_y_dps` | Y 轴角速度 | dps | `raw->gyro_y` | `raw / 32.8 - bias` | Mahony 中会取反统一方向 |
| `gyro_z_dps` | Z 轴角速度 | dps | `raw->gyro_z` | `raw / 32.8 - bias` | yaw 积分主要依赖 |
| `temp_c` | 芯片温度 | degC | `raw->temp` | `raw / 333.87 + 21` | 当前只保存，不参与补偿 |

## 8. 配置磁力计初始化

AK8963 是 MPU9250 内部挂载的独立磁力计芯片，不是 MPU9250 主寄存器的一部分。新手最容易犯的错误是只初始化 MPU9250 主芯片，然后直接读磁力计地址，结果 AK8963 一直读不到。

当前工程磁力计初始化入口是 `ak8963_init()`：

```c
static uint8_t ak8963_init(void)
{
    mpu_set_ak8963_by_mcu();                         // 打开旁路访问

    if (AK8963_CheckDeviceID() != 0)                  // 检查 AK8963 ID
    {
        Debug_Print("[AK8963] device ID check failed\r\n");
        return 0U;
    }

    AK8963_EnterPowerDownMode();                      // 模式切换前先掉电
    AK8963_EnterFuseROMMode();                        // 进入 Fuse ROM
    AK8963_AdjustSensitivity();                       // 读取 ASA 并计算灵敏度系数
    AK8963_EnterPowerDownMode();                      // 退出 Fuse ROM 后回到掉电
    AK8963_EnterContinuousMeasurementMode();          // 进入 16bit 连续测量模式

    if (AK8963_CheckDataReady() == 0)                  // 检查是否有新数据
    {
        Debug_Print("[AK8963] data not ready\r\n");
        return 0U;
    }

    Debug_Print("[AK8963] data ready\r\n");
    return 1U;
}
```

## 8.1 访问 AK8963

MCU 不能直接像读 MPU9250 主寄存器那样读 AK8963，原因是 AK8963 挂在 MPU9250 内部辅助 I2C 总线上。当前工程使用 bypass 模式，让外部 MCU 通过 MPU9250 旁路直接访问 `0x0C`。

- function：关闭 MPU9250 内部 I2C master，打开 BYPASS_EN。
- why：让 MCU 的软件 I2C 能直接访问 AK8963。
- if not：MPU9250 WHO_AM_I 正常，但 AK8963 WIA 读不到或读 `0xFF`。
- before：MPU9250 主芯片已经唤醒。
- operate：`USER_CTRL.I2C_MST_EN=0`，`INT_PIN_CFG.BYPASS_EN=1`。
- register：见下表。
- code：当前工程 `mpu_set_ak8963_by_mcu()`。
- check：读 `AK8963_REG_WIA` 返回 `0x48`。
- debug：如果失败，检查 USER_CTRL 是否仍启用 I2C master。
- next：读 AK8963 WHO_AM_I。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| USER_CTRL | `0x6A` | bit5 | I2C_MST_EN | `0` | 关闭 MPU9250 内部 I2C master | 允许 bypass |
| INT_PIN_CFG | `0x37` | bit1 | BYPASS_EN | `1` | 外部 MCU 直接访问辅助 I2C | 访问 AK8963 `0x0C` |

当前工程代码：

```c
void mpu_set_ak8963_by_mcu(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_USER_CTRL_REG);  // 读取 USER_CTRL

    val &= (uint8_t)~(1U << 5U);                                          // 关闭 MPU9250 内部 I2C 主机模式
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_USER_CTRL_REG, val);           // 写回 USER_CTRL

    val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_INT_PIN_CFG_REG);         // 读取 INT_PIN_CFG
    val |= (uint8_t)(1U << 1U);                                            // 打开 BYPASS_EN
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_INT_PIN_CFG_REG, val);          // 写回旁路配置
}
```

## 8.2 检查 AK8963 设备 ID

AK8963 的设备 ID 寄存器叫 `WIA`，地址 `0x00`，正常返回 `0x48`。

- function：读取 `AK8963_REG_WIA`。
- why：确认旁路通道、磁力计地址和芯片存在。
- if not：后续读 ASA、设置模式都不可靠。
- before：`BYPASS_EN` 已打开。
- operate：`I2C_ReadReg(0x0C, 0x00)`。
- register：见下表。
- code：当前工程 `AK8963_CheckDeviceID()`。
- check：返回 0 表示 ID 正确。
- debug：检查 bypass、地址 `0x0C`、USER_CTRL、模块是否真带 AK8963。
- next：进入 Power Down。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| WIA | `0x00` | bit[7:0] | WHO_AM_I | 只读，正常 `0x48` | AK8963 设备 ID | `AK8963_CheckDeviceID()` |

当前工程代码：

```c
int AK8963_CheckDeviceID(void)
{
    uint8_t device_id = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_WIA); // 读取 WIA 设备 ID

    return (device_id == AK8963_WHO_AM_I_VALUE) ? 0 : -1;              // 0 表示 ID 正确
}
```

## 8.3 AK8963 power-down

AK8963 切换工作模式前，推荐先进入 Power Down。不要从一个模式直接跳到另一个模式，否则可能模式切换不生效或数据状态异常。

- function：写 `CNTL1=0x00`。
- why：为 Fuse ROM、连续测量模式切换提供干净起点。
- if not：ASA 可能读不到，连续测量模式可能没进。
- before：AK8963 ID 正确。
- operate：写 `AK8963_REG_CNTL1`。
- register：见下表。
- code：当前工程 `AK8963_EnterPowerDownMode()`。
- check：延时 10ms 后再切换下一模式。
- debug：如果模式切换后数据不 ready，先确认是否有这个延时。
- next：进入 Fuse ROM。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| CNTL1 | `0x0A` | bit[3:0] | MODE | `0x00` | Power Down | 模式切换前后的中间状态 |

```c
void AK8963_EnterPowerDownMode(void)
{
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x00U); // 写入 Power-down 模式
    vTaskDelay(pdMS_TO_TICKS(10));                           // 等待模式切换完成
}
```

## 8.4 进入 Fuse ROM 模式

Fuse ROM 保存 AK8963 出厂灵敏度调整值，也就是 `ASA`。只有进入 Fuse ROM access 模式，才能读取 `ASAX / ASAY / ASAZ`。

- function：写 `CNTL1=0x0F`。
- why：读取三轴 ASA 灵敏度修正值。
- if not：ASA 读数无效，磁力计 uT 转换会丢失出厂修正。
- before：已经进入 Power Down 并延时。
- operate：写 `AK8963_REG_CNTL1`。
- register：见下表。
- code：当前工程 `AK8963_EnterFuseROMMode()`。
- check：延时 10ms 后读 ASA。
- debug：如果 ASA 为 0 或 255，检查是否真的进入 Fuse ROM。
- next：读取 ASA。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| CNTL1 | `0x0A` | bit[3:0] | MODE | `0x0F` | Fuse ROM access | 允许读取 ASA |

```c
void AK8963_EnterFuseROMMode(void)
{
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x0FU); // 写入 Fuse ROM access 模式
    vTaskDelay(pdMS_TO_TICKS(10));                           // 等待模式切换完成
}
```

## 8.5 读取灵敏度补偿系数 ASA

ASA 是 AK8963 出厂写入的三轴灵敏度调整值。raw 转 uT 时不能只乘 `0.15f`，还要乘 ASA 修正系数。

- function：读取 `ASAX / ASAY / ASAZ` 并计算 `g_ak8963_sensitivity[]`。
- why：消除芯片出厂灵敏度差异。
- if not：磁场三轴比例会偏，yaw 和校准拟合都会受影响。
- before：AK8963 已进入 Fuse ROM。
- operate：读 `0x10 / 0x11 / 0x12`。
- register：见下表。
- code：当前工程 `AK8963_AdjustSensitivity()`。
- check：ASA 一般不应全 0 或全 255。
- debug：异常时重新 Power Down -> Fuse ROM -> 读 ASA。
- next：退出 Fuse ROM，回到 Power Down。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| ASAX | `0x10` | bit[7:0] | ASA_X | 只读 | X 轴灵敏度调整值 | `g_ak8963_asa[0]` |
| ASAY | `0x11` | bit[7:0] | ASA_Y | 只读 | Y 轴灵敏度调整值 | `g_ak8963_asa[1]` |
| ASAZ | `0x12` | bit[7:0] | ASA_Z | 只读 | Z 轴灵敏度调整值 | `g_ak8963_asa[2]` |

ASA 修正公式：

```text
sensitivity = ((ASA - 128) * 0.5 / 128) + 1
```

当前工程代码：

```c
void AK8963_AdjustSensitivity(void)
{
    g_ak8963_asa[0] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAX);       // 读取 X 轴 ASA
    g_ak8963_asa[1] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAY);       // 读取 Y 轴 ASA
    g_ak8963_asa[2] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAZ);       // 读取 Z 轴 ASA

    g_ak8963_sensitivity[0] = (((float)g_ak8963_asa[0] - 128.0f) * 0.5f / 128.0f) + 1.0f; // X 轴系数
    g_ak8963_sensitivity[1] = (((float)g_ak8963_asa[1] - 128.0f) * 0.5f / 128.0f) + 1.0f; // Y 轴系数
    g_ak8963_sensitivity[2] = (((float)g_ak8963_asa[2] - 128.0f) * 0.5f / 128.0f) + 1.0f; // Z 轴系数
}
```

## 8.6 退出 Fuse ROM，重新进入 Power Down

读取 ASA 后不能一直停留在 Fuse ROM，因为 Fuse ROM 不是正常测量模式。下一步要设置连续测量模式，所以必须重新进入 Power Down。

- function：退出 Fuse ROM。
- why：AK8963 要从 Power Down 切到连续测量模式。
- if not：后续 `CNTL1=0x16` 可能不可靠。
- before：ASA 已读取并保存。
- operate：写 `CNTL1=0x00`，延时 10ms。
- register：`CNTL1`。
- code：当前工程再次调用 `AK8963_EnterPowerDownMode()`。
- check：下一步进入连续测量模式后 `ST1.DRDY` 能置位。
- debug：如果 `data not ready`，确认这一步是否执行。
- next：设置磁力计测量模式。

## 8.7 设置磁力计测量模式

AK8963 的 `CNTL1` 控制输出位宽和测量模式。常见选择：

| 模式 | CNTL1 | 说明 |
|---|---|---|
| Power Down | `0x00` | 掉电模式 |
| 单次测量 | `0x01` | 触发一次测量 |
| Continuous 1 | `0x02` | 8Hz 连续测量 |
| Continuous 2 | `0x06` | 100Hz 连续测量 |
| 14-bit 输出 | bit4 = `0` | 分辨率较低 |
| 16-bit 输出 | bit4 = `1` | 当前工程使用 |
| Fuse ROM | `0x0F` | 读取 ASA |

当前工程写 `0x16`：

```text
0x16 = 0001 0110b
bit4 = 1      -> 16-bit 输出
bit[3:0] = 6  -> Continuous measurement mode 2，100Hz
```

- function：设置 AK8963 为 16bit + 100Hz 连续测量。
- why：给 50Hz 的 `ImuTask` 提供足够新的磁场数据。
- if not：8Hz 磁力计对 yaw 修正会慢，单次模式还需要每次触发。
- before：已从 Fuse ROM 回到 Power Down。
- operate：写 `CNTL1=0x16`。
- register：见下表。
- code：当前工程 `AK8963_EnterContinuousMeasurementMode()`。
- check：`ST1.DRDY` 能周期置位。
- debug：如果无数据，检查模式值和延时。
- next：检查 AK8963 初始化是否成功。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| CNTL1 | `0x0A` | bit4 | BIT | `1` | 16-bit 输出 | `0x16` |
| CNTL1 | `0x0A` | bit[3:0] | MODE | `0x06` | 连续测量模式 2，100Hz | `0x16` |

```c
void AK8963_EnterContinuousMeasurementMode(void)
{
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x16U); // 设置 16 位连续测量模式 2
    vTaskDelay(pdMS_TO_TICKS(10));                           // 等待模式切换完成
}
```

## 8.8 检查 AK8963 初始化是否成功

AK8963 初始化成功至少要满足：

| 检查项 | 成功标志 | 当前工程位置 |
|---|---|---|
| bypass | 能访问 `0x0C` | `mpu_set_ak8963_by_mcu()` |
| 设备 ID | `WIA=0x48` | `AK8963_CheckDeviceID()` |
| ASA | 三轴读数合理 | `AK8963_AdjustSensitivity()` |
| 测量模式 | `CNTL1=0x16` 后延时 | `AK8963_EnterContinuousMeasurementMode()` |
| ST1 | `DRDY=1` | `AK8963_CheckDataReady()` |
| ST2 | `HOFL=0` | `AK8963_Read_Axis()` |
| 数据 | 三轴不是长期全 0 | `AK8963_Read_Mag_UT()` |

当前工程调试打印：

```text
[AK8963] device ID check ok
[AK8963] data ready
```

当前工程在进入连续模式后只检查一次 `AK8963_CheckDataReady()`，如果刚好还没到一帧数据，可能返回初始化失败。推荐新手版加入短时间重试。下面是推荐整理后的写法，不一定完全等同于当前工程原代码：

```c
/**
 * @brief  等待 AK8963 首帧数据准备好。
 * @retval 0 成功；负数表示超时。
 */
int AK8963_WaitFirstDataReady(void)
{
    uint8_t retry;                                                        // 重试次数

    for (retry = 0U; retry < 20U; retry++)
    {
        if (AK8963_CheckDataReady() != 0)
        {
            return 0;                                                     // ST1.DRDY 已置位
        }

        vTaskDelay(pdMS_TO_TICKS(5));                                     // 等待下一次磁力计转换
    }

    return -1;                                                            // 超时未等到数据
}
```

## 9. 读取磁力计三轴原始数据

磁力计 raw 读取比六轴更容易写错，重点是三件事：

1. 先看 `ST1.DRDY`；
2. 数据是低字节在前；
3. 读完必须带上 `ST2` 并检查 `HOFL`。

## 9.1 检查数据准备状态 ST1

- function：检查 `ST1.DRDY`。
- why：AK8963 连续测量不是每个 MCU 读取瞬间都有新数据。
- if not：可能读到旧数据或无效数据。
- before：AK8963 已进入连续测量模式。
- operate：读 `AK8963_REG_ST1`，检查 bit0。
- register：见下表。
- code：当前工程 `AK8963_CheckDataReady()`。
- check：返回 1 表示 ready。
- debug：如果一直不 ready，检查连续测量模式、bypass、读 ST2。
- next：读取 `HXL..ST2`。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| ST1 | `0x02` | bit0 | DRDY | 只读，`1` 表示 ready | 有新磁力计数据 | `AK8963_CheckDataReady()` |

当前工程代码：

```c
int AK8963_CheckDataReady(void)
{
    uint8_t status = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ST1); // 读取 ST1 状态寄存器

    return ((status & 0x01U) != 0U) ? 1 : 0;                        // bit0 为 1 表示数据就绪
}
```

注意：`User/Inc/mpu9250_driver.h` 里 `AK8963_CheckDataReady()` 的 `@retval` 注释写成了“0 数据就绪；负数表示未就绪或异常”，但当前实现是“1 数据就绪；0 未就绪”。新手看代码时以 `.c` 文件实现为准。

## 9.2 读取磁力计数据

AK8963 数据寄存器从 `HXL` 开始，低字节在前。这一点和 MPU9250 六轴“高字节在前”不同。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| HXL | `0x03` | bit[7:0] | HX_L | 只读 | X 磁场低字节 | `frame[0]` |
| HXH | `0x04` | bit[7:0] | HX_H | 只读 | X 磁场高字节 | `frame[1]` |
| HYL | `0x05` | bit[7:0] | HY_L | 只读 | Y 磁场低字节 | `frame[2]` |
| HYH | `0x06` | bit[7:0] | HY_H | 只读 | Y 磁场高字节 | `frame[3]` |
| HZL | `0x07` | bit[7:0] | HZ_L | 只读 | Z 磁场低字节 | `frame[4]` |
| HZH | `0x08` | bit[7:0] | HZ_H | 只读 | Z 磁场高字节 | `frame[5]` |
| ST2 | `0x09` | bit[7:0] | 状态 2 | 只读 | 溢出和锁存释放 | `frame[6]` |

当前工程代码：

```c
raw->mag_x = (int16_t)(((uint16_t)frame[1] << 8U) | (uint16_t)frame[0]); // AK8963 低字节在前
raw->mag_y = (int16_t)(((uint16_t)frame[3] << 8U) | (uint16_t)frame[2]); // 合成 Y 轴 raw
raw->mag_z = (int16_t)(((uint16_t)frame[5] << 8U) | (uint16_t)frame[4]); // 合成 Z 轴 raw
```

推荐写法和当前工程一致：一次读取 7 字节，从 `HXL` 到 `ST2`。

## 9.3 必须读取 ST2

AK8963 的 `ST2` 有两个工程意义：

1. 检查 `HOFL` 溢出位；
2. 释放本帧数据锁存，让下一帧可以更新。

- function：读磁力计数据时带上 `ST2`。
- why：不读 `ST2` 可能影响下一帧更新。
- if not：后续 DRDY 或数据更新会异常，表现为磁力计卡住。
- before：ST1 已 ready。
- operate：连续读 `HXL..ST2` 共 7 字节。
- register：见下表。
- code：当前工程 `AK8963_Read_Axis()` 已正确读取 `ST2`。
- check：`frame[6] & 0x08` 应为 0。
- debug：如果频繁 HOFL，远离强磁场或电机电流线。
- next：判断数据是否有效。

| 寄存器名称 | 地址 | bit 位 | bit 名称 | 设置值 | 含义 | 当前工程用途 |
|---|---|---|---|---|---|---|
| ST2 | `0x09` | bit3 | HOFL | 只读，`1` 表示溢出 | 本帧磁场无效 | `AK8963_Read_Axis()` 返回 `-3` |

当前工程代码：

```c
if ((frame[6] & 0x08U) != 0U) // 检查磁力计溢出标志 HOFL
{
    return -3;                  // 磁力计数据溢出
}
```

## 9.4 判断磁力计数据是否有效

`mag_valid` 或磁力计读取成功不应该只看 I2C 是否成功。以下情况应视为无效：

| 无效情况 | 当前工程处理 |
|---|---|
| `ST1.DRDY=0` | `AK8963_Read_Axis()` 返回 `-1` |
| I2C 读取失败 | 返回 `-2` |
| `ST2.HOFL=1` | 返回 `-3` |
| 数据全 0 | 当前没有单独检查，推荐增加 |
| 磁场模长太小或太大 | `MPU9250_MahonyUpdate()` 用 `15uT..100uT` 判断 |
| 周围磁干扰强 | 通过校准日志和航向验证发现 |

`ImuService_ReadPhys()` 的降级逻辑是：

```c
if (AK8963_Read_Mag_UT(mag) != 0)                 // 磁力计本帧失败
{
    return IMU_SERVICE_READ_6AXIS_OK;              // 六轴仍然有效，退回 6 轴融合
}

return IMU_SERVICE_READ_9AXIS_OK;                  // 六轴和磁力计都有效
```

## 10. 磁力计数据补偿与物理量转换

磁力计 raw 不能直接用于 yaw。完整处理顺序是：

```text
raw int16
  -> raw * ASA 修正 * 0.15uT/LSB
  -> 减去硬铁 offset
  -> 乘软铁 scale 或 3x3 correction matrix
  -> 得到校准后的 mag_x_ut / mag_y_ut / mag_z_ut
```

## 10.1 磁力计 raw 转物理量

- function：把 AK8963 raw 转成 uT。
- why：Mahony 和校准算法需要实际磁场单位或至少一致比例的物理量。
- if not：ASA 不参与会带来三轴比例误差。
- before：ASA 已读取，`g_ak8963_sensitivity[]` 已计算。
- operate：`mag_ut = raw * sensitivity * 0.15f`。
- register：raw 来自 `HXL..HZH`，ASA 来自 `ASAX..ASAZ`。
- code：当前工程 `ak8963_convert_raw_to_ut()`。
- check：地磁模长通常在几十 uT 量级。
- debug：如果模长接近 0 或几百 uT，检查字节顺序、ASA、环境磁干扰。
- next：做 offset 补偿。

当前工程常量：

```c
#define AK8963_16BIT_UT_PER_LSB 0.15f // AK8963 16 位模式下每 LSB 对应微特斯拉
```

当前工程代码：

```c
mag_out->mag_x_ut = (float)raw->mag_x * g_ak8963_sensitivity[0] * AK8963_16BIT_UT_PER_LSB; // X 轴 uT
mag_out->mag_y_ut = (float)raw->mag_y * g_ak8963_sensitivity[1] * AK8963_16BIT_UT_PER_LSB; // Y 轴 uT
mag_out->mag_z_ut = (float)raw->mag_z * g_ak8963_sensitivity[2] * AK8963_16BIT_UT_PER_LSB; // Z 轴 uT
```

## 10.2 offset 补偿

offset 是硬铁干扰造成的磁场中心偏移。硬铁干扰来自固定磁性材料、电流回路、螺丝、电池线等。正确做法是先把磁场点云中心移回原点：

```text
centered_x = mag_x - offset_x
centered_y = mag_y - offset_y
centered_z = mag_z - offset_z
```

当前工程 offset 保存位置：

```c
static float g_mag_offset_ut[3] = {0.0f, 0.0f, 0.0f}; // 磁力计硬铁偏移，单位 uT
```

当前工程在 `AK8963_CalibrateMag()` 拟合成功后写入：

```c
g_mag_offset_ut[0] = fit_params[0]; // 保存 X 轴硬铁偏移
g_mag_offset_ut[1] = fit_params[1]; // 保存 Y 轴硬铁偏移
g_mag_offset_ut[2] = fit_params[2]; // 保存 Z 轴硬铁偏移
```

## 10.3 scale 补偿

scale 是软铁干扰造成的三轴拉伸/压缩差异。传统入门写法是每轴一个 scale：

```text
corrected_x = centered_x * scale_x
corrected_y = centered_y * scale_y
corrected_z = centered_z * scale_z
```

当前工程已经升级为 3x3 软铁矩阵：

```c
static float g_mag_correction[3][3] = {{1.0f, 0.0f, 0.0f},
                                       {0.0f, 1.0f, 0.0f},
                                       {0.0f, 0.0f, 1.0f}};
```

这比单独 scale 更强，因为 3x3 矩阵可以处理轴间耦合和旋转过的椭球。

## 10.4 完整补偿公式

用户原目录里的基础公式必须保留：

```text
mag_corrected_x = (mag_x - offset_x) * scale_x
mag_corrected_y = (mag_y - offset_y) * scale_y
mag_corrected_z = (mag_z - offset_z) * scale_z
```

当前工程实际使用的是扩展后的矩阵形式：

```text
centered = raw_ut - offset
corrected = M * centered
```

当前工程代码：

```c
centered_x = mag->mag_x_ut - g_mag_offset_ut[0];                 // 去除 X 轴硬铁偏移
centered_y = mag->mag_y_ut - g_mag_offset_ut[1];                 // 去除 Y 轴硬铁偏移
centered_z = mag->mag_z_ut - g_mag_offset_ut[2];                 // 去除 Z 轴硬铁偏移

corrected_x = g_mag_correction[0][0] * centered_x +              // 软铁矩阵第 0 行作用到校正向量
              g_mag_correction[0][1] * centered_y +
              g_mag_correction[0][2] * centered_z;
corrected_y = g_mag_correction[1][0] * centered_x +              // 软铁矩阵第 1 行作用到校正向量
              g_mag_correction[1][1] * centered_y +
              g_mag_correction[1][2] * centered_z;
corrected_z = g_mag_correction[2][0] * centered_x +              // 软铁矩阵第 2 行作用到校正向量
              g_mag_correction[2][1] * centered_y +
              g_mag_correction[2][2] * centered_z;

mag->mag_x_ut = corrected_x;                                     // 写回校准后的 X 轴磁场
mag->mag_y_ut = corrected_y;                                     // 写回校准后的 Y 轴磁场
mag->mag_z_ut = corrected_z;                                     // 写回校准后的 Z 轴磁场
```

## 11. 零偏校准

传感器上电后不能直接完全相信。即使静止不动，陀螺仪也可能输出非零，水平放置时加速度计也不一定刚好是 `0 / 0 / 1g`，磁力计还会被环境硬铁/软铁干扰。

## 11.1 零偏校准整体目的

- function：建立 raw 转物理量之后的校准参数。
- why：把固定偏差从每一帧数据中扣掉。
- if not：陀螺仪 yaw 漂移更快，加速度 roll/pitch 有固定倾斜误差，磁力计 yaw 偏移和变形。
- before：I2C、WHO_AM_I、六轴配置、AK8963 初始化都已完成。
- operate：初始化阶段采样，运行阶段每帧扣除。
- register：校准不是寄存器配置，而是软件参数。
- code：`MPU9250_CalibrateGyro()`、`MPU9250_CalibrateAccel()`、`AK8963_CalibrateMag()`。
- check：校准后静止 gyro 接近 0，acc 模长接近 1，磁力计拟合日志成功。
- debug：校准失败先看动作和日志，不要先调 Mahony。
- next：分别实现三类校准。

校准参数要和 raw 转物理量区分开：量程比例是由寄存器配置决定的固定换算，校准 bias/offset/矩阵是当前板子和当前环境的误差补偿。

## 11.2 陀螺仪校准

陀螺仪静止时理论输出应该接近 `0dps`，实际会有 bias。当前工程在初始化阶段静止采样 1000 次，每次间隔 10ms，总约 10 秒。

当前工程代码：

```c
void MPU9250_CalibrateGyro(uint16_t samples, uint16_t delay_ms)
{
    MPU9250_raw_Data raw;                                               // 暂存 MPU9250 原始值
    int32_t gyro_x_sum = 0;                                             // X 轴陀螺仪原始值累加
    int32_t gyro_y_sum = 0;                                             // Y 轴陀螺仪原始值累加
    int32_t gyro_z_sum = 0;                                             // Z 轴陀螺仪原始值累加
    uint16_t i;                                                         // 采样索引

    if (samples == 0U)
    {
        AppStatus_SetCalState(APP_CAL_STATE_FAIL);                      // 采样参数异常，提示标定失败
        return;
    }

    for (i = 0U; i < samples; i++)
    {
        MPU9250_ReadAxis(&raw);                                         // 读取六轴原始值
        gyro_x_sum += raw.gyro_x;                                       // 累加 X 轴陀螺仪原始值
        gyro_y_sum += raw.gyro_y;                                       // 累加 Y 轴陀螺仪原始值
        gyro_z_sum += raw.gyro_z;                                       // 累加 Z 轴陀螺仪原始值
        vTaskDelay(pdMS_TO_TICKS(delay_ms));                            // 按指定间隔等待下一次采样
    }

    g_gyro_bias_dps[0] = ((float)gyro_x_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS; // X 零偏
    g_gyro_bias_dps[1] = ((float)gyro_y_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS; // Y 零偏
    g_gyro_bias_dps[2] = ((float)gyro_z_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS; // Z 零偏
}
```

后续每帧在 `MPU9250_ConvertToPhysical()` 中扣除 `g_gyro_bias_dps[]`。校准时必须保持板子静止，否则会把真实转动误认为零偏。

## 11.3 加速度计校准

静止时加速度计测到的是重力方向，不是 X/Y/Z 都等于 0。水平放置时，通常 X/Y 接近 0，Z 接近 1g 或 -1g，具体取决于安装方向。

当前工程做了加速度计 bias 校准：

```c
void MPU9250_CalibrateAccel(uint16_t samples, uint16_t delay_ms)
{
    MPU9250_raw_Data raw;                                               // 暂存 MPU9250 原始值
    float ax_sum = 0.0f;                                                // X 轴加速度 g 值累加
    float ay_sum = 0.0f;                                                // Y 轴加速度 g 值累加
    float az_sum = 0.0f;                                                // Z 轴加速度 g 值累加
    uint16_t i;                                                         // 采样索引

    if (samples == 0U)
    {
        AppStatus_SetCalState(APP_CAL_STATE_FAIL);                      // 采样参数异常
        return;
    }

    for (i = 0U; i < samples; i++)
    {
        MPU9250_ReadAxis(&raw);                                         // 读取六轴原始值
        ax_sum += (float)raw.accel_x / MPU9250_ACCEL_LSB_PER_G;          // 累加 X 轴 g 值
        ay_sum += (float)raw.accel_y / MPU9250_ACCEL_LSB_PER_G;          // 累加 Y 轴 g 值
        az_sum += (float)raw.accel_z / MPU9250_ACCEL_LSB_PER_G;          // 累加 Z 轴 g 值
        vTaskDelay(pdMS_TO_TICKS(delay_ms));                            // 等待下一次采样
    }

    g_accel_bias_g[0] = ax_sum / (float)samples;                        // X 轴静止零偏
    g_accel_bias_g[1] = ay_sum / (float)samples;                        // Y 轴静止零偏
    g_accel_bias_g[2] = (az_sum / (float)samples) - 1.0f;                // Z 轴扣除 1g 重力
}
```

这套写法假设校准时板子 Z 轴方向接近 `+1g`。如果板子安装方向不同，或者校准姿态不是水平，需要调整“哪一轴扣 1g”和符号。

## 11.4 磁力计校准

磁力计校准必须旋转采样。只在一个平面转不够，因为三维椭球拟合需要 X/Y/Z 都扫开。当前工程采样 500 次，每次 20ms，总约 10 秒；最少有效样本数是 64。

当前工程使用的是“min/max 初值 + 3D 椭球拟合 + 3x3 软铁矩阵”的方法，不是简单的每轴 scale。

关键保护条件：

| 保护项 | 当前工程条件 | 失败日志 |
|---|---|---|
| 有效样本数 | `valid_count >= 64` | `calibrate mag rejected valid=...` |
| 三轴覆盖半径 | 每轴半径 `>= 5uT` | `calibrate mag rejected radius=...` |
| 椭球拟合 | `ak8963_fit_ellipsoid()` 成功 | `calibrate mag rejected fit failed` |
| 椭球参数 | 可生成软铁矩阵 | `invalid ellipsoid` |
| 拟合可信度 | 中心和半径不能明显偏离采样范围 | `implausible fit` |

校准成功会写入：

```c
g_mag_offset_ut[0] = fit_params[0];                                      // X 硬铁偏移
g_mag_offset_ut[1] = fit_params[1];                                      // Y 硬铁偏移
g_mag_offset_ut[2] = fit_params[2];                                      // Z 硬铁偏移
g_mag_correction[row][col] = candidate_correction[row][col];             // 软铁矩阵
```

实际操作建议：

```text
1. 水平慢转一圈；
2. 前后翻滚一圈；
3. 左右翻滚一圈；
4. 做两组斜 45 度转动，补齐球面空白；
5. 远离电机、大电流线、磁铁、扬声器、铁桌。
```

## 11.5 校准状态和 LED 提示

当前工程有校准状态和 LED 提示，状态定义在 `User/Inc/app_status.h`：

| 状态 | 含义 | LED 行为 | 当前工程代码 |
|---|---|---|---|
| `APP_CAL_STATE_SELF_CHECK` | 上电自检 | 红绿同时短亮一次 | `LedTask()` |
| `APP_CAL_STATE_STATIC` | 陀螺仪和加速度计静止校准 | 绿灯 500ms 慢闪 | `LedTask()` |
| `APP_CAL_STATE_MAG_ROTATE` | 磁力计旋转校准 | 绿灯 150ms 快闪 | `LedTask()` |
| `APP_CAL_STATE_SUCCESS` | 标定成功 | 绿灯常亮 3 秒 | `LedTask()` |
| `APP_CAL_STATE_FAIL` | 标定失败 | 红灯 150ms 快闪 | `LedTask()` |
| `APP_CAL_STATE_IDLE` | 正常运行 | 绿灯短亮，每 2 秒一次 | `LedTask()` |

状态设置位置：

| 阶段 | 设置代码 |
|---|---|
| 驱动初始化开始 | `AppStatus_SetCalState(APP_CAL_STATE_SELF_CHECK)` |
| 静止校准 | `AppStatus_SetCalState(APP_CAL_STATE_STATIC)` |
| 磁力计旋转校准 | `AppStatus_SetCalState(APP_CAL_STATE_MAG_ROTATE)` |
| 校准失败 | `AppStatus_SetCalState(APP_CAL_STATE_FAIL)` |
| 校准成功 | `AppStatus_SetCalState(APP_CAL_STATE_SUCCESS)` |

当前工程 LED 执行逻辑在 `User/Src/app_tasks.c` 的 `LedTask()`，底层红绿灯控制在 `LedService_Set()` 和 `BSP_LED_Set()`。

## 12. A+M 初始姿态解算

A+M 指用 Accel + Magnetometer 直接算一组初始 `roll / pitch / yaw`，再把它作为融合初值。它适合学习姿态几何，也适合改善融合启动收敛。

当前工程需要特别说明：`User/Src/mpu9250_driver.c` 当前未找到可编译的 `MPU9250_ComputeEuler_FromAccMag()` 或 `MPU9250_GetEulerDeg()`。文件里只保留了注释：

```c
// EulerAngle_t g_euler_acc_mag = {0.0f, 0.0f, 0.0f}; // 旧的加速度计+磁力计直接解算角，目前保留为学习/调试参考
```

也就是说，当前运行主线没有使用 A+M 初始姿态，`MPU9250_MahonyInit()` 直接把四元数初始化为 `{1,0,0,0}`。下面章节给出工程推荐写法，但必须明确：它不是当前工程现役函数。

## 12.1 为什么需要初始姿态

- function：在融合开始前给四元数一个接近真实姿态的初值。
- why：初始值太差会导致融合收敛慢，yaw 可能需要较长时间被磁力计拉回。
- if not：当前工程仍能跑，但启动阶段姿态可能从默认水平朝向逐步收敛。
- before：已有加速度计和校准后的磁力计物理量。
- operate：用加速度算 roll/pitch，用倾斜补偿后的磁力计算 yaw。
- register：不直接操作寄存器。
- code：当前工程未找到该函数，下面是推荐新增函数。
- check：水平静止时 roll/pitch 接近 0，朝北/东/南/西 yaw 大致正确。
- debug：如果方向错，先查轴向映射和磁力计校准。
- next：把初始欧拉角转换为四元数。

## 12.2 用加速度计计算 roll / pitch

静止时加速度计测的是重力方向，所以可以估算横滚和俯仰。但运动状态下有外加速度，不能完全相信。

推荐写法：

```c
/**
 * @brief  用加速度计估算 roll 和 pitch。
 * @param  imu 六轴物理量。
 * @param  roll_rad 输出横滚角，单位 rad。
 * @param  pitch_rad 输出俯仰角，单位 rad。
 * @retval 0 成功；负数表示输入无效。
 */
int MPU9250_ComputeRollPitchFromAccel(const MPU9250_Physical_Data *imu,
                                      float *roll_rad,
                                      float *pitch_rad)
{
    float ax;                                                            // X 轴加速度
    float ay;                                                            // Y 轴加速度
    float az;                                                            // Z 轴加速度

    if ((imu == 0) || (roll_rad == 0) || (pitch_rad == 0))
    {
        return -1;                                                       // 参数无效
    }

    ax = imu->accel_x_g;                                                 // 取 X 轴 g 值
    ay = -imu->accel_y_g;                                                // 按当前工程融合方向统一 Y 轴
    az = imu->accel_z_g;                                                 // 取 Z 轴 g 值

    *roll_rad = atan2f(ay, az);                                          // 由重力方向计算 roll
    *pitch_rad = atan2f(-ax, sqrtf(ay * ay + az * az));                  // 由重力方向计算 pitch

    return 0;                                                            // 计算成功
}
```

## 12.3 用磁力计计算 yaw

磁力计提供航向信息，但不能简单 `atan2(my, mx)`，因为机体倾斜后磁力计的水平分量会混入垂直分量。必须先用 roll/pitch 做倾斜补偿。

当前工程没有现役的直接 yaw 计算函数，推荐新增函数时要复用当前工程轴向约定：`mag_y` 和 `mag_z` 在 Mahony 中被取反。

## 12.4 磁力计倾斜补偿

推荐写法：

```c
/**
 * @brief  用加速度 roll/pitch 对磁力计做倾斜补偿并计算 yaw。
 * @param  mag 校准后的磁力计物理量，单位 uT。
 * @param  roll_rad 横滚角，单位 rad。
 * @param  pitch_rad 俯仰角，单位 rad。
 * @param  yaw_rad 输出数学坐标系 yaw，单位 rad。
 * @retval 0 成功；负数表示输入无效。
 */
int MPU9250_ComputeYawFromTiltCompMag(const AK8963_Physical_Data *mag,
                                      float roll_rad,
                                      float pitch_rad,
                                      float *yaw_rad)
{
    float mx;                                                            // X 轴磁场
    float my;                                                            // Y 轴磁场
    float mz;                                                            // Z 轴磁场
    float cr;                                                            // cos(roll)
    float sr;                                                            // sin(roll)
    float cp;                                                            // cos(pitch)
    float sp;                                                            // sin(pitch)
    float mx2;                                                           // 倾斜补偿后的水平 X 分量
    float my2;                                                           // 倾斜补偿后的水平 Y 分量

    if ((mag == 0) || (yaw_rad == 0))
    {
        return -1;                                                       // 参数无效
    }

    mx = mag->mag_x_ut;                                                  // 当前工程 X 磁场方向
    my = -mag->mag_y_ut;                                                 // 当前工程 Y 磁场方向取反
    mz = -mag->mag_z_ut;                                                 // 当前工程 Z 磁场方向取反

    cr = cosf(roll_rad);                                                 // 计算 roll 余弦
    sr = sinf(roll_rad);                                                 // 计算 roll 正弦
    cp = cosf(pitch_rad);                                                // 计算 pitch 余弦
    sp = sinf(pitch_rad);                                                // 计算 pitch 正弦

    mx2 = mx * cp + mz * sp;                                             // 俯仰补偿后的水平 X
    my2 = mx * sr * sp + my * cr - mz * sr * cp;                         // 横滚和俯仰补偿后的水平 Y

    *yaw_rad = atan2f(-my2, mx2);                                        // 计算磁航向角
    return 0;                                                            // 计算成功
}
```

如果要把 A+M 结果作为 Mahony 初始四元数，还需要新增“欧拉角转四元数”函数，并让 `MPU9250_MahonyInit()` 支持传入初始 roll/pitch/yaw。当前工程没有这一步，当前主线依赖 Mahony 后续迭代收敛。

## 13. 姿态融合 Fusion / Filtering

姿态融合的目的，是综合三类传感器优点：

- 陀螺仪响应快，但积分会漂；
- 加速度计可以修正 roll/pitch，但受运动加速度影响；
- 磁力计可以修正 yaw，但受磁干扰影响。

当前工程使用 Mahony 思路，公开接口是 `MPU9250_MahonyInit()`、`MPU9250_MahonyUpdate()`、`MPU9250_MahonyUpdateIMU()`、`MPU9250_GetEulerFusedDeg()`。

## 13.1 姿态融合的目的

- function：用陀螺仪积分姿态，再用加速度计和磁力计误差修正。
- why：单独一种传感器都不够稳定。
- if not：只用陀螺会漂，只用加速度和磁力计会抖且响应慢。
- before：六轴和磁力计物理量都已校准。
- operate：每个 `dt` 调一次 Mahony 更新。
- register：不直接操作寄存器。
- code：当前工程在 `ImuTask()` 中根据读取结果选择 9 轴或 6 轴融合。
- check：静止姿态稳定，yaw 不长期漂移。
- debug：先确认输入单位，再调 `Kp/Ki`。
- next：理解 Mahony 九轴融合。

## 13.2 Mahony 九轴融合

当前工程初始化：

```c
MPU9250_MahonyInit(0.3f, 0.0f); // Kp=0.3，Ki=0
```

含义：

| 参数/数据 | 作用 |
|---|---|
| `Kp` / `g_kp` | 比例修正增益，误差越大修正越强 |
| `Ki` / `g_ki` | 积分修正增益，用于长期 bias 修正，当前为 0 |
| `gyro` | 快速响应，积分更新四元数 |
| `accel` | 修正 roll/pitch 的重力方向 |
| `mag` | 修正 yaw 的地磁方向 |
| `g_q` | 当前四元数 |
| `g_euler_fused` | 四元数转换后的融合欧拉角，内部为弧度 |

当前工程还做了动态加速度修正权重：

```c
acc_error = fabsf(norm_acc - 1.0f);                         // 当前加速度模长和 1g 的偏差
acc_diff = fabsf(norm_acc - g_acc_norm_prev);               // 当前帧与上一帧变化
kp_dynamic = g_kp;                                          // 默认比例增益

if ((acc_error < 0.04f) && (acc_diff < 0.02f))
{
    kp_dynamic = g_kp * 2.0f;                                // 接近静止时提高修正权重
}
else if ((acc_error > 0.25f) || (acc_diff > 0.25f))
{
    kp_dynamic = g_kp * 0.35f;                               // 动态运动时降低加速度权重
}
```

## 13.3 六轴融合和九轴融合区别

| 融合类型 | 输入 | 能稳定什么 | yaw 表现 | 当前工程函数 |
|---|---|---|---|---|
| 六轴融合 | gyro + accel | roll / pitch | yaw 会随陀螺积分漂移 | `MPU9250_MahonyUpdateIMU()` |
| 九轴融合 | gyro + accel + mag | roll / pitch / yaw | yaw 可被磁力计长期约束 | `MPU9250_MahonyUpdate()` |

当前工程在任务层判断：

```c
if (imu_read_ok == IMU_SERVICE_READ_9AXIS_OK)
{
    MPU9250_MahonyUpdate(&phys, &mag_physical, imu_dt); // 九轴融合
}
else
{
    MPU9250_MahonyUpdateIMU(&phys, imu_dt);             // 磁力计失败时退回六轴融合
}
```

即使 `ImuService_ReadPhys()` 返回 9 轴成功，`MPU9250_MahonyUpdate()` 内部还会用磁场模长检查：

```c
norm_mag = sqrtf(mx * mx + my * my + mz * mz);                     // 计算磁场模长
mag_valid = ((norm_mag > 15.0f) && (norm_mag < 100.0f)) ? 1U : 0U;  // 判断磁场是否可信
```

## 13.4 融合周期 dt

`dt` 是每次融合更新之间的时间，单位秒。当前工程：

```c
const float imu_dt = (float)APP_IMU_TASK_PERIOD_MS * 0.001f; // 20ms -> 0.02s
```

`dt` 必须和 `ImuTask` 周期匹配：

- `dt` 太小：同样角速度积分出来的角度偏小，姿态响应慢；
- `dt` 太大：积分过量，姿态变化被放大；
- 软件 I2C 读取耗时如果超过任务周期，固定 `dt` 就和真实周期不一致。

当前工程使用 `vTaskDelayUntil()` 保持周期，并处理超时：

```c
period_blocked = xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(APP_IMU_TASK_PERIOD_MS)); // 维持 20ms 周期
if (period_blocked == pdFALSE)                                                         // 本轮超过配置周期
{
    lastWakeTime = xTaskGetTickCount();                                                 // 重新对齐周期基准
    vTaskDelay(pdMS_TO_TICKS(1));                                                       // 防止饿死低优先级任务
}
```

当前工程存在的工程风险：`imu_dt` 是配置周期，不是每轮实测耗时。如果软件 I2C 偶尔超过 20ms，融合仍用 0.02s。更严谨的写法是用 tick 或硬件定时器计算真实 `dt`。

## 13.5 输出 roll / pitch / yaw

当前工程由四元数转欧拉角：

```c
g_euler_fused.roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                            1.0f - 2.0f * (q1 * q1 + q2 * q2)); // roll

pitch_sin = 2.0f * (q0 * q2 - q3 * q1);                         // pitch 正弦
pitch_sin = mpu9250_clampf(pitch_sin, -1.0f, 1.0f);              // 防止 asinf 越界
g_euler_fused.pitch = asinf(pitch_sin);                          // pitch

g_euler_fused.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                           1.0f - 2.0f * (q2 * q2 + q3 * q3));   // yaw
```

输出角度函数：

```c
void MPU9250_GetEulerFusedDeg(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    const float rad2deg = 57.295779513f;                         // 弧度转角度

    if (roll_deg != 0)
    {
        *roll_deg = g_euler_fused.roll * rad2deg;                // 输出横滚角
    }
    if (pitch_deg != 0)
    {
        *pitch_deg = g_euler_fused.pitch * rad2deg;              // 输出俯仰角
    }
    if (yaw_deg != 0)
    {
        *yaw_deg = mpu9250_math_yaw_to_heading_deg(g_euler_fused.yaw * rad2deg); // 转导航航向角
    }
}
```

当前工程对外航向约定是导航/无人机习惯：

```text
北 = 0
东 = +90
南 = -180 或 +180
西 = -90
范围 = (-180, 180]
```

姿态输出结构体在 `User/Inc/app_types.h`：

```c
typedef struct
{
    float roll_deg;        // 横滚角，单位度
    float pitch_deg;       // 俯仰角，单位度
    float yaw_deg;         // 航向角，单位度
    uint32_t timestamp_ms; // 数据时间戳
    uint8_t valid;         // 数据有效标志
} AttitudeData_t;
```

## 14. FreeRTOS 中的九轴任务运行逻辑

九轴驱动最终不是单独运行，而是在 FreeRTOS 任务中周期执行。当前工程把初始化、采样融合、遥测组包分开，这是合理分层。

## 14.1 InitTask 的职责

`InitTask` 在 `User/Src/app_tasks.c`：

- 初始化 LED；
- 初始化调试串口；
- 调用 `ImuService_Init()`；
- 成功后设置 `APP_STATUS_IMU_READY`；
- 初始化 Mahony；
- 当前阶段默认置位 GNSS / MQTT / NET ready；
- 删除自身。

关键代码：

```c
imu_ret = ImuService_Init();                         // 初始化 IMU 驱动和校准流程
if (imu_ret == 1)
{
    AppStatus_Set(APP_STATUS_IMU_READY);             // 标记 IMU 可用
    MPU9250_MahonyInit(0.3f, 0.0f);                  // 初始化 Mahony 姿态融合参数
}
else
{
    AppStatus_Set(APP_STATUS_IMU_ERROR);             // 标记 IMU 异常
}
```

## 14.2 ImuTask 的职责

`ImuTask` 做周期采样和融合：

1. 等待 `APP_STATUS_IMU_READY`；
2. 调用 `ImuService_ReadPhys()`；
3. 9 轴成功时调用 `MPU9250_MahonyUpdate()`；
4. 磁力计失败但六轴有效时调用 `MPU9250_MahonyUpdateIMU()`；
5. 调用 `MPU9250_GetEulerFusedDeg()`；
6. 填充 `AttitudeData_t`；
7. `xQueueOverwrite(qAttitude, &attitude)`。

关键代码：

```c
imu_read_ok = ImuService_ReadPhys(&phys, &mag_physical);     // 读取六轴和可选磁力计

if (imu_read_ok == IMU_SERVICE_READ_9AXIS_OK)
{
    MPU9250_MahonyUpdate(&phys, &mag_physical, imu_dt);      // 使用九轴融合
}
else
{
    MPU9250_MahonyUpdateIMU(&phys, imu_dt);                  // 退回六轴融合
}

MPU9250_GetEulerFusedDeg(&roll_deg, &pitch_deg, &yaw_deg);   // 读取融合后的欧拉角
attitude.roll_deg = roll_deg;                                // 写入横滚角
attitude.pitch_deg = pitch_deg;                              // 写入俯仰角
attitude.yaw_deg = yaw_deg;                                  // 写入航向角
attitude.timestamp_ms = xTaskGetTickCount();                 // 写入时间戳
attitude.valid = 1U;                                         // 标记有效

xQueueOverwrite(qAttitude, &attitude);                       // 覆盖最新姿态
```

## 14.3 TelemetryTask 的关系

`TelemetryTask` 不直接读 MPU9250。它只从 `qAttitude` 读取最新姿态，再和 GNSS 数据一起组包。

好处：

- 传感器读取只在 `ImuTask`；
- 姿态融合只在 `ImuTask`；
- 遥测格式只在 `TelemetryTask` / `Telemetry_BuildMqttMsg()`；
- 后续改 MQTT、串口、4G 模块，不需要改传感器驱动。

当前遥测 JSON：

```c
snprintf(msg->payload,
         MQTT_PAYLOAD_MAX_LEN,
         "{\"att_valid\":%u,\"gnss_valid\":%u,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"lat\":%.6f,\"lon\":%.6f}",
         att->valid,
         gnss->fix_valid,
         att->roll_deg,
         att->pitch_deg,
         att->yaw_deg,
         gnss->latitude,
         gnss->longitude);
```

## 14.4 软件 I2C 和任务周期关系

软件 I2C 会占用 CPU，因为每一位都靠 GPIO 翻转和延时完成。当前每轮 IMU 至少要：

- 连续读 MPU9250 14 字节；
- 读 AK8963 ST1；
- 连续读 AK8963 7 字节；
- 做 Mahony 计算；
- 写队列。

当前配置：

| 项目 | 当前值 |
|---|---|
| `APP_IMU_TASK_PERIOD_MS` | `20ms` |
| `APP_TASK_IMU_PRIORITY` | `3` |
| `APP_TASK_MODEM_PRIORITY` | `2` |
| `APP_TASK_TELEMETRY_PRIORITY` | `2` |
| `APP_TASK_LED_PRIORITY` | `1` |

如果 IMU 周期太短或优先级太高，可能影响串口、LED、通信任务。当前工程已经在周期超时时 `vTaskDelay(1ms)` 主动让出 CPU，这是一个保护。

优化方向：

- 软件 I2C 改硬件 I2C；
- 六轴和磁力计读取函数增加错误返回；
- 用真实 tick 计算 `dt`；
- 降低磁力计读取频率，六轴 50Hz，磁力计 25Hz 也可以；
- 校准阶段和运行阶段分开任务优先级。

## 15. 调试与错误处理

调试要按链路顺序，不要直接改融合参数。

## 15.1 MPU9250 WHO_AM_I 失败怎么排查

| 可能原因 | 排查方式 | 当前工程相关信息 |
|---|---|---|
| I2C 地址错误 | AD0 接低通常 `0x68`，接高通常 `0x69` | 当前 `MPU9250_I2C_ADDR=0x68` |
| SDA/SCL 接线错误 | 逻辑分析仪看 START 和 ACK | 当前 PB6/PB7 |
| 上拉电阻问题 | SCL/SDA 空闲是否为高 | 软件 I2C 开漏 + 内部上拉 |
| 供电问题 | 模块是否 3.3V，GND 是否共地 | 先看 WHO_AM_I |
| 软件 I2C 时序问题 | 调整 `I2C_SOFT_DELAY_COUNT` | 当前默认 100 |
| 芯片型号不一致 | WHO_AM_I 不是 `0x71` | `mpu9250_check_device()` 会报错 |

当前调试打印：

```text
[MPU9250] WHO_AM_I read failed
[MPU9250] WHO_AM_I = 0x..
[MPU9250] WHO_AM_I value error
```

## 15.2 AK8963 读不到怎么排查

| 可能原因 | 排查方式 | 当前工程相关信息 |
|---|---|---|
| bypass 没打开 | 检查 `INT_PIN_CFG.BYPASS_EN=1` | `mpu_set_ak8963_by_mcu()` |
| AK8963 地址错误 | 确认 `0x0C` 是否 ACK | `AK8963_I2C_ADDR7=0x0C` |
| USER_CTRL 冲突 | `I2C_MST_EN` 必须为 0 | 当前清 bit5 |
| 没有 Power Down 切模式 | 每次模式切换前回 `0x00` | 当前有 |
| 延时不足 | Power Down/Fuse/Continuous 后延时 | 当前 10ms |
| 模块不带 AK8963 | 有些模块可能不是 9 轴版本 | WIA 不会返回 `0x48` |

当前调试打印：

```text
[AK8963] device ID check failed
[AK8963] device ID check ok
[AK8963] data not ready
[AK8963] data ready
```

## 15.3 磁力计数据无效怎么排查

| 现象 | 优先检查 |
|---|---|
| `ST1` 没 ready | 连续模式是否 `0x16`，是否等待足够时间 |
| ST2 溢出 | 周围是否有强磁场或电机电流 |
| 没读 ST2 | 必须连续读 HXL..ST2 |
| 全 0 或固定值 | bypass、地址、模式、锁存释放 |
| 标定失败 valid 少 | 采样动作太快/太少，DRDY 没跟上 |
| radius 太小 | 没做完整 3D 旋转 |
| fit failed | 点云覆盖差或环境磁干扰强 |

当前磁力计校准日志：

```text
[AK8963] mag calibration progress ...
[AK8963] raw bounds ...
[AK8963] seed center=...
[AK8963] calibrate mag rejected ...
[AK8963] calibrate mag ok ...
```

## 15.4 姿态角异常怎么排查

| 异常 | 优先排查 |
|---|---|
| roll/pitch 方向反 | `MPU9250_MahonyUpdate()` 里的 `ay = -imu->accel_y_g` 等轴向映射 |
| yaw 偏 90 度 | `mpu9250_math_yaw_to_heading_deg()` 航向转换 |
| yaw 漂移 | 磁力计是否失败退回六轴 |
| 静止抖动大 | DLPF、I2C 噪声、任务周期、Kp |
| 运动时被拉偏 | 加速度动态太大，`kp_dynamic` 是否合适 |
| 姿态发散 | gyro dps 和 rad/s 是否混用，`dt` 是否正确 |
| 四方向不等间隔 | 磁力计未校准好或环境干扰 |

当前工程关键调试点：

- `Debug_Printf("[TelemetryTask] ... roll=... pitch=... yaw=...")` 当前在 `TelemetryTask` 中实际调用了 `Debug_Print(log_buf)`；
- `MPU9250_Read_PowerMgmt()` 可看电源管理寄存器；
- `AK8963_CalibrateMag()` 可看磁力计点云和拟合结果。

## 16. 当前工程文件和函数索引

| 功能 | 文件名 | 函数名/宏定义 | 作用 | 被谁调用 | 初始化阶段/周期运行阶段 | 备注 |
| -- | --- | ------- | -- | ---- | ------------ | -- |
| 软件 I2C 初始化 | `User/Src/bsp_i2c_soft.c` | `BSP_I2C_Soft_Init()` | 配置 PB6/PB7 开漏并恢复总线 | `MPU9250_Driver_Init()` | 初始化 | 当前使用软件 I2C |
| I2C 单字节读 | `User/Src/bsp_i2c_soft.c` | `I2C_ReadReg()` | 读取一个寄存器，失败默认 `0xFF` | 多个驱动函数 | 初始化/周期 | 简单但不返回错误码 |
| I2C 单字节写 | `User/Src/bsp_i2c_soft.c` | `I2C_WriteReg()` | 写一个寄存器 | 初始化配置函数 | 初始化 | 复用 `I2C_WriteRegs()` |
| I2C 连续读 | `User/Src/bsp_i2c_soft.c` | `I2C_ReadRegs()` | 从起始寄存器连续读取 | `MPU9250_ReadAxis()`、`AK8963_Read_Axis()` | 周期运行 | 六轴和磁力计都依赖 |
| I2C 连续写 | `User/Src/bsp_i2c_soft.c` | `I2C_WriteRegs()` | 连续写多个字节 | `I2C_WriteReg()` | 初始化 | 当前单字节写复用 |
| MPU9250 WHO_AM_I 检测 | `User/Src/mpu9250_driver.c` | `mpu9250_check_device()` | 判断 ID 是否为 `0x71` | `MPU9250_Driver_Init()` | 初始化 | 静态函数 |
| MPU9250 WHO_AM_I 读取 | `User/Src/mpu9250_driver.c` | `MPU9250_Driver_ReadWhoAmI()` | 读取寄存器 `0x75` | `mpu9250_check_device()` | 初始化 | 实现中 1 成功 |
| MPU9250 软复位 | `User/Src/mpu9250_driver.c` | `MPU9250_SoftReset()` | 写 `PWR_MGMT_1.bit7` | `mpu9250_config_six_axis()` | 初始化 | 延时 100ms |
| MPU9250 唤醒 | `User/Src/mpu9250_driver.c` | `mpu_set_clock_to_auto()` | 清 `SLEEP` | `mpu9250_config_six_axis()` | 初始化 | 同时设时钟 |
| MPU9250 时钟配置 | `User/Src/mpu9250_driver.c` | `mpu_set_clock_to_auto()` | `CLKSEL=1` | `mpu9250_config_six_axis()` | 初始化 | X gyro PLL |
| 六轴 standby 配置 | `User/Src/mpu9250_driver.c` | `mpu_enable_six_axis()` | 清 `PWR_MGMT_2` 低 6 位 | `mpu9250_config_six_axis()` | 初始化 | 使能加速度和陀螺仪 |
| 陀螺仪 DLPF 配置 | `User/Src/mpu9250_driver.c` | `mpu_set_dlpf_cfg_3()` | `CONFIG.DLPF_CFG=3` | `mpu9250_config_six_axis()` | 初始化 | 降低 gyro 噪声 |
| 采样率配置 | `User/Src/mpu9250_driver.c` | `mpu_set_sample_rate_200hz()` | `SMPLRT_DIV=0x04` | `mpu9250_config_six_axis()` | 初始化 | 约 200Hz |
| 加速度计 DLPF 配置 | `User/Src/mpu9250_driver.c` | `mpu_set_accel_dlpf()` | `ACCEL_CONFIG2=...3` | `mpu9250_config_six_axis()` | 初始化 | 清高带宽旁路 |
| 陀螺仪量程配置 | `User/Src/mpu9250_driver.c` | `mpu_set_gyro_config()` | `FS_SEL=2` | `mpu9250_config_six_axis()` | 初始化 | `+-1000dps` |
| 加速度计量程配置 | `User/Src/mpu9250_driver.c` | `mpu_set_accel_range()` | `AFS_SEL=2` | `mpu9250_config_six_axis()` | 初始化 | `+-8g` |
| 六轴原始数据读取 | `User/Src/mpu9250_driver.c` | `MPU9250_ReadAxis()` | 连续读 14 字节并合成 raw | `ImuService_ReadRaw()`、`ImuService_ReadPhys()`、校准函数 | 周期运行/校准 | 当前无错误返回 |
| 六轴物理量转换 | `User/Src/mpu9250_driver.c` | `MPU9250_ConvertToPhysical()` | raw 转 g/dps/degC 并扣 bias | `ImuService_ReadPhys()` | 周期运行 | 使用 `4096` 和 `32.8` |
| AK8963 访问通道配置 | `User/Src/mpu9250_driver.c` | `mpu_set_ak8963_by_mcu()` | 关闭 I2C master，打开 bypass | `ak8963_init()` | 初始化 | 访问 `0x0C` 前必须做 |
| AK8963 WHO_AM_I 检测 | `User/Src/mpu9250_driver.c` | `AK8963_CheckDeviceID()` | 检查 WIA 是否 `0x48` | `ak8963_init()` | 初始化 | 只返回匹配/不匹配 |
| AK8963 Power Down | `User/Src/mpu9250_driver.c` | `AK8963_EnterPowerDownMode()` | 写 `CNTL1=0x00` | `ak8963_init()` | 初始化 | 切模式前后调用 |
| AK8963 Fuse ROM | `User/Src/mpu9250_driver.c` | `AK8963_EnterFuseROMMode()` | 写 `CNTL1=0x0F` | `ak8963_init()` | 初始化 | 用于读取 ASA |
| AK8963 ASA 读取 | `User/Src/mpu9250_driver.c` | `AK8963_AdjustSensitivity()` | 读 ASAX/Y/Z 并算系数 | `ak8963_init()` | 初始化 | 保存到 `g_ak8963_sensitivity` |
| AK8963 连续测量模式配置 | `User/Src/mpu9250_driver.c` | `AK8963_EnterContinuousMeasurementMode()` | 写 `CNTL1=0x16` | `ak8963_init()` | 初始化 | 16bit + 100Hz |
| ST1 检查 | `User/Src/mpu9250_driver.c` | `AK8963_CheckDataReady()` | 检查 DRDY | `ak8963_init()`、`AK8963_Read_Axis()` | 初始化/周期 | 实现中 1 ready |
| 磁力计原始数据读取 | `User/Src/mpu9250_driver.c` | `AK8963_Read_Axis()` | 连续读 HXL..ST2 | `AK8963_Read_Mag_UT()`、`AK8963_CalibrateMag()` | 周期/校准 | 低字节在前 |
| ST2 检查 | `User/Src/mpu9250_driver.c` | `AK8963_Read_Axis()` | 检查 `HOFL` | `AK8963_Read_Mag_UT()` | 周期运行 | 溢出返回 `-3` |
| 磁力计物理量转换 | `User/Src/mpu9250_driver.c` | `ak8963_convert_raw_to_ut()` | raw 转 uT | `AK8963_Calibrate()`、`AK8963_CalibrateMag()` | 周期/校准 | 静态函数 |
| 磁力计 offset / scale 补偿 | `User/Src/mpu9250_driver.c` | `ak8963_apply_mag_calibration()` | `M * (raw - center)` | `AK8963_Calibrate()` | 周期运行 | 3x3 矩阵 |
| 陀螺仪校准 | `User/Src/mpu9250_driver.c` | `MPU9250_CalibrateGyro()` | 静止求平均 bias | `MPU9250_Driver_Init()` | 初始化 | 1000 次，10ms |
| 加速度计校准 | `User/Src/mpu9250_driver.c` | `MPU9250_CalibrateAccel()` | 静止求 bias，Z 扣 1g | `MPU9250_Driver_Init()` | 初始化 | 假设水平 `+Z` |
| 磁力计校准 | `User/Src/mpu9250_driver.c` | `AK8963_CalibrateMag()` | 3D 点云椭球拟合 | `MPU9250_Driver_Init()` | 初始化 | 500 次，20ms |
| A+M 初始姿态解算 | 当前工程未找到现役函数 | 推荐新增 | 用 accel + mag 算初值 | 当前未被调用 | 推荐扩展 | 旧路径已停用 |
| Mahony 初始化 | `User/Src/mpu9250_driver.c` | `MPU9250_MahonyInit()` | 初始化 Kp/Ki、四元数、误差积分 | `InitTask()` | 初始化 | 当前单位四元数起步 |
| Mahony 九轴融合 | `User/Src/mpu9250_driver.c` | `MPU9250_MahonyUpdate()` | gyro+accel+mag 更新四元数 | `ImuTask()` | 周期运行 | mag 模长 15..100uT |
| Mahony 六轴融合 | `User/Src/mpu9250_driver.c` | `MPU9250_MahonyUpdateIMU()` | gyro+accel 更新四元数 | `ImuTask()` | 周期运行 | 磁力计失败时降级 |
| 欧拉角输出 | `User/Src/mpu9250_driver.c` | `MPU9250_GetEulerFusedDeg()` | 弧度转度，yaw 转导航航向 | `ImuTask()` | 周期运行 | 输出到 `AttitudeData_t` |
| InitTask | `User/Src/app_tasks.c` | `InitTask()` | 初始化服务和 IMU | FreeRTOS 调度器 | 初始化 | 成功置 `IMU_READY` |
| ImuTask | `User/Src/app_tasks.c` | `ImuTask()` | 周期读取和融合 | FreeRTOS 调度器 | 周期运行 | 写 `qAttitude` |
| TelemetryTask | `User/Src/app_tasks.c` | `TelemetryTask()` | 读取姿态/GNSS 并组包 | FreeRTOS 调度器 | 周期运行 | 当前会打印遥测日志 |
| qAttitude | `User/Src/freertos_objects.c` | `qAttitude` | 姿态最新值队列 | `ImuTask` 写，`TelemetryTask` 读 | 周期运行 | 长度 1 |
| AppStatus 状态位 | `User/Src/app_status.c` | `AppStatus_Set/IsSet/Clear()` | 系统 ready/error gate | 多任务 | 初始化/周期 | 基于 EventGroup |
| LED 校准状态提示 | `User/Src/app_tasks.c` | `LedTask()` | 按校准状态显示红绿灯 | FreeRTOS 调度器 | 周期运行 | 状态来自 `AppStatus_GetCalState()` |

## 17. 新手照着写的完整任务清单

* [ ] 第 1 步：完成软件 I2C GPIO 初始化
* [ ] 第 2 步：完成 I2C 单字节写寄存器函数
* [ ] 第 3 步：完成 I2C 单字节读寄存器函数
* [ ] 第 4 步：完成 I2C 多字节连续读函数
* [ ] 第 5 步：读取 MPU9250 WHO_AM_I
* [ ] 第 6 步：对 MPU9250 执行软复位
* [ ] 第 7 步：唤醒 MPU9250
* [ ] 第 8 步：设置 MPU9250 时钟源
* [ ] 第 9 步：确保六轴未 standby
* [ ] 第 10 步：配置陀螺仪 DLPF 和采样率
* [ ] 第 11 步：配置加速度计 DLPF 和采样率
* [ ] 第 12 步：配置陀螺仪量程
* [ ] 第 13 步：配置加速度计量程
* [ ] 第 14 步：读取六轴 14 字节原始数据
* [ ] 第 15 步：组合六轴高低字节
* [ ] 第 16 步：六轴 raw 转物理量
* [ ] 第 17 步：打开 AK8963 访问通道
* [ ] 第 18 步：读取 AK8963 WHO_AM_I
* [ ] 第 19 步：AK8963 进入 Power Down
* [ ] 第 20 步：AK8963 进入 Fuse ROM 模式
* [ ] 第 21 步：读取 ASAX / ASAY / ASAZ
* [ ] 第 22 步：退出 Fuse ROM，重新 Power Down
* [ ] 第 23 步：设置 AK8963 16bit 连续测量模式
* [ ] 第 24 步：检查 ST1 数据准备状态
* [ ] 第 25 步：读取磁力计三轴 raw 数据
* [ ] 第 26 步：读取 ST2 并判断溢出
* [ ] 第 27 步：磁力计 raw 转物理量
* [ ] 第 28 步：执行陀螺仪零偏校准
* [ ] 第 29 步：执行加速度计校准
* [ ] 第 30 步：执行磁力计校准
* [ ] 第 31 步：计算磁力计 offset / scale
* [ ] 第 32 步：对磁力计数据做补偿
* [ ] 第 33 步：用加速度计计算 roll / pitch
* [ ] 第 34 步：用磁力计计算 yaw
* [ ] 第 35 步：做磁力计倾斜补偿
* [ ] 第 36 步：初始化 Mahony / Fusion
* [ ] 第 37 步：周期运行九轴融合
* [ ] 第 38 步：输出 roll / pitch / yaw
* [ ] 第 39 步：在 ImuTask 中周期读取和融合
* [ ] 第 40 步：将姿态数据发送到 qAttitude
* [ ] 第 41 步：TelemetryTask 读取姿态并组包输出

## 18. 最终完整流程总结

| 顺序 | 阶段 | 要做什么 | 关键寄存器/函数 | 成功标志 | 失败排查 |
| -- | -- | ---- | -------- | ---- | ---- |
| 1 | 软件 I2C 准备 | 初始化 PB6/PB7，恢复总线 | `BSP_I2C_Soft_Init()` | SCL/SDA 空闲为高 | 接线、上拉、GPIO 模式 |
| 2 | MPU9250 WHO_AM_I 检测 | 读取主芯片 ID | `WHO_AM_I=0x75`，`MPU9250_Driver_ReadWhoAmI()` | 返回 `0x71` | 地址、AD0、供电、I2C 时序 |
| 3 | MPU9250 软复位 | 写复位位 | `PWR_MGMT_1.bit7=1`，`MPU9250_SoftReset()` | 延时后仍能读 ID | 延时不足、I2C 写失败 |
| 4 | 唤醒 MPU9250 | 清睡眠位 | `PWR_MGMT_1.bit6=0`，`mpu_set_clock_to_auto()` | `SLEEP=0` | 写寄存器失败 |
| 5 | 设置时钟源 | 选择 PLL | `PWR_MGMT_1.CLKSEL=1` | 低三位为 `001` | 读回确认 |
| 6 | 使能六轴 | 清 standby | `PWR_MGMT_2[5:0]=0`，`mpu_enable_six_axis()` | 六轴都更新 | 某轴固定为 0 |
| 7 | 配置陀螺仪 DLPF 和采样率 | 降噪并设置内部采样 | `CONFIG=...3`，`SMPLRT_DIV=4` | 静止 gyro 噪声可接受 | DLPF、采样率、振动 |
| 8 | 配置加速度计 DLPF 和采样率 | 降低振动影响 | `ACCEL_CONFIG2`，`mpu_set_accel_dlpf()` | 静止模长约 1g | 量程、安装、振动 |
| 9 | 配置陀螺仪量程 | 设置 `+-1000dps` | `GYRO_CONFIG.FS_SEL=2` | 比例 `32.8 LSB/dps` | 饱和或分辨率不合适 |
| 10 | 配置加速度计量程 | 设置 `+-8g` | `ACCEL_CONFIG.AFS_SEL=2` | 比例 `4096 LSB/g` | 静止 1g 不对 |
| 11 | 读取六轴原始数据 | 连续读 14 字节 | `MPU9250_ReadAxis()` | raw 随姿态变化 | I2C、睡眠、standby |
| 12 | 六轴 raw 转物理量 | 转 g/dps/degC | `MPU9250_ConvertToPhysical()` | 静止 acc 约 1g，gyro 约 0 | 常量和量程不匹配 |
| 13 | 打开 AK8963 访问通道 | bypass | `USER_CTRL.bit5=0`，`INT_PIN_CFG.bit1=1` | AK8963 地址 ACK | bypass、I2C master 冲突 |
| 14 | AK8963 WHO_AM_I 检测 | 读 WIA | `AK8963_CheckDeviceID()` | 返回 `0x48` | 地址、bypass、模块型号 |
| 15 | AK8963 power-down | 模式切换起点 | `CNTL1=0x00` | 延时后可切模式 | 没延时 |
| 16 | AK8963 进入 Fuse ROM | 读取 ASA | `CNTL1=0x0F` | ASA 合理 | 模式切换失败 |
| 17 | 读取 ASA | 读出厂灵敏度 | `ASAX/ASAY/ASAZ` | 系数约在 1 附近 | ASA 全 0/255 |
| 18 | 退出 Fuse ROM | 回 Power Down | `CNTL1=0x00` | 可进入测量模式 | 顺序错误 |
| 19 | 设置 AK8963 连续测量模式 | 16bit + 100Hz | `CNTL1=0x16` | `ST1.DRDY=1` | 模式值、延时 |
| 20 | 检查 ST1 | 判断新数据 | `AK8963_CheckDataReady()` | ready 返回 1 | 连续模式、ST2 锁存 |
| 21 | 读取磁力计原始数据 | 连续读 HXL..ST2 | `AK8963_Read_Axis()` | raw 非零且变化 | 字节顺序、I2C |
| 22 | 读取 ST2 并判断溢出 | 检查 HOFL | `ST2.bit3` | `HOFL=0` | 强磁场、没读 ST2 |
| 23 | 磁力计 raw 转物理量 | 转 uT | `ak8963_convert_raw_to_ut()` | 地磁几十 uT | ASA、比例、字节序 |
| 24 | 陀螺仪零偏校准 | 静止求平均 | `MPU9250_CalibrateGyro()` | 静止 dps 接近 0 | 校准时移动 |
| 25 | 加速度计校准 | 静止水平求 bias | `MPU9250_CalibrateAccel()` | 静止模长约 1g | 姿态假设不符 |
| 26 | 磁力计校准 | 3D 旋转采样 | `AK8963_CalibrateMag()` | `calibrate mag ok` | 样本少、半径小、fit failed |
| 27 | 磁力计 offset / scale 补偿 | 应用硬铁/软铁 | `ak8963_apply_mag_calibration()` | 四方向 yaw 间隔接近 90 度 | 环境干扰、动作不充分 |
| 28 | A+M 初始 roll / pitch / yaw 解算 | 推荐新增 | 当前工程未找到现役函数 | 推荐函数输出合理角度 | 轴向和倾斜补偿 |
| 29 | Mahony 九轴姿态融合 | 更新四元数 | `MPU9250_MahonyUpdate()` | yaw 不长期漂移 | 单位、dt、mag_valid |
| 30 | ImuTask 周期运行 | 20ms 周期采样融合 | `ImuTask()`，`APP_IMU_TASK_PERIOD_MS` | 周期刷新姿态 | 软件 I2C 超时 |
| 31 | 姿态数据进入 qAttitude | 覆盖最新姿态 | `xQueueOverwrite(qAttitude, &attitude)` | 队列有 valid 姿态 | IMU_READY、队列创建 |
| 32 | TelemetryTask 组包输出 | JSON 包含姿态 | `Telemetry_BuildMqttMsg()` | payload 有 roll/pitch/yaw | 姿态无效、队列为空 |

这条链路写完以后，新手就可以按顺序验证：先看 I2C ACK，再看 `0x71 / 0x48`，再看六轴 raw，再看 uT 磁场，再看校准日志，最后才看姿态角。这样排查不会乱。
