# MPU9250 + AK8963 姿态角通用复用笔记

这份笔记以当前 AeroPush 工程为例，但写法按“以后换一个 STM32 工程也能复用”的方式整理。只要硬件仍然是 MPU9250 内置六轴 + AK8963 磁力计，并且通信方式还是 I2C，核心思路不用重写，主要改 I2C 引脚、地址、任务周期和坐标轴方向。

## 1. 当前工程结论

当前工程的姿态角主链路已经打通：

```text
APP_Main()
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
  -> MPU9250_GetEulerFusedDeg(&roll_deg, &pitch_deg, &yaw_deg)
  -> AttitudeData_t
  -> xQueueOverwrite(qAttitude, &attitude)

TelemetryTask()
  -> xQueuePeek(qAttitude, &attitude, 0)
  -> Telemetry_BuildMqttMsg()
  -> JSON 中带 roll / pitch / yaw
```

也就是说，MPU9250 和 AK8963 从初始化、校准、周期采样、Mahony 融合，到最终拿到 `roll_deg / pitch_deg / yaw_deg` 并写入 `qAttitude`，已经是完整路径。

当前工程里真正对外使用的姿态角来自 Mahony 融合路径：`g_euler_fused -> MPU9250_GetEulerFusedDeg() -> AttitudeData_t -> qAttitude`。旧的“加速度计 + 磁力计直接解算欧拉角”路径已经不作为运行主线。

## 2. 代码文件分工

| 文件 | 作用 |
| --- | --- |
| `User/Inc/bsp_i2c_soft.h` | 软件 I2C 引脚、端口、地址宏。移植时优先改这里。 |
| `User/Src/bsp_i2c_soft.c` | 软件 I2C 时序、读写寄存器、总线恢复。 |
| `User/Inc/mpu9250_driver.h` | MPU9250/AK8963 数据结构和驱动接口声明。 |
| `User/Src/mpu9250_driver.c` | 设备初始化、寄存器配置、三轴转换、校准、Mahony 融合。 |
| `User/Src/imu_service.c` | 任务层和底层驱动之间的薄封装。 |
| `User/Src/app_tasks.c` | `InitTask` 初始化 IMU，`ImuTask` 周期融合姿态，`TelemetryTask` 读取姿态。 |
| `User/Src/freertos_objects.c` | 创建 `qAttitude`，队列长度为 1，只保留最新姿态。 |
| `User/Src/telemetry_service.c` | 将姿态角打包到遥测 JSON。 |
| `Docs/magnetometer_calibration_notes.md` | 磁力计校准细节和验证方法。 |

## 3. 最小移植清单

如果以后重新开发 MPU9250，先按这个顺序处理：

1. 改 I2C 引脚。

   当前工程在 `User/Inc/bsp_i2c_soft.h`：

   ```c
   #define I2C_GPIO_PORT GPIOB
   #define I2C_GPIO_CLK  RCC_AHB1Periph_GPIOB
   #define I2C_SCL_PIN   GPIO_Pin_6
   #define I2C_SDA_PIN   GPIO_Pin_7
   ```

   换板子时通常只改这四个宏。如果换成硬件 I2C，就保留上层接口名，把 `I2C_ReadRegs()`、`I2C_WriteRegs()` 等函数改成调用硬件 I2C。

2. 确认 I2C 地址。

   ```c
   #define MPU9250_I2C_ADDR7 0x68U
   #define AK8963_I2C_ADDR7  0x0CU
   ```

   MPU9250 的 AD0 脚会影响地址：AD0 接低通常是 `0x68`，接高通常是 `0x69`。AK8963 在旁路模式下一般是 `0x0C`。

3. 确认电气条件。

   - VCC 使用 3.3V；
   - SCL/SDA 需要上拉；
   - MCU 和模块必须共地；
   - 如果模块上已有上拉，外部上拉不要过强；
   - 先用 `WHO_AM_I` 判断 I2C 是否真的通。

4. 确认坐标轴方向。

   当前工程在融合时做了坐标方向统一，例如 Y 轴取反、磁力计 Y/Z 取反。换安装方向后，不一定只改引脚，可能还要改 `MPU9250_MahonyUpdate()` 和 `MPU9250_MahonyUpdateIMU()` 中的轴向映射。

5. 确认任务周期。

   当前 `APP_IMU_TASK_PERIOD_MS` 是 20ms，Mahony 使用：

   ```c
   const float imu_dt = (float)APP_IMU_TASK_PERIOD_MS * 0.001f;
   ```

   如果改成 10ms、5ms 或别的周期，`dt` 必须同步准确，否则陀螺积分会漂。

## 4. 初始化顺序

MPU9250 初始化主入口是：

```c
uint8_t MPU9250_Driver_Init(void);
```

它当前做四件事：

1. 初始化 I2C 总线。

   ```text
   BSP_I2C_Soft_Init()
     -> 配置 SCL/SDA 为开漏输出
     -> 释放 SCL/SDA
     -> BSP_I2C_Soft_RecoverBus()
   ```

2. 检查 MPU9250。

   ```text
   MPU9250_Driver_ReadWhoAmI()
     -> 读 0x75
     -> 正常值应为 0x71
   ```

3. 配置 MPU9250 六轴。

   ```text
   MPU9250_SoftReset()
   mpu_set_clock_to_auto()
   mpu_enable_six_axis()
   mpu_set_dlpf_cfg_3()
   mpu_set_sample_rate_200hz()
   mpu_set_accel_dlpf()
   mpu_set_gyro_config()
   mpu_set_accel_range()
   ```

   当前量程和换算关系：

   | 传感器 | 当前配置 | 换算常量 |
   | --- | --- | --- |
   | 加速度计 | +-8g | `4096.0f` LSB/g |
   | 陀螺仪 | +-1000dps | `32.8f` LSB/dps |
   | 磁力计 | 16 位 | `0.15f` uT/LSB，再乘 ASA 修正 |

4. 初始化 AK8963。

   ```text
   mpu_set_ak8963_by_mcu()
     -> 关闭 MPU9250 内部 I2C 主机
     -> 打开 BYPASS_EN

   AK8963_CheckDeviceID()
     -> 读 WIA
     -> 正常值应为 0x48

   AK8963_EnterPowerDownMode()
   AK8963_EnterFuseROMMode()
   AK8963_AdjustSensitivity()
   AK8963_EnterPowerDownMode()
   AK8963_EnterContinuousMeasurementMode()
   AK8963_CheckDataReady()
   ```

   AK8963 的关键模式值：

   | 模式 | CNTL1 |
   | --- | --- |
   | Power-down | `0x00` |
   | Fuse ROM access | `0x0F` |
   | 16 位连续测量模式 2 | `0x16` |

## 5. 启动阶段校准

当前初始化成功前会做三类校准：

```c
MPU9250_CalibrateGyro(1000U, 10U);
MPU9250_CalibrateAccel(1000U, 10U);
AK8963_CalibrateMag(500U, 20U);
```

含义：

| 校准 | 动作要求 | 当前算法 |
| --- | --- | --- |
| 陀螺仪零偏 | 板子静止 | 连续采样求平均，得到三轴 dps 零偏 |
| 加速度计零偏 | 板子静止且尽量水平 | 连续采样求平均，Z 轴扣除静止重力 1g |
| 磁力计硬铁/软铁 | 板子绕各方向充分旋转 | 采样点云，先用 min/max 估中心，再做 3D 椭球拟合 |

磁力计最终应用的形式是：

```text
corrected = M * (raw_ut - center)
```

其中：

- `center` 保存在 `g_mag_offset_ut[3]`，表示硬铁偏移；
- `M` 保存在 `g_mag_correction[3][3]`，表示软铁修正矩阵；
- 运行时 `AK8963_Calibrate()` 会把每一帧磁场都转换并校正。

磁力计校准通过时，串口日志应能看到类似：

```text
[AK8963] calibrate mag ok center=... radius=... fit=...
[AK8963] mag matrix row0=...
[AK8963] mag matrix row1=...
[AK8963] mag matrix row2=...
[AK8963] mag calibration done
```

磁力计校准失败时，重点看这些拒绝原因：

| 日志/条件 | 常见原因 |
| --- | --- |
| `calibrate mag rejected valid=...` | 有效样本少于 64，可能旋转不充分或数据未就绪太多 |
| `calibrate mag rejected radius...` | 三轴覆盖半径太小，通常是没转够 |
| `calibrate mag rejected fit failed` | 点云质量差或数值无法收敛 |
| `calibrate mag rejected implausible fit...` | 拟合结果不可信，环境磁干扰或点云分布异常 |

## 6. 周期采样和物理量转换

任务层每周期调用：

```c
imu_read_ok = ImuService_ReadPhys(&phys, &mag_physical);
```

内部顺序：

```text
MPU9250_ReadAxis(&raw)
  -> 读 ACCEL_XOUT_H 起始的一帧六轴和温度数据

MPU9250_ConvertToPhysical(&raw, &phys)
  -> raw accel / 4096.0f - accel_bias
  -> raw gyro / 32.8f - gyro_bias
  -> temp = raw / 333.87f + 21.0f

AK8963_Read_Mag_UT(&mag)
  -> AK8963_Read_Axis()
  -> 读 HXL 到 ST2
  -> 检查 DRDY 和 HOFL
  -> AK8963_Calibrate()
```

`ImuService_ReadPhys()` 有三种返回值：

| 返回值 | 含义 | 后续融合 |
| --- | --- | --- |
| `IMU_SERVICE_READ_9AXIS_OK` | 六轴和磁力计都有效 | 使用 9 轴 Mahony |
| `IMU_SERVICE_READ_6AXIS_OK` | 六轴有效，磁力计本帧失败 | 退回 6 轴 Mahony |
| `IMU_SERVICE_READ_FAIL` | 输出参数无效或六轴读取不可用 | 本轮不刷新姿态 |

## 7. Mahony 融合主线

初始化时调用：

```c
MPU9250_MahonyInit(0.3f, 0.0f);
```

它会重置：

- 比例增益 `g_kp`；
- 积分增益 `g_ki`；
- 四元数 `g_q = {1, 0, 0, 0}`；
- 误差积分项 `g_exInt/g_eyInt/g_ezInt`；
- 融合欧拉角 `g_euler_fused`。

运行时选择：

```c
if (imu_read_ok == IMU_SERVICE_READ_9AXIS_OK)
{
    MPU9250_MahonyUpdate(&phys, &mag_physical, imu_dt);
}
else
{
    MPU9250_MahonyUpdateIMU(&phys, imu_dt);
}
```

区别：

| 函数 | 使用数据 | 特点 |
| --- | --- | --- |
| `MPU9250_MahonyUpdate()` | 加速度计 + 陀螺仪 + 磁力计 | 能用磁场修正 yaw 长期漂移 |
| `MPU9250_MahonyUpdateIMU()` | 加速度计 + 陀螺仪 | 能维持 roll/pitch，yaw 会随陀螺积分漂移 |

`MPU9250_MahonyUpdate()` 里还会检查磁场强度，当前认为 15uT 到 100uT 之间才可信。如果磁场模长异常，即使本帧读到了磁力计，也会降低或取消磁力计对融合的影响。

融合结果内部以弧度存在 `g_euler_fused`，最后通过：

```c
MPU9250_GetEulerFusedDeg(&roll_deg, &pitch_deg, &yaw_deg);
```

转成角度输出。

## 8. 航向角约定

当前工程对外输出的是无人机/导航习惯的航向角：

```text
北 = 0 度
东 = +90 度
南 = -180 或 +180 度
西 = -90 度
范围 = (-180, 180]
```

代码里先由四元数算数学坐标系 yaw，再通过：

```c
mpu9250_math_yaw_to_heading_deg()
```

转换成导航航向角。

如果以后发现方向反了，不要先乱改 Mahony 公式，先按这个顺序检查：

1. 板子的 X/Y/Z 安装方向是否和代码假设一致；
2. `MPU9250_MahonyUpdate()` 里加速度、陀螺仪、磁力计的正负号是否匹配；
3. `mpu9250_math_yaw_to_heading_deg()` 的航向转换是否符合你的产品定义；
4. 磁力计校准是否真的通过，而不是靠 6 轴模式在跑。

## 9. 姿态输出接口

当前姿态输出结构体是：

```c
typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint32_t timestamp_ms;
    uint8_t valid;
} AttitudeData_t;
```

`ImuTask` 每周期填充它：

```c
attitude.roll_deg = roll_deg;
attitude.pitch_deg = pitch_deg;
attitude.yaw_deg = yaw_deg;
attitude.timestamp_ms = xTaskGetTickCount();
attitude.valid = 1U;
xQueueOverwrite(qAttitude, &attitude);
```

`qAttitude` 的长度是 1：

```c
qAttitude = xQueueCreate(1, sizeof(AttitudeData_t));
```

这表示它不是历史队列，而是“最新值缓存”。消费端读取时应使用 `xQueuePeek()` 或接收最新值，不要期望里面保存多帧姿态历史。

遥测打包路径：

```text
TelemetryTask()
  -> xQueuePeek(qAttitude, &attitude, 0)
  -> Telemetry_BuildMqttMsg()
  -> {"roll":..., "pitch":..., "yaw":...}
```

如果以后不用 FreeRTOS，可以把 `qAttitude` 换成一个全局 `AttitudeData_t g_attitude`，但要自己处理并发访问。

## 10. 复用到新工程的推荐骨架

最小主流程可以写成这样：

```c
static void AppImuInit(void)
{
    if (MPU9250_Driver_Init() != 0U)
    {
        MPU9250_MahonyInit(0.3f, 0.0f);
    }
}

static void AppImuLoop(void)
{
    MPU9250_Physical_Data imu = {0};
    AK8963_Physical_Data mag = {0};
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    uint8_t ret;

    ret = ImuService_ReadPhys(&imu, &mag);
    if (ret == IMU_SERVICE_READ_9AXIS_OK)
    {
        MPU9250_MahonyUpdate(&imu, &mag, 0.02f);
    }
    else if (ret == IMU_SERVICE_READ_6AXIS_OK)
    {
        MPU9250_MahonyUpdateIMU(&imu, 0.02f);
    }
    else
    {
        return;
    }

    MPU9250_GetEulerFusedDeg(&roll, &pitch, &yaw);
}
```

如果不用当前工程的 `ImuService_ReadPhys()`，也可以直接调用底层：

```text
MPU9250_ReadAxis()
MPU9250_ConvertToPhysical()
AK8963_Read_Mag_UT()
MPU9250_MahonyUpdate()
MPU9250_GetEulerFusedDeg()
```

## 11. 调试顺序

第一次移植时，不要一上来就看姿态角。按这个顺序验证：

1. I2C 基础通信。

   - MPU9250 `WHO_AM_I` 应为 `0x71`；
   - AK8963 `WIA` 应为 `0x48`。

2. 六轴原始值。

   - 静止时加速度模长应接近 1g；
   - 静止时陀螺仪应接近 0dps；
   - 摇动板子时原始值应变化。

3. AK8963 原始磁场。

   - `ST1.DRDY` 要能置位；
   - 读取时必须带上 `ST2`，否则 AK8963 可能不释放下一帧；
   - `ST2.HOFL` 置位表示磁场溢出，本帧不要用。

4. 校准日志。

   - 必须看到 `calibrate mag ok` 和 `mag calibration done`；
   - 如果只有初始化成功但磁力计校准失败，yaw 不要直接当可信结果。

5. 姿态角。

   - 水平静置时 roll/pitch 应接近 0；
   - 绕 X/Y 轴倾斜时 roll/pitch 方向应符合定义；
   - 水平朝北/东/南/西时 yaw 应大致落在 0/+90/180/-90；
   - 如果 yaw 四个方向间隔不接近 90 度，优先重做磁力计校准。

## 12. 常见问题定位表

| 现象 | 优先检查 |
| --- | --- |
| MPU9250 `WHO_AM_I` 读不到 | SCL/SDA 接反、地址 0x68/0x69 错、上拉不足、供电不对 |
| MPU9250 能读，AK8963 读不到 | 没开 BYPASS_EN、没关 MPU9250 内部 I2C 主机、模块不是带 AK8963 的版本 |
| AK8963 一直 data not ready | 连续测量模式没进、模式切换延时不足、没有读 ST2 |
| roll/pitch 方向反 | 板子安装方向和代码轴向映射不一致 |
| yaw 方向反或偏 90 度 | 磁力计轴向映射或航向角转换不匹配 |
| yaw 漂移明显 | 磁力计未参与融合，可能退回了 6 轴模式 |
| yaw 四个方向不均匀 | 磁力计硬铁/软铁校准不够好，或附近有磁干扰 |
| 静止时姿态抖 | I2C 读数不稳定、任务周期不稳、`dt` 不准、滤波和增益不合适 |
| 运动时 roll/pitch 被拉偏 | 加速度动态太大，Mahony 中加速度修正权重需要调 |

## 13. 真正需要改的地方

一般换同类工程，只需要动这些位置：

| 改动目标 | 位置 |
| --- | --- |
| 换 I2C 引脚 | `User/Inc/bsp_i2c_soft.h` 的端口、时钟、SCL、SDA 宏 |
| 换硬件 I2C | 保留 `I2C_ReadRegs()` / `I2C_WriteRegs()` 接口，替换其底层实现 |
| MPU9250 地址变成 0x69 | `MPU9250_I2C_ADDR` |
| 改任务周期 | `APP_IMU_TASK_PERIOD_MS`，并确认 Mahony `dt` 同步 |
| 改姿态发布方式 | `ImuTask` 写入姿态后的输出部分 |
| 改航向角定义 | `mpu9250_math_yaw_to_heading_deg()` |
| 改安装方向 | `MPU9250_MahonyUpdate()` 和 `MPU9250_MahonyUpdateIMU()` 的轴向符号 |
| 改校准策略 | `MPU9250_CalibrateGyro()`、`MPU9250_CalibrateAccel()`、`AK8963_CalibrateMag()` |

如果只是同一块 MPU9250 模块换到另一组 I2C 引脚，且板子安装方向不变，通常改 `bsp_i2c_soft.h` 的引脚宏就够了。如果模块方向、安装姿态、AD0 地址、任务周期或 I2C 实现方式变了，就按上表同步改。

