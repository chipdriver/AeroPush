#ifndef __MPU9250_DRIVER_H__ // 防止头文件重复包含
#define __MPU9250_DRIVER_H__

#include <stdint.h> // 提供固定宽度整数类型
#include "bsp_i2c_soft.h" // 提供软件 I2C 读写接口
#include "debug_log.h" // 提供调试日志接口
#include "FreeRTOS.h" // 提供 FreeRTOS 基础类型
#include "task.h" // 提供 FreeRTOS 延时接口

#define MPU9250_I2C_ADDR 0x68U // MPU9250 7 位 I2C 地址
#define MPU9250_I2C_ADDR7 MPU9250_I2C_ADDR // MPU9250 7 位地址别名
#define AK8963_I2C_ADDR7 0x0CU // AK8963 7 位 I2C 地址
#define MPU9250_REG_WHO_AM_I 0x75U // MPU9250 设备 ID 寄存器
#define MPU9250_WHO_AM_I_VALUE 0x71U // MPU9250 正常设备 ID

typedef struct // MPU9250 六轴原始寄存器数据
{
    int16_t accel_x; // 加速度 X 轴原始值
    int16_t accel_y; // 加速度 Y 轴原始值
    int16_t accel_z; // 加速度 Z 轴原始值
    int16_t gyro_x; // 陀螺仪 X 轴原始值
    int16_t gyro_y; // 陀螺仪 Y 轴原始值
    int16_t gyro_z; // 陀螺仪 Z 轴原始值
    int16_t temp; // 温度原始值
} MPU9250_raw_Data;

typedef struct // MPU9250 六轴物理量
{
    float accel_x_g; // 加速度 X 轴，单位 g
    float accel_y_g; // 加速度 Y 轴，单位 g
    float accel_z_g; // 加速度 Z 轴，单位 g
    float gyro_x_dps; // 角速度 X 轴，单位 dps
    float gyro_y_dps; // 角速度 Y 轴，单位 dps
    float gyro_z_dps; // 角速度 Z 轴，单位 dps
    float temp_c; // 温度，单位摄氏度
} MPU9250_Physical_Data;

typedef struct // AK8963 磁力计原始数据
{
    int16_t mag_x; // 磁力计 X 轴原始值
    int16_t mag_y; // 磁力计 Y 轴原始值
    int16_t mag_z; // 磁力计 Z 轴原始值
} AK8963_raw_Data;

typedef struct // AK8963 磁力计物理量
{
    float mag_x_ut; // 磁场 X 轴，单位 uT
    float mag_y_ut; // 磁场 Y 轴，单位 uT
    float mag_z_ut; // 磁场 Z 轴，单位 uT
} AK8963_Physical_Data;

typedef struct // 欧拉角，内部通常用弧度保存
{
    float roll; // 横滚角
    float pitch; // 俯仰角
    float yaw; // 航向角
} EulerAngle_t;

typedef struct // 姿态四元数
{
    float q0; // 四元数标量分量
    float q1; // 四元数 X 分量
    float q2; // 四元数 Y 分量
    float q3; // 四元数 Z 分量
} Quaternion_t;

extern EulerAngle_t g_euler_fused; // Mahony 融合后的欧拉角状态

/**
 * @brief 初始化 MPU9250、AK8963，并完成启动阶段校准。
 * @retval 1 初始化成功；0 初始化失败。
 */
uint8_t MPU9250_Driver_Init(void); // 初始化 MPU9250 驱动

/**
 * @brief 读取 MPU9250 WHO_AM_I。
 * @param id 输出设备 ID。
 * @retval 0 读取成功；非 0 表示参数无效。
 */
uint8_t MPU9250_Driver_ReadWhoAmI(uint8_t *id); // 读取 MPU9250 设备 ID

/**
 * @brief 软件复位 MPU9250。
 * @retval None
 */
void MPU9250_SoftReset(void); // 复位 MPU9250

/**
 * @brief 读取电源管理寄存器并打印调试信息。
 * @retval None
 */
void MPU9250_Read_PowerMgmt(void); // 读取电源管理状态

/**
 * @brief 唤醒 MPU9250 并选择自动时钟源。
 * @retval None
 */
void mpu_set_clock_to_auto(void); // 配置 MPU9250 时钟源

/**
 * @brief 使能三轴加速度计和三轴陀螺仪。
 * @retval None
 */
void mpu_enable_six_axis(void); // 使能六轴传感器

/**
 * @brief 配置陀螺仪 DLPF。
 * @retval None
 */
void mpu_set_dlpf_cfg_3(void); // 配置陀螺仪低通滤波

/**
 * @brief 设置六轴共用采样率分频为约 200Hz。
 * @retval None
 */
void mpu_set_sample_rate_200hz(void); // 配置六轴采样率

/**
 * @brief 配置加速度计 DLPF。
 * @retval None
 */
void mpu_set_accel_dlpf(void); // 配置加速度计低通滤波

/**
 * @brief 配置陀螺仪量程。
 * @retval None
 */
void mpu_set_gyro_config(void); // 配置陀螺仪量程

/**
 * @brief 配置加速度计量程。
 * @retval None
 */
void mpu_set_accel_range(void); // 配置加速度计量程

/**
 * @brief 读取 MPU9250 加速度计、陀螺仪和温度原始寄存器。
 * @param raw 输出原始采样数据。
 * @retval None
 */
void MPU9250_ReadAxis(MPU9250_raw_Data *raw); // 读取六轴原始值

/**
 * @brief 将 MPU9250 原始值转换成 g、dps 和摄氏度物理量。
 * @param raw 输入原始采样数据。
 * @param physical 输出六轴物理量。
 * @retval None
 */
void MPU9250_ConvertToPhysical(const MPU9250_raw_Data *raw, MPU9250_Physical_Data *physical); // 转换 MPU9250 物理量

/**
 * @brief 配置 MPU9250 进入旁路模式，让 MCU 直接访问 AK8963。
 * @retval None
 */
void mpu_set_ak8963_by_mcu(void); // 打开磁力计旁路访问

/**
 * @brief 检查 AK8963 设备 ID。
 * @retval 0 ID 正确；负数表示异常。
 */
int AK8963_CheckDeviceID(void); // 检查 AK8963 设备 ID

/**
 * @brief 让 AK8963 进入掉电模式。
 * @retval None
 */
void AK8963_EnterPowerDownMode(void); // 切换 AK8963 到掉电模式

/**
 * @brief 让 AK8963 进入 Fuse ROM 模式。
 * @retval None
 */
void AK8963_EnterFuseROMMode(void); // 切换 AK8963 到 Fuse ROM 模式

/**
 * @brief 读取 AK8963 灵敏度调整值。
 * @retval None
 */
void AK8963_AdjustSensitivity(void); // 读取磁力计灵敏度调整值

/**
 * @brief 让 AK8963 进入连续测量模式。
 * @retval None
 */
void AK8963_EnterContinuousMeasurementMode(void); // 开启磁力计连续测量

/**
 * @brief 检查 AK8963 数据是否就绪。
 * @retval 0 数据就绪；负数表示未就绪或异常。
 */
int AK8963_CheckDataReady(void); // 检查磁力计数据就绪

/**
 * @brief 读取 AK8963 三轴磁力计原始值。
 * @param raw 输出磁力计原始数据。
 * @retval 0 读取成功；负数表示失败。
 */
int AK8963_Read_Axis(AK8963_raw_Data *raw); // 读取磁力计原始值

/**
 * @brief 将 AK8963 原始磁场值转换并应用校准。
 * @param raw 输入磁力计原始数据。
 * @param physical 输出磁力计物理量。
 * @retval None
 */
void AK8963_Calibrate(const AK8963_raw_Data *raw, AK8963_Physical_Data *physical); // 转换并校准磁力计数据

/**
 * @brief 读取并校准 AK8963 磁力计物理量。
 * @param mag_out 输出磁力计物理量。
 * @retval 0 读取成功；非 0 表示失败。
 */
int AK8963_Read_Mag_UT(AK8963_Physical_Data *mag_out); // 读取磁力计物理量

/**
 * @brief 采样并计算陀螺仪零偏。
 * @param samples 校准采样次数。
 * @param delay_ms 采样间隔毫秒数。
 * @retval None
 */
void MPU9250_CalibrateGyro(uint16_t samples, uint16_t delay_ms); // 校准陀螺仪零偏

/**
 * @brief 采样并计算加速度计零偏。
 * @param samples 校准采样次数。
 * @param delay_ms 采样间隔毫秒数。
 * @retval None
 */
void MPU9250_CalibrateAccel(uint16_t samples, uint16_t delay_ms); // 校准加速度计零偏

/**
 * @brief 采集磁力计样本并计算硬铁和软铁校准参数。
 * @param samples 校准采样次数。
 * @param delay_ms 采样间隔毫秒数。
 * @retval None
 */
void AK8963_CalibrateMag(uint16_t samples, uint16_t delay_ms); // 校准磁力计硬铁和软铁

/*
 * 旧的加速度计 + 磁力计直接解算欧拉角接口已停用。
 * 当前运行时使用 Mahony 融合接口输出姿态。
 */

/**
 * @brief 初始化 Mahony 姿态融合四元数和误差积分项。
 * @param kp Mahony 比例修正增益。
 * @param ki Mahony 积分修正增益。
 * @retval None
 */
void MPU9250_MahonyInit(float kp, float ki); // 初始化 Mahony 融合

/**
 * @brief 使用九轴数据执行 Mahony 姿态融合更新。
 * @param imu MPU9250 六轴物理量数据。
 * @param mag AK8963 磁力计物理量数据。
 * @param dt 姿态融合采样周期秒数。
 * @retval None
 */
void MPU9250_MahonyUpdate(const MPU9250_Physical_Data *imu, // 输入六轴物理量
                          const AK8963_Physical_Data *mag, // 输入磁力计物理量
                          float dt); // 输入融合周期秒数

/**
 * @brief 使用六轴 IMU 数据执行 Mahony 姿态融合更新。
 * @param imu MPU9250 六轴物理量数据。
 * @param dt 姿态融合采样周期秒数。
 * @retval None
 */
void MPU9250_MahonyUpdateIMU(const MPU9250_Physical_Data *imu, float dt); // 使用六轴更新 Mahony 融合

/**
 * @brief 读取 Mahony 融合后的欧拉角角度值。
 * @param roll_deg 输出横滚角角度值。
 * @param pitch_deg 输出俯仰角角度值。
 * @param yaw_deg 输出航向角角度值。
 * @retval None
 */
void MPU9250_GetEulerFusedDeg(float *roll_deg, float *pitch_deg, float *yaw_deg); // 读取融合姿态角

#endif // __MPU9250_DRIVER_H__
