#ifndef __MPU9250_DRIVER_H__ // 检查 __MPU9250_DRIVER_H__ 是否未定义，防止头文件重复包含
#define __MPU9250_DRIVER_H__ // 定义 __MPU9250_DRIVER_H__ 变量

#include <stdint.h> // 引入 stdint.h 提供的接口、宏和类型定义
#include "bsp_i2c_soft.h" // 引入 bsp_i2c_soft.h 提供的接口、宏和类型定义
#include "debug_log.h" // 引入 debug_log.h 提供的接口、宏和类型定义
#include "FreeRTOS.h" // 引入 FreeRTOS.h 提供的接口、宏和类型定义
#include "task.h" // 引入 task.h 提供的接口、宏和类型定义

#define MPU9250_I2C_ADDR 0x68U // 定义 MPU9250_I2C_ADDR 变量为 0x68U
#define MPU9250_I2C_ADDR7 MPU9250_I2C_ADDR // 定义 MPU9250_I2C_ADDR7 变量为 MPU9250_I2C_ADDR
#define AK8963_I2C_ADDR7 0x0CU // 定义 AK8963_I2C_ADDR7 变量为 0x0CU
#define MPU9250_REG_WHO_AM_I 0x75U // 定义 MPU9250_REG_WHO_AM_I 寄存器地址为 0x75U
#define MPU9250_WHO_AM_I_VALUE 0x71U // 定义 MPU9250_WHO_AM_I_VALUE 变量为 0x71U

typedef struct // 开始定义结构体类型，用于集中保存相关数据字段
{ // 进入当前代码块
    int16_t accel_x; // 声明 accel_x 变量，供后续计算、状态保存或模块间传递使用
    int16_t accel_y; // 声明 accel_y 变量，供后续计算、状态保存或模块间传递使用
    int16_t accel_z; // 声明 accel_z 变量，供后续计算、状态保存或模块间传递使用
    int16_t gyro_x; // 声明 gyro_x 变量，供后续计算、状态保存或模块间传递使用
    int16_t gyro_y; // 声明 gyro_y 变量，供后续计算、状态保存或模块间传递使用
    int16_t gyro_z; // 声明 gyro_z 变量，供后续计算、状态保存或模块间传递使用
    int16_t temp; // 声明 temp 变量，供后续计算、状态保存或模块间传递使用
} MPU9250_raw_Data; // 结束结构体定义，并声明结构体类型名 MPU9250_raw_Data

typedef struct // 开始定义结构体类型，用于集中保存相关数据字段
{ // 进入当前代码块
    float accel_x_g; // 声明 accel_x_g 变量，供后续计算、状态保存或模块间传递使用
    float accel_y_g; // 声明 accel_y_g 变量，供后续计算、状态保存或模块间传递使用
    float accel_z_g; // 声明 accel_z_g 变量，供后续计算、状态保存或模块间传递使用
    float gyro_x_dps; // 声明 gyro_x_dps 变量，供后续计算、状态保存或模块间传递使用
    float gyro_y_dps; // 声明 gyro_y_dps 变量，供后续计算、状态保存或模块间传递使用
    float gyro_z_dps; // 声明 gyro_z_dps 变量，供后续计算、状态保存或模块间传递使用
    float temp_c; // 声明 temp_c 变量，供后续计算、状态保存或模块间传递使用
} MPU9250_Physical_Data; // 结束结构体定义，并声明结构体类型名 MPU9250_Physical_Data

typedef struct // 开始定义结构体类型，用于集中保存相关数据字段
{ // 进入当前代码块
    int16_t mag_x; // 声明 mag_x 变量，供后续计算、状态保存或模块间传递使用
    int16_t mag_y; // 声明 mag_y 变量，供后续计算、状态保存或模块间传递使用
    int16_t mag_z; // 声明 mag_z 变量，供后续计算、状态保存或模块间传递使用
} AK8963_raw_Data; // 结束结构体定义，并声明结构体类型名 AK8963_raw_Data

typedef struct // 开始定义结构体类型，用于集中保存相关数据字段
{ // 进入当前代码块
    float mag_x_ut; // 声明 mag_x_ut 变量，供后续计算、状态保存或模块间传递使用
    float mag_y_ut; // 声明 mag_y_ut 变量，供后续计算、状态保存或模块间传递使用
    float mag_z_ut; // 声明 mag_z_ut 变量，供后续计算、状态保存或模块间传递使用
} AK8963_Physical_Data; // 结束结构体定义，并声明结构体类型名 AK8963_Physical_Data

typedef struct // 开始定义结构体类型，用于集中保存相关数据字段
{ // 进入当前代码块
    float roll; // 声明 roll 变量，供后续计算、状态保存或模块间传递使用
    float pitch; // 声明 pitch 变量，供后续计算、状态保存或模块间传递使用
    float yaw; // 声明 yaw 变量，供后续计算、状态保存或模块间传递使用
} EulerAngle_t; // 结束结构体定义，并声明结构体类型名 EulerAngle_t

typedef struct // 开始定义结构体类型，用于集中保存相关数据字段
{ // 进入当前代码块
    float q0; // 声明 四元数标量分量，供后续计算、状态保存或模块间传递使用
    float q1; // 声明 四元数 X 分量，供后续计算、状态保存或模块间传递使用
    float q2; // 声明 四元数 Y 分量，供后续计算、状态保存或模块间传递使用
    float q3; // 声明 四元数 Z 分量，供后续计算、状态保存或模块间传递使用
} Quaternion_t; // 结束结构体定义，并声明结构体类型名 Quaternion_t

//extern EulerAngle_t g_euler_acc_mag; // 声明 g_euler_acc_mag 全局状态变量，供后续计算、状态保存或模块间传递使用
extern EulerAngle_t g_euler_fused; // 声明 g_euler_fused 全局状态变量，供后续计算、状态保存或模块间传递使用
/**
 * @brief 初始化 MPU9250 和 AK8963 并完成基础校准。
 * @retval 函数执行结果或计算得到的返回值。
 */
uint8_t MPU9250_Driver_Init(void); // 声明MPU9250_Driver_Init 函数签名：初始化 MPU9250 和 AK8963 并完成基础校准
/**
 * @brief MPU9250_Driver_ReadWhoAmI 函数。
 * @param id 设备 ID 输出变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
uint8_t MPU9250_Driver_ReadWhoAmI(uint8_t *id); // 声明MPU9250_Driver_ReadWhoAmI 函数签名：MPU9250_Driver_ReadWhoAmI 函数
/**
 * @brief MPU9250_SoftReset 函数。
 * @retval None
 */
void MPU9250_SoftReset(void); // 声明MPU9250_SoftReset 函数签名：MPU9250_SoftReset 函数
/**
 * @brief MPU9250_Read_PowerMgmt 函数。
 * @retval None
 */
void MPU9250_Read_PowerMgmt(void); // 声明MPU9250_Read_PowerMgmt 函数签名：MPU9250_Read_PowerMgmt 函数
/**
 * @brief mpu_set_clock_to_auto 函数。
 * @retval None
 */
void mpu_set_clock_to_auto(void); // 声明mpu_set_clock_to_auto 函数签名：mpu_set_clock_to_auto 函数
/**
 * @brief mpu_enable_six_axis 函数。
 * @retval None
 */
void mpu_enable_six_axis(void); // 声明mpu_enable_six_axis 函数签名：mpu_enable_six_axis 函数
/**
 * @brief mpu_set_dlpf_cfg_3 函数。
 * @retval None
 */
void mpu_set_dlpf_cfg_3(void); // 声明mpu_set_dlpf_cfg_3 函数签名：mpu_set_dlpf_cfg_3 函数
/**
 * @brief mpu_set_sample_rate_200hz 函数。
 * @retval None
 */
void mpu_set_sample_rate_200hz(void); // 声明mpu_set_sample_rate_200hz 函数签名：mpu_set_sample_rate_200hz 函数
/**
 * @brief mpu_set_accel_dlpf 函数。
 * @retval None
 */
void mpu_set_accel_dlpf(void); // 声明mpu_set_accel_dlpf 函数签名：mpu_set_accel_dlpf 函数
/**
 * @brief mpu_set_gyro_config 函数。
 * @retval None
 */
void mpu_set_gyro_config(void); // 声明mpu_set_gyro_config 函数签名：mpu_set_gyro_config 函数
/**
 * @brief mpu_set_accel_range 函数。
 * @retval None
 */
void mpu_set_accel_range(void); // 声明mpu_set_accel_range 函数签名：mpu_set_accel_range 函数
/**
 * @brief 读取 MPU9250 加速度计、陀螺仪和温度原始寄存器。
 * @param raw 传感器原始采样数据结构体。
 * @retval None
 */
void MPU9250_ReadAxis(MPU9250_raw_Data *raw); // 声明MPU9250_ReadAxis 函数签名：读取 MPU9250 加速度计、陀螺仪和温度原始寄存器
/**
 * @brief 将 MPU9250 原始值转换成 g、dps 和摄氏度物理量。
 * @param raw 传感器原始采样数据结构体。
 * @param physical MPU9250 六轴物理量输出结构体。
 * @retval None
 */
void MPU9250_ConvertToPhysical(const MPU9250_raw_Data *raw, MPU9250_Physical_Data *physical); // 声明MPU9250_ConvertToPhysical 函数签名：将 MPU9250 原始值转换成 g、dps 和摄氏度物理量
/**
 * @brief mpu_set_ak8963_by_mcu 函数。
 * @retval None
 */
void mpu_set_ak8963_by_mcu(void); // 声明mpu_set_ak8963_by_mcu 函数签名：mpu_set_ak8963_by_mcu 函数
/**
 * @brief AK8963_CheckDeviceID 函数。
 * @retval 函数执行结果或计算得到的返回值。
 */
int AK8963_CheckDeviceID(void); // 声明AK8963_CheckDeviceID 函数签名：AK8963_CheckDeviceID 函数
/**
 * @brief AK8963_EnterPowerDownMode 函数。
 * @retval None
 */
void AK8963_EnterPowerDownMode(void); // 声明AK8963_EnterPowerDownMode 函数签名：AK8963_EnterPowerDownMode 函数
/**
 * @brief AK8963_EnterFuseROMMode 函数。
 * @retval None
 */
void AK8963_EnterFuseROMMode(void); // 声明AK8963_EnterFuseROMMode 函数签名：AK8963_EnterFuseROMMode 函数
/**
 * @brief AK8963_AdjustSensitivity 函数。
 * @retval None
 */
void AK8963_AdjustSensitivity(void); // 声明AK8963_AdjustSensitivity 函数签名：AK8963_AdjustSensitivity 函数
/**
 * @brief AK8963_EnterContinuousMeasurementMode 函数。
 * @retval None
 */
void AK8963_EnterContinuousMeasurementMode(void); // 声明AK8963_EnterContinuousMeasurementMode 函数签名：AK8963_EnterContinuousMeasurementMode 函数
/**
 * @brief AK8963_CheckDataReady 函数。
 * @retval 函数执行结果或计算得到的返回值。
 */
int AK8963_CheckDataReady(void); // 声明AK8963_CheckDataReady 函数签名：AK8963_CheckDataReady 函数
/**
 * @brief 读取 AK8963 三轴磁力计原始值。
 * @param raw 传感器原始采样数据结构体。
 * @retval 函数执行结果或计算得到的返回值。
 */
int AK8963_Read_Axis(AK8963_raw_Data *raw); // 声明AK8963_Read_Axis 函数签名：读取 AK8963 三轴磁力计原始值
/**
 * @brief 将 AK8963 原始磁场值转换并应用校准。
 * @param raw 传感器原始采样数据结构体。
 * @param physical MPU9250 六轴物理量输出结构体。
 * @retval None
 */
void AK8963_Calibrate(const AK8963_raw_Data *raw, AK8963_Physical_Data *physical); // 声明AK8963_Calibrate 函数签名：将 AK8963 原始磁场值转换并应用校准
/**
 * @brief 读取并校准 AK8963 磁力计物理量。
 * @param mag_out mag_out 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
int AK8963_Read_Mag_UT(AK8963_Physical_Data *mag_out); // 声明AK8963_Read_Mag_UT 函数签名：读取并校准 AK8963 磁力计物理量
/**
 * @brief MPU9250_CalibrateGyro 函数。
 * @param samples 校准采样次数。
 * @param delay_ms 采样间隔毫秒数。
 * @retval None
 */
void MPU9250_CalibrateGyro(uint16_t samples, uint16_t delay_ms); // 声明MPU9250_CalibrateGyro 函数签名：MPU9250_CalibrateGyro 函数
/**
 * @brief MPU9250_CalibrateAccel 函数。
 * @param samples 校准采样次数。
 * @param delay_ms 采样间隔毫秒数。
 * @retval None
 */
void MPU9250_CalibrateAccel(uint16_t samples, uint16_t delay_ms); // 声明MPU9250_CalibrateAccel 函数签名：MPU9250_CalibrateAccel 函数
/**
 * @brief 采集磁力计样本并计算硬铁和软铁校准参数。
 * @param samples 校准采样次数。
 * @param delay_ms 采样间隔毫秒数。
 * @retval None
 */
void AK8963_CalibrateMag(uint16_t samples, uint16_t delay_ms); // 声明AK8963_CalibrateMag 函数签名：采集磁力计样本并计算硬铁和软铁校准参数
/**
 * @brief 使用加速度计和磁力计直接计算欧拉角。
 * @param imu MPU9250 六轴物理量数据。
 * @param mag AK8963 磁力计物理量数据。
 * @retval None
 */
//void MPU9250_ComputeEuler_FromAccMag(const MPU9250_Physical_Data *imu, // 声明MPU9250_ComputeEuler_FromAccMag 函数签名：使用加速度计和磁力计直接计算欧拉角
//                                     const AK8963_Physical_Data *mag); // 声明MPU9250_ComputeEuler_FromAccMag 函数签名：使用加速度计和磁力计直接计算欧拉角
/**
 * @brief 读取加速度计磁力计解算的欧拉角角度值。
 * @param roll_deg 横滚角角度值。
 * @param pitch_deg 俯仰角角度值。
 * @param yaw_deg 航向角角度值。
 * @retval None
 */
//void MPU9250_GetEulerDeg(float *roll_deg, float *pitch_deg, float *yaw_deg); // 声明MPU9250_GetEulerDeg 函数签名：读取加速度计磁力计解算的欧拉角角度值
/**
 * @brief 初始化 Mahony 姿态融合四元数和误差积分项。
 * @param kp Mahony 比例修正增益。
 * @param ki Mahony 积分修正增益。
 * @retval None
 */
void MPU9250_MahonyInit(float kp, float ki); // 声明MPU9250_MahonyInit 函数签名：初始化 Mahony 姿态融合四元数和误差积分项
/**
 * @brief 使用九轴数据执行 Mahony 姿态融合更新。
 * @param imu MPU9250 六轴物理量数据。
 * @param mag AK8963 磁力计物理量数据。
 * @param dt 姿态融合采样周期秒数。
 * @retval None
 */
void MPU9250_MahonyUpdate(const MPU9250_Physical_Data *imu, // 声明MPU9250_MahonyUpdate 函数签名：使用九轴数据执行 Mahony 姿态融合更新
                          const AK8963_Physical_Data *mag, // 声明MPU9250_MahonyUpdate 函数签名：使用九轴数据执行 Mahony 姿态融合更新
                          float dt); // 声明MPU9250_MahonyUpdate 函数签名：使用九轴数据执行 Mahony 姿态融合更新
/**
 * @brief 使用六轴 IMU 数据执行 Mahony 姿态融合更新。
 * @param imu MPU9250 六轴物理量数据。
 * @param dt 姿态融合采样周期秒数。
 * @retval None
 */
void MPU9250_MahonyUpdateIMU(const MPU9250_Physical_Data *imu, float dt); // 声明MPU9250_MahonyUpdateIMU 函数签名：使用六轴 IMU 数据执行 Mahony 姿态融合更新
/**
 * @brief 读取 Mahony 融合后的欧拉角角度值。
 * @param roll_deg 横滚角角度值。
 * @param pitch_deg 俯仰角角度值。
 * @param yaw_deg 航向角角度值。
 * @retval None
 */
void MPU9250_GetEulerFusedDeg(float *roll_deg, float *pitch_deg, float *yaw_deg); // 声明MPU9250_GetEulerFusedDeg 函数签名：读取 Mahony 融合后的欧拉角角度值
#endif // 结束当前条件编译或头文件保护范围
