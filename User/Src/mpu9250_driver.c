/**
 * @file    mpu9250_driver.c
 * @brief   MPU9250 六轴与 AK8963 磁力计驱动实现。
 */
#include "mpu9250_driver.h"                                                               // 包含所需头文件

#define MPU9250_PWR_MGMT_1_REG 0x6BU                                                      // 定义本文件使用的宏
#define MPU9250_PWR_MGMT_2_REG 0x6CU                                                      // 定义本文件使用的宏
#define MPU9250_USER_CTRL_REG 0x6AU                                                       // 定义本文件使用的宏
#define MPU9250_INT_PIN_CFG_REG 0x37U                                                     // 定义本文件使用的宏
#define MPU9250_CONFIG_REG 0x1AU                                                          // 定义本文件使用的宏
#define MPU9250_SMPLRT_DIV_REG 0x19U                                                      // 定义本文件使用的宏
#define MPU9250_ACCEL_CONFIG_REG 0x1CU                                                    // 定义本文件使用的宏
#define MPU9250_ACCEL_CONFIG2_REG 0x1DU                                                   // 定义本文件使用的宏
#define MPU9250_GYRO_CONFIG_REG 0x1BU                                                     // 定义本文件使用的宏
#define MPU9250_ACCEL_XOUT_H_REG 0x3BU                                                    // 定义本文件使用的宏
#define MPU9250_ACCEL_YOUT_H_REG 0x3DU                                                    // 定义本文件使用的宏
#define MPU9250_ACCEL_ZOUT_H_REG 0x3FU                                                    // 定义本文件使用的宏
#define MPU9250_TEMP_OUT_H_REG 0x41U                                                      // 定义本文件使用的宏
#define MPU9250_GYRO_XOUT_H_REG 0x43U                                                     // 定义本文件使用的宏
#define MPU9250_GYRO_YOUT_H_REG 0x45U                                                     // 定义本文件使用的宏
#define MPU9250_GYRO_ZOUT_H_REG 0x47U                                                     // 定义本文件使用的宏

#define AK8963_REG_WIA 0x00U                                                              // 定义本文件使用的宏
#define AK8963_REG_ST1 0x02U                                                              // 定义本文件使用的宏
#define AK8963_REG_HXL 0x03U                                                              // 定义本文件使用的宏
#define AK8963_REG_HYL 0x05U                                                              // 定义本文件使用的宏
#define AK8963_REG_HZL 0x07U                                                              // 定义本文件使用的宏
#define AK8963_REG_ST2 0x09U                                                              // 定义本文件使用的宏
#define AK8963_REG_CNTL1 0x0AU                                                            // 定义本文件使用的宏
#define AK8963_REG_ASAX 0x10U                                                             // 定义本文件使用的宏
#define AK8963_REG_ASAY 0x11U                                                             // 定义本文件使用的宏
#define AK8963_REG_ASAZ 0x12U                                                             // 定义本文件使用的宏
#define AK8963_WHO_AM_I_VALUE 0x48U                                                       // 定义本文件使用的宏

#define MPU9250_ACCEL_LSB_PER_G 4096.0f                                                   // 定义本文件使用的宏
#define MPU9250_GYRO_LSB_PER_DPS 32.8f                                                    // 定义本文件使用的宏
#define AK8963_16BIT_UT_PER_LSB 0.15f                                                     // 定义本文件使用的宏
#define AK8963_MAG_RADIUS_MIN_UT 0.001f                                                   // 定义本文件使用的宏

static uint8_t g_ak8963_asa[3] = {0U, 0U, 0U};                                            // 定义本文件内部静态变量
static float g_ak8963_sensitivity[3] = {1.0f, 1.0f, 1.0f};                                // 定义本文件内部静态变量

static float g_gyro_bias_dps[3] = {0.0f, 0.0f, 0.0f};                                     // 定义本文件内部静态变量
static float g_accel_bias_g[3] = {0.0f, 0.0f, 0.0f};                                      // 定义本文件内部静态变量
static float g_mag_offset_ut[3] = {0.0f, 0.0f, 0.0f};                                     // 定义本文件内部静态变量
static float g_mag_scale[3] = {1.0f, 1.0f, 1.0f};                                         // 定义本文件内部静态变量

/**
 * @brief  读取 MPU9250 的 16 位大端有符号寄存器值。
 * @param  reg 高字节寄存器地址。
 * @return 组合后的 16 位有符号值。
 */
static int16_t mpu9250_read_word(uint8_t reg)                                             // 说明当前代码行
{                                                                                         // 进入代码块
    uint8_t high_byte = I2C_ReadReg(MPU9250_I2C_ADDR7, reg);                              // 定义局部变量
    uint8_t low_byte = I2C_ReadReg(MPU9250_I2C_ADDR7, (uint8_t)(reg + 1U));               // 定义局部变量

    return (int16_t)(((uint16_t)high_byte << 8U) | (uint16_t)low_byte);                   // 返回函数执行结果
}                                                                                         // 结束代码块

/**
 * @brief  读取 AK8963 的 16 位小端有符号寄存器值。
 * @param  low_reg 低字节寄存器地址。
 * @return 组合后的 16 位有符号值。
 */
static int16_t ak8963_read_word(uint8_t low_reg)                                          // 说明当前代码行
{                                                                                         // 进入代码块
    uint8_t low_byte = I2C_ReadReg(AK8963_I2C_ADDR7, low_reg);                            // 定义局部变量
    uint8_t high_byte = I2C_ReadReg(AK8963_I2C_ADDR7, (uint8_t)(low_reg + 1U));           // 定义局部变量

    return (int16_t)(((uint16_t)high_byte << 8U) | (uint16_t)low_byte);                   // 返回函数执行结果
}                                                                                         // 结束代码块

/**
 * @brief  将 AK8963 原始值转换为只经过 ASA 灵敏度补偿的 uT 数据。
 * @param  raw 磁力计原始数据。
 * @param  mag_out 输出的基础磁场物理量。
 * @retval None
 */
static void ak8963_convert_raw_to_ut(const AK8963_raw_Data *raw, AK8963_Physical_Data *mag_out) // 说明当前代码行
{                                                                                         // 进入代码块
    if ((raw == 0) || (mag_out == 0))                                                     // 判断条件是否成立
    {                                                                                     // 进入代码块
        return;                                                                           // 返回函数执行结果
    }                                                                                     // 结束代码块

    mag_out->mag_x_ut = (float)raw->mag_x * g_ak8963_sensitivity[0] * AK8963_16BIT_UT_PER_LSB; // 给变量或寄存器写入新值
    mag_out->mag_y_ut = (float)raw->mag_y * g_ak8963_sensitivity[1] * AK8963_16BIT_UT_PER_LSB; // 给变量或寄存器写入新值
    mag_out->mag_z_ut = (float)raw->mag_z * g_ak8963_sensitivity[2] * AK8963_16BIT_UT_PER_LSB; // 给变量或寄存器写入新值
}                                                                                         // 结束代码块

/**
 * @brief  应用磁力计硬铁偏移和软铁缩放校准。
 * @param  mag 待校准的磁场物理量，函数会原地更新。
 * @retval None
 */
static void ak8963_apply_mag_calibration(AK8963_Physical_Data *mag)                       // 说明当前代码行
{                                                                                         // 进入代码块
    if (mag == 0)                                                                         // 判断条件是否成立
    {                                                                                     // 进入代码块
        return;                                                                           // 返回函数执行结果
    }                                                                                     // 结束代码块

    mag->mag_x_ut = (mag->mag_x_ut - g_mag_offset_ut[0]) * g_mag_scale[0];                // 给变量或寄存器写入新值
    mag->mag_y_ut = (mag->mag_y_ut - g_mag_offset_ut[1]) * g_mag_scale[1];                // 给变量或寄存器写入新值
    mag->mag_z_ut = (mag->mag_z_ut - g_mag_offset_ut[2]) * g_mag_scale[2];                // 给变量或寄存器写入新值
}                                                                                         // 结束代码块

/**
 * @brief  读取并校验 MPU9250 设备 ID。
 * @param  None
 * @return 0 表示 ID 正确，负数表示读取失败或 ID 错误。
 */
static int mpu9250_check_device(void)                                                     // 说明当前代码行
{                                                                                         // 进入代码块
    uint8_t id = 0U;                                                                      // 定义局部变量

    if (MPU9250_Driver_ReadWhoAmI(&id) == 0U)                                             // 判断条件是否成立
    {                                                                                     // 进入代码块
        Debug_Print("[MPU9250] WHO_AM_I read failed\r\n");                                // 输出调试日志
        return -1;                                                                        // 返回函数执行结果
    }                                                                                     // 结束代码块

    Debug_Printf("[MPU9250] WHO_AM_I = 0x%02X\r\n", id);                                  // 输出调试日志

    if (id != MPU9250_WHO_AM_I_VALUE)                                                     // 判断条件是否成立
    {                                                                                     // 进入代码块
        Debug_Print("[MPU9250] WHO_AM_I value error\r\n");                                // 输出调试日志
        return -2;                                                                        // 返回函数执行结果
    }                                                                                     // 结束代码块

    Debug_Print("[MPU9250] WHO_AM_I check ok\r\n");                                       // 输出调试日志
    return 0;                                                                             // 返回函数执行结果
}                                                                                         // 结束代码块

/**
 * @brief  配置 MPU9250 六轴传感器的基础寄存器。
 * @param  None
 * @retval None
 */
static void mpu9250_config_six_axis(void)                                                 // 说明当前代码行
{                                                                                         // 进入代码块
    MPU9250_SoftReset();                                                                  // 调用 MPU9250 驱动接口
    mpu_set_clock_to_auto();                                                              // 调用函数执行对应操作
    mpu_enable_six_axis();                                                                // 调用函数执行对应操作
    mpu_set_dlpf_cfg_3();                                                                 // 调用函数执行对应操作
    mpu_set_sample_rate_200hz();                                                          // 调用函数执行对应操作
    mpu_set_accel_dlpf();                                                                 // 调用函数执行对应操作
    mpu_set_gyro_config();                                                                // 调用函数执行对应操作
    mpu_set_accel_range();                                                                // 调用函数执行对应操作
    MPU9250_Read_PowerMgmt();                                                             // 调用 MPU9250 驱动接口
}                                                                                         // 结束代码块

/**
 * @brief  初始化 AK8963 磁力计并进入连续测量模式。
 * @param  None
 * @return 1 表示初始化成功，0 表示初始化失败。
 */
static uint8_t ak8963_init(void)                                                          // 说明当前代码行
{                                                                                         // 进入代码块
    mpu_set_ak8963_by_mcu();                                                              // 调用函数执行对应操作

    if (AK8963_CheckDeviceID() != 0)                                                      // 判断条件是否成立
    {                                                                                     // 进入代码块
        Debug_Print("[AK8963] device ID check failed\r\n");                               // 输出调试日志
        return 0U;                                                                        // 返回函数执行结果
    }                                                                                     // 结束代码块

    Debug_Print("[AK8963] device ID check ok\r\n");                                       // 输出调试日志

    AK8963_EnterPowerDownMode();                                                          // 调用 AK8963 磁力计接口
    AK8963_EnterFuseROMMode();                                                            // 调用 AK8963 磁力计接口
    AK8963_AdjustSensitivity();                                                           // 调用 AK8963 磁力计接口
    AK8963_EnterPowerDownMode();                                                          // 调用 AK8963 磁力计接口
    AK8963_EnterContinuousMeasurementMode();                                              // 调用 AK8963 磁力计接口

    if (AK8963_CheckDataReady() == 0)                                                     // 判断条件是否成立
    {                                                                                     // 进入代码块
        Debug_Print("[AK8963] data not ready\r\n");                                       // 输出调试日志
        return 0U;                                                                        // 返回函数执行结果
    }                                                                                     // 结束代码块

    Debug_Print("[AK8963] data ready\r\n");                                               // 输出调试日志
    return 1U;                                                                            // 返回函数执行结果
}                                                                                         // 结束代码块

/**
 * @brief  读取 MPU9250 WHO_AM_I 寄存器。
 * @param  id 保存读取结果的指针。
 * @return 1 表示读取成功，0 表示读取失败。
 */
uint8_t MPU9250_Driver_ReadWhoAmI(uint8_t *id)                                            // 定义局部变量
{                                                                                         // 进入代码块
    int ret;                                                                              // 定义局部变量

    if (id == 0)                                                                          // 判断条件是否成立
    {                                                                                     // 进入代码块
        return 0U;                                                                        // 返回函数执行结果
    }                                                                                     // 结束代码块

    ret = I2C_ReadRegData(MPU9250_I2C_ADDR7, MPU9250_REG_WHO_AM_I, id);                   // 给变量或寄存器写入新值
    return (ret == 0) ? 1U : 0U;                                                          // 返回函数执行结果
}                                                                                         // 结束代码块

/**
 * @brief  软复位 MPU9250。
 * @param  None
 * @retval None
 */
void MPU9250_SoftReset(void)                                                              // 说明当前代码行
{                                                                                         // 进入代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG);                 // 定义局部变量

    val &= (uint8_t)~(1U << 7U);                                                          // 给变量或寄存器写入新值
    val |= (uint8_t)(1U << 7U);                                                           // 给变量或寄存器写入新值
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG, val);                         // 执行软件 I2C 操作

    vTaskDelay(pdMS_TO_TICKS(100));                                                       // 让当前任务延时等待
}                                                                                         // 结束代码块

/**
 * @brief  读取并打印 MPU9250 电源管理寄存器。
 * @param  None
 * @retval None
 */
void MPU9250_Read_PowerMgmt(void)                                                         // 说明当前代码行
{                                                                                         // 进入代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG);                 // 定义局部变量

    Debug_Printf("[MPU9250] PWR_MGMT_1 = 0x%02X\r\n", val);                               // 输出调试日志
}                                                                                         // 结束代码块

/**
 * @brief  唤醒 MPU9250 并设置 PLL 时钟源。
 * @param  None
 * @retval None
 */
void mpu_set_clock_to_auto(void)                                                          // 说明当前代码行
{                                                                                         // 进入代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG);                 // 定义局部变量

    val &= (uint8_t)~((1U << 6U) | 0x07U);                                                // 给变量或寄存器写入新值
    val |= 0x01U;                                                                         // 给变量或寄存器写入新值
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG, val);                         // 执行软件 I2C 操作
}                                                                                         // 结束代码块

/**
 * @brief  启用 MPU9250 加速度计和陀螺仪六轴。
 * @param  None
 * @retval None
 */
void mpu_enable_six_axis(void)                                                            // 说明当前代码行
{                                                                                         // 进入代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_2_REG);                 // 定义局部变量

    val &= (uint8_t)~0x3FU;                                                               // 给变量或寄存器写入新值
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_2_REG, val);                         // 执行软件 I2C 操作
}                                                                                         // 结束代码块

/**
 * @brief  配置陀螺仪低通滤波为 DLPF_CFG=3。
 * @param  None
 * @retval None
 */
void mpu_set_dlpf_cfg_3(void)                                                             // 说明当前代码行
{                                                                                         // 进入代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_CONFIG_REG);                     // 定义局部变量

    val = (uint8_t)((val & (uint8_t)~0x07U) | 0x03U);                                     // 给变量或寄存器写入新值
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_CONFIG_REG, val);                             // 执行软件 I2C 操作
}                                                                                         // 结束代码块

/**
 * @brief  配置 MPU9250 采样率为 200Hz。
 * @param  None
 * @retval None
 */
void mpu_set_sample_rate_200hz(void)                                                      // 说明当前代码行
{                                                                                         // 进入代码块
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_SMPLRT_DIV_REG, 0x04U);                       // 执行软件 I2C 操作
}                                                                                         // 结束代码块

/**
 * @brief  配置加速度计低通滤波。
 * @param  None
 * @retval None
 */
void mpu_set_accel_dlpf(void)                                                             // 说明当前代码行
{                                                                                         // 进入代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG2_REG);              // 定义局部变量

    val &= (uint8_t)~0x07U;                                                               // 给变量或寄存器写入新值
    val |= 0x03U;                                                                         // 给变量或寄存器写入新值
    val &= (uint8_t)~(1U << 3U);                                                          // 给变量或寄存器写入新值
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG2_REG, val);                      // 执行软件 I2C 操作
}                                                                                         // 结束代码块

/**
 * @brief  配置陀螺仪量程为 +/-1000 dps。
 * @param  None
 * @retval None
 */
void mpu_set_gyro_config(void)                                                            // 说明当前代码行
{                                                                                         // 进入代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_GYRO_CONFIG_REG);                // 定义局部变量

    val &= (uint8_t)~0x03U;                                                               // 给变量或寄存器写入新值
    val &= (uint8_t)~(3U << 3U);                                                          // 给变量或寄存器写入新值
    val |= (uint8_t)(2U << 3U);                                                           // 给变量或寄存器写入新值
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_GYRO_CONFIG_REG, val);                        // 执行软件 I2C 操作
}                                                                                         // 结束代码块

/**
 * @brief  配置加速度计量程为 +/-8g。
 * @param  None
 * @retval None
 */
void mpu_set_accel_range(void)                                                            // 说明当前代码行
{                                                                                         // 进入代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG_REG);               // 定义局部变量

    val &= (uint8_t)~(3U << 3U);                                                          // 给变量或寄存器写入新值
    val |= (uint8_t)(2U << 3U);                                                           // 给变量或寄存器写入新值
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG_REG, val);                       // 执行软件 I2C 操作
}                                                                                         // 结束代码块

/**
 * @brief  读取 MPU9250 六轴原始数据。
 * @param  raw 保存六轴原始数据的指针。
 * @retval None
 */
void MPU9250_ReadAxis(MPU9250_raw_Data *raw)                                              // 说明当前代码行
{                                                                                         // 进入代码块
    if (raw == 0)                                                                         // 判断条件是否成立
    {                                                                                     // 进入代码块
        return;                                                                           // 返回函数执行结果
    }                                                                                     // 结束代码块

    raw->accel_x = mpu9250_read_word(MPU9250_ACCEL_XOUT_H_REG);                           // 给变量或寄存器写入新值
    raw->accel_y = mpu9250_read_word(MPU9250_ACCEL_YOUT_H_REG);                           // 给变量或寄存器写入新值
    raw->accel_z = mpu9250_read_word(MPU9250_ACCEL_ZOUT_H_REG);                           // 给变量或寄存器写入新值
    raw->gyro_x = mpu9250_read_word(MPU9250_GYRO_XOUT_H_REG);                             // 给变量或寄存器写入新值
    raw->gyro_y = mpu9250_read_word(MPU9250_GYRO_YOUT_H_REG);                             // 给变量或寄存器写入新值
    raw->gyro_z = mpu9250_read_word(MPU9250_GYRO_ZOUT_H_REG);                             // 给变量或寄存器写入新值
    raw->temp = mpu9250_read_word(MPU9250_TEMP_OUT_H_REG);                                // 给变量或寄存器写入新值
}                                                                                         // 结束代码块

/**
 * @brief  将 MPU9250 原始六轴数据转换为物理量并应用零偏。
 * @param  raw 原始六轴数据。
 * @param  physical 输出的六轴物理量。
 * @retval None
 */
void MPU9250_ConvertToPhysical(const MPU9250_raw_Data *raw, MPU9250_Physical_Data *physical) // 说明当前代码行
{                                                                                         // 进入代码块
    if ((raw == 0) || (physical == 0))                                                    // 判断条件是否成立
    {                                                                                     // 进入代码块
        return;                                                                           // 返回函数执行结果
    }                                                                                     // 结束代码块

    physical->accel_x_g = (float)raw->accel_x / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[0]; // 给变量或寄存器写入新值
    physical->accel_y_g = (float)raw->accel_y / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[1]; // 给变量或寄存器写入新值
    physical->accel_z_g = (float)raw->accel_z / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[2]; // 给变量或寄存器写入新值
    physical->gyro_x_dps = (float)raw->gyro_x / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[0]; // 给变量或寄存器写入新值
    physical->gyro_y_dps = (float)raw->gyro_y / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[1]; // 给变量或寄存器写入新值
    physical->gyro_z_dps = (float)raw->gyro_z / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[2]; // 给变量或寄存器写入新值
    physical->temp_c = ((float)raw->temp / 333.87f) + 21.0f;                              // 给变量或寄存器写入新值
}                                                                                         // 结束代码块

/**
 * @brief  配置 MPU9250 BYPASS，让 MCU 直接访问 AK8963。
 * @param  None
 * @retval None
 */
void mpu_set_ak8963_by_mcu(void)                                                          // 说明当前代码行
{                                                                                         // 进入代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_USER_CTRL_REG);                  // 定义局部变量

    val &= (uint8_t)~(1U << 5U);                                                          // 给变量或寄存器写入新值
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_USER_CTRL_REG, val);                          // 执行软件 I2C 操作

    val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_INT_PIN_CFG_REG);                        // 给变量或寄存器写入新值
    val |= (uint8_t)(1U << 1U);                                                           // 给变量或寄存器写入新值
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_INT_PIN_CFG_REG, val);                        // 执行软件 I2C 操作
}                                                                                         // 结束代码块

/**
 * @brief  检查 AK8963 设备 ID。
 * @param  None
 * @return 0 表示 ID 正确，-1 表示 ID 错误。
 */
int AK8963_CheckDeviceID(void)                                                            // 定义局部变量
{                                                                                         // 进入代码块
    uint8_t device_id = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_WIA);                    // 定义局部变量

    return (device_id == AK8963_WHO_AM_I_VALUE) ? 0 : -1;                                 // 返回函数执行结果
}                                                                                         // 结束代码块

/**
 * @brief  使 AK8963 进入 Power-down 模式。
 * @param  None
 * @retval None
 */
void AK8963_EnterPowerDownMode(void)                                                      // 说明当前代码行
{                                                                                         // 进入代码块
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x00U);                              // 执行软件 I2C 操作
    vTaskDelay(pdMS_TO_TICKS(10));                                                        // 让当前任务延时等待
}                                                                                         // 结束代码块

/**
 * @brief  使 AK8963 进入 Fuse ROM 模式。
 * @param  None
 * @retval None
 */
void AK8963_EnterFuseROMMode(void)                                                        // 说明当前代码行
{                                                                                         // 进入代码块
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x0FU);                              // 执行软件 I2C 操作
    vTaskDelay(pdMS_TO_TICKS(10));                                                        // 让当前任务延时等待
}                                                                                         // 结束代码块

/**
 * @brief  读取 AK8963 Fuse ROM 中的 ASA 灵敏度补偿系数。
 * @param  None
 * @retval None
 */
void AK8963_AdjustSensitivity(void)                                                       // 说明当前代码行
{                                                                                         // 进入代码块
    g_ak8963_asa[0] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAX);                     // 给变量或寄存器写入新值
    g_ak8963_asa[1] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAY);                     // 给变量或寄存器写入新值
    g_ak8963_asa[2] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAZ);                     // 给变量或寄存器写入新值

    g_ak8963_sensitivity[0] = (((float)g_ak8963_asa[0] - 128.0f) * 0.5f / 128.0f) + 1.0f; // 给变量或寄存器写入新值
    g_ak8963_sensitivity[1] = (((float)g_ak8963_asa[1] - 128.0f) * 0.5f / 128.0f) + 1.0f; // 给变量或寄存器写入新值
    g_ak8963_sensitivity[2] = (((float)g_ak8963_asa[2] - 128.0f) * 0.5f / 128.0f) + 1.0f; // 给变量或寄存器写入新值
}                                                                                         // 结束代码块

/**
 * @brief  使 AK8963 进入 16-bit 100Hz 连续测量模式。
 * @param  None
 * @retval None
 */
void AK8963_EnterContinuousMeasurementMode(void)                                          // 说明当前代码行
{                                                                                         // 进入代码块
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x16U);                              // 执行软件 I2C 操作
    vTaskDelay(pdMS_TO_TICKS(10));                                                        // 让当前任务延时等待
}                                                                                         // 结束代码块

/**
 * @brief  检查 AK8963 数据准备标志。
 * @param  None
 * @return 1 表示数据已准备好，0 表示未准备好。
 */
int AK8963_CheckDataReady(void)                                                           // 定义局部变量
{                                                                                         // 进入代码块
    uint8_t status = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ST1);                       // 定义局部变量

    return ((status & 0x01U) != 0U) ? 1 : 0;                                              // 返回函数执行结果
}                                                                                         // 结束代码块

/**
 * @brief  读取 AK8963 三轴原始磁力计数据。
 * @param  raw 保存磁力计原始数据的指针。
 * @return 0 表示成功，-1 表示未就绪，-2 表示读取失败，-3 表示磁场溢出。
 */
int AK8963_Read_Axis(AK8963_raw_Data *raw)                                                // 定义局部变量
{                                                                                         // 进入代码块
    uint8_t st2;                                                                          // 定义局部变量

    if (raw == 0)                                                                         // 判断条件是否成立
    {                                                                                     // 进入代码块
        return -2;                                                                        // 返回函数执行结果
    }                                                                                     // 结束代码块

    if (AK8963_CheckDataReady() == 0)                                                     // 判断条件是否成立
    {                                                                                     // 进入代码块
        return -1;                                                                        // 返回函数执行结果
    }                                                                                     // 结束代码块

    raw->mag_x = ak8963_read_word(AK8963_REG_HXL);                                        // 给变量或寄存器写入新值
    raw->mag_y = ak8963_read_word(AK8963_REG_HYL);                                        // 给变量或寄存器写入新值
    raw->mag_z = ak8963_read_word(AK8963_REG_HZL);                                        // 给变量或寄存器写入新值

    st2 = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ST2);                                  // 给变量或寄存器写入新值
    if (st2 == 0xFFU)                                                                     // 判断条件是否成立
    {                                                                                     // 进入代码块
        return -2;                                                                        // 返回函数执行结果
    }                                                                                     // 结束代码块

    if ((st2 & 0x08U) != 0U)                                                              // 判断条件是否成立
    {                                                                                     // 进入代码块
        return -3;                                                                        // 返回函数执行结果
    }                                                                                     // 结束代码块

    return 0;                                                                             // 返回函数执行结果
}                                                                                         // 结束代码块

/**
 * @brief  将 AK8963 原始数据转换为最终校准后的 uT 数据。
 * @param  raw 原始磁力计数据。
 * @param  physical 输出的校准后磁场物理量。
 * @retval None
 */
void AK8963_Calibrate(const AK8963_raw_Data *raw, AK8963_Physical_Data *physical)         // 说明当前代码行
{                                                                                         // 进入代码块
    ak8963_convert_raw_to_ut(raw, physical);                                              // 调用函数执行对应操作
    ak8963_apply_mag_calibration(physical);                                               // 调用函数执行对应操作
}                                                                                         // 结束代码块

/**
 * @brief  读取一帧 AK8963 磁力计数据并输出校准后的 uT 数据。
 * @param  mag_out 保存磁力计物理量的指针。
 * @return 0 表示成功，其他值沿用 AK8963_Read_Axis 的错误码。
 */
int AK8963_Read_Mag_UT(AK8963_Physical_Data *mag_out)                                     // 定义局部变量
{                                                                                         // 进入代码块
    AK8963_raw_Data raw;                                                                  // 定义局部变量
    int ret;                                                                              // 定义局部变量

    if (mag_out == 0)                                                                     // 判断条件是否成立
    {                                                                                     // 进入代码块
        return -2;                                                                        // 返回函数执行结果
    }                                                                                     // 结束代码块

    ret = AK8963_Read_Axis(&raw);                                                         // 给变量或寄存器写入新值
    if (ret != 0)                                                                         // 判断条件是否成立
    {                                                                                     // 进入代码块
        return ret;                                                                       // 返回函数执行结果
    }                                                                                     // 结束代码块

    AK8963_Calibrate(&raw, mag_out);                                                      // 调用 AK8963 磁力计接口
    return 0;                                                                             // 返回函数执行结果
}                                                                                         // 结束代码块

/**
 * @brief  校准陀螺仪零偏。
 * @param  samples 采样次数。
 * @param  delay_ms 每次采样之间的延时，单位 ms。
 * @retval None
 */
void MPU9250_CalibrateGyro(uint16_t samples, uint16_t delay_ms)                           // 说明当前代码行
{                                                                                         // 进入代码块
    MPU9250_raw_Data raw;                                                                 // 定义局部变量
    int32_t gyro_x_sum = 0;                                                               // 定义局部变量
    int32_t gyro_y_sum = 0;                                                               // 定义局部变量
    int32_t gyro_z_sum = 0;                                                               // 定义局部变量
    uint16_t i;                                                                           // 定义局部变量

    if (samples == 0U)                                                                    // 判断条件是否成立
    {                                                                                     // 进入代码块
        return;                                                                           // 返回函数执行结果
    }                                                                                     // 结束代码块

    for (i = 0U; i < samples; i++)                                                        // 开始循环采样或遍历
    {                                                                                     // 进入代码块
        MPU9250_ReadAxis(&raw);                                                           // 调用 MPU9250 驱动接口
        gyro_x_sum += raw.gyro_x;                                                         // 累加或更新当前变量
        gyro_y_sum += raw.gyro_y;                                                         // 累加或更新当前变量
        gyro_z_sum += raw.gyro_z;                                                         // 累加或更新当前变量
        vTaskDelay(pdMS_TO_TICKS(delay_ms));                                              // 让当前任务延时等待
    }                                                                                     // 结束代码块

    g_gyro_bias_dps[0] = ((float)gyro_x_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS; // 给变量或寄存器写入新值
    g_gyro_bias_dps[1] = ((float)gyro_y_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS; // 给变量或寄存器写入新值
    g_gyro_bias_dps[2] = ((float)gyro_z_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS; // 给变量或寄存器写入新值
}                                                                                         // 结束代码块

/**
 * @brief  校准加速度计静态零偏。
 * @param  samples 采样次数。
 * @param  delay_ms 每次采样之间的延时，单位 ms。
 * @note   调用时模块需要水平静止，Z 轴朝上。
 * @retval None
 */
void MPU9250_CalibrateAccel(uint16_t samples, uint16_t delay_ms)                          // 说明当前代码行
{                                                                                         // 进入代码块
    MPU9250_raw_Data raw;                                                                 // 定义局部变量
    float ax_sum = 0.0f;                                                                  // 定义局部变量
    float ay_sum = 0.0f;                                                                  // 定义局部变量
    float az_sum = 0.0f;                                                                  // 定义局部变量
    uint16_t i;                                                                           // 定义局部变量

    if (samples == 0U)                                                                    // 判断条件是否成立
    {                                                                                     // 进入代码块
        return;                                                                           // 返回函数执行结果
    }                                                                                     // 结束代码块

    for (i = 0U; i < samples; i++)                                                        // 开始循环采样或遍历
    {                                                                                     // 进入代码块
        MPU9250_ReadAxis(&raw);                                                           // 调用 MPU9250 驱动接口
        ax_sum += (float)raw.accel_x / MPU9250_ACCEL_LSB_PER_G;                           // 累加或更新当前变量
        ay_sum += (float)raw.accel_y / MPU9250_ACCEL_LSB_PER_G;                           // 累加或更新当前变量
        az_sum += (float)raw.accel_z / MPU9250_ACCEL_LSB_PER_G;                           // 累加或更新当前变量
        vTaskDelay(pdMS_TO_TICKS(delay_ms));                                              // 让当前任务延时等待
    }                                                                                     // 结束代码块

    g_accel_bias_g[0] = (ax_sum / (float)samples);                                        // 给变量或寄存器写入新值
    g_accel_bias_g[1] = (ay_sum / (float)samples);                                        // 给变量或寄存器写入新值
    g_accel_bias_g[2] = (az_sum / (float)samples) - 1.0f;                                 // 给变量或寄存器写入新值
}                                                                                         // 结束代码块

/**
 * @brief  校准 AK8963 磁力计硬铁偏移和软铁缩放。
 * @param  samples 采样次数。
 * @param  delay_ms 每次采样之间的延时，单位 ms。
 * @note   调用期间需要持续旋转模块，尽量覆盖完整三维方向。
 * @retval None
 */
void AK8963_CalibrateMag(uint16_t samples, uint16_t delay_ms)                             // 说明当前代码行
{                                                                                         // 进入代码块
    AK8963_raw_Data raw;                                                                  // 定义局部变量
    AK8963_Physical_Data mag;                                                             // 定义局部变量
    float min_x = 99999.0f;                                                               // 定义局部变量
    float min_y = 99999.0f;                                                               // 定义局部变量
    float min_z = 99999.0f;                                                               // 定义局部变量
    float max_x = -99999.0f;                                                              // 定义局部变量
    float max_y = -99999.0f;                                                              // 定义局部变量
    float max_z = -99999.0f;                                                              // 定义局部变量
    float radius_x;                                                                       // 定义局部变量
    float radius_y;                                                                       // 定义局部变量
    float radius_z;                                                                       // 定义局部变量
    float radius_avg;                                                                     // 定义局部变量
    uint16_t valid_count = 0U;                                                            // 定义局部变量
    uint16_t i;                                                                           // 定义局部变量

    if (samples == 0U)                                                                    // 判断条件是否成立
    {                                                                                     // 进入代码块
        return;                                                                           // 返回函数执行结果
    }                                                                                     // 结束代码块

    g_mag_offset_ut[0] = 0.0f;                                                            // 给变量或寄存器写入新值
    g_mag_offset_ut[1] = 0.0f;                                                            // 给变量或寄存器写入新值
    g_mag_offset_ut[2] = 0.0f;                                                            // 给变量或寄存器写入新值
    g_mag_scale[0] = 1.0f;                                                                // 给变量或寄存器写入新值
    g_mag_scale[1] = 1.0f;                                                                // 给变量或寄存器写入新值
    g_mag_scale[2] = 1.0f;                                                                // 给变量或寄存器写入新值

    for (i = 0U; i < samples; i++)                                                        // 开始循环采样或遍历
    {                                                                                     // 进入代码块
        if (AK8963_Read_Axis(&raw) == 0)                                                  // 判断条件是否成立
        {                                                                                 // 进入代码块
            ak8963_convert_raw_to_ut(&raw, &mag);                                         // 调用函数执行对应操作

            if (mag.mag_x_ut < min_x)                                                     // 判断条件是否成立
            {                                                                             // 进入代码块
                min_x = mag.mag_x_ut;                                                     // 给变量或寄存器写入新值
            }                                                                             // 结束代码块
            if (mag.mag_x_ut > max_x)                                                     // 判断条件是否成立
            {                                                                             // 进入代码块
                max_x = mag.mag_x_ut;                                                     // 给变量或寄存器写入新值
            }                                                                             // 结束代码块
            if (mag.mag_y_ut < min_y)                                                     // 判断条件是否成立
            {                                                                             // 进入代码块
                min_y = mag.mag_y_ut;                                                     // 给变量或寄存器写入新值
            }                                                                             // 结束代码块
            if (mag.mag_y_ut > max_y)                                                     // 判断条件是否成立
            {                                                                             // 进入代码块
                max_y = mag.mag_y_ut;                                                     // 给变量或寄存器写入新值
            }                                                                             // 结束代码块
            if (mag.mag_z_ut < min_z)                                                     // 判断条件是否成立
            {                                                                             // 进入代码块
                min_z = mag.mag_z_ut;                                                     // 给变量或寄存器写入新值
            }                                                                             // 结束代码块
            if (mag.mag_z_ut > max_z)                                                     // 判断条件是否成立
            {                                                                             // 进入代码块
                max_z = mag.mag_z_ut;                                                     // 给变量或寄存器写入新值
            }                                                                             // 结束代码块
            valid_count++;                                                                // 递增计数值
        }                                                                                 // 结束代码块

        vTaskDelay(pdMS_TO_TICKS(delay_ms));                                              // 让当前任务延时等待
    }                                                                                     // 结束代码块

    if (valid_count == 0U)                                                                // 判断条件是否成立
    {                                                                                     // 进入代码块
        return;                                                                           // 返回函数执行结果
    }                                                                                     // 结束代码块

    g_mag_offset_ut[0] = (min_x + max_x) * 0.5f;                                          // 给变量或寄存器写入新值
    g_mag_offset_ut[1] = (min_y + max_y) * 0.5f;                                          // 给变量或寄存器写入新值
    g_mag_offset_ut[2] = (min_z + max_z) * 0.5f;                                          // 给变量或寄存器写入新值

    radius_x = (max_x - min_x) * 0.5f;                                                    // 给变量或寄存器写入新值
    radius_y = (max_y - min_y) * 0.5f;                                                    // 给变量或寄存器写入新值
    radius_z = (max_z - min_z) * 0.5f;                                                    // 给变量或寄存器写入新值

    if ((radius_x < AK8963_MAG_RADIUS_MIN_UT) ||                                          // 判断条件是否成立
        (radius_y < AK8963_MAG_RADIUS_MIN_UT) ||                                          // 说明当前代码行
        (radius_z < AK8963_MAG_RADIUS_MIN_UT))                                            // 说明当前代码行
    {                                                                                     // 进入代码块
        g_mag_scale[0] = 1.0f;                                                            // 给变量或寄存器写入新值
        g_mag_scale[1] = 1.0f;                                                            // 给变量或寄存器写入新值
        g_mag_scale[2] = 1.0f;                                                            // 给变量或寄存器写入新值
        return;                                                                           // 返回函数执行结果
    }                                                                                     // 结束代码块

    radius_avg = (radius_x + radius_y + radius_z) / 3.0f;                                 // 给变量或寄存器写入新值
    g_mag_scale[0] = radius_avg / radius_x;                                               // 给变量或寄存器写入新值
    g_mag_scale[1] = radius_avg / radius_y;                                               // 给变量或寄存器写入新值
    g_mag_scale[2] = radius_avg / radius_z;                                               // 给变量或寄存器写入新值
}                                                                                         // 结束代码块

/**
 * @brief  初始化 MPU9250 六轴和 AK8963 磁力计。
 * @param  None
 * @return 1 表示初始化成功，0 表示初始化失败。
 */
uint8_t MPU9250_Driver_Init(void)                                                         // 定义局部变量
{                                                                                         // 进入代码块
    Debug_Print("[MPU9250] driver init start\r\n");                                       // 输出调试日志

    BSP_I2C_Soft_Init();                                                                  // 调用 BSP 底层接口
    Debug_Print("[MPU9250] soft i2c init ok\r\n");                                        // 输出调试日志

    if (mpu9250_check_device() != 0)                                                      // 判断条件是否成立
    {                                                                                     // 进入代码块
        Debug_Print("[MPU9250] driver init failed\r\n");                                  // 输出调试日志
        return 0U;                                                                        // 返回函数执行结果
    }                                                                                     // 结束代码块

    mpu9250_config_six_axis();                                                            // 调用函数执行对应操作

    if (ak8963_init() == 0U)                                                              // 判断条件是否成立
    {                                                                                     // 进入代码块
        return 0U;                                                                        // 返回函数执行结果
    }                                                                                     // 结束代码块

    MPU9250_CalibrateGyro(1000U, 10U);                                                    // 调用 MPU9250 驱动接口
    MPU9250_CalibrateAccel(1000U, 10U);                                                   // 调用 MPU9250 驱动接口
    Debug_Print("[AK8963] calibrate mag start\r\n");                                      // 输出调试日志
    AK8963_CalibrateMag(500U, 20U);                                                       // 调用 AK8963 磁力计接口

    Debug_Print("[MPU9250] driver init ok\r\n");                                          // 输出调试日志
    return 1U;                                                                            // 返回函数执行结果
}                                                                                         // 结束代码块
