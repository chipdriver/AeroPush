/**
 * @file    mpu9250_driver.c
 * @brief   MPU9250 六轴与 AK8963 磁力计驱动实现。
 */
#include "mpu9250_driver.h" // 引入 mpu9250_driver.h 提供的接口、宏和类型定义
#include "app_config.h" // 引入 app_config.h 提供的接口、宏和类型定义
#include <math.h> // 引入 math.h 提供的接口、宏和类型定义

#define MPU9250_PWR_MGMT_1_REG 0x6BU // 定义 MPU9250_PWR_MGMT_1_REG 寄存器地址为 0x6BU
#define MPU9250_PWR_MGMT_2_REG 0x6CU // 定义 MPU9250_PWR_MGMT_2_REG 寄存器地址为 0x6CU
#define MPU9250_USER_CTRL_REG 0x6AU // 定义 MPU9250_USER_CTRL_REG 寄存器地址为 0x6AU
#define MPU9250_INT_PIN_CFG_REG 0x37U // 定义 MPU9250_INT_PIN_CFG_REG 寄存器地址为 0x37U
#define MPU9250_CONFIG_REG 0x1AU // 定义 MPU9250_CONFIG_REG 寄存器地址为 0x1AU
#define MPU9250_SMPLRT_DIV_REG 0x19U // 定义 MPU9250_SMPLRT_DIV_REG 寄存器地址为 0x19U
#define MPU9250_ACCEL_CONFIG_REG 0x1CU // 定义 MPU9250_ACCEL_CONFIG_REG 寄存器地址为 0x1CU
#define MPU9250_ACCEL_CONFIG2_REG 0x1DU // 定义 MPU9250_ACCEL_CONFIG2_REG 寄存器地址为 0x1DU
#define MPU9250_GYRO_CONFIG_REG 0x1BU // 定义 MPU9250_GYRO_CONFIG_REG 寄存器地址为 0x1BU
#define MPU9250_ACCEL_XOUT_H_REG 0x3BU // 定义 MPU9250_ACCEL_XOUT_H_REG 寄存器地址为 0x3BU
#define MPU9250_ACCEL_YOUT_H_REG 0x3DU // 定义 MPU9250_ACCEL_YOUT_H_REG 寄存器地址为 0x3DU
#define MPU9250_ACCEL_ZOUT_H_REG 0x3FU // 定义 MPU9250_ACCEL_ZOUT_H_REG 寄存器地址为 0x3FU
#define MPU9250_TEMP_OUT_H_REG 0x41U // 定义 MPU9250_TEMP_OUT_H_REG 寄存器地址为 0x41U
#define MPU9250_GYRO_XOUT_H_REG 0x43U // 定义 MPU9250_GYRO_XOUT_H_REG 寄存器地址为 0x43U
#define MPU9250_GYRO_YOUT_H_REG 0x45U // 定义 MPU9250_GYRO_YOUT_H_REG 寄存器地址为 0x45U
#define MPU9250_GYRO_ZOUT_H_REG 0x47U // 定义 MPU9250_GYRO_ZOUT_H_REG 寄存器地址为 0x47U

#define AK8963_REG_WIA 0x00U // 定义 AK8963_REG_WIA 寄存器地址为 0x00U
#define AK8963_REG_ST1 0x02U // 定义 AK8963_REG_ST1 寄存器地址为 0x02U
#define AK8963_REG_HXL 0x03U // 定义 AK8963_REG_HXL 寄存器地址为 0x03U
#define AK8963_REG_HYL 0x05U // 定义 AK8963_REG_HYL 寄存器地址为 0x05U
#define AK8963_REG_HZL 0x07U // 定义 AK8963_REG_HZL 寄存器地址为 0x07U
#define AK8963_REG_ST2 0x09U // 定义 AK8963_REG_ST2 寄存器地址为 0x09U
#define AK8963_REG_CNTL1 0x0AU // 定义 AK8963_REG_CNTL1 寄存器地址为 0x0AU
#define AK8963_REG_ASAX 0x10U // 定义 AK8963_REG_ASAX 寄存器地址为 0x10U
#define AK8963_REG_ASAY 0x11U // 定义 AK8963_REG_ASAY 寄存器地址为 0x11U
#define AK8963_REG_ASAZ 0x12U // 定义 AK8963_REG_ASAZ 寄存器地址为 0x12U
#define AK8963_WHO_AM_I_VALUE 0x48U // 定义 AK8963_WHO_AM_I_VALUE 变量为 0x48U

#define MPU9250_ACCEL_LSB_PER_G 4096.0f // 定义 MPU9250_ACCEL_LSB_PER_G 变量为 4096.0f
#define MPU9250_GYRO_LSB_PER_DPS 32.8f // 定义 MPU9250_GYRO_LSB_PER_DPS 变量为 32.8f
#define AK8963_16BIT_UT_PER_LSB 0.15f // 定义 AK8963_16BIT_UT_PER_LSB 变量为 0.15f
#define AK8963_MAG_RADIUS_MIN_UT 5.0f // 定义 AK8963_MAG_RADIUS_MIN_UT 变量为 5.0f
#define AK8963_MAG_CAL_MAX_SAMPLES 500U // 定义 AK8963_MAG_CAL_MAX_SAMPLES 变量为 500U
#define AK8963_MAG_FIT_PARAM_COUNT 9U // 定义 AK8963_MAG_FIT_PARAM_COUNT 变量为 9U
#define AK8963_MAG_FIT_MAX_ITERATIONS 24U // 定义 AK8963_MAG_FIT_MAX_ITERATIONS 变量为 24U
#define AK8963_MAG_FIT_MIN_VALID_SAMPLES 64U // 定义 AK8963_MAG_FIT_MIN_VALID_SAMPLES 变量为 64U
#define AK8963_MAG_FIT_INITIAL_DAMPING 1.0e-3f // 定义 AK8963_MAG_FIT_INITIAL_DAMPING 变量为 1.0e-3f
#define AK8963_MAG_FIT_MIN_EIGENVALUE 1.0e-8f // 定义 AK8963_MAG_FIT_MIN_EIGENVALUE 变量为 1.0e-8f
#define AK8963_MAG_FIT_MAX_EIGENVALUE 1.0f // 定义 AK8963_MAG_FIT_MAX_EIGENVALUE 变量为 1.0f

static uint8_t g_ak8963_asa[3] = {0U, 0U, 0U}; // 定义 g_ak8963_asa 全局状态变量，初始值设置为 {0U, 0U, 0U}
static float g_ak8963_sensitivity[3] = {1.0f, 1.0f, 1.0f}; // 定义 g_ak8963_sensitivity 全局状态变量，初始值设置为 {1.0f, 1.0f, 1.0f} 字段值

static float g_gyro_bias_dps[3] = {0.0f, 0.0f, 0.0f}; // 定义 g_gyro_bias_dps 全局状态变量，初始值设置为 {0.0f, 0.0f, 0.0f} 字段值
static float g_accel_bias_g[3] = {0.0f, 0.0f, 0.0f}; // 定义 g_accel_bias_g 全局状态变量，初始值设置为 {0.0f, 0.0f, 0.0f} 字段值
static float g_mag_offset_ut[3] = {0.0f, 0.0f, 0.0f}; // 定义 g_mag_offset_ut 全局状态变量，初始值设置为 {0.0f, 0.0f, 0.0f} 字段值
static float g_mag_correction[3][3] = {{1.0f, 0.0f, 0.0f}, // 继续传入 static float g_mag_correction[3][3] = {{1.0f, 0.0f, 0.0f} 字段值，作为当前多行调用或初始化列表的一项
                                       {0.0f, 1.0f, 0.0f}, // 继续传入 {0.0f, 1.0f, 0.0f} 字段值，作为当前多行调用或初始化列表的一项
                                       {0.0f, 0.0f, 1.0f}}; // 执行 {0.0f, 0.0f, 1.0f}};，完成当前上下文中的具体处理
static AK8963_Physical_Data g_mag_cal_samples[AK8963_MAG_CAL_MAX_SAMPLES]; // 声明 g_mag_cal_samples 全局状态变量，供后续计算、状态保存或模块间传递使用
static float g_mag_fit_matrix[AK8963_MAG_FIT_PARAM_COUNT][AK8963_MAG_FIT_PARAM_COUNT]; // 执行 static float g_mag_fit_matrix[AK8963_MAG_FIT_PARAM_COUNT][AK8963_MAG_FIT_PARAM_COUNT];，完成当前上下文中的具体处理
static float g_mag_fit_vector[AK8963_MAG_FIT_PARAM_COUNT]; // 声明 g_mag_fit_vector 全局状态变量，供后续计算、状态保存或模块间传递使用
static float g_mag_fit_delta[AK8963_MAG_FIT_PARAM_COUNT]; // 声明 g_mag_fit_delta 全局状态变量，供后续计算、状态保存或模块间传递使用
static float g_mag_fit_candidate[AK8963_MAG_FIT_PARAM_COUNT]; // 声明 g_mag_fit_candidate 全局状态变量，供后续计算、状态保存或模块间传递使用

/**
 * @brief mpu9250_read_word 函数。
 * @param reg reg 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
static int16_t mpu9250_read_word(uint8_t reg) // 定义mpu9250_read_word 函数签名：mpu9250_read_word 函数
{ // 进入当前代码块
    uint8_t high_byte = I2C_ReadReg(MPU9250_I2C_ADDR7, reg); // 定义 high_byte 变量，初始值设置为 I2C_ReadReg(MPU9250_I2C_ADDR7, reg)
    uint8_t low_byte = I2C_ReadReg(MPU9250_I2C_ADDR7, (uint8_t)(reg + 1U)); // 定义 low_byte 变量，初始值设置为 I2C_ReadReg(MPU9250_I2C_ADDR7, (uint8_t)(reg + 1U)) 的计算结果

    return (int16_t)(((uint16_t)high_byte << 8U) | (uint16_t)low_byte); // 将 (int16_t)(((uint16_t)high_byte << 8U) | (uint16_t)low_byte) 返回给调用者
} // 结束当前代码块

/**
 * @brief ak8963_read_word 函数。
 * @param low_reg low_reg 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
static int16_t ak8963_read_word(uint8_t low_reg) // 定义ak8963_read_word 函数签名：ak8963_read_word 函数
{ // 进入当前代码块
    uint8_t low_byte = I2C_ReadReg(AK8963_I2C_ADDR7, low_reg); // 定义 low_byte 变量，初始值设置为 I2C_ReadReg(AK8963_I2C_ADDR7, low_reg)
    uint8_t high_byte = I2C_ReadReg(AK8963_I2C_ADDR7, (uint8_t)(low_reg + 1U)); // 定义 high_byte 变量，初始值设置为 I2C_ReadReg(AK8963_I2C_ADDR7, (uint8_t)(low_reg + 1U)) 的计算结果

    return (int16_t)(((uint16_t)high_byte << 8U) | (uint16_t)low_byte); // 将 (int16_t)(((uint16_t)high_byte << 8U) | (uint16_t)low_byte) 返回给调用者
} // 结束当前代码块

/**
 * @brief ak8963_convert_raw_to_ut 函数。
 * @param raw 传感器原始采样数据结构体。
 * @param mag_out mag_out 变量。
 * @retval None
 */
static void ak8963_convert_raw_to_ut(const AK8963_raw_Data *raw, AK8963_Physical_Data *mag_out) // 定义ak8963_convert_raw_to_ut 函数签名：ak8963_convert_raw_to_ut 函数
{ // 进入当前代码块
    if ((raw == 0) || (mag_out == 0)) // 判断 (raw == 0) || (mag_out == 0) 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    mag_out->mag_x_ut = (float)raw->mag_x * g_ak8963_sensitivity[0] * AK8963_16BIT_UT_PER_LSB; // 把 (float)raw->mag_x * g_ak8963_sensitivity[0] * AK8963_16BIT_UT_PER_LSB 字段值 写入 mag_out->mag_x_ut 字段值
    mag_out->mag_y_ut = (float)raw->mag_y * g_ak8963_sensitivity[1] * AK8963_16BIT_UT_PER_LSB; // 把 (float)raw->mag_y * g_ak8963_sensitivity[1] * AK8963_16BIT_UT_PER_LSB 字段值 写入 mag_out->mag_y_ut 字段值
    mag_out->mag_z_ut = (float)raw->mag_z * g_ak8963_sensitivity[2] * AK8963_16BIT_UT_PER_LSB; // 把 (float)raw->mag_z * g_ak8963_sensitivity[2] * AK8963_16BIT_UT_PER_LSB 字段值 写入 mag_out->mag_z_ut 字段值
} // 结束当前代码块

/**
 * @brief ak8963_apply_mag_calibration 函数。
 * @param mag AK8963 磁力计物理量数据。
 * @retval None
 */
static void ak8963_apply_mag_calibration(AK8963_Physical_Data *mag) // 定义ak8963_apply_mag_calibration 函数签名：ak8963_apply_mag_calibration 函数
{ // 进入当前代码块
    float centered_x; // 声明 centered_x 变量，供后续计算、状态保存或模块间传递使用
    float centered_y; // 声明 centered_y 变量，供后续计算、状态保存或模块间传递使用
    float centered_z; // 声明 centered_z 变量，供后续计算、状态保存或模块间传递使用
    float corrected_x; // 声明 corrected_x 变量，供后续计算、状态保存或模块间传递使用
    float corrected_y; // 声明 corrected_y 变量，供后续计算、状态保存或模块间传递使用
    float corrected_z; // 声明 corrected_z 变量，供后续计算、状态保存或模块间传递使用

    if (mag == 0) // 判断 mag == 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    centered_x = mag->mag_x_ut - g_mag_offset_ut[0]; // 把 mag->mag_x_ut - g_mag_offset_ut[0] 字段值 写入 centered_x 变量
    centered_y = mag->mag_y_ut - g_mag_offset_ut[1]; // 把 mag->mag_y_ut - g_mag_offset_ut[1] 字段值 写入 centered_y 变量
    centered_z = mag->mag_z_ut - g_mag_offset_ut[2]; // 把 mag->mag_z_ut - g_mag_offset_ut[2] 字段值 写入 centered_z 变量

    corrected_x = g_mag_correction[0][0] * centered_x + // 将 corrected_x = g_mag_correction[0][0] * centered_x 的计算结果 作为当前多行表达式的一部分继续累加
                  g_mag_correction[0][1] * centered_y + // 将 g_mag_correction[0][1] * centered_y 的计算结果 作为当前多行表达式的一部分继续累加
                  g_mag_correction[0][2] * centered_z; // 执行 g_mag_correction[0][2] * centered_z;，完成当前上下文中的具体处理
    corrected_y = g_mag_correction[1][0] * centered_x + // 将 corrected_y = g_mag_correction[1][0] * centered_x 的计算结果 作为当前多行表达式的一部分继续累加
                  g_mag_correction[1][1] * centered_y + // 将 g_mag_correction[1][1] * centered_y 的计算结果 作为当前多行表达式的一部分继续累加
                  g_mag_correction[1][2] * centered_z; // 执行 g_mag_correction[1][2] * centered_z;，完成当前上下文中的具体处理
    corrected_z = g_mag_correction[2][0] * centered_x + // 将 corrected_z = g_mag_correction[2][0] * centered_x 的计算结果 作为当前多行表达式的一部分继续累加
                  g_mag_correction[2][1] * centered_y + // 将 g_mag_correction[2][1] * centered_y 的计算结果 作为当前多行表达式的一部分继续累加
                  g_mag_correction[2][2] * centered_z; // 执行 g_mag_correction[2][2] * centered_z;，完成当前上下文中的具体处理

    mag->mag_x_ut = corrected_x; // 把 corrected_x 变量 写入 mag->mag_x_ut 字段值
    mag->mag_y_ut = corrected_y; // 把 corrected_y 变量 写入 mag->mag_y_ut 字段值
    mag->mag_z_ut = corrected_z; // 把 corrected_z 变量 写入 mag->mag_z_ut 字段值
} // 结束当前代码块

/**
 * @brief ak8963_zero_fit_system 函数。
 * @param matrix 椭球拟合矩阵。
 * @param vector 椭球拟合右端向量。
 * @retval None
 */
static void ak8963_zero_fit_system(float matrix[AK8963_MAG_FIT_PARAM_COUNT][AK8963_MAG_FIT_PARAM_COUNT], // 定义ak8963_zero_fit_system 函数签名：ak8963_zero_fit_system 函数
                                   float vector[AK8963_MAG_FIT_PARAM_COUNT]) // 定义ak8963_zero_fit_system 函数签名：ak8963_zero_fit_system 函数
{ // 进入当前代码块
    uint8_t row; // 声明 矩阵行索引，供后续计算、状态保存或模块间传递使用
    uint8_t col; // 声明 矩阵列索引，供后续计算、状态保存或模块间传递使用

    for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 按照 row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        vector[row] = 0.0f; // 把 0.0f 字段值 写入 vector[row]
        for (col = 0U; col < AK8963_MAG_FIT_PARAM_COUNT; col++) // 按照 col = 0U; col < AK8963_MAG_FIT_PARAM_COUNT; col++ 的初始化、边界和步进条件重复执行循环体
        { // 进入当前代码块
            matrix[row][col] = 0.0f; // 把 0.0f 字段值 写入 matrix[row][col]
        } // 结束当前代码块
    } // 结束当前代码块
} // 结束当前代码块

/**
 * @brief ak8963_eval_ellipsoid_residual 函数。
 * @param params 椭球拟合参数数组。
 * @param sample 单个磁力计校准样本。
 * @retval 函数执行结果或计算得到的返回值。
 */
static float ak8963_eval_ellipsoid_residual(const float params[AK8963_MAG_FIT_PARAM_COUNT], // 定义ak8963_eval_ellipsoid_residual 函数签名：ak8963_eval_ellipsoid_residual 函数
                                            const AK8963_Physical_Data *sample) // 定义ak8963_eval_ellipsoid_residual 函数签名：ak8963_eval_ellipsoid_residual 函数
{ // 进入当前代码块
    float dx = sample->mag_x_ut - params[0]; // 定义 dx 变量，初始值设置为 sample->mag_x_ut - params[0] 字段值
    float dy = sample->mag_y_ut - params[1]; // 定义 dy 变量，初始值设置为 sample->mag_y_ut - params[1] 字段值
    float dz = sample->mag_z_ut - params[2]; // 定义 dz 变量，初始值设置为 sample->mag_z_ut - params[2] 字段值

    return params[3] * dx * dx + // 将 return params[3] * dx * dx 的计算结果 作为当前多行表达式的一部分继续累加
           2.0f * params[4] * dx * dy + // 将 2.0f * params[4] * dx * dy 字段值 作为当前多行表达式的一部分继续累加
           2.0f * params[5] * dx * dz + // 将 2.0f * params[5] * dx * dz 字段值 作为当前多行表达式的一部分继续累加
           params[6] * dy * dy + // 将 params[6] * dy * dy 的计算结果 作为当前多行表达式的一部分继续累加
           2.0f * params[7] * dy * dz + // 将 2.0f * params[7] * dy * dz 字段值 作为当前多行表达式的一部分继续累加
           params[8] * dz * dz - // 将 params[8] * dz * dz 的计算结果 作为当前多行表达式的一部分继续相减
           1.0f; // 执行 1.0f;，完成当前上下文中的具体处理
} // 结束当前代码块

/**
 * @brief ak8963_compute_fit_error 函数。
 * @param params 椭球拟合参数数组。
 * @param sample_count 有效校准样本数量。
 * @retval 函数执行结果或计算得到的返回值。
 */
static float ak8963_compute_fit_error(const float params[AK8963_MAG_FIT_PARAM_COUNT], uint16_t sample_count) // 定义ak8963_compute_fit_error 函数签名：ak8963_compute_fit_error 函数
{ // 进入当前代码块
    uint16_t i; // 声明 循环索引，供后续计算、状态保存或模块间传递使用
    float residual; // 声明 residual 变量，供后续计算、状态保存或模块间传递使用
    float sum_sq = 0.0f; // 定义 sum_sq 变量，初始值设置为 0.0f 字段值

    for (i = 0U; i < sample_count; i++) // 按照 i = 0U; i < sample_count; i++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        residual = ak8963_eval_ellipsoid_residual(params, &g_mag_cal_samples[i]); // 把 ak8963_eval_ellipsoid_residual(params, &g_mag_cal_samples[i]) 写入 residual 变量
        sum_sq += residual * residual; // 把 residual * residual 的计算结果 累加到 sum_sq 变量
    } // 结束当前代码块

    return sum_sq / (float)sample_count; // 将 sum_sq / (float)sample_count 的计算结果 返回给调用者
} // 结束当前代码块

/**
 * @brief ak8963_build_fit_system 函数。
 * @param params 椭球拟合参数数组。
 * @param sample_count 有效校准样本数量。
 * @param matrix 椭球拟合矩阵。
 * @param vector 椭球拟合右端向量。
 * @retval None
 */
static void ak8963_build_fit_system(const float params[AK8963_MAG_FIT_PARAM_COUNT], // 定义ak8963_build_fit_system 函数签名：ak8963_build_fit_system 函数
                                    uint16_t sample_count, // 定义ak8963_build_fit_system 函数签名：ak8963_build_fit_system 函数
                                    float matrix[AK8963_MAG_FIT_PARAM_COUNT][AK8963_MAG_FIT_PARAM_COUNT], // 定义ak8963_build_fit_system 函数签名：ak8963_build_fit_system 函数
                                    float vector[AK8963_MAG_FIT_PARAM_COUNT]) // 定义ak8963_build_fit_system 函数签名：ak8963_build_fit_system 函数
{ // 进入当前代码块
    uint16_t i; // 声明 循环索引，供后续计算、状态保存或模块间传递使用
    uint8_t row; // 声明 矩阵行索引，供后续计算、状态保存或模块间传递使用
    uint8_t col; // 声明 矩阵列索引，供后续计算、状态保存或模块间传递使用
    float dx; // 声明 dx 变量，供后续计算、状态保存或模块间传递使用
    float dy; // 声明 dy 变量，供后续计算、状态保存或模块间传递使用
    float dz; // 声明 dz 变量，供后续计算、状态保存或模块间传递使用
    float residual; // 声明 residual 变量，供后续计算、状态保存或模块间传递使用
    float jac[AK8963_MAG_FIT_PARAM_COUNT]; // 声明 jac 变量，供后续计算、状态保存或模块间传递使用

    ak8963_zero_fit_system(matrix, vector); // 调用ak8963_zero_fit_system 函数，参数为 matrix, vector

    for (i = 0U; i < sample_count; i++) // 按照 i = 0U; i < sample_count; i++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        dx = g_mag_cal_samples[i].mag_x_ut - params[0]; // 把 g_mag_cal_samples[i].mag_x_ut - params[0] 字段值 写入 dx 变量
        dy = g_mag_cal_samples[i].mag_y_ut - params[1]; // 把 g_mag_cal_samples[i].mag_y_ut - params[1] 字段值 写入 dy 变量
        dz = g_mag_cal_samples[i].mag_z_ut - params[2]; // 把 g_mag_cal_samples[i].mag_z_ut - params[2] 字段值 写入 dz 变量
        residual = ak8963_eval_ellipsoid_residual(params, &g_mag_cal_samples[i]); // 把 ak8963_eval_ellipsoid_residual(params, &g_mag_cal_samples[i]) 写入 residual 变量

        jac[0] = -2.0f * (params[3] * dx + params[4] * dy + params[5] * dz); // 把 -2.0f * (params[3] * dx + params[4] * dy + params[5] * dz) 字段值 写入 jac[0]
        jac[1] = -2.0f * (params[4] * dx + params[6] * dy + params[7] * dz); // 把 -2.0f * (params[4] * dx + params[6] * dy + params[7] * dz) 字段值 写入 jac[1]
        jac[2] = -2.0f * (params[5] * dx + params[7] * dy + params[8] * dz); // 把 -2.0f * (params[5] * dx + params[7] * dy + params[8] * dz) 字段值 写入 jac[2]
        jac[3] = dx * dx; // 把 dx * dx 的计算结果 写入 jac[3]
        jac[4] = 2.0f * dx * dy; // 把 2.0f * dx * dy 字段值 写入 jac[4]
        jac[5] = 2.0f * dx * dz; // 把 2.0f * dx * dz 字段值 写入 jac[5]
        jac[6] = dy * dy; // 把 dy * dy 的计算结果 写入 jac[6]
        jac[7] = 2.0f * dy * dz; // 把 2.0f * dy * dz 字段值 写入 jac[7]
        jac[8] = dz * dz; // 把 dz * dz 的计算结果 写入 jac[8]

        for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 按照 row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++ 的初始化、边界和步进条件重复执行循环体
        { // 进入当前代码块
            vector[row] -= jac[row] * residual; // 执行 vector[row] -= jac[row] * residual;，完成当前上下文中的具体处理
            for (col = row; col < AK8963_MAG_FIT_PARAM_COUNT; col++) // 按照 col = row; col < AK8963_MAG_FIT_PARAM_COUNT; col++ 的初始化、边界和步进条件重复执行循环体
            { // 进入当前代码块
                matrix[row][col] += jac[row] * jac[col]; // 把 jac[row] * jac[col] 的计算结果 累加到 matrix[row][col]
            } // 结束当前代码块
        } // 结束当前代码块
    } // 结束当前代码块

    for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 按照 row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        for (col = 0U; col < row; col++) // 按照 col = 0U; col < row; col++ 的初始化、边界和步进条件重复执行循环体
        { // 进入当前代码块
            matrix[row][col] = matrix[col][row]; // 把 matrix[col][row] 写入 matrix[row][col]
        } // 结束当前代码块
    } // 结束当前代码块
} // 结束当前代码块

/**
 * @brief ak8963_solve_fit_system 函数。
 * @param matrix 椭球拟合矩阵。
 * @param vector 椭球拟合右端向量。
 * @param solution 线性方程求解结果。
 * @retval 函数执行结果或计算得到的返回值。
 */
static uint8_t ak8963_solve_fit_system(float matrix[AK8963_MAG_FIT_PARAM_COUNT][AK8963_MAG_FIT_PARAM_COUNT], // 定义ak8963_solve_fit_system 函数签名：ak8963_solve_fit_system 函数
                                       float vector[AK8963_MAG_FIT_PARAM_COUNT], // 定义ak8963_solve_fit_system 函数签名：ak8963_solve_fit_system 函数
                                       float solution[AK8963_MAG_FIT_PARAM_COUNT]) // 定义ak8963_solve_fit_system 函数签名：ak8963_solve_fit_system 函数
{ // 进入当前代码块
    uint8_t pivot; // 声明 pivot 变量，供后续计算、状态保存或模块间传递使用
    uint8_t row; // 声明 矩阵行索引，供后续计算、状态保存或模块间传递使用
    uint8_t col; // 声明 矩阵列索引，供后续计算、状态保存或模块间传递使用
    uint8_t max_row; // 声明 max_row 变量，供后续计算、状态保存或模块间传递使用
    float max_abs; // 声明 max_abs 变量，供后续计算、状态保存或模块间传递使用
    float candidate_abs; // 声明 candidate_abs 变量，供后续计算、状态保存或模块间传递使用
    float temp; // 声明 temp 变量，供后续计算、状态保存或模块间传递使用
    float factor; // 声明 factor 变量，供后续计算、状态保存或模块间传递使用
    float sum; // 声明 sum 变量，供后续计算、状态保存或模块间传递使用

    for (pivot = 0U; pivot < AK8963_MAG_FIT_PARAM_COUNT; pivot++) // 按照 pivot = 0U; pivot < AK8963_MAG_FIT_PARAM_COUNT; pivot++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        max_row = pivot; // 把 pivot 变量 写入 max_row 变量
        max_abs = fabsf(matrix[pivot][pivot]); // 把 fabsf(matrix[pivot][pivot]) 写入 max_abs 变量
        for (row = (uint8_t)(pivot + 1U); row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 按照 row = (uint8_t)(pivot + 1U); row < AK8963_MAG_FIT_PARAM_COUNT; row++ 的初始化、边界和步进条件重复执行循环体
        { // 进入当前代码块
            candidate_abs = fabsf(matrix[row][pivot]); // 把 fabsf(matrix[row][pivot]) 写入 candidate_abs 变量
            if (candidate_abs > max_abs) // 判断 candidate_abs > max_abs 是否成立，以选择后续执行路径
            { // 进入当前代码块
                max_abs = candidate_abs; // 把 candidate_abs 变量 写入 max_abs 变量
                max_row = row; // 把 矩阵行索引 写入 max_row 变量
            } // 结束当前代码块
        } // 结束当前代码块

        if (max_abs < 1.0e-12f) // 判断 max_abs < 1.0e-12f 是否成立，以选择后续执行路径
        { // 进入当前代码块
            return 0U; // 将 0 返回给调用者
        } // 结束当前代码块

        if (max_row != pivot) // 判断 max_row != pivot 是否成立，以选择后续执行路径
        { // 进入当前代码块
            for (col = pivot; col < AK8963_MAG_FIT_PARAM_COUNT; col++) // 按照 col = pivot; col < AK8963_MAG_FIT_PARAM_COUNT; col++ 的初始化、边界和步进条件重复执行循环体
            { // 进入当前代码块
                temp = matrix[pivot][col]; // 把 matrix[pivot][col] 写入 temp 变量
                matrix[pivot][col] = matrix[max_row][col]; // 把 matrix[max_row][col] 写入 matrix[pivot][col]
                matrix[max_row][col] = temp; // 把 temp 变量 写入 matrix[max_row][col]
            } // 结束当前代码块
            temp = vector[pivot]; // 把 vector[pivot] 写入 temp 变量
            vector[pivot] = vector[max_row]; // 把 vector[max_row] 写入 vector[pivot]
            vector[max_row] = temp; // 把 temp 变量 写入 vector[max_row]
        } // 结束当前代码块

        for (row = (uint8_t)(pivot + 1U); row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 按照 row = (uint8_t)(pivot + 1U); row < AK8963_MAG_FIT_PARAM_COUNT; row++ 的初始化、边界和步进条件重复执行循环体
        { // 进入当前代码块
            factor = matrix[row][pivot] / matrix[pivot][pivot]; // 把 matrix[row][pivot] / matrix[pivot][pivot] 的计算结果 写入 factor 变量
            matrix[row][pivot] = 0.0f; // 把 0.0f 字段值 写入 matrix[row][pivot]
            for (col = (uint8_t)(pivot + 1U); col < AK8963_MAG_FIT_PARAM_COUNT; col++) // 按照 col = (uint8_t)(pivot + 1U); col < AK8963_MAG_FIT_PARAM_COUNT; col++ 的初始化、边界和步进条件重复执行循环体
            { // 进入当前代码块
                matrix[row][col] -= factor * matrix[pivot][col]; // 执行 matrix[row][col] -= factor * matrix[pivot][col];，完成当前上下文中的具体处理
            } // 结束当前代码块
            vector[row] -= factor * vector[pivot]; // 执行 vector[row] -= factor * vector[pivot];，完成当前上下文中的具体处理
        } // 结束当前代码块
    } // 结束当前代码块

    for (row = AK8963_MAG_FIT_PARAM_COUNT; row > 0U; row--) // 按照 row = AK8963_MAG_FIT_PARAM_COUNT; row > 0U; row-- 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        uint8_t idx = (uint8_t)(row - 1U); // 定义 idx 变量，初始值设置为 (uint8_t)(row - 1U) 的计算结果
        sum = vector[idx]; // 把 vector[idx] 写入 sum 变量
        for (col = (uint8_t)(idx + 1U); col < AK8963_MAG_FIT_PARAM_COUNT; col++) // 按照 col = (uint8_t)(idx + 1U); col < AK8963_MAG_FIT_PARAM_COUNT; col++ 的初始化、边界和步进条件重复执行循环体
        { // 进入当前代码块
            sum -= matrix[idx][col] * solution[col]; // 执行 sum -= matrix[idx][col] * solution[col];，完成当前上下文中的具体处理
        } // 结束当前代码块

        if (fabsf(matrix[idx][idx]) < 1.0e-12f) // 判断 fabsf(matrix[idx][idx]) < 1.0e-12f 是否成立，以选择后续执行路径
        { // 进入当前代码块
            return 0U; // 将 0 返回给调用者
        } // 结束当前代码块

        solution[idx] = sum / matrix[idx][idx]; // 把 sum / matrix[idx][idx] 的计算结果 写入 solution[idx]
    } // 结束当前代码块

    return 1U; // 将 1U 返回给调用者
} // 结束当前代码块

/**
 * @brief ak8963_params_to_matrix 函数。
 * @param params 椭球拟合参数数组。
 * @param matrix 椭球拟合矩阵。
 * @retval None
 */
static void ak8963_params_to_matrix(const float params[AK8963_MAG_FIT_PARAM_COUNT], float matrix[3][3]) // 定义ak8963_params_to_matrix 函数签名：ak8963_params_to_matrix 函数
{ // 进入当前代码块
    matrix[0][0] = params[3]; // 把 params[3] 写入 matrix[0][0]
    matrix[0][1] = params[4]; // 把 params[4] 写入 matrix[0][1]
    matrix[0][2] = params[5]; // 把 params[5] 写入 matrix[0][2]
    matrix[1][0] = params[4]; // 把 params[4] 写入 matrix[1][0]
    matrix[1][1] = params[6]; // 把 params[6] 写入 matrix[1][1]
    matrix[1][2] = params[7]; // 把 params[7] 写入 matrix[1][2]
    matrix[2][0] = params[5]; // 把 params[5] 写入 matrix[2][0]
    matrix[2][1] = params[7]; // 把 params[7] 写入 matrix[2][1]
    matrix[2][2] = params[8]; // 把 params[8] 写入 matrix[2][2]
} // 结束当前代码块

/**
 * @brief ak8963_jacobi_eigen_symmetric3 函数。
 * @param input input 变量。
 * @param eigenvalues eigenvalues 变量。
 * @param eigenvectors eigenvectors 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
static uint8_t ak8963_jacobi_eigen_symmetric3(const float input[3][3], float eigenvalues[3], float eigenvectors[3][3]) // 定义ak8963_jacobi_eigen_symmetric3 函数签名：ak8963_jacobi_eigen_symmetric3 函数
{ // 进入当前代码块
    float a[3][3]; // 执行 float a[3][3];，完成当前上下文中的具体处理
    uint8_t row; // 声明 矩阵行索引，供后续计算、状态保存或模块间传递使用
    uint8_t col; // 声明 矩阵列索引，供后续计算、状态保存或模块间传递使用
    uint8_t iter; // 声明 iter 变量，供后续计算、状态保存或模块间传递使用
    uint8_t p; // 声明 p 变量，供后续计算、状态保存或模块间传递使用
    uint8_t q; // 声明 q 变量，供后续计算、状态保存或模块间传递使用
    float max_off_diag; // 声明 max_off_diag 变量，供后续计算、状态保存或模块间传递使用
    float off_diag; // 声明 off_diag 变量，供后续计算、状态保存或模块间传递使用
    float app; // 声明 app 变量，供后续计算、状态保存或模块间传递使用
    float aqq; // 声明 aqq 变量，供后续计算、状态保存或模块间传递使用
    float apq; // 声明 apq 变量，供后续计算、状态保存或模块间传递使用
    float phi; // 声明 phi 变量，供后续计算、状态保存或模块间传递使用
    float c; // 声明 c 变量，供后续计算、状态保存或模块间传递使用
    float s; // 声明 s 变量，供后续计算、状态保存或模块间传递使用
    float aip; // 声明 aip 变量，供后续计算、状态保存或模块间传递使用
    float aiq; // 声明 aiq 变量，供后续计算、状态保存或模块间传递使用
    float vip; // 声明 vip 变量，供后续计算、状态保存或模块间传递使用
    float viq; // 声明 viq 变量，供后续计算、状态保存或模块间传递使用

    for (row = 0U; row < 3U; row++) // 按照 row = 0U; row < 3U; row++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        for (col = 0U; col < 3U; col++) // 按照 col = 0U; col < 3U; col++ 的初始化、边界和步进条件重复执行循环体
        { // 进入当前代码块
            a[row][col] = input[row][col]; // 把 input[row][col] 写入 a[row][col]
            eigenvectors[row][col] = (row == col) ? 1.0f : 0.0f; // 把 (row == col) ? 1.0f : 0.0f 字段值 写入 eigenvectors[row][col]
        } // 结束当前代码块
    } // 结束当前代码块

    for (iter = 0U; iter < 18U; iter++) // 按照 iter = 0U; iter < 18U; iter++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        p = 0U; // 把 0 写入 p 变量
        q = 1U; // 把 1U 写入 q 变量
        max_off_diag = fabsf(a[0][1]); // 把 fabsf(a[0][1]) 写入 max_off_diag 变量

        off_diag = fabsf(a[0][2]); // 把 fabsf(a[0][2]) 写入 off_diag 变量
        if (off_diag > max_off_diag) // 判断 off_diag > max_off_diag 是否成立，以选择后续执行路径
        { // 进入当前代码块
            max_off_diag = off_diag; // 把 off_diag 变量 写入 max_off_diag 变量
            p = 0U; // 把 0 写入 p 变量
            q = 2U; // 把 2U 写入 q 变量
        } // 结束当前代码块

        off_diag = fabsf(a[1][2]); // 把 fabsf(a[1][2]) 写入 off_diag 变量
        if (off_diag > max_off_diag) // 判断 off_diag > max_off_diag 是否成立，以选择后续执行路径
        { // 进入当前代码块
            max_off_diag = off_diag; // 把 off_diag 变量 写入 max_off_diag 变量
            p = 1U; // 把 1U 写入 p 变量
            q = 2U; // 把 2U 写入 q 变量
        } // 结束当前代码块

        if (max_off_diag < 1.0e-9f) // 判断 max_off_diag < 1.0e-9f 是否成立，以选择后续执行路径
        { // 进入当前代码块
            break; // 结束当前循环或分支处理
        } // 结束当前代码块

        app = a[p][p]; // 把 a[p][p] 写入 app 变量
        aqq = a[q][q]; // 把 a[q][q] 写入 aqq 变量
        apq = a[p][q]; // 把 a[p][q] 写入 apq 变量
        phi = 0.5f * atan2f(2.0f * apq, aqq - app); // 把 0.5f * atan2f(2.0f * apq, aqq - app) 字段值 写入 phi 变量
        c = cosf(phi); // 把 cosf(phi) 写入 c 变量
        s = sinf(phi); // 把 sinf(phi) 写入 s 变量

        for (row = 0U; row < 3U; row++) // 按照 row = 0U; row < 3U; row++ 的初始化、边界和步进条件重复执行循环体
        { // 进入当前代码块
            if ((row != p) && (row != q)) // 判断 (row != p) && (row != q) 是否成立，以选择后续执行路径
            { // 进入当前代码块
                aip = a[row][p]; // 把 a[row][p] 写入 aip 变量
                aiq = a[row][q]; // 把 a[row][q] 写入 aiq 变量
                a[row][p] = c * aip - s * aiq; // 把 c * aip - s * aiq 的计算结果 写入 a[row][p]
                a[p][row] = a[row][p]; // 把 a[row][p] 写入 a[p][row]
                a[row][q] = s * aip + c * aiq; // 把 s * aip + c * aiq 的计算结果 写入 a[row][q]
                a[q][row] = a[row][q]; // 把 a[row][q] 写入 a[q][row]
            } // 结束当前代码块
        } // 结束当前代码块

        a[p][p] = c * c * app - 2.0f * s * c * apq + s * s * aqq; // 把 c * c * app - 2.0f * s * c * apq + s * s * aqq 字段值 写入 a[p][p]
        a[q][q] = s * s * app + 2.0f * s * c * apq + c * c * aqq; // 把 s * s * app + 2.0f * s * c * apq + c * c * aqq 字段值 写入 a[q][q]
        a[p][q] = 0.0f; // 把 0.0f 字段值 写入 a[p][q]
        a[q][p] = 0.0f; // 把 0.0f 字段值 写入 a[q][p]

        for (row = 0U; row < 3U; row++) // 按照 row = 0U; row < 3U; row++ 的初始化、边界和步进条件重复执行循环体
        { // 进入当前代码块
            vip = eigenvectors[row][p]; // 把 eigenvectors[row][p] 写入 vip 变量
            viq = eigenvectors[row][q]; // 把 eigenvectors[row][q] 写入 viq 变量
            eigenvectors[row][p] = c * vip - s * viq; // 把 c * vip - s * viq 的计算结果 写入 eigenvectors[row][p]
            eigenvectors[row][q] = s * vip + c * viq; // 把 s * vip + c * viq 的计算结果 写入 eigenvectors[row][q]
        } // 结束当前代码块
    } // 结束当前代码块

    for (row = 0U; row < 3U; row++) // 按照 row = 0U; row < 3U; row++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        eigenvalues[row] = a[row][row]; // 把 a[row][row] 写入 eigenvalues[row]
        if ((eigenvalues[row] < AK8963_MAG_FIT_MIN_EIGENVALUE) || // 调用if 函数，参数为 (eigenvalues[row] < AK8963_MAG_FIT_MIN_EIGENVALUE) ||
            (eigenvalues[row] > AK8963_MAG_FIT_MAX_EIGENVALUE)) // 执行 (eigenvalues[row] > AK8963_MAG_FIT_MAX_EIGENVALUE))，完成当前上下文中的具体处理
        { // 进入当前代码块
            return 0U; // 将 0 返回给调用者
        } // 结束当前代码块
    } // 结束当前代码块

    return 1U; // 将 1U 返回给调用者
} // 结束当前代码块

/**
 * @brief ak8963_build_softiron_matrix 函数。
 * @param params 椭球拟合参数数组。
 * @param correction correction 变量。
 * @param radius_ut radius_ut 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
static uint8_t ak8963_build_softiron_matrix(const float params[AK8963_MAG_FIT_PARAM_COUNT], // 定义ak8963_build_softiron_matrix 函数签名：ak8963_build_softiron_matrix 函数
                                            float correction[3][3], // 定义ak8963_build_softiron_matrix 函数签名：ak8963_build_softiron_matrix 函数
                                            float *radius_ut) // 定义ak8963_build_softiron_matrix 函数签名：ak8963_build_softiron_matrix 函数
{ // 进入当前代码块
    float quadratic[3][3]; // 执行 float quadratic[3][3];，完成当前上下文中的具体处理
    float eigenvalues[3]; // 声明 eigenvalues 变量，供后续计算、状态保存或模块间传递使用
    float eigenvectors[3][3]; // 执行 float eigenvectors[3][3];，完成当前上下文中的具体处理
    float sqrt_lambda[3]; // 声明 sqrt_lambda 变量，供后续计算、状态保存或模块间传递使用
    float axis_radius[3]; // 声明 axis_radius 变量，供后续计算、状态保存或模块间传递使用
    uint8_t row; // 声明 矩阵行索引，供后续计算、状态保存或模块间传递使用
    uint8_t col; // 声明 矩阵列索引，供后续计算、状态保存或模块间传递使用
    uint8_t axis; // 声明 axis 变量，供后续计算、状态保存或模块间传递使用

    ak8963_params_to_matrix(params, quadratic); // 调用ak8963_params_to_matrix 函数，参数为 params, quadratic
    if (ak8963_jacobi_eigen_symmetric3(quadratic, eigenvalues, eigenvectors) == 0U) // 判断 ak8963_jacobi_eigen_symmetric3(quadratic, eigenvalues, eigenvectors) == 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return 0U; // 将 0 返回给调用者
    } // 结束当前代码块

    *radius_ut = 0.0f;
    for (axis = 0U; axis < 3U; axis++) // 按照 axis = 0U; axis < 3U; axis++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        sqrt_lambda[axis] = sqrtf(eigenvalues[axis]); // 把 sqrtf(eigenvalues[axis]) 写入 sqrt_lambda[axis]
        axis_radius[axis] = 1.0f / sqrt_lambda[axis]; // 把 1.0f / sqrt_lambda[axis] 字段值 写入 axis_radius[axis]
        *radius_ut += axis_radius[axis];
    } // 结束当前代码块
    *radius_ut /= 3.0f;

    if (*radius_ut < AK8963_MAG_RADIUS_MIN_UT) // 判断 *radius_ut < AK8963_MAG_RADIUS_MIN_UT 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return 0U; // 将 0 返回给调用者
    } // 结束当前代码块

    for (row = 0U; row < 3U; row++) // 按照 row = 0U; row < 3U; row++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        for (col = 0U; col < 3U; col++) // 按照 col = 0U; col < 3U; col++ 的初始化、边界和步进条件重复执行循环体
        { // 进入当前代码块
            correction[row][col] = 0.0f; // 把 0.0f 字段值 写入 correction[row][col]
            for (axis = 0U; axis < 3U; axis++) // 按照 axis = 0U; axis < 3U; axis++ 的初始化、边界和步进条件重复执行循环体
            { // 进入当前代码块
                correction[row][col] += (*radius_ut) * // 执行 correction[row][col] += (*radius_ut) *，完成当前上下文中的具体处理
                                        eigenvectors[row][axis] * // 执行 eigenvectors[row][axis] *，完成当前上下文中的具体处理
                                        sqrt_lambda[axis] * // 执行 sqrt_lambda[axis] *，完成当前上下文中的具体处理
                                        eigenvectors[col][axis]; // 执行 eigenvectors[col][axis];，完成当前上下文中的具体处理
            } // 结束当前代码块
        } // 结束当前代码块
    } // 结束当前代码块

    return 1U; // 将 1U 返回给调用者
} // 结束当前代码块

/**
 * @brief ak8963_fit_ellipsoid 函数。
 * @param sample_count 有效校准样本数量。
 * @param params 椭球拟合参数数组。
 * @param fit_error fit_error 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
static uint8_t ak8963_fit_ellipsoid(uint16_t sample_count, // 定义ak8963_fit_ellipsoid 函数签名：ak8963_fit_ellipsoid 函数
                                    float params[AK8963_MAG_FIT_PARAM_COUNT], // 定义ak8963_fit_ellipsoid 函数签名：ak8963_fit_ellipsoid 函数
                                    float *fit_error) // 定义ak8963_fit_ellipsoid 函数签名：ak8963_fit_ellipsoid 函数
{ // 进入当前代码块
    uint8_t iter; // 声明 iter 变量，供后续计算、状态保存或模块间传递使用
    uint8_t row; // 声明 矩阵行索引，供后续计算、状态保存或模块间传递使用
    float damping = AK8963_MAG_FIT_INITIAL_DAMPING; // 定义 damping 变量，初始值设置为 AK8963_MAG_FIT_INITIAL_DAMPING 变量
    float current_error = ak8963_compute_fit_error(params, sample_count); // 定义 current_error 变量，初始值设置为 ak8963_compute_fit_error(params, sample_count)
    float candidate_error; // 声明 candidate_error 变量，供后续计算、状态保存或模块间传递使用
    float max_delta; // 声明 max_delta 变量，供后续计算、状态保存或模块间传递使用

    for (iter = 0U; iter < AK8963_MAG_FIT_MAX_ITERATIONS; iter++) // 按照 iter = 0U; iter < AK8963_MAG_FIT_MAX_ITERATIONS; iter++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        ak8963_build_fit_system(params, sample_count, g_mag_fit_matrix, g_mag_fit_vector); // 调用ak8963_build_fit_system 函数，参数为 params, sample_count, g_mag_fit_matrix, g_mag_fit_vector
        for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 按照 row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++ 的初始化、边界和步进条件重复执行循环体
        { // 进入当前代码块
            g_mag_fit_matrix[row][row] += damping; // 把 damping 变量 累加到 g_mag_fit_matrix[row][row]
            g_mag_fit_delta[row] = 0.0f; // 把 0.0f 字段值 写入 g_mag_fit_delta[row]
        } // 结束当前代码块

        if (ak8963_solve_fit_system(g_mag_fit_matrix, g_mag_fit_vector, g_mag_fit_delta) == 0U) // 判断 ak8963_solve_fit_system(g_mag_fit_matrix, g_mag_fit_vector, g_mag_fit_delta) == 0U 是否成立，以选择后续执行路径
        { // 进入当前代码块
            return 0U; // 将 0 返回给调用者
        } // 结束当前代码块

        max_delta = 0.0f; // 把 0.0f 字段值 写入 max_delta 变量
        for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 按照 row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++ 的初始化、边界和步进条件重复执行循环体
        { // 进入当前代码块
            g_mag_fit_candidate[row] = params[row] + g_mag_fit_delta[row]; // 把 params[row] + g_mag_fit_delta[row] 的计算结果 写入 g_mag_fit_candidate[row]
            if (fabsf(g_mag_fit_delta[row]) > max_delta) // 判断 fabsf(g_mag_fit_delta[row]) > max_delta 是否成立，以选择后续执行路径
            { // 进入当前代码块
                max_delta = fabsf(g_mag_fit_delta[row]); // 把 fabsf(g_mag_fit_delta[row]) 写入 max_delta 变量
            } // 结束当前代码块
        } // 结束当前代码块

        candidate_error = ak8963_compute_fit_error(g_mag_fit_candidate, sample_count); // 把 ak8963_compute_fit_error(g_mag_fit_candidate, sample_count) 写入 candidate_error 变量
        if (candidate_error < current_error) // 判断 candidate_error < current_error 是否成立，以选择后续执行路径
        { // 进入当前代码块
            for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 按照 row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++ 的初始化、边界和步进条件重复执行循环体
            { // 进入当前代码块
                params[row] = g_mag_fit_candidate[row]; // 把 g_mag_fit_candidate[row] 写入 params[row]
            } // 结束当前代码块
            current_error = candidate_error; // 把 candidate_error 变量 写入 current_error 变量
            damping *= 0.35f; // 执行 damping *= 0.35f;，完成当前上下文中的具体处理
            if (damping < 1.0e-7f) // 判断 damping < 1.0e-7f 是否成立，以选择后续执行路径
            { // 进入当前代码块
                damping = 1.0e-7f; // 把 1.0e-7f 字段值 写入 damping 变量
            } // 结束当前代码块

            if (max_delta < 1.0e-5f) // 判断 max_delta < 1.0e-5f 是否成立，以选择后续执行路径
            { // 进入当前代码块
                break; // 结束当前循环或分支处理
            } // 结束当前代码块
        } // 结束当前代码块
        else // 处理前面判断条件不成立时的备用逻辑
        { // 进入当前代码块
            damping *= 10.0f; // 执行 damping *= 10.0f;，完成当前上下文中的具体处理
            if (damping > 1.0e6f) // 判断 damping > 1.0e6f 是否成立，以选择后续执行路径
            { // 进入当前代码块
                return 0U; // 将 0 返回给调用者
            } // 结束当前代码块
        } // 结束当前代码块
    } // 结束当前代码块

    *fit_error = current_error;
    return 1U; // 将 1U 返回给调用者
} // 结束当前代码块
/**
 * @brief mpu9250_check_device 函数。
 * @retval 函数执行结果或计算得到的返回值。
 */
static int mpu9250_check_device(void) // 定义mpu9250_check_device 函数签名：mpu9250_check_device 函数
{ // 进入当前代码块
    uint8_t id = 0U; // 定义 设备 ID 输出变量，初始值设置为 0

    if (MPU9250_Driver_ReadWhoAmI(&id) == 0U) // 判断 MPU9250_Driver_ReadWhoAmI(&id) == 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        Debug_Print("[MPU9250] WHO_AM_I read failed\r\n"); // 调用通过调试串口输出字符串，参数为 "[MPU9250] WHO_AM_I read failed\r\n"
        return -1; // 将 -1 的计算结果 返回给调用者
    } // 结束当前代码块

    Debug_Printf("[MPU9250] WHO_AM_I = 0x%02X\r\n", id); // 调用格式化并通过调试串口输出调试信息，参数为 "[MPU9250] WHO_AM_I = 0x%02X\r\n", id

    if (id != MPU9250_WHO_AM_I_VALUE) // 判断 id != MPU9250_WHO_AM_I_VALUE 是否成立，以选择后续执行路径
    { // 进入当前代码块
        Debug_Print("[MPU9250] WHO_AM_I value error\r\n"); // 调用通过调试串口输出字符串，参数为 "[MPU9250] WHO_AM_I value error\r\n"
        return -2; // 将 -2 的计算结果 返回给调用者
    } // 结束当前代码块

    Debug_Print("[MPU9250] WHO_AM_I check ok\r\n"); // 调用通过调试串口输出字符串，参数为 "[MPU9250] WHO_AM_I check ok\r\n"
    return 0; // 将 0 返回给调用者
} // 结束当前代码块

/**
 * @brief 配置 MPU9250 六轴初始化参数。
 *
 * 六轴指三轴加速度计和三轴陀螺仪。
 * 这里完成复位、唤醒、使能、滤波、采样率和量程配置。
 *
 * @retval None
 */
static void mpu9250_config_six_axis(void) // 配置 MPU9250 的加速度计和陀螺仪
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

/**
 * @brief ak8963_init 函数。
 * @retval 函数执行结果或计算得到的返回值。
 */
static uint8_t ak8963_init(void) // 定义ak8963_init 函数签名：ak8963_init 函数
{ // 进入当前代码块
    mpu_set_ak8963_by_mcu(); // 调用mpu_set_ak8963_by_mcu 函数

    if (AK8963_CheckDeviceID() != 0) // 判断 AK8963_CheckDeviceID() != 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        Debug_Print("[AK8963] device ID check failed\r\n"); // 调用通过调试串口输出字符串，参数为 "[AK8963] device ID check failed\r\n"
        return 0U; // 将 0 返回给调用者
    } // 结束当前代码块

    Debug_Print("[AK8963] device ID check ok\r\n"); // 调用通过调试串口输出字符串，参数为 "[AK8963] device ID check ok\r\n"

    AK8963_EnterPowerDownMode(); // 调用AK8963_EnterPowerDownMode 函数
    AK8963_EnterFuseROMMode(); // 调用AK8963_EnterFuseROMMode 函数
    AK8963_AdjustSensitivity(); // 调用AK8963_AdjustSensitivity 函数
    AK8963_EnterPowerDownMode(); // 调用AK8963_EnterPowerDownMode 函数
    AK8963_EnterContinuousMeasurementMode(); // 调用AK8963_EnterContinuousMeasurementMode 函数

    if (AK8963_CheckDataReady() == 0) // 判断 AK8963_CheckDataReady() == 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        Debug_Print("[AK8963] data not ready\r\n"); // 调用通过调试串口输出字符串，参数为 "[AK8963] data not ready\r\n"
        return 0U; // 将 0 返回给调用者
    } // 结束当前代码块

    Debug_Print("[AK8963] data ready\r\n"); // 调用通过调试串口输出字符串，参数为 "[AK8963] data ready\r\n"
    return 1U; // 将 1U 返回给调用者
} // 结束当前代码块

/**
 * @brief MPU9250_Driver_ReadWhoAmI 函数。
 * @param id 设备 ID 输出变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
uint8_t MPU9250_Driver_ReadWhoAmI(uint8_t *id) // 定义MPU9250_Driver_ReadWhoAmI 函数签名：MPU9250_Driver_ReadWhoAmI 函数
{ // 进入当前代码块
    int ret; // 声明 函数返回状态变量，供后续计算、状态保存或模块间传递使用

    if (id == 0) // 判断 id == 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return 0U; // 将 0 返回给调用者
    } // 结束当前代码块

    ret = I2C_ReadRegData(MPU9250_I2C_ADDR7, MPU9250_REG_WHO_AM_I, id); // 把 I2C_ReadRegData(MPU9250_I2C_ADDR7, MPU9250_REG_WHO_AM_I, id) 写入 函数返回状态变量
    return (ret == 0) ? 1U : 0U; // 将 (ret == 0) ? 1U : 0U 返回给调用者
} // 结束当前代码块

/**
 * @brief MPU9250_SoftReset 函数。
 * @retval None
 */
void MPU9250_SoftReset(void) // 定义MPU9250_SoftReset 函数签名：MPU9250_SoftReset 函数
{ // 进入当前代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG); // 定义 寄存器临时读写值，初始值设置为 I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG)

    val &= (uint8_t)~(1U << 7U); // 执行 val &= (uint8_t)~(1U << 7U);，完成当前上下文中的具体处理
    val |= (uint8_t)(1U << 7U); // 执行 val |= (uint8_t)(1U << 7U);，完成当前上下文中的具体处理
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG, val); // 通过软件 I2C 写入指定设备寄存器，参数为 MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG, val

    vTaskDelay(pdMS_TO_TICKS(100)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(100)
} // 结束当前代码块

/**
 * @brief MPU9250_Read_PowerMgmt 函数。
 * @retval None
 */
void MPU9250_Read_PowerMgmt(void) // 定义MPU9250_Read_PowerMgmt 函数签名：MPU9250_Read_PowerMgmt 函数
{ // 进入当前代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG); // 定义 寄存器临时读写值，初始值设置为 I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG)

    Debug_Printf("[MPU9250] PWR_MGMT_1 = 0x%02X\r\n", val); // 调用格式化并通过调试串口输出调试信息，参数为 "[MPU9250] PWR_MGMT_1 = 0x%02X\r\n", val
} // 结束当前代码块

/**
 * @brief mpu_set_clock_to_auto 函数。
 * @retval None
 */
void mpu_set_clock_to_auto(void) // 定义mpu_set_clock_to_auto 函数签名：mpu_set_clock_to_auto 函数
{ // 进入当前代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG); // 定义 寄存器临时读写值，初始值设置为 I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG)

    val &= (uint8_t)~((1U << 6U) | 0x07U); // 执行 val &= (uint8_t)~((1U << 6U) | 0x07U);，完成当前上下文中的具体处理
    val |= 0x01U; // 执行 val |= 0x01U;，完成当前上下文中的具体处理
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG, val); // 通过软件 I2C 写入指定设备寄存器，参数为 MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG, val
} // 结束当前代码块

/**
 * @brief mpu_enable_six_axis 函数。
 * @retval None
 */
void mpu_enable_six_axis(void) // 定义mpu_enable_six_axis 函数签名：mpu_enable_six_axis 函数
{ // 进入当前代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_2_REG); // 定义 寄存器临时读写值，初始值设置为 I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_2_REG)

    val &= (uint8_t)~0x3FU; // 执行 val &= (uint8_t)~0x3FU;，完成当前上下文中的具体处理
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_2_REG, val); // 通过软件 I2C 写入指定设备寄存器，参数为 MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_2_REG, val
} // 结束当前代码块

/**
 * @brief mpu_set_dlpf_cfg_3 函数。
 * @retval None
 */
void mpu_set_dlpf_cfg_3(void) // 定义mpu_set_dlpf_cfg_3 函数签名：mpu_set_dlpf_cfg_3 函数
{ // 进入当前代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_CONFIG_REG); // 定义 寄存器临时读写值，初始值设置为 I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_CONFIG_REG)

    val = (uint8_t)((val & (uint8_t)~0x07U) | 0x03U); // 把 (uint8_t)((val & (uint8_t)~0x07U) | 0x03U) 写入 寄存器临时读写值
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_CONFIG_REG, val); // 通过软件 I2C 写入指定设备寄存器，参数为 MPU9250_I2C_ADDR7, MPU9250_CONFIG_REG, val
} // 结束当前代码块

/**
 * @brief mpu_set_sample_rate_200hz 函数。
 * @retval None
 */
void mpu_set_sample_rate_200hz(void) // 定义mpu_set_sample_rate_200hz 函数签名：mpu_set_sample_rate_200hz 函数
{ // 进入当前代码块
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_SMPLRT_DIV_REG, 0x04U); // 通过软件 I2C 写入指定设备寄存器，参数为 MPU9250_I2C_ADDR7, MPU9250_SMPLRT_DIV_REG, 0x04U
} // 结束当前代码块

/**
 * @brief mpu_set_accel_dlpf 函数。
 * @retval None
 */
void mpu_set_accel_dlpf(void) // 定义mpu_set_accel_dlpf 函数签名：mpu_set_accel_dlpf 函数
{ // 进入当前代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG2_REG); // 定义 寄存器临时读写值，初始值设置为 I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG2_REG)

    val &= (uint8_t)~0x07U; // 执行 val &= (uint8_t)~0x07U;，完成当前上下文中的具体处理
    val |= 0x03U; // 执行 val |= 0x03U;，完成当前上下文中的具体处理
    val &= (uint8_t)~(1U << 3U); // 执行 val &= (uint8_t)~(1U << 3U);，完成当前上下文中的具体处理
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG2_REG, val); // 通过软件 I2C 写入指定设备寄存器，参数为 MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG2_REG, val
} // 结束当前代码块

/**
 * @brief mpu_set_gyro_config 函数。
 * @retval None
 */
void mpu_set_gyro_config(void) // 定义mpu_set_gyro_config 函数签名：mpu_set_gyro_config 函数
{ // 进入当前代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_GYRO_CONFIG_REG); // 定义 寄存器临时读写值，初始值设置为 I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_GYRO_CONFIG_REG)

    val &= (uint8_t)~0x03U; // 执行 val &= (uint8_t)~0x03U;，完成当前上下文中的具体处理
    val &= (uint8_t)~(3U << 3U); // 执行 val &= (uint8_t)~(3U << 3U);，完成当前上下文中的具体处理
    val |= (uint8_t)(2U << 3U); // 执行 val |= (uint8_t)(2U << 3U);，完成当前上下文中的具体处理
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_GYRO_CONFIG_REG, val); // 通过软件 I2C 写入指定设备寄存器，参数为 MPU9250_I2C_ADDR7, MPU9250_GYRO_CONFIG_REG, val
} // 结束当前代码块

/**
 * @brief mpu_set_accel_range 函数。
 * @retval None
 */
void mpu_set_accel_range(void) // 定义mpu_set_accel_range 函数签名：mpu_set_accel_range 函数
{ // 进入当前代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG_REG); // 定义 寄存器临时读写值，初始值设置为 I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG_REG)

    val &= (uint8_t)~(3U << 3U); // 执行 val &= (uint8_t)~(3U << 3U);，完成当前上下文中的具体处理
    val |= (uint8_t)(2U << 3U); // 执行 val |= (uint8_t)(2U << 3U);，完成当前上下文中的具体处理
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG_REG, val); // 通过软件 I2C 写入指定设备寄存器，参数为 MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG_REG, val
} // 结束当前代码块

/**
 * @brief 读取 MPU9250 加速度计、陀螺仪和温度原始寄存器。
 * @param raw 传感器原始采样数据结构体。
 * @retval None
 */
void MPU9250_ReadAxis(MPU9250_raw_Data *raw) // 定义MPU9250_ReadAxis 函数签名：读取 MPU9250 加速度计、陀螺仪和温度原始寄存器
{ // 进入当前代码块
    if (raw == 0) // 判断 raw == 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    raw->accel_x = mpu9250_read_word(MPU9250_ACCEL_XOUT_H_REG); // 把 mpu9250_read_word(MPU9250_ACCEL_XOUT_H_REG) 写入 raw->accel_x 字段值
    raw->accel_y = mpu9250_read_word(MPU9250_ACCEL_YOUT_H_REG); // 把 mpu9250_read_word(MPU9250_ACCEL_YOUT_H_REG) 写入 raw->accel_y 字段值
    raw->accel_z = mpu9250_read_word(MPU9250_ACCEL_ZOUT_H_REG); // 把 mpu9250_read_word(MPU9250_ACCEL_ZOUT_H_REG) 写入 raw->accel_z 字段值
    raw->gyro_x = mpu9250_read_word(MPU9250_GYRO_XOUT_H_REG); // 把 mpu9250_read_word(MPU9250_GYRO_XOUT_H_REG) 写入 raw->gyro_x 字段值
    raw->gyro_y = mpu9250_read_word(MPU9250_GYRO_YOUT_H_REG); // 把 mpu9250_read_word(MPU9250_GYRO_YOUT_H_REG) 写入 raw->gyro_y 字段值
    raw->gyro_z = mpu9250_read_word(MPU9250_GYRO_ZOUT_H_REG); // 把 mpu9250_read_word(MPU9250_GYRO_ZOUT_H_REG) 写入 raw->gyro_z 字段值
    raw->temp = mpu9250_read_word(MPU9250_TEMP_OUT_H_REG); // 把 mpu9250_read_word(MPU9250_TEMP_OUT_H_REG) 写入 raw->temp 字段值
} // 结束当前代码块

/**
 * @brief 将 MPU9250 原始值转换成 g、dps 和摄氏度物理量。
 * @param raw 传感器原始采样数据结构体。
 * @param physical MPU9250 六轴物理量输出结构体。
 * @retval None
 */
void MPU9250_ConvertToPhysical(const MPU9250_raw_Data *raw, MPU9250_Physical_Data *physical) // 定义MPU9250_ConvertToPhysical 函数签名：将 MPU9250 原始值转换成 g、dps 和摄氏度物理量
{ // 进入当前代码块
    if ((raw == 0) || (physical == 0)) // 判断 (raw == 0) || (physical == 0) 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    physical->accel_x_g = (float)raw->accel_x / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[0]; // 把 (float)raw->accel_x / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[0] 字段值 写入 physical->accel_x_g 字段值
    physical->accel_y_g = (float)raw->accel_y / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[1]; // 把 (float)raw->accel_y / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[1] 字段值 写入 physical->accel_y_g 字段值
    physical->accel_z_g = (float)raw->accel_z / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[2]; // 把 (float)raw->accel_z / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[2] 字段值 写入 physical->accel_z_g 字段值
    physical->gyro_x_dps = (float)raw->gyro_x / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[0]; // 把 (float)raw->gyro_x / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[0] 字段值 写入 physical->gyro_x_dps 字段值
    physical->gyro_y_dps = (float)raw->gyro_y / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[1]; // 把 (float)raw->gyro_y / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[1] 字段值 写入 physical->gyro_y_dps 字段值
    physical->gyro_z_dps = (float)raw->gyro_z / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[2]; // 把 (float)raw->gyro_z / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[2] 字段值 写入 physical->gyro_z_dps 字段值
    physical->temp_c = ((float)raw->temp / 333.87f) + 21.0f; // 把 ((float)raw->temp / 333.87f) + 21.0f 字段值 写入 physical->temp_c 字段值
} // 结束当前代码块

/**
 * @brief mpu_set_ak8963_by_mcu 函数。
 * @retval None
 */
void mpu_set_ak8963_by_mcu(void) // 定义mpu_set_ak8963_by_mcu 函数签名：mpu_set_ak8963_by_mcu 函数
{ // 进入当前代码块
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_USER_CTRL_REG); // 定义 寄存器临时读写值，初始值设置为 I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_USER_CTRL_REG)

    val &= (uint8_t)~(1U << 5U); // 执行 val &= (uint8_t)~(1U << 5U);，完成当前上下文中的具体处理
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_USER_CTRL_REG, val); // 通过软件 I2C 写入指定设备寄存器，参数为 MPU9250_I2C_ADDR7, MPU9250_USER_CTRL_REG, val

    val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_INT_PIN_CFG_REG); // 把 I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_INT_PIN_CFG_REG) 写入 寄存器临时读写值
    val |= (uint8_t)(1U << 1U); // 执行 val |= (uint8_t)(1U << 1U);，完成当前上下文中的具体处理
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_INT_PIN_CFG_REG, val); // 通过软件 I2C 写入指定设备寄存器，参数为 MPU9250_I2C_ADDR7, MPU9250_INT_PIN_CFG_REG, val
} // 结束当前代码块

/**
 * @brief AK8963_CheckDeviceID 函数。
 * @retval 函数执行结果或计算得到的返回值。
 */
int AK8963_CheckDeviceID(void) // 定义AK8963_CheckDeviceID 函数签名：AK8963_CheckDeviceID 函数
{ // 进入当前代码块
    uint8_t device_id = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_WIA); // 定义 device_id 变量，初始值设置为 I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_WIA)

    return (device_id == AK8963_WHO_AM_I_VALUE) ? 0 : -1; // 将 (device_id == AK8963_WHO_AM_I_VALUE) ? 0 : -1 的计算结果 返回给调用者
} // 结束当前代码块

/**
 * @brief AK8963_EnterPowerDownMode 函数。
 * @retval None
 */
void AK8963_EnterPowerDownMode(void) // 定义AK8963_EnterPowerDownMode 函数签名：AK8963_EnterPowerDownMode 函数
{ // 进入当前代码块
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x00U); // 通过软件 I2C 写入指定设备寄存器，参数为 AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x00U
    vTaskDelay(pdMS_TO_TICKS(10)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(10)
} // 结束当前代码块

/**
 * @brief AK8963_EnterFuseROMMode 函数。
 * @retval None
 */
void AK8963_EnterFuseROMMode(void) // 定义AK8963_EnterFuseROMMode 函数签名：AK8963_EnterFuseROMMode 函数
{ // 进入当前代码块
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x0FU); // 通过软件 I2C 写入指定设备寄存器，参数为 AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x0FU
    vTaskDelay(pdMS_TO_TICKS(10)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(10)
} // 结束当前代码块

/**
 * @brief AK8963_AdjustSensitivity 函数。
 * @retval None
 */
void AK8963_AdjustSensitivity(void) // 定义AK8963_AdjustSensitivity 函数签名：AK8963_AdjustSensitivity 函数
{ // 进入当前代码块
    g_ak8963_asa[0] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAX); // 把 I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAX) 写入 g_ak8963_asa[0]
    g_ak8963_asa[1] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAY); // 把 I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAY) 写入 g_ak8963_asa[1]
    g_ak8963_asa[2] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAZ); // 把 I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAZ) 写入 g_ak8963_asa[2]

    g_ak8963_sensitivity[0] = (((float)g_ak8963_asa[0] - 128.0f) * 0.5f / 128.0f) + 1.0f; // 把 (((float)g_ak8963_asa[0] - 128.0f) * 0.5f / 128.0f) + 1.0f 字段值 写入 g_ak8963_sensitivity[0]
    g_ak8963_sensitivity[1] = (((float)g_ak8963_asa[1] - 128.0f) * 0.5f / 128.0f) + 1.0f; // 把 (((float)g_ak8963_asa[1] - 128.0f) * 0.5f / 128.0f) + 1.0f 字段值 写入 g_ak8963_sensitivity[1]
    g_ak8963_sensitivity[2] = (((float)g_ak8963_asa[2] - 128.0f) * 0.5f / 128.0f) + 1.0f; // 把 (((float)g_ak8963_asa[2] - 128.0f) * 0.5f / 128.0f) + 1.0f 字段值 写入 g_ak8963_sensitivity[2]
} // 结束当前代码块

/**
 * @brief AK8963_EnterContinuousMeasurementMode 函数。
 * @retval None
 */
void AK8963_EnterContinuousMeasurementMode(void) // 定义AK8963_EnterContinuousMeasurementMode 函数签名：AK8963_EnterContinuousMeasurementMode 函数
{ // 进入当前代码块
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x16U); // 通过软件 I2C 写入指定设备寄存器，参数为 AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x16U
    vTaskDelay(pdMS_TO_TICKS(10)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(10)
} // 结束当前代码块

/**
 * @brief AK8963_CheckDataReady 函数。
 * @retval 函数执行结果或计算得到的返回值。
 */
int AK8963_CheckDataReady(void) // 定义AK8963_CheckDataReady 函数签名：AK8963_CheckDataReady 函数
{ // 进入当前代码块
    uint8_t status = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ST1); // 定义 status 变量，初始值设置为 I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ST1)

    return ((status & 0x01U) != 0U) ? 1 : 0; // 将 ((status & 0x01U) != 0U) ? 1 : 0 返回给调用者
} // 结束当前代码块

/**
 * @brief 读取 AK8963 三轴磁力计原始值。
 * @param raw 传感器原始采样数据结构体。
 * @retval 函数执行结果或计算得到的返回值。
 */
int AK8963_Read_Axis(AK8963_raw_Data *raw) // 定义AK8963_Read_Axis 函数签名：读取 AK8963 三轴磁力计原始值
{ // 进入当前代码块
    uint8_t frame[7]; // 声明 frame 变量，供后续计算、状态保存或模块间传递使用

    if (raw == 0) // 判断 raw == 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return -2; // 将 -2 的计算结果 返回给调用者
    } // 结束当前代码块

    if (AK8963_CheckDataReady() == 0) // 判断 AK8963_CheckDataReady() == 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return -1; // 将 -1 的计算结果 返回给调用者
    } // 结束当前代码块

    /* Read HXL..ST2 in one burst so the AK8963 releases the latched frame cleanly. */
    if (I2C_ReadRegs(AK8963_I2C_ADDR7, AK8963_REG_HXL, frame, (uint16_t)sizeof(frame)) != 0) // 判断 I2C_ReadRegs(AK8963_I2C_ADDR7, AK8963_REG_HXL, frame, (uint16_t)sizeof(frame)) != 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return -2; // 将 -2 的计算结果 返回给调用者
    } // 结束当前代码块

    raw->mag_x = (int16_t)(((uint16_t)frame[1] << 8U) | (uint16_t)frame[0]); // 把 (int16_t)(((uint16_t)frame[1] << 8U) | (uint16_t)frame[0]) 写入 raw->mag_x 字段值
    raw->mag_y = (int16_t)(((uint16_t)frame[3] << 8U) | (uint16_t)frame[2]); // 把 (int16_t)(((uint16_t)frame[3] << 8U) | (uint16_t)frame[2]) 写入 raw->mag_y 字段值
    raw->mag_z = (int16_t)(((uint16_t)frame[5] << 8U) | (uint16_t)frame[4]); // 把 (int16_t)(((uint16_t)frame[5] << 8U) | (uint16_t)frame[4]) 写入 raw->mag_z 字段值

    if ((frame[6] & 0x08U) != 0U) // 判断 (frame[6] & 0x08U) != 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return -3; // 将 -3 的计算结果 返回给调用者
    } // 结束当前代码块

    return 0; // 将 0 返回给调用者
} // 结束当前代码块

/**
 * @brief 将 AK8963 原始磁场值转换并应用校准。
 * @param raw 传感器原始采样数据结构体。
 * @param physical MPU9250 六轴物理量输出结构体。
 * @retval None
 */
void AK8963_Calibrate(const AK8963_raw_Data *raw, AK8963_Physical_Data *physical) // 定义AK8963_Calibrate 函数签名：将 AK8963 原始磁场值转换并应用校准
{ // 进入当前代码块
    ak8963_convert_raw_to_ut(raw, physical); // 调用ak8963_convert_raw_to_ut 函数，参数为 raw, physical
    ak8963_apply_mag_calibration(physical); // 调用ak8963_apply_mag_calibration 函数，参数为 physical
} // 结束当前代码块

/**
 * @brief 读取并校准 AK8963 磁力计物理量。
 * @param mag_out mag_out 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
int AK8963_Read_Mag_UT(AK8963_Physical_Data *mag_out) // 定义AK8963_Read_Mag_UT 函数签名：读取并校准 AK8963 磁力计物理量
{ // 进入当前代码块
    AK8963_raw_Data raw; // 声明 传感器原始采样数据结构体，供后续计算、状态保存或模块间传递使用
    int ret; // 声明 函数返回状态变量，供后续计算、状态保存或模块间传递使用

    if (mag_out == 0) // 判断 mag_out == 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return -2; // 将 -2 的计算结果 返回给调用者
    } // 结束当前代码块

    ret = AK8963_Read_Axis(&raw); // 把 AK8963_Read_Axis(&raw) 写入 函数返回状态变量
    if (ret != 0) // 判断 ret != 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return ret; // 将 函数返回状态变量 返回给调用者
    } // 结束当前代码块

    AK8963_Calibrate(&raw, mag_out); // 调用将 AK8963 原始磁场值转换并应用校准，参数为 &raw, mag_out
    return 0; // 将 0 返回给调用者
} // 结束当前代码块

/**
 * @brief MPU9250_CalibrateGyro 函数。
 * @param samples 校准采样次数。
 * @param delay_ms 采样间隔毫秒数。
 * @retval None
 */
void MPU9250_CalibrateGyro(uint16_t samples, uint16_t delay_ms) // 定义MPU9250_CalibrateGyro 函数签名：MPU9250_CalibrateGyro 函数
{ // 进入当前代码块
    MPU9250_raw_Data raw; // 声明 传感器原始采样数据结构体，供后续计算、状态保存或模块间传递使用
    int32_t gyro_x_sum = 0; // 定义 gyro_x_sum 变量，初始值设置为 0
    int32_t gyro_y_sum = 0; // 定义 gyro_y_sum 变量，初始值设置为 0
    int32_t gyro_z_sum = 0; // 定义 gyro_z_sum 变量，初始值设置为 0
    uint16_t i; // 声明 循环索引，供后续计算、状态保存或模块间传递使用

    if (samples == 0U) // 判断 samples == 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    for (i = 0U; i < samples; i++) // 按照 i = 0U; i < samples; i++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        MPU9250_ReadAxis(&raw); // 调用读取 MPU9250 加速度计、陀螺仪和温度原始寄存器，参数为 &raw
        gyro_x_sum += raw.gyro_x; // 把 raw.gyro_x 字段值 累加到 gyro_x_sum 变量
        gyro_y_sum += raw.gyro_y; // 把 raw.gyro_y 字段值 累加到 gyro_y_sum 变量
        gyro_z_sum += raw.gyro_z; // 把 raw.gyro_z 字段值 累加到 gyro_z_sum 变量
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(delay_ms)
    } // 结束当前代码块

    g_gyro_bias_dps[0] = ((float)gyro_x_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS; // 把 ((float)gyro_x_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS 的计算结果 写入 g_gyro_bias_dps[0]
    g_gyro_bias_dps[1] = ((float)gyro_y_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS; // 把 ((float)gyro_y_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS 的计算结果 写入 g_gyro_bias_dps[1]
    g_gyro_bias_dps[2] = ((float)gyro_z_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS; // 把 ((float)gyro_z_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS 的计算结果 写入 g_gyro_bias_dps[2]
} // 结束当前代码块

/**
 * @brief MPU9250_CalibrateAccel 函数。
 * @param samples 校准采样次数。
 * @param delay_ms 采样间隔毫秒数。
 * @retval None
 */
void MPU9250_CalibrateAccel(uint16_t samples, uint16_t delay_ms) // 定义MPU9250_CalibrateAccel 函数签名：MPU9250_CalibrateAccel 函数
{ // 进入当前代码块
    MPU9250_raw_Data raw; // 声明 传感器原始采样数据结构体，供后续计算、状态保存或模块间传递使用
    float ax_sum = 0.0f; // 定义 ax_sum 变量，初始值设置为 0.0f 字段值
    float ay_sum = 0.0f; // 定义 ay_sum 变量，初始值设置为 0.0f 字段值
    float az_sum = 0.0f; // 定义 az_sum 变量，初始值设置为 0.0f 字段值
    uint16_t i; // 声明 循环索引，供后续计算、状态保存或模块间传递使用

    if (samples == 0U) // 判断 samples == 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    for (i = 0U; i < samples; i++) // 按照 i = 0U; i < samples; i++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        MPU9250_ReadAxis(&raw); // 调用读取 MPU9250 加速度计、陀螺仪和温度原始寄存器，参数为 &raw
        ax_sum += (float)raw.accel_x / MPU9250_ACCEL_LSB_PER_G; // 把 (float)raw.accel_x / MPU9250_ACCEL_LSB_PER_G 字段值 累加到 ax_sum 变量
        ay_sum += (float)raw.accel_y / MPU9250_ACCEL_LSB_PER_G; // 把 (float)raw.accel_y / MPU9250_ACCEL_LSB_PER_G 字段值 累加到 ay_sum 变量
        az_sum += (float)raw.accel_z / MPU9250_ACCEL_LSB_PER_G; // 把 (float)raw.accel_z / MPU9250_ACCEL_LSB_PER_G 字段值 累加到 az_sum 变量
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(delay_ms)
    } // 结束当前代码块

    g_accel_bias_g[0] = ax_sum / (float)samples; // 把 ax_sum / (float)samples 的计算结果 写入 g_accel_bias_g[0]
    g_accel_bias_g[1] = ay_sum / (float)samples; // 把 ay_sum / (float)samples 的计算结果 写入 g_accel_bias_g[1]
    g_accel_bias_g[2] = (az_sum / (float)samples) - 1.0f; // 把 (az_sum / (float)samples) - 1.0f 字段值 写入 g_accel_bias_g[2]
} // 结束当前代码块

/**
 * @brief 采集磁力计样本并计算硬铁和软铁校准参数。
 * @param samples 校准采样次数。
 * @param delay_ms 采样间隔毫秒数。
 * @retval None
 */
void AK8963_CalibrateMag(uint16_t samples, uint16_t delay_ms) // 定义AK8963_CalibrateMag 函数签名：采集磁力计样本并计算硬铁和软铁校准参数
{ // 进入当前代码块
    AK8963_raw_Data raw; // 声明 传感器原始采样数据结构体，供后续计算、状态保存或模块间传递使用
    AK8963_Physical_Data mag; // 声明 AK8963 磁力计物理量数据，供后续计算、状态保存或模块间传递使用
    float min_x = 99999.0f; // 定义 min_x 变量，初始值设置为 99999.0f 字段值
    float min_y = 99999.0f; // 定义 min_y 变量，初始值设置为 99999.0f 字段值
    float min_z = 99999.0f; // 定义 min_z 变量，初始值设置为 99999.0f 字段值
    float max_x = -99999.0f; // 定义 max_x 变量，初始值设置为 -99999.0f 字段值
    float max_y = -99999.0f; // 定义 max_y 变量，初始值设置为 -99999.0f 字段值
    float max_z = -99999.0f; // 定义 max_z 变量，初始值设置为 -99999.0f 字段值
    float radius_x; // 声明 radius_x 变量，供后续计算、状态保存或模块间传递使用
    float radius_y; // 声明 radius_y 变量，供后续计算、状态保存或模块间传递使用
    float radius_z; // 声明 radius_z 变量，供后续计算、状态保存或模块间传递使用
    float seed_center_x; // 声明 seed_center_x 变量，供后续计算、状态保存或模块间传递使用
    float seed_center_y; // 声明 seed_center_y 变量，供后续计算、状态保存或模块间传递使用
    float seed_center_z; // 声明 seed_center_z 变量，供后续计算、状态保存或模块间传递使用
    float radius_avg; // 声明 radius_avg 变量，供后续计算、状态保存或模块间传递使用
    float fit_params[AK8963_MAG_FIT_PARAM_COUNT]; // 声明 fit_params 变量，供后续计算、状态保存或模块间传递使用
    float fit_error = 0.0f; // 定义 fit_error 变量，初始值设置为 0.0f 字段值
    float fitted_radius_ut = 0.0f; // 定义 fitted_radius_ut 变量，初始值设置为 0.0f 字段值
    float candidate_correction[3][3]; // 执行 float candidate_correction[3][3];，完成当前上下文中的具体处理
    uint16_t valid_count = 0U; // 定义 valid_count 变量，初始值设置为 0
    uint16_t requested_samples = samples; // 定义 requested_samples 变量，初始值设置为 校准采样次数
    uint16_t not_ready_count = 0U; // 定义 not_ready_count 变量，初始值设置为 0
    uint16_t read_fail_count = 0U; // 定义 read_fail_count 变量，初始值设置为 0
    uint16_t overflow_count = 0U; // 定义 overflow_count 变量，初始值设置为 0
    uint16_t i; // 声明 循环索引，供后续计算、状态保存或模块间传递使用
    uint8_t row; // 声明 矩阵行索引，供后续计算、状态保存或模块间传递使用
    uint8_t col; // 声明 矩阵列索引，供后续计算、状态保存或模块间传递使用
    int read_ret; // 声明 read_ret 变量，供后续计算、状态保存或模块间传递使用

    if (samples == 0U) // 判断 samples == 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    if (samples > AK8963_MAG_CAL_MAX_SAMPLES) // 判断 samples > AK8963_MAG_CAL_MAX_SAMPLES 是否成立，以选择后续执行路径
    { // 进入当前代码块
        samples = AK8963_MAG_CAL_MAX_SAMPLES; // 把 AK8963_MAG_CAL_MAX_SAMPLES 变量 写入 校准采样次数
        Debug_Printf("[AK8963] mag calibration samples limited %u->%u\r\n", // 调用格式化并通过调试串口输出调试信息，参数为 "[AK8963] mag calibration samples limited %u->%u\r\n",
                     requested_samples, // 继续传入 requested_samples 变量，作为当前多行调用或初始化列表的一项
                     samples); // 执行 samples);，完成当前上下文中的具体处理
    } // 结束当前代码块

    Debug_Print("[AK8963] mag calibration preparing\r\n"); // 调用通过调试串口输出字符串，参数为 "[AK8963] mag calibration preparing\r\n"
    Debug_Print("[AK8963] get ready to rotate slowly through all 3 axes\r\n"); // 调用通过调试串口输出字符串，参数为 "[AK8963] get ready to rotate slowly through all 3 axes\r\n"
    Debug_Print("[AK8963] start in 3...\r\n"); // 调用通过调试串口输出字符串，参数为 "[AK8963] start in 3...\r\n"
    vTaskDelay(pdMS_TO_TICKS(1000)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(1000)
    Debug_Print("[AK8963] start in 2...\r\n"); // 调用通过调试串口输出字符串，参数为 "[AK8963] start in 2...\r\n"
    vTaskDelay(pdMS_TO_TICKS(1000)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(1000)
    Debug_Print("[AK8963] start in 1...\r\n"); // 调用通过调试串口输出字符串，参数为 "[AK8963] start in 1...\r\n"
    vTaskDelay(pdMS_TO_TICKS(1000)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(1000)
    Debug_Print("[AK8963] mag calibration begin,  keep moving\r\n"); // 调用通过调试串口输出字符串，参数为 "[AK8963] mag calibration begin,  keep moving\r\n"

    for (i = 0U; i < samples; i++) // 按照 i = 0U; i < samples; i++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        read_ret = AK8963_Read_Axis(&raw); // 把 AK8963_Read_Axis(&raw) 写入 read_ret 变量
        if (read_ret == 0) // 判断 read_ret == 0 是否成立，以选择后续执行路径
        { // 进入当前代码块
            ak8963_convert_raw_to_ut(&raw, &mag); // 调用ak8963_convert_raw_to_ut 函数，参数为 &raw, &mag

#if (APP_MAG_POINT_CLOUD_DEBUG_ENABLE != 0U) // 根据芯片型号或编译选项选择参与编译的代码
            if ((APP_MAG_POINT_CLOUD_PRINT_DIV == 0U) || // 调用if 函数，参数为 (APP_MAG_POINT_CLOUD_PRINT_DIV == 0U) ||
                ((valid_count % APP_MAG_POINT_CLOUD_PRINT_DIV) == 0U)) // 执行 ((valid_count % APP_MAG_POINT_CLOUD_PRINT_DIV) == 0U))，完成当前上下文中的具体处理
            { // 进入当前代码块
                Debug_Printf("[AK8963_RAW_UT] %.2f,%.2f,%.2f\r\n", // 调用格式化并通过调试串口输出调试信息，参数为 "[AK8963_RAW_UT] %.2f,%.2f,%.2f\r\n",
                             mag.mag_x_ut, // 继续传入 mag.mag_x_ut 字段值，作为当前多行调用或初始化列表的一项
                             mag.mag_y_ut, // 继续传入 mag.mag_y_ut 字段值，作为当前多行调用或初始化列表的一项
                             mag.mag_z_ut); // 执行 mag.mag_z_ut);，完成当前上下文中的具体处理
            } // 结束当前代码块
#endif // 结束当前条件编译或头文件保护范围

            if (valid_count < AK8963_MAG_CAL_MAX_SAMPLES) // 判断 valid_count < AK8963_MAG_CAL_MAX_SAMPLES 是否成立，以选择后续执行路径
            { // 进入当前代码块
                g_mag_cal_samples[valid_count] = mag; // 把 AK8963 磁力计物理量数据 写入 g_mag_cal_samples[valid_count]
            } // 结束当前代码块

            if (mag.mag_x_ut < min_x) // 判断 mag.mag_x_ut < min_x 是否成立，以选择后续执行路径
            { // 进入当前代码块
                min_x = mag.mag_x_ut; // 把 mag.mag_x_ut 字段值 写入 min_x 变量
            } // 结束当前代码块
            if (mag.mag_x_ut > max_x) // 判断 mag.mag_x_ut > max_x 是否成立，以选择后续执行路径
            { // 进入当前代码块
                max_x = mag.mag_x_ut; // 把 mag.mag_x_ut 字段值 写入 max_x 变量
            } // 结束当前代码块
            if (mag.mag_y_ut < min_y) // 判断 mag.mag_y_ut < min_y 是否成立，以选择后续执行路径
            { // 进入当前代码块
                min_y = mag.mag_y_ut; // 把 mag.mag_y_ut 字段值 写入 min_y 变量
            } // 结束当前代码块
            if (mag.mag_y_ut > max_y) // 判断 mag.mag_y_ut > max_y 是否成立，以选择后续执行路径
            { // 进入当前代码块
                max_y = mag.mag_y_ut; // 把 mag.mag_y_ut 字段值 写入 max_y 变量
            } // 结束当前代码块
            if (mag.mag_z_ut < min_z) // 判断 mag.mag_z_ut < min_z 是否成立，以选择后续执行路径
            { // 进入当前代码块
                min_z = mag.mag_z_ut; // 把 mag.mag_z_ut 字段值 写入 min_z 变量
            } // 结束当前代码块
            if (mag.mag_z_ut > max_z) // 判断 mag.mag_z_ut > max_z 是否成立，以选择后续执行路径
            { // 进入当前代码块
                max_z = mag.mag_z_ut; // 把 mag.mag_z_ut 字段值 写入 max_z 变量
            } // 结束当前代码块
            valid_count++; // 将 valid_count 变量 自增 1，用于推进计数或索引
        } // 结束当前代码块
        else if (read_ret == -1) // 当前一条件不成立时，继续判断 read_ret == -1
        { // 进入当前代码块
            not_ready_count++; // 将 not_ready_count 变量 自增 1，用于推进计数或索引
        } // 结束当前代码块
        else if (read_ret == -2) // 当前一条件不成立时，继续判断 read_ret == -2
        { // 进入当前代码块
            read_fail_count++; // 将 read_fail_count 变量 自增 1，用于推进计数或索引
        } // 结束当前代码块
        else if (read_ret == -3) // 当前一条件不成立时，继续判断 read_ret == -3
        { // 进入当前代码块
            overflow_count++; // 将 overflow_count 变量 自增 1，用于推进计数或索引
        } // 结束当前代码块

        if ((((uint16_t)(i + 1U)) % 50U) == 0U) // 判断 (((uint16_t)(i + 1U)) % 50U) == 0U 是否成立，以选择后续执行路径
        { // 进入当前代码块
            Debug_Printf("[AK8963] mag calibration progress %u/%u valid=%u nr=%u io=%u ov=%u\r\n", // 调用格式化并通过调试串口输出调试信息，参数为 "[AK8963] mag calibration progress %u/%u valid=%u nr=%u io=%u ov=%u\r\n",
                         (uint16_t)(i + 1U), // 继续传入 (uint16_t)(i + 1U) 的计算结果，作为当前多行调用或初始化列表的一项
                         samples, // 继续传入 校准采样次数，作为当前多行调用或初始化列表的一项
                         valid_count, // 继续传入 valid_count 变量，作为当前多行调用或初始化列表的一项
                         not_ready_count, // 继续传入 not_ready_count 变量，作为当前多行调用或初始化列表的一项
                         read_fail_count, // 继续传入 read_fail_count 变量，作为当前多行调用或初始化列表的一项
                         overflow_count); // 执行 overflow_count);，完成当前上下文中的具体处理
        } // 结束当前代码块

        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // 让当前任务阻塞指定时间以释放 CPU，参数为 pdMS_TO_TICKS(delay_ms)
    } // 结束当前代码块

    if (valid_count < AK8963_MAG_FIT_MIN_VALID_SAMPLES) // 判断 valid_count < AK8963_MAG_FIT_MIN_VALID_SAMPLES 是否成立，以选择后续执行路径
    { // 进入当前代码块
        Debug_Printf("[AK8963] calibrate mag rejected valid=%u\r\n", valid_count); // 调用格式化并通过调试串口输出调试信息，参数为 "[AK8963] calibrate mag rejected valid=%u\r\n", valid_count
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    radius_x = (max_x - min_x) * 0.5f; // 把 (max_x - min_x) * 0.5f 字段值 写入 radius_x 变量
    radius_y = (max_y - min_y) * 0.5f; // 把 (max_y - min_y) * 0.5f 字段值 写入 radius_y 变量
    radius_z = (max_z - min_z) * 0.5f; // 把 (max_z - min_z) * 0.5f 字段值 写入 radius_z 变量

    seed_center_x = (min_x + max_x) * 0.5f; // 把 (min_x + max_x) * 0.5f 字段值 写入 seed_center_x 变量
    seed_center_y = (min_y + max_y) * 0.5f; // 把 (min_y + max_y) * 0.5f 字段值 写入 seed_center_y 变量
    seed_center_z = (min_z + max_z) * 0.5f; // 把 (min_z + max_z) * 0.5f 字段值 写入 seed_center_z 变量
    radius_avg = (radius_x + radius_y + radius_z) / 3.0f; // 把 (radius_x + radius_y + radius_z) / 3.0f 字段值 写入 radius_avg 变量
    Debug_Printf("[AK8963] raw bounds x=%.1f..%.1f y=%.1f..%.1f z=%.1f..%.1f\r\n", // 调用格式化并通过调试串口输出调试信息，参数为 "[AK8963] raw bounds x=%.1f..%.1f y=%.1f..%.1f z=%.1f..%.1f\r\n",
                 min_x, max_x, min_y, max_y, min_z, max_z); // 执行 min_x, max_x, min_y, max_y, min_z, max_z);，完成当前上下文中的具体处理
    Debug_Printf("[AK8963] seed center=%.1f/%.1f/%.1f radius=%.1f/%.1f/%.1f avg=%.1f\r\n", // 调用格式化并通过调试串口输出调试信息，参数为 "[AK8963] seed center=%.1f/%.1f/%.1f radius=%.1f/%.1f/%.1f avg=%.1f\r\n",
                 seed_center_x, // 继续传入 seed_center_x 变量，作为当前多行调用或初始化列表的一项
                 seed_center_y, // 继续传入 seed_center_y 变量，作为当前多行调用或初始化列表的一项
                 seed_center_z, // 继续传入 seed_center_z 变量，作为当前多行调用或初始化列表的一项
                 radius_x, // 继续传入 radius_x 变量，作为当前多行调用或初始化列表的一项
                 radius_y, // 继续传入 radius_y 变量，作为当前多行调用或初始化列表的一项
                 radius_z, // 继续传入 radius_z 变量，作为当前多行调用或初始化列表的一项
                 radius_avg); // 执行 radius_avg);，完成当前上下文中的具体处理

    if ((radius_x < AK8963_MAG_RADIUS_MIN_UT) || // 调用if 函数，参数为 (radius_x < AK8963_MAG_RADIUS_MIN_UT) ||
        (radius_y < AK8963_MAG_RADIUS_MIN_UT) || // 执行 (radius_y < AK8963_MAG_RADIUS_MIN_UT) ||，完成当前上下文中的具体处理
        (radius_z < AK8963_MAG_RADIUS_MIN_UT)) // 执行 (radius_z < AK8963_MAG_RADIUS_MIN_UT))，完成当前上下文中的具体处理
    { // 进入当前代码块
        Debug_Printf("[AK8963] calibrate mag rejected radius=%.1f/%.1f/%.1f\r\n", // 调用格式化并通过调试串口输出调试信息，参数为 "[AK8963] calibrate mag rejected radius=%.1f/%.1f/%.1f\r\n",
                     radius_x, // 继续传入 radius_x 变量，作为当前多行调用或初始化列表的一项
                     radius_y, // 继续传入 radius_y 变量，作为当前多行调用或初始化列表的一项
                     radius_z); // 执行 radius_z);，完成当前上下文中的具体处理
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    fit_params[0] = (min_x + max_x) * 0.5f; // 把 (min_x + max_x) * 0.5f 字段值 写入 fit_params[0]
    fit_params[1] = (min_y + max_y) * 0.5f; // 把 (min_y + max_y) * 0.5f 字段值 写入 fit_params[1]
    fit_params[2] = (min_z + max_z) * 0.5f; // 把 (min_z + max_z) * 0.5f 字段值 写入 fit_params[2]
    fit_params[3] = 1.0f / (radius_x * radius_x); // 把 1.0f / (radius_x * radius_x) 字段值 写入 fit_params[3]
    fit_params[4] = 0.0f; // 把 0.0f 字段值 写入 fit_params[4]
    fit_params[5] = 0.0f; // 把 0.0f 字段值 写入 fit_params[5]
    fit_params[6] = 1.0f / (radius_y * radius_y); // 把 1.0f / (radius_y * radius_y) 字段值 写入 fit_params[6]
    fit_params[7] = 0.0f; // 把 0.0f 字段值 写入 fit_params[7]
    fit_params[8] = 1.0f / (radius_z * radius_z); // 把 1.0f / (radius_z * radius_z) 字段值 写入 fit_params[8]

    if (ak8963_fit_ellipsoid(valid_count, fit_params, &fit_error) == 0U) // 判断 ak8963_fit_ellipsoid(valid_count, fit_params, &fit_error) == 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        Debug_Print("[AK8963] calibrate mag rejected fit failed\r\n"); // 调用通过调试串口输出字符串，参数为 "[AK8963] calibrate mag rejected fit failed\r\n"
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    if (ak8963_build_softiron_matrix(fit_params, candidate_correction, &fitted_radius_ut) == 0U) // 判断 ak8963_build_softiron_matrix(fit_params, candidate_correction, &fitted_radius_ut) == 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        Debug_Print("[AK8963] calibrate mag rejected invalid ellipsoid\r\n"); // 调用通过调试串口输出字符串，参数为 "[AK8963] calibrate mag rejected invalid ellipsoid\r\n"
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    if ((fit_params[0] < min_x) || (fit_params[0] > max_x) || // 调用if 函数，参数为 (fit_params[0] < min_x) || (fit_params[0] > max_x) ||
        (fit_params[1] < min_y) || (fit_params[1] > max_y) || // 执行 (fit_params[1] < min_y) || (fit_params[1] > max_y) ||，完成当前上下文中的具体处理
        (fit_params[2] < min_z) || (fit_params[2] > max_z) || // 执行 (fit_params[2] < min_z) || (fit_params[2] > max_z) ||，完成当前上下文中的具体处理
        (fitted_radius_ut < (radius_avg * 0.5f)) || // 执行 (fitted_radius_ut < (radius_avg * 0.5f)) ||，完成当前上下文中的具体处理
        (fitted_radius_ut > (radius_avg * 1.5f))) // 执行 (fitted_radius_ut > (radius_avg * 1.5f)))，完成当前上下文中的具体处理
    { // 进入当前代码块
        Debug_Printf("[AK8963] calibrate mag rejected implausible fit center=%.1f/%.1f/%.1f radius=%.1f seed_avg=%.1f\r\n", // 调用格式化并通过调试串口输出调试信息，参数为 "[AK8963] calibrate mag rejected implausible fit center=%.1f/%.1f/%.1f radius=%.1f seed_avg=%.1f\r\n",
                     fit_params[0], // 继续传入 fit_params[0]，作为当前多行调用或初始化列表的一项
                     fit_params[1], // 继续传入 fit_params[1]，作为当前多行调用或初始化列表的一项
                     fit_params[2], // 继续传入 fit_params[2]，作为当前多行调用或初始化列表的一项
                     fitted_radius_ut, // 继续传入 fitted_radius_ut 变量，作为当前多行调用或初始化列表的一项
                     radius_avg); // 执行 radius_avg);，完成当前上下文中的具体处理
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    g_mag_offset_ut[0] = fit_params[0]; // 把 fit_params[0] 写入 g_mag_offset_ut[0]
    g_mag_offset_ut[1] = fit_params[1]; // 把 fit_params[1] 写入 g_mag_offset_ut[1]
    g_mag_offset_ut[2] = fit_params[2]; // 把 fit_params[2] 写入 g_mag_offset_ut[2]
    for (row = 0U; row < 3U; row++) // 按照 row = 0U; row < 3U; row++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        for (col = 0U; col < 3U; col++) // 按照 col = 0U; col < 3U; col++ 的初始化、边界和步进条件重复执行循环体
        { // 进入当前代码块
            g_mag_correction[row][col] = candidate_correction[row][col]; // 把 candidate_correction[row][col] 写入 g_mag_correction[row][col]
        } // 结束当前代码块
    } // 结束当前代码块

    Debug_Printf("[AK8963] calibrate mag ok center=%.1f/%.1f/%.1f radius=%.1f fit=%.4f\r\n", // 调用格式化并通过调试串口输出调试信息，参数为 "[AK8963] calibrate mag ok center=%.1f/%.1f/%.1f radius=%.1f fit=%.4f\r\n",
                 g_mag_offset_ut[0], // 继续传入 g_mag_offset_ut[0]，作为当前多行调用或初始化列表的一项
                 g_mag_offset_ut[1], // 继续传入 g_mag_offset_ut[1]，作为当前多行调用或初始化列表的一项
                 g_mag_offset_ut[2], // 继续传入 g_mag_offset_ut[2]，作为当前多行调用或初始化列表的一项
                 fitted_radius_ut, // 继续传入 fitted_radius_ut 变量，作为当前多行调用或初始化列表的一项
                 fit_error); // 执行 fit_error);，完成当前上下文中的具体处理
    Debug_Printf("[AK8963] mag matrix row0=%.3f,%.3f,%.3f\r\n", // 调用格式化并通过调试串口输出调试信息，参数为 "[AK8963] mag matrix row0=%.3f,%.3f,%.3f\r\n",
                 g_mag_correction[0][0], // 继续传入 g_mag_correction[0][0]，作为当前多行调用或初始化列表的一项
                 g_mag_correction[0][1], // 继续传入 g_mag_correction[0][1]，作为当前多行调用或初始化列表的一项
                 g_mag_correction[0][2]); // 执行 g_mag_correction[0][2]);，完成当前上下文中的具体处理
    Debug_Printf("[AK8963] mag matrix row1=%.3f,%.3f,%.3f\r\n", // 调用格式化并通过调试串口输出调试信息，参数为 "[AK8963] mag matrix row1=%.3f,%.3f,%.3f\r\n",
                 g_mag_correction[1][0], // 继续传入 g_mag_correction[1][0]，作为当前多行调用或初始化列表的一项
                 g_mag_correction[1][1], // 继续传入 g_mag_correction[1][1]，作为当前多行调用或初始化列表的一项
                 g_mag_correction[1][2]); // 执行 g_mag_correction[1][2]);，完成当前上下文中的具体处理
    Debug_Printf("[AK8963] mag matrix row2=%.3f,%.3f,%.3f\r\n", // 调用格式化并通过调试串口输出调试信息，参数为 "[AK8963] mag matrix row2=%.3f,%.3f,%.3f\r\n",
                 g_mag_correction[2][0], // 继续传入 g_mag_correction[2][0]，作为当前多行调用或初始化列表的一项
                 g_mag_correction[2][1], // 继续传入 g_mag_correction[2][1]，作为当前多行调用或初始化列表的一项
                 g_mag_correction[2][2]); // 执行 g_mag_correction[2][2]);，完成当前上下文中的具体处理
    Debug_Print("[AK8963] mag calibration done\r\n"); // 调用通过调试串口输出字符串，参数为 "[AK8963] mag calibration done\r\n"
} // 结束当前代码块

/**
 * @brief 初始化 MPU9250、AK8963，并完成启动阶段校准。
 *
 * 主要做五件事：
 * 1. 打印驱动初始化开始日志；
 * 2. 初始化软件 I2C；
 * 3. 检查 MPU9250 并配置六轴参数；
 * 4. 初始化 AK8963 磁力计；
 * 5. 执行陀螺仪、加速度计、磁力计校准。
 *
 * @retval 1 初始化成功；0 初始化失败。
 */
uint8_t MPU9250_Driver_Init(void)
{
    /* 2. 初始化软件 I2C */
    BSP_I2C_Soft_Init(); // 初始化软件 I2C 总线

    /* 3. 检查 MPU9250 并配置六轴 */
    if (mpu9250_check_device() != 0) // 检查 MPU9250 设备 ID
    {
        Debug_Print("[MPU9250] driver init failed\r\n"); // 输出 MPU9250 检查失败日志
        return 0U; // MPU9250 检查失败
    }

    mpu9250_config_six_axis(); // 配置加速度计和陀螺仪参数

    /* 4. 初始化 AK8963 磁力计 */
    if (ak8963_init() == 0U) // 初始化磁力计并读取灵敏度参数
    {
        return 0U; // 磁力计初始化失败
    }

    /* 5. 启动阶段校准 */
    MPU9250_CalibrateGyro(1000U, 10U); // 校准陀螺仪零偏
    MPU9250_CalibrateAccel(1000U, 10U); // 校准加速度计零偏
    AK8963_CalibrateMag(500U, 20U); // 校准磁力计硬铁和软铁误差
    Debug_Print("[MPU9250] driver init ok\r\n"); // 输出驱动初始化成功日志

    return 1U; // 驱动初始化成功
}

/*=================================================================================姿态解算与融合=================================================================================*/

//EulerAngle_t g_euler_acc_mag = {0.0f, 0.0f, 0.0f}; // 定义 g_euler_acc_mag 全局状态变量，初始值设置为 {0.0f, 0.0f, 0.0f} 字段值
EulerAngle_t g_euler_fused = {0.0f, 0.0f, 0.0f}; // 定义 g_euler_fused 全局状态变量，初始值设置为 {0.0f, 0.0f, 0.0f} 字段值

static Quaternion_t g_q = {1.0f, 0.0f, 0.0f, 0.0f}; // 定义 g_q 全局状态变量，初始值设置为 {1.0f, 0.0f, 0.0f, 0.0f} 字段值
static float g_exInt = 0.0f; // 定义 g_exInt 全局状态变量，初始值设置为 0.0f 字段值
static float g_eyInt = 0.0f; // 定义 g_eyInt 全局状态变量，初始值设置为 0.0f 字段值
static float g_ezInt = 0.0f; // 定义 g_ezInt 全局状态变量，初始值设置为 0.0f 字段值
static float g_kp = 0.3f; // 定义 g_kp 全局状态变量，初始值设置为 0.3f 字段值
static float g_ki = 0.0f; // 定义 g_ki 全局状态变量，初始值设置为 0.0f 字段值
static float g_acc_norm_prev = 1.0f; // 定义 g_acc_norm_prev 全局状态变量，初始值设置为 1.0f 字段值

/**
 * @brief mpu9250_clampf 函数。
 * @param value value 变量。
 * @param min_value min_value 变量。
 * @param max_value max_value 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
static float mpu9250_clampf(float value, float min_value, float max_value) // 定义mpu9250_clampf 函数签名：mpu9250_clampf 函数
{ // 进入当前代码块
    if (value < min_value) // 判断 value < min_value 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return min_value; // 将 min_value 变量 返回给调用者
    } // 结束当前代码块
    if (value > max_value) // 判断 value > max_value 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return max_value; // 将 max_value 变量 返回给调用者
    } // 结束当前代码块
    return value; // 将 value 变量 返回给调用者
} // 结束当前代码块

/**
 * @brief mpu9250_wrap_angle_deg 函数。
 * @param angle_deg angle_deg 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
static float mpu9250_wrap_angle_deg(float angle_deg) // 定义mpu9250_wrap_angle_deg 函数签名：mpu9250_wrap_angle_deg 函数
{ // 进入当前代码块
    while (angle_deg > 180.0f) // 当 angle_deg > 180.0f 成立时持续执行循环体
    { // 进入当前代码块
        angle_deg -= 360.0f; // 执行 angle_deg -= 360.0f;，完成当前上下文中的具体处理
    } // 结束当前代码块
    while (angle_deg <= -180.0f) // 当 angle_deg <= -180.0f 成立时持续执行循环体
    { // 进入当前代码块
        angle_deg += 360.0f; // 把 360.0f 字段值 累加到 angle_deg 变量
    } // 结束当前代码块
    return angle_deg; // 将 angle_deg 变量 返回给调用者
} // 结束当前代码块

/**
 * @brief mpu9250_math_yaw_to_heading_deg 函数。
 * @param math_yaw_deg math_yaw_deg 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
static float mpu9250_math_yaw_to_heading_deg(float math_yaw_deg) // 定义mpu9250_math_yaw_to_heading_deg 函数签名：mpu9250_math_yaw_to_heading_deg 函数
{ // 进入当前代码块
    /*
     * Internal yaw uses the mathematical convention:
     *   east = 0 deg, counter-clockwise positive.
     * Externally expose the navigation convention used by aircraft:
     *   north = 0 deg, east = +90 deg, range = (-180, 180].
     */
    return mpu9250_wrap_angle_deg(90.0f - math_yaw_deg); // 将 mpu9250_wrap_angle_deg(90.0f - math_yaw_deg) 字段值 返回给调用者
} // 结束当前代码块

/**
 * @brief mpu9250_update_euler_from_quaternion 函数。
 * @retval None
 */
static void mpu9250_update_euler_from_quaternion(void) // 定义mpu9250_update_euler_from_quaternion 函数签名：mpu9250_update_euler_from_quaternion 函数
{ // 进入当前代码块
    float q0 = g_q.q0; // 定义 四元数标量分量，初始值设置为 g_q.q0 字段值
    float q1 = g_q.q1; // 定义 四元数 X 分量，初始值设置为 g_q.q1 字段值
    float q2 = g_q.q2; // 定义 四元数 Y 分量，初始值设置为 g_q.q2 字段值
    float q3 = g_q.q3; // 定义 四元数 Z 分量，初始值设置为 g_q.q3 字段值
    float pitch_sin; // 声明 pitch_sin 变量，供后续计算、状态保存或模块间传递使用

    g_euler_fused.roll = atan2f(2.0f * (q0 * q1 + q2 * q3), // 继续传入 g_euler_fused.roll = atan2f(2.0f * (q0 * q1 + q2 * q3) 字段值，作为当前多行调用或初始化列表的一项
                                1.0f - 2.0f * (q1 * q1 + q2 * q2)); // 执行 1.0f - 2.0f * (q1 * q1 + q2 * q2));，完成当前上下文中的具体处理

    pitch_sin = 2.0f * (q0 * q2 - q3 * q1); // 把 2.0f * (q0 * q2 - q3 * q1) 字段值 写入 pitch_sin 变量
    pitch_sin = mpu9250_clampf(pitch_sin, -1.0f, 1.0f); // 把 mpu9250_clampf(pitch_sin, -1.0f, 1.0f) 字段值 写入 pitch_sin 变量
    g_euler_fused.pitch = asinf(pitch_sin); // 把 asinf(pitch_sin) 写入 g_euler_fused.pitch 字段值

    g_euler_fused.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), // 继续传入 g_euler_fused.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2) 字段值，作为当前多行调用或初始化列表的一项
                               1.0f - 2.0f * (q2 * q2 + q3 * q3)); // 执行 1.0f - 2.0f * (q2 * q2 + q3 * q3));，完成当前上下文中的具体处理
} // 结束当前代码块

/**
 * @brief mpu9250_integrate_quaternion 函数。
 * @param gx X 轴角速度弧度值。
 * @param gy Y 轴角速度弧度值。
 * @param gz Z 轴角速度弧度值。
 * @param dt 姿态融合采样周期秒数。
 * @retval None
 */
static void mpu9250_integrate_quaternion(float gx, float gy, float gz, float dt) // 定义mpu9250_integrate_quaternion 函数签名：mpu9250_integrate_quaternion 函数
{ // 进入当前代码块
    float q0 = g_q.q0; // 定义 四元数标量分量，初始值设置为 g_q.q0 字段值
    float q1 = g_q.q1; // 定义 四元数 X 分量，初始值设置为 g_q.q1 字段值
    float q2 = g_q.q2; // 定义 四元数 Y 分量，初始值设置为 g_q.q2 字段值
    float q3 = g_q.q3; // 定义 四元数 Z 分量，初始值设置为 g_q.q3 字段值
    float qDot0; // 声明 qDot0 变量，供后续计算、状态保存或模块间传递使用
    float qDot1; // 声明 qDot1 变量，供后续计算、状态保存或模块间传递使用
    float qDot2; // 声明 qDot2 变量，供后续计算、状态保存或模块间传递使用
    float qDot3; // 声明 qDot3 变量，供后续计算、状态保存或模块间传递使用
    float norm_q; // 声明 四元数模长，供后续计算、状态保存或模块间传递使用

    qDot0 = -0.5f * (q1 * gx + q2 * gy + q3 * gz); // 把 -0.5f * (q1 * gx + q2 * gy + q3 * gz) 字段值 写入 qDot0 变量
    qDot1 =  0.5f * (q0 * gx + q2 * gz - q3 * gy); // 把 0.5f * (q0 * gx + q2 * gz - q3 * gy) 字段值 写入 qDot1 变量
    qDot2 =  0.5f * (q0 * gy - q1 * gz + q3 * gx); // 把 0.5f * (q0 * gy - q1 * gz + q3 * gx) 字段值 写入 qDot2 变量
    qDot3 =  0.5f * (q0 * gz + q1 * gy - q2 * gx); // 把 0.5f * (q0 * gz + q1 * gy - q2 * gx) 字段值 写入 qDot3 变量

    q0 += qDot0 * dt; // 把 qDot0 * dt 的计算结果 累加到 四元数标量分量
    q1 += qDot1 * dt; // 把 qDot1 * dt 的计算结果 累加到 四元数 X 分量
    q2 += qDot2 * dt; // 把 qDot2 * dt 的计算结果 累加到 四元数 Y 分量
    q3 += qDot3 * dt; // 把 qDot3 * dt 的计算结果 累加到 四元数 Z 分量

    norm_q = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3); // 把 sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3) 的计算结果 写入 四元数模长
    if (norm_q < 1e-6f) // 判断 norm_q < 1e-6f 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    g_q.q0 = q0 / norm_q; // 把 q0 / norm_q 的计算结果 写入 g_q.q0 字段值
    g_q.q1 = q1 / norm_q; // 把 q1 / norm_q 的计算结果 写入 g_q.q1 字段值
    g_q.q2 = q2 / norm_q; // 把 q2 / norm_q 的计算结果 写入 g_q.q2 字段值
    g_q.q3 = q3 / norm_q; // 把 q3 / norm_q 的计算结果 写入 g_q.q3 字段值

    mpu9250_update_euler_from_quaternion(); // 调用mpu9250_update_euler_from_quaternion 函数
} // 结束当前代码块

/**
 * @brief 使用加速度计和磁力计直接计算欧拉角。
 * @param imu MPU9250 六轴物理量数据。
 * @param mag AK8963 磁力计物理量数据。
 * @retval None
 */
// void MPU9250_ComputeEuler_FromAccMag(const MPU9250_Physical_Data *imu, // 定义MPU9250_ComputeEuler_FromAccMag 函数签名：使用加速度计和磁力计直接计算欧拉角
//                                      const AK8963_Physical_Data *mag) // 定义MPU9250_ComputeEuler_FromAccMag 函数签名：使用加速度计和磁力计直接计算欧拉角
// { // 进入当前代码块
//     float ax; // 声明 X 轴加速度分量，供后续计算、状态保存或模块间传递使用
//     float ay; // 声明 Y 轴加速度分量，供后续计算、状态保存或模块间传递使用
//     float az; // 声明 Z 轴加速度分量，供后续计算、状态保存或模块间传递使用
//     float mx; // 声明 X 轴磁场分量，供后续计算、状态保存或模块间传递使用
//     float my; // 声明 Y 轴磁场分量，供后续计算、状态保存或模块间传递使用
//     float mz; // 声明 Z 轴磁场分量，供后续计算、状态保存或模块间传递使用
//     float norm; // 声明 norm 变量，供后续计算、状态保存或模块间传递使用
//     float roll; // 声明 roll 变量，供后续计算、状态保存或模块间传递使用
//     float pitch; // 声明 pitch 变量，供后续计算、状态保存或模块间传递使用
//     float sinRoll; // 声明 sinRoll 变量，供后续计算、状态保存或模块间传递使用
//     float cosRoll; // 声明 cosRoll 变量，供后续计算、状态保存或模块间传递使用
//     float sinPitch; // 声明 sinPitch 变量，供后续计算、状态保存或模块间传递使用
//     float cosPitch; // 声明 cosPitch 变量，供后续计算、状态保存或模块间传递使用
//     float mx2; // 声明 mx2 变量，供后续计算、状态保存或模块间传递使用
//     float my2; // 声明 my2 变量，供后续计算、状态保存或模块间传递使用

//     if ((imu == 0) || (mag == 0)) // 判断 (imu == 0) || (mag == 0) 是否成立，以选择后续执行路径
//     { // 进入当前代码块
//         return; // 当前条件不满足继续处理，直接返回调用者
//     } // 结束当前代码块

//     ax =  imu->accel_x_g; // 把 imu->accel_x_g 字段值 写入 X 轴加速度分量
//     ay = -imu->accel_y_g; // 把 -imu->accel_y_g 字段值 写入 Y 轴加速度分量
//     az =  imu->accel_z_g; // 把 imu->accel_z_g 字段值 写入 Z 轴加速度分量

//     mx =  mag->mag_x_ut; // 把 mag->mag_x_ut 字段值 写入 X 轴磁场分量
//     my = -mag->mag_y_ut; // 把 -mag->mag_y_ut 字段值 写入 Y 轴磁场分量
//     mz = -mag->mag_z_ut; // 把 -mag->mag_z_ut 字段值 写入 Z 轴磁场分量

//     norm = sqrtf(ax * ax + ay * ay + az * az); // 把 sqrtf(ax * ax + ay * ay + az * az) 的计算结果 写入 norm 变量
//     if (norm < 1e-6f) // 判断 norm < 1e-6f 是否成立，以选择后续执行路径
//     { // 进入当前代码块
//         return; // 当前条件不满足继续处理，直接返回调用者
//     } // 结束当前代码块
//     ax /= norm; // 执行 ax /= norm;，完成当前上下文中的具体处理
//     ay /= norm; // 执行 ay /= norm;，完成当前上下文中的具体处理
//     az /= norm; // 执行 az /= norm;，完成当前上下文中的具体处理

//     roll = atan2f(ay, az); // 把 atan2f(ay, az) 写入 roll 变量
//     pitch = atan2f(-ax, sqrtf(ay * ay + az * az)); // 把 atan2f(-ax, sqrtf(ay * ay + az * az)) 的计算结果 写入 pitch 变量

//     norm = sqrtf(mx * mx + my * my + mz * mz); // 把 sqrtf(mx * mx + my * my + mz * mz) 的计算结果 写入 norm 变量
//     if (norm < 1e-6f) // 判断 norm < 1e-6f 是否成立，以选择后续执行路径
//     { // 进入当前代码块
//         g_euler_acc_mag.roll = roll; // 把 roll 变量 写入 g_euler_acc_mag.roll 字段值
//         g_euler_acc_mag.pitch = pitch; // 把 pitch 变量 写入 g_euler_acc_mag.pitch 字段值
//         return; // 当前条件不满足继续处理，直接返回调用者
//     } // 结束当前代码块
//     mx /= norm; // 执行 mx /= norm;，完成当前上下文中的具体处理
//     my /= norm; // 执行 my /= norm;，完成当前上下文中的具体处理
//     mz /= norm; // 执行 mz /= norm;，完成当前上下文中的具体处理

//     sinRoll = sinf(roll); // 把 sinf(roll) 写入 sinRoll 变量
//     cosRoll = cosf(roll); // 把 cosf(roll) 写入 cosRoll 变量
//     sinPitch = sinf(pitch); // 把 sinf(pitch) 写入 sinPitch 变量
//     cosPitch = cosf(pitch); // 把 cosf(pitch) 写入 cosPitch 变量

//     mx2 = mx * cosPitch + mz * sinPitch; // 把 mx * cosPitch + mz * sinPitch 的计算结果 写入 mx2 变量
//     my2 = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch; // 把 mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch 的计算结果 写入 my2 变量

//     g_euler_acc_mag.roll = roll; // 把 roll 变量 写入 g_euler_acc_mag.roll 字段值
//     g_euler_acc_mag.pitch = pitch; // 把 pitch 变量 写入 g_euler_acc_mag.pitch 字段值
//     g_euler_acc_mag.yaw = atan2f(-my2, mx2); // 把 atan2f(-my2, mx2) 的计算结果 写入 g_euler_acc_mag.yaw 字段值
// } // 结束当前代码块

/**
 * @brief 读取加速度计磁力计解算的欧拉角角度值。
 * @param roll_deg 横滚角角度值。
 * @param pitch_deg 俯仰角角度值。
 * @param yaw_deg 航向角角度值。
 * @retval None
 */
// void MPU9250_GetEulerDeg(float *roll_deg, float *pitch_deg, float *yaw_deg) // 定义MPU9250_GetEulerDeg 函数签名：读取加速度计磁力计解算的欧拉角角度值
// { // 进入当前代码块
//     const float rad2deg = 57.295779513f; // 定义 rad2deg 变量，初始值设置为 57.295779513f 字段值

//     if (roll_deg != 0) // 判断 roll_deg != 0 是否成立，以选择后续执行路径
//     { // 进入当前代码块
//         *roll_deg = g_euler_acc_mag.roll * rad2deg;
//     } // 结束当前代码块
//     if (pitch_deg != 0) // 判断 pitch_deg != 0 是否成立，以选择后续执行路径
//     { // 进入当前代码块
//         *pitch_deg = g_euler_acc_mag.pitch * rad2deg;
//     } // 结束当前代码块
//     if (yaw_deg != 0) // 判断 yaw_deg != 0 是否成立，以选择后续执行路径
//     { // 进入当前代码块
//         *yaw_deg = mpu9250_math_yaw_to_heading_deg(g_euler_acc_mag.yaw * rad2deg);
//     } // 结束当前代码块
// } // 结束当前代码块

/**
 * @brief 初始化 Mahony 姿态融合四元数和误差积分项。
 * @param kp Mahony 比例修正增益。
 * @param ki Mahony 积分修正增益。
 * @retval None
 */
void MPU9250_MahonyInit(float kp, float ki) // 定义MPU9250_MahonyInit 函数签名：初始化 Mahony 姿态融合四元数和误差积分项
{ // 进入当前代码块
    g_kp = kp; // 把 Mahony 比例修正增益 写入 g_kp 全局状态变量
    g_ki = ki; // 把 Mahony 积分修正增益 写入 g_ki 全局状态变量

    g_q.q0 = 1.0f; // 把 1.0f 字段值 写入 g_q.q0 字段值
    g_q.q1 = 0.0f; // 把 0.0f 字段值 写入 g_q.q1 字段值
    g_q.q2 = 0.0f; // 把 0.0f 字段值 写入 g_q.q2 字段值
    g_q.q3 = 0.0f; // 把 0.0f 字段值 写入 g_q.q3 字段值

    g_exInt = 0.0f; // 把 0.0f 字段值 写入 g_exInt 全局状态变量
    g_eyInt = 0.0f; // 把 0.0f 字段值 写入 g_eyInt 全局状态变量
    g_ezInt = 0.0f; // 把 0.0f 字段值 写入 g_ezInt 全局状态变量
    g_acc_norm_prev = 1.0f; // 把 1.0f 字段值 写入 g_acc_norm_prev 全局状态变量

    // g_euler_acc_mag.roll = 0.0f; // 把 0.0f 字段值 写入 g_euler_acc_mag.roll 字段值
    // g_euler_acc_mag.pitch = 0.0f; // 把 0.0f 字段值 写入 g_euler_acc_mag.pitch 字段值
    // g_euler_acc_mag.yaw = 0.0f; // 把 0.0f 字段值 写入 g_euler_acc_mag.yaw 字段值
    g_euler_fused.roll = 0.0f; // 把 0.0f 字段值 写入 g_euler_fused.roll 字段值
    g_euler_fused.pitch = 0.0f; // 把 0.0f 字段值 写入 g_euler_fused.pitch 字段值
    g_euler_fused.yaw = 0.0f; // 把 0.0f 字段值 写入 g_euler_fused.yaw 字段值
} // 结束当前代码块

/**
 * @brief 使用九轴数据执行 Mahony 姿态融合更新。
 * @param imu MPU9250 六轴物理量数据。
 * @param mag AK8963 磁力计物理量数据。
 * @param dt 姿态融合采样周期秒数。
 * @retval None
 */
void MPU9250_MahonyUpdate(const MPU9250_Physical_Data *imu, // 定义MPU9250_MahonyUpdate 函数签名：使用九轴数据执行 Mahony 姿态融合更新
                          const AK8963_Physical_Data *mag, // 定义MPU9250_MahonyUpdate 函数签名：使用九轴数据执行 Mahony 姿态融合更新
                          float dt) // 定义MPU9250_MahonyUpdate 函数签名：使用九轴数据执行 Mahony 姿态融合更新
{ // 进入当前代码块
    const float deg2rad = 0.01745329252f; // 定义 deg2rad 变量，初始值设置为 0.01745329252f 字段值
    const float mag_min_ut = 15.0f; // 定义 mag_min_ut 变量，初始值设置为 15.0f 字段值
    const float mag_max_ut = 100.0f; // 定义 mag_max_ut 变量，初始值设置为 100.0f 字段值
    const float int_lim = 0.1f; // 定义 int_lim 变量，初始值设置为 0.1f 字段值
    float ax; // 声明 X 轴加速度分量，供后续计算、状态保存或模块间传递使用
    float ay; // 声明 Y 轴加速度分量，供后续计算、状态保存或模块间传递使用
    float az; // 声明 Z 轴加速度分量，供后续计算、状态保存或模块间传递使用
    float gx; // 声明 X 轴角速度弧度值，供后续计算、状态保存或模块间传递使用
    float gy; // 声明 Y 轴角速度弧度值，供后续计算、状态保存或模块间传递使用
    float gz; // 声明 Z 轴角速度弧度值，供后续计算、状态保存或模块间传递使用
    float mx; // 声明 X 轴磁场分量，供后续计算、状态保存或模块间传递使用
    float my; // 声明 Y 轴磁场分量，供后续计算、状态保存或模块间传递使用
    float mz; // 声明 Z 轴磁场分量，供后续计算、状态保存或模块间传递使用
    float norm_acc; // 声明 加速度向量模长，供后续计算、状态保存或模块间传递使用
    float norm_mag; // 声明 磁场向量模长，供后续计算、状态保存或模块间传递使用
    float acc_error; // 声明 acc_error 变量，供后续计算、状态保存或模块间传递使用
    float acc_diff; // 声明 acc_diff 变量，供后续计算、状态保存或模块间传递使用
    float kp_dynamic; // 声明 kp_dynamic 变量，供后续计算、状态保存或模块间传递使用
    float q0; // 声明 四元数标量分量，供后续计算、状态保存或模块间传递使用
    float q1; // 声明 四元数 X 分量，供后续计算、状态保存或模块间传递使用
    float q2; // 声明 四元数 Y 分量，供后续计算、状态保存或模块间传递使用
    float q3; // 声明 四元数 Z 分量，供后续计算、状态保存或模块间传递使用
    float vx; // 声明 vx 变量，供后续计算、状态保存或模块间传递使用
    float vy; // 声明 vy 变量，供后续计算、状态保存或模块间传递使用
    float vz; // 声明 vz 变量，供后续计算、状态保存或模块间传递使用
    float ex; // 声明 X 轴姿态误差修正量，供后续计算、状态保存或模块间传递使用
    float ey; // 声明 Y 轴姿态误差修正量，供后续计算、状态保存或模块间传递使用
    float ez; // 声明 Z 轴姿态误差修正量，供后续计算、状态保存或模块间传递使用
    float hx; // 声明 hx 变量，供后续计算、状态保存或模块间传递使用
    float hy; // 声明 hy 变量，供后续计算、状态保存或模块间传递使用
    float hz; // 声明 hz 变量，供后续计算、状态保存或模块间传递使用
    float bx; // 声明 bx 变量，供后续计算、状态保存或模块间传递使用
    float bz; // 声明 bz 变量，供后续计算、状态保存或模块间传递使用
    float wx; // 声明 wx 变量，供后续计算、状态保存或模块间传递使用
    float wy; // 声明 wy 变量，供后续计算、状态保存或模块间传递使用
    float wz; // 声明 wz 变量，供后续计算、状态保存或模块间传递使用
    uint8_t mag_valid; // 声明 mag_valid 变量，供后续计算、状态保存或模块间传递使用

    if ((imu == 0) || (mag == 0) || (dt <= 0.0f) || (dt > 0.2f)) // 判断 (imu == 0) || (mag == 0) || (dt <= 0.0f) || (dt > 0.2f) 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    ax =  imu->accel_x_g; // 把 imu->accel_x_g 字段值 写入 X 轴加速度分量
    ay = -imu->accel_y_g; // 把 -imu->accel_y_g 字段值 写入 Y 轴加速度分量
    az =  imu->accel_z_g; // 把 imu->accel_z_g 字段值 写入 Z 轴加速度分量

    gx =  imu->gyro_x_dps * deg2rad; // 把 imu->gyro_x_dps * deg2rad 字段值 写入 X 轴角速度弧度值
    gy = -imu->gyro_y_dps * deg2rad; // 把 -imu->gyro_y_dps * deg2rad 字段值 写入 Y 轴角速度弧度值
    gz =  imu->gyro_z_dps * deg2rad; // 把 imu->gyro_z_dps * deg2rad 字段值 写入 Z 轴角速度弧度值

    mx =  mag->mag_x_ut; // 把 mag->mag_x_ut 字段值 写入 X 轴磁场分量
    my = -mag->mag_y_ut; // 把 -mag->mag_y_ut 字段值 写入 Y 轴磁场分量
    mz = -mag->mag_z_ut; // 把 -mag->mag_z_ut 字段值 写入 Z 轴磁场分量

    norm_acc = sqrtf(ax * ax + ay * ay + az * az); // 把 sqrtf(ax * ax + ay * ay + az * az) 的计算结果 写入 加速度向量模长
    if (norm_acc < 1e-6f) // 判断 norm_acc < 1e-6f 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    norm_mag = sqrtf(mx * mx + my * my + mz * mz); // 把 sqrtf(mx * mx + my * my + mz * mz) 的计算结果 写入 磁场向量模长
    mag_valid = ((norm_mag > mag_min_ut) && (norm_mag < mag_max_ut)) ? 1U : 0U; // 把 ((norm_mag > mag_min_ut) && (norm_mag < mag_max_ut)) ? 1U : 0U 写入 mag_valid 变量

    acc_error = fabsf(norm_acc - 1.0f); // 把 fabsf(norm_acc - 1.0f) 字段值 写入 acc_error 变量
    acc_diff = fabsf(norm_acc - g_acc_norm_prev); // 把 fabsf(norm_acc - g_acc_norm_prev) 的计算结果 写入 acc_diff 变量
    kp_dynamic = g_kp; // 把 g_kp 全局状态变量 写入 kp_dynamic 变量
    if ((acc_error < 0.04f) && (acc_diff < 0.02f)) // 判断 (acc_error < 0.04f) && (acc_diff < 0.02f) 是否成立，以选择后续执行路径
    { // 进入当前代码块
        kp_dynamic = g_kp * 2.0f; // 把 g_kp * 2.0f 字段值 写入 kp_dynamic 变量
    } // 结束当前代码块
    else if ((acc_error > 0.25f) || (acc_diff > 0.25f)) // 当前一条件不成立时，继续判断 (acc_error > 0.25f) || (acc_diff > 0.25f)
    { // 进入当前代码块
        kp_dynamic = g_kp * 0.35f; // 把 g_kp * 0.35f 字段值 写入 kp_dynamic 变量
    } // 结束当前代码块
    g_acc_norm_prev = norm_acc; // 把 加速度向量模长 写入 g_acc_norm_prev 全局状态变量

    ax /= norm_acc; // 执行 ax /= norm_acc;，完成当前上下文中的具体处理
    ay /= norm_acc; // 执行 ay /= norm_acc;，完成当前上下文中的具体处理
    az /= norm_acc; // 执行 az /= norm_acc;，完成当前上下文中的具体处理

    if (norm_mag > 1e-6f) // 判断 norm_mag > 1e-6f 是否成立，以选择后续执行路径
    { // 进入当前代码块
        mx /= norm_mag; // 执行 mx /= norm_mag;，完成当前上下文中的具体处理
        my /= norm_mag; // 执行 my /= norm_mag;，完成当前上下文中的具体处理
        mz /= norm_mag; // 执行 mz /= norm_mag;，完成当前上下文中的具体处理
    } // 结束当前代码块
    else // 处理前面判断条件不成立时的备用逻辑
    { // 进入当前代码块
        mag_valid = 0U; // 把 0 写入 mag_valid 变量
    } // 结束当前代码块

    q0 = g_q.q0; // 把 g_q.q0 字段值 写入 四元数标量分量
    q1 = g_q.q1; // 把 g_q.q1 字段值 写入 四元数 X 分量
    q2 = g_q.q2; // 把 g_q.q2 字段值 写入 四元数 Y 分量
    q3 = g_q.q3; // 把 g_q.q3 字段值 写入 四元数 Z 分量

    vx = 2.0f * (q1 * q3 - q0 * q2); // 把 2.0f * (q1 * q3 - q0 * q2) 字段值 写入 vx 变量
    vy = 2.0f * (q0 * q1 + q2 * q3); // 把 2.0f * (q0 * q1 + q2 * q3) 字段值 写入 vy 变量
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // 把 q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3 的计算结果 写入 vz 变量

    ex = ay * vz - az * vy; // 把 ay * vz - az * vy 的计算结果 写入 X 轴姿态误差修正量
    ey = az * vx - ax * vz; // 把 az * vx - ax * vz 的计算结果 写入 Y 轴姿态误差修正量
    ez = ax * vy - ay * vx; // 把 ax * vy - ay * vx 的计算结果 写入 Z 轴姿态误差修正量

    if (mag_valid != 0U) // 判断 mag_valid != 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        hx = 2.0f * mx * (0.5f - q2 * q2 - q3 * q3) // 执行 hx = 2.0f * mx * (0.5f - q2 * q2 - q3 * q3)，完成当前上下文中的具体处理
           + 2.0f * my * (q1 * q2 - q0 * q3) // 执行 + 2.0f * my * (q1 * q2 - q0 * q3)，完成当前上下文中的具体处理
           + 2.0f * mz * (q1 * q3 + q0 * q2); // 执行 + 2.0f * mz * (q1 * q3 + q0 * q2);，完成当前上下文中的具体处理
        hy = 2.0f * mx * (q1 * q2 + q0 * q3) // 执行 hy = 2.0f * mx * (q1 * q2 + q0 * q3)，完成当前上下文中的具体处理
           + 2.0f * my * (0.5f - q1 * q1 - q3 * q3) // 执行 + 2.0f * my * (0.5f - q1 * q1 - q3 * q3)，完成当前上下文中的具体处理
           + 2.0f * mz * (q2 * q3 - q0 * q1); // 执行 + 2.0f * mz * (q2 * q3 - q0 * q1);，完成当前上下文中的具体处理
        hz = 2.0f * mx * (q1 * q3 - q0 * q2) // 执行 hz = 2.0f * mx * (q1 * q3 - q0 * q2)，完成当前上下文中的具体处理
           + 2.0f * my * (q2 * q3 + q0 * q1) // 执行 + 2.0f * my * (q2 * q3 + q0 * q1)，完成当前上下文中的具体处理
           + 2.0f * mz * (0.5f - q1 * q1 - q2 * q2); // 执行 + 2.0f * mz * (0.5f - q1 * q1 - q2 * q2);，完成当前上下文中的具体处理

        bx = sqrtf(hx * hx + hy * hy); // 把 sqrtf(hx * hx + hy * hy) 的计算结果 写入 bx 变量
        bz = hz; // 把 hz 变量 写入 bz 变量

        wx = 2.0f * bx * (0.5f - q2 * q2 - q3 * q3) // 执行 wx = 2.0f * bx * (0.5f - q2 * q2 - q3 * q3)，完成当前上下文中的具体处理
           + 2.0f * bz * (q1 * q3 - q0 * q2); // 执行 + 2.0f * bz * (q1 * q3 - q0 * q2);，完成当前上下文中的具体处理
        wy = 2.0f * bx * (q1 * q2 - q0 * q3) // 执行 wy = 2.0f * bx * (q1 * q2 - q0 * q3)，完成当前上下文中的具体处理
           + 2.0f * bz * (q0 * q1 + q2 * q3); // 执行 + 2.0f * bz * (q0 * q1 + q2 * q3);，完成当前上下文中的具体处理
        wz = 2.0f * bx * (q0 * q2 + q1 * q3) // 执行 wz = 2.0f * bx * (q0 * q2 + q1 * q3)，完成当前上下文中的具体处理
           + 2.0f * bz * (0.5f - q1 * q1 - q2 * q2); // 执行 + 2.0f * bz * (0.5f - q1 * q1 - q2 * q2);，完成当前上下文中的具体处理

        ex += my * wz - mz * wy; // 把 my * wz - mz * wy 的计算结果 累加到 X 轴姿态误差修正量
        ey += mz * wx - mx * wz; // 把 mz * wx - mx * wz 的计算结果 累加到 Y 轴姿态误差修正量
        ez += mx * wy - my * wx; // 把 mx * wy - my * wx 的计算结果 累加到 Z 轴姿态误差修正量
    } // 结束当前代码块

    if (g_ki > 0.0f) // 判断 g_ki > 0.0f 是否成立，以选择后续执行路径
    { // 进入当前代码块
        g_exInt += ex * g_ki * dt; // 把 ex * g_ki * dt 的计算结果 累加到 g_exInt 全局状态变量
        g_eyInt += ey * g_ki * dt; // 把 ey * g_ki * dt 的计算结果 累加到 g_eyInt 全局状态变量
        g_ezInt += ez * g_ki * dt; // 把 ez * g_ki * dt 的计算结果 累加到 g_ezInt 全局状态变量

        g_exInt = mpu9250_clampf(g_exInt, -int_lim, int_lim); // 把 mpu9250_clampf(g_exInt, -int_lim, int_lim) 的计算结果 写入 g_exInt 全局状态变量
        g_eyInt = mpu9250_clampf(g_eyInt, -int_lim, int_lim); // 把 mpu9250_clampf(g_eyInt, -int_lim, int_lim) 的计算结果 写入 g_eyInt 全局状态变量
        g_ezInt = mpu9250_clampf(g_ezInt, -int_lim, int_lim); // 把 mpu9250_clampf(g_ezInt, -int_lim, int_lim) 的计算结果 写入 g_ezInt 全局状态变量

        gx += g_exInt; // 把 g_exInt 全局状态变量 累加到 X 轴角速度弧度值
        gy += g_eyInt; // 把 g_eyInt 全局状态变量 累加到 Y 轴角速度弧度值
        gz += g_ezInt; // 把 g_ezInt 全局状态变量 累加到 Z 轴角速度弧度值
    } // 结束当前代码块
    else // 处理前面判断条件不成立时的备用逻辑
    { // 进入当前代码块
        g_exInt = 0.0f; // 把 0.0f 字段值 写入 g_exInt 全局状态变量
        g_eyInt = 0.0f; // 把 0.0f 字段值 写入 g_eyInt 全局状态变量
        g_ezInt = 0.0f; // 把 0.0f 字段值 写入 g_ezInt 全局状态变量
    } // 结束当前代码块

    gx += kp_dynamic * ex; // 把 kp_dynamic * ex 的计算结果 累加到 X 轴角速度弧度值
    gy += kp_dynamic * ey; // 把 kp_dynamic * ey 的计算结果 累加到 Y 轴角速度弧度值
    gz += kp_dynamic * ez; // 把 kp_dynamic * ez 的计算结果 累加到 Z 轴角速度弧度值

    mpu9250_integrate_quaternion(gx, gy, gz, dt); // 调用mpu9250_integrate_quaternion 函数，参数为 gx, gy, gz, dt
} // 结束当前代码块

/**
 * @brief 使用六轴 IMU 数据执行 Mahony 姿态融合更新。
 * @param imu MPU9250 六轴物理量数据。
 * @param dt 姿态融合采样周期秒数。
 * @retval None
 */
void MPU9250_MahonyUpdateIMU(const MPU9250_Physical_Data *imu, float dt) // 定义MPU9250_MahonyUpdateIMU 函数签名：使用六轴 IMU 数据执行 Mahony 姿态融合更新
{ // 进入当前代码块
    const float deg2rad = 0.01745329252f; // 定义 deg2rad 变量，初始值设置为 0.01745329252f 字段值
    const float int_lim = 0.1f; // 定义 int_lim 变量，初始值设置为 0.1f 字段值
    float ax; // 声明 X 轴加速度分量，供后续计算、状态保存或模块间传递使用
    float ay; // 声明 Y 轴加速度分量，供后续计算、状态保存或模块间传递使用
    float az; // 声明 Z 轴加速度分量，供后续计算、状态保存或模块间传递使用
    float gx; // 声明 X 轴角速度弧度值，供后续计算、状态保存或模块间传递使用
    float gy; // 声明 Y 轴角速度弧度值，供后续计算、状态保存或模块间传递使用
    float gz; // 声明 Z 轴角速度弧度值，供后续计算、状态保存或模块间传递使用
    float norm; // 声明 norm 变量，供后续计算、状态保存或模块间传递使用
    float q0; // 声明 四元数标量分量，供后续计算、状态保存或模块间传递使用
    float q1; // 声明 四元数 X 分量，供后续计算、状态保存或模块间传递使用
    float q2; // 声明 四元数 Y 分量，供后续计算、状态保存或模块间传递使用
    float q3; // 声明 四元数 Z 分量，供后续计算、状态保存或模块间传递使用
    float vx; // 声明 vx 变量，供后续计算、状态保存或模块间传递使用
    float vy; // 声明 vy 变量，供后续计算、状态保存或模块间传递使用
    float vz; // 声明 vz 变量，供后续计算、状态保存或模块间传递使用
    float ex; // 声明 X 轴姿态误差修正量，供后续计算、状态保存或模块间传递使用
    float ey; // 声明 Y 轴姿态误差修正量，供后续计算、状态保存或模块间传递使用
    float ez; // 声明 Z 轴姿态误差修正量，供后续计算、状态保存或模块间传递使用

    if ((imu == 0) || (dt <= 0.0f) || (dt > 0.2f)) // 判断 (imu == 0) || (dt <= 0.0f) || (dt > 0.2f) 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    ax =  imu->accel_x_g; // 把 imu->accel_x_g 字段值 写入 X 轴加速度分量
    ay = -imu->accel_y_g; // 把 -imu->accel_y_g 字段值 写入 Y 轴加速度分量
    az =  imu->accel_z_g; // 把 imu->accel_z_g 字段值 写入 Z 轴加速度分量

    gx =  imu->gyro_x_dps * deg2rad; // 把 imu->gyro_x_dps * deg2rad 字段值 写入 X 轴角速度弧度值
    gy = -imu->gyro_y_dps * deg2rad; // 把 -imu->gyro_y_dps * deg2rad 字段值 写入 Y 轴角速度弧度值
    gz =  imu->gyro_z_dps * deg2rad; // 把 imu->gyro_z_dps * deg2rad 字段值 写入 Z 轴角速度弧度值

    norm = sqrtf(ax * ax + ay * ay + az * az); // 把 sqrtf(ax * ax + ay * ay + az * az) 的计算结果 写入 norm 变量
    if (norm < 1e-6f) // 判断 norm < 1e-6f 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    ax /= norm; // 执行 ax /= norm;，完成当前上下文中的具体处理
    ay /= norm; // 执行 ay /= norm;，完成当前上下文中的具体处理
    az /= norm; // 执行 az /= norm;，完成当前上下文中的具体处理

    q0 = g_q.q0; // 把 g_q.q0 字段值 写入 四元数标量分量
    q1 = g_q.q1; // 把 g_q.q1 字段值 写入 四元数 X 分量
    q2 = g_q.q2; // 把 g_q.q2 字段值 写入 四元数 Y 分量
    q3 = g_q.q3; // 把 g_q.q3 字段值 写入 四元数 Z 分量

    vx = 2.0f * (q1 * q3 - q0 * q2); // 把 2.0f * (q1 * q3 - q0 * q2) 字段值 写入 vx 变量
    vy = 2.0f * (q0 * q1 + q2 * q3); // 把 2.0f * (q0 * q1 + q2 * q3) 字段值 写入 vy 变量
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // 把 q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3 的计算结果 写入 vz 变量

    ex = ay * vz - az * vy; // 把 ay * vz - az * vy 的计算结果 写入 X 轴姿态误差修正量
    ey = az * vx - ax * vz; // 把 az * vx - ax * vz 的计算结果 写入 Y 轴姿态误差修正量
    ez = ax * vy - ay * vx; // 把 ax * vy - ay * vx 的计算结果 写入 Z 轴姿态误差修正量

    if (g_ki > 0.0f) // 判断 g_ki > 0.0f 是否成立，以选择后续执行路径
    { // 进入当前代码块
        g_exInt += ex * g_ki * dt; // 把 ex * g_ki * dt 的计算结果 累加到 g_exInt 全局状态变量
        g_eyInt += ey * g_ki * dt; // 把 ey * g_ki * dt 的计算结果 累加到 g_eyInt 全局状态变量
        g_ezInt += ez * g_ki * dt; // 把 ez * g_ki * dt 的计算结果 累加到 g_ezInt 全局状态变量

        g_exInt = mpu9250_clampf(g_exInt, -int_lim, int_lim); // 把 mpu9250_clampf(g_exInt, -int_lim, int_lim) 的计算结果 写入 g_exInt 全局状态变量
        g_eyInt = mpu9250_clampf(g_eyInt, -int_lim, int_lim); // 把 mpu9250_clampf(g_eyInt, -int_lim, int_lim) 的计算结果 写入 g_eyInt 全局状态变量
        g_ezInt = mpu9250_clampf(g_ezInt, -int_lim, int_lim); // 把 mpu9250_clampf(g_ezInt, -int_lim, int_lim) 的计算结果 写入 g_ezInt 全局状态变量

        gx += g_exInt; // 把 g_exInt 全局状态变量 累加到 X 轴角速度弧度值
        gy += g_eyInt; // 把 g_eyInt 全局状态变量 累加到 Y 轴角速度弧度值
        gz += g_ezInt; // 把 g_ezInt 全局状态变量 累加到 Z 轴角速度弧度值
    } // 结束当前代码块
    else // 处理前面判断条件不成立时的备用逻辑
    { // 进入当前代码块
        g_exInt = 0.0f; // 把 0.0f 字段值 写入 g_exInt 全局状态变量
        g_eyInt = 0.0f; // 把 0.0f 字段值 写入 g_eyInt 全局状态变量
        g_ezInt = 0.0f; // 把 0.0f 字段值 写入 g_ezInt 全局状态变量
    } // 结束当前代码块

    gx += g_kp * ex; // 把 g_kp * ex 的计算结果 累加到 X 轴角速度弧度值
    gy += g_kp * ey; // 把 g_kp * ey 的计算结果 累加到 Y 轴角速度弧度值
    gz += g_kp * ez; // 把 g_kp * ez 的计算结果 累加到 Z 轴角速度弧度值

    mpu9250_integrate_quaternion(gx, gy, gz, dt); // 调用mpu9250_integrate_quaternion 函数，参数为 gx, gy, gz, dt
} // 结束当前代码块

/**
 * @brief 读取 Mahony 融合后的欧拉角，并由弧度转换为角度。
 * @param roll_deg 横滚角角度值。
 * @param pitch_deg 俯仰角角度值。
 * @param yaw_deg 航向角角度值。
 * @retval None
 */
void MPU9250_GetEulerFusedDeg(float *roll_deg, float *pitch_deg, float *yaw_deg) // 定义MPU9250_GetEulerFusedDeg 函数签名：读取 Mahony 融合后的欧拉角角度值
{ // 进入当前代码块
    const float rad2deg = 57.295779513f; // 定义 rad2deg 变量，初始值设置为 57.295779513f 字段值

    if (roll_deg != 0) // 判断 roll_deg != 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        *roll_deg = g_euler_fused.roll * rad2deg;
    } // 结束当前代码块
    if (pitch_deg != 0) // 判断 pitch_deg != 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        *pitch_deg = g_euler_fused.pitch * rad2deg;
    } // 结束当前代码块
    if (yaw_deg != 0) // 判断 yaw_deg != 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        *yaw_deg = mpu9250_math_yaw_to_heading_deg(g_euler_fused.yaw * rad2deg);
    } // 结束当前代码块
} // 结束当前代码块








