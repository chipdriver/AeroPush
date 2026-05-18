/**
 * @file    mpu9250_driver.c
 * @brief   MPU9250 六轴与 AK8963 磁力计驱动实现。
 */
#include "mpu9250_driver.h"
#include "app_config.h"
#include <math.h>

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
#define AK8963_MAG_RADIUS_MIN_UT 5.0f                                                   // 定义本文件使用的宏
#define AK8963_MAG_CAL_MAX_SAMPLES 500U
#define AK8963_MAG_FIT_PARAM_COUNT 9U
#define AK8963_MAG_FIT_MAX_ITERATIONS 24U
#define AK8963_MAG_FIT_MIN_VALID_SAMPLES 64U
#define AK8963_MAG_FIT_INITIAL_DAMPING 1.0e-3f
#define AK8963_MAG_FIT_MIN_EIGENVALUE 1.0e-8f
#define AK8963_MAG_FIT_MAX_EIGENVALUE 1.0f

static uint8_t g_ak8963_asa[3] = {0U, 0U, 0U};                                            // 定义本文件内部静态变量
static float g_ak8963_sensitivity[3] = {1.0f, 1.0f, 1.0f};                                // 定义本文件内部静态变量

static float g_gyro_bias_dps[3] = {0.0f, 0.0f, 0.0f};                                     // 定义本文件内部静态变量
static float g_accel_bias_g[3] = {0.0f, 0.0f, 0.0f};                                      // 定义本文件内部静态变量
static float g_mag_offset_ut[3] = {0.0f, 0.0f, 0.0f};                                     // 定义本文件内部静态变量
static float g_mag_correction[3][3] = {{1.0f, 0.0f, 0.0f},
                                       {0.0f, 1.0f, 0.0f},
                                       {0.0f, 0.0f, 1.0f}};
static AK8963_Physical_Data g_mag_cal_samples[AK8963_MAG_CAL_MAX_SAMPLES];
static float g_mag_fit_matrix[AK8963_MAG_FIT_PARAM_COUNT][AK8963_MAG_FIT_PARAM_COUNT];
static float g_mag_fit_vector[AK8963_MAG_FIT_PARAM_COUNT];
static float g_mag_fit_delta[AK8963_MAG_FIT_PARAM_COUNT];
static float g_mag_fit_candidate[AK8963_MAG_FIT_PARAM_COUNT];

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
{
    float centered_x;
    float centered_y;
    float centered_z;
    float corrected_x;
    float corrected_y;
    float corrected_z;

    if (mag == 0)
    {
        return;
    }

    centered_x = mag->mag_x_ut - g_mag_offset_ut[0];
    centered_y = mag->mag_y_ut - g_mag_offset_ut[1];
    centered_z = mag->mag_z_ut - g_mag_offset_ut[2];

    corrected_x = g_mag_correction[0][0] * centered_x +
                  g_mag_correction[0][1] * centered_y +
                  g_mag_correction[0][2] * centered_z;
    corrected_y = g_mag_correction[1][0] * centered_x +
                  g_mag_correction[1][1] * centered_y +
                  g_mag_correction[1][2] * centered_z;
    corrected_z = g_mag_correction[2][0] * centered_x +
                  g_mag_correction[2][1] * centered_y +
                  g_mag_correction[2][2] * centered_z;

    mag->mag_x_ut = corrected_x;
    mag->mag_y_ut = corrected_y;
    mag->mag_z_ut = corrected_z;
}

static void ak8963_zero_fit_system(float matrix[AK8963_MAG_FIT_PARAM_COUNT][AK8963_MAG_FIT_PARAM_COUNT],
                                   float vector[AK8963_MAG_FIT_PARAM_COUNT])
{
    uint8_t row;
    uint8_t col;

    for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++)
    {
        vector[row] = 0.0f;
        for (col = 0U; col < AK8963_MAG_FIT_PARAM_COUNT; col++)
        {
            matrix[row][col] = 0.0f;
        }
    }
}

static float ak8963_eval_ellipsoid_residual(const float params[AK8963_MAG_FIT_PARAM_COUNT],
                                            const AK8963_Physical_Data *sample)
{
    float dx = sample->mag_x_ut - params[0];
    float dy = sample->mag_y_ut - params[1];
    float dz = sample->mag_z_ut - params[2];

    return params[3] * dx * dx +
           2.0f * params[4] * dx * dy +
           2.0f * params[5] * dx * dz +
           params[6] * dy * dy +
           2.0f * params[7] * dy * dz +
           params[8] * dz * dz -
           1.0f;
}

static float ak8963_compute_fit_error(const float params[AK8963_MAG_FIT_PARAM_COUNT], uint16_t sample_count)
{
    uint16_t i;
    float residual;
    float sum_sq = 0.0f;

    for (i = 0U; i < sample_count; i++)
    {
        residual = ak8963_eval_ellipsoid_residual(params, &g_mag_cal_samples[i]);
        sum_sq += residual * residual;
    }

    return sum_sq / (float)sample_count;
}

static void ak8963_build_fit_system(const float params[AK8963_MAG_FIT_PARAM_COUNT],
                                    uint16_t sample_count,
                                    float matrix[AK8963_MAG_FIT_PARAM_COUNT][AK8963_MAG_FIT_PARAM_COUNT],
                                    float vector[AK8963_MAG_FIT_PARAM_COUNT])
{
    uint16_t i;
    uint8_t row;
    uint8_t col;
    float dx;
    float dy;
    float dz;
    float residual;
    float jac[AK8963_MAG_FIT_PARAM_COUNT];

    ak8963_zero_fit_system(matrix, vector);

    for (i = 0U; i < sample_count; i++)
    {
        dx = g_mag_cal_samples[i].mag_x_ut - params[0];
        dy = g_mag_cal_samples[i].mag_y_ut - params[1];
        dz = g_mag_cal_samples[i].mag_z_ut - params[2];
        residual = ak8963_eval_ellipsoid_residual(params, &g_mag_cal_samples[i]);

        jac[0] = -2.0f * (params[3] * dx + params[4] * dy + params[5] * dz);
        jac[1] = -2.0f * (params[4] * dx + params[6] * dy + params[7] * dz);
        jac[2] = -2.0f * (params[5] * dx + params[7] * dy + params[8] * dz);
        jac[3] = dx * dx;
        jac[4] = 2.0f * dx * dy;
        jac[5] = 2.0f * dx * dz;
        jac[6] = dy * dy;
        jac[7] = 2.0f * dy * dz;
        jac[8] = dz * dz;

        for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++)
        {
            vector[row] -= jac[row] * residual;
            for (col = row; col < AK8963_MAG_FIT_PARAM_COUNT; col++)
            {
                matrix[row][col] += jac[row] * jac[col];
            }
        }
    }

    for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++)
    {
        for (col = 0U; col < row; col++)
        {
            matrix[row][col] = matrix[col][row];
        }
    }
}

static uint8_t ak8963_solve_fit_system(float matrix[AK8963_MAG_FIT_PARAM_COUNT][AK8963_MAG_FIT_PARAM_COUNT],
                                       float vector[AK8963_MAG_FIT_PARAM_COUNT],
                                       float solution[AK8963_MAG_FIT_PARAM_COUNT])
{
    uint8_t pivot;
    uint8_t row;
    uint8_t col;
    uint8_t max_row;
    float max_abs;
    float candidate_abs;
    float temp;
    float factor;
    float sum;

    for (pivot = 0U; pivot < AK8963_MAG_FIT_PARAM_COUNT; pivot++)
    {
        max_row = pivot;
        max_abs = fabsf(matrix[pivot][pivot]);
        for (row = (uint8_t)(pivot + 1U); row < AK8963_MAG_FIT_PARAM_COUNT; row++)
        {
            candidate_abs = fabsf(matrix[row][pivot]);
            if (candidate_abs > max_abs)
            {
                max_abs = candidate_abs;
                max_row = row;
            }
        }

        if (max_abs < 1.0e-12f)
        {
            return 0U;
        }

        if (max_row != pivot)
        {
            for (col = pivot; col < AK8963_MAG_FIT_PARAM_COUNT; col++)
            {
                temp = matrix[pivot][col];
                matrix[pivot][col] = matrix[max_row][col];
                matrix[max_row][col] = temp;
            }
            temp = vector[pivot];
            vector[pivot] = vector[max_row];
            vector[max_row] = temp;
        }

        for (row = (uint8_t)(pivot + 1U); row < AK8963_MAG_FIT_PARAM_COUNT; row++)
        {
            factor = matrix[row][pivot] / matrix[pivot][pivot];
            matrix[row][pivot] = 0.0f;
            for (col = (uint8_t)(pivot + 1U); col < AK8963_MAG_FIT_PARAM_COUNT; col++)
            {
                matrix[row][col] -= factor * matrix[pivot][col];
            }
            vector[row] -= factor * vector[pivot];
        }
    }

    for (row = AK8963_MAG_FIT_PARAM_COUNT; row > 0U; row--)
    {
        uint8_t idx = (uint8_t)(row - 1U);
        sum = vector[idx];
        for (col = (uint8_t)(idx + 1U); col < AK8963_MAG_FIT_PARAM_COUNT; col++)
        {
            sum -= matrix[idx][col] * solution[col];
        }

        if (fabsf(matrix[idx][idx]) < 1.0e-12f)
        {
            return 0U;
        }

        solution[idx] = sum / matrix[idx][idx];
    }

    return 1U;
}

static void ak8963_params_to_matrix(const float params[AK8963_MAG_FIT_PARAM_COUNT], float matrix[3][3])
{
    matrix[0][0] = params[3];
    matrix[0][1] = params[4];
    matrix[0][2] = params[5];
    matrix[1][0] = params[4];
    matrix[1][1] = params[6];
    matrix[1][2] = params[7];
    matrix[2][0] = params[5];
    matrix[2][1] = params[7];
    matrix[2][2] = params[8];
}

static uint8_t ak8963_jacobi_eigen_symmetric3(const float input[3][3], float eigenvalues[3], float eigenvectors[3][3])
{
    float a[3][3];
    uint8_t row;
    uint8_t col;
    uint8_t iter;
    uint8_t p;
    uint8_t q;
    float max_off_diag;
    float off_diag;
    float app;
    float aqq;
    float apq;
    float phi;
    float c;
    float s;
    float aip;
    float aiq;
    float vip;
    float viq;

    for (row = 0U; row < 3U; row++)
    {
        for (col = 0U; col < 3U; col++)
        {
            a[row][col] = input[row][col];
            eigenvectors[row][col] = (row == col) ? 1.0f : 0.0f;
        }
    }

    for (iter = 0U; iter < 18U; iter++)
    {
        p = 0U;
        q = 1U;
        max_off_diag = fabsf(a[0][1]);

        off_diag = fabsf(a[0][2]);
        if (off_diag > max_off_diag)
        {
            max_off_diag = off_diag;
            p = 0U;
            q = 2U;
        }

        off_diag = fabsf(a[1][2]);
        if (off_diag > max_off_diag)
        {
            max_off_diag = off_diag;
            p = 1U;
            q = 2U;
        }

        if (max_off_diag < 1.0e-9f)
        {
            break;
        }

        app = a[p][p];
        aqq = a[q][q];
        apq = a[p][q];
        phi = 0.5f * atan2f(2.0f * apq, aqq - app);
        c = cosf(phi);
        s = sinf(phi);

        for (row = 0U; row < 3U; row++)
        {
            if ((row != p) && (row != q))
            {
                aip = a[row][p];
                aiq = a[row][q];
                a[row][p] = c * aip - s * aiq;
                a[p][row] = a[row][p];
                a[row][q] = s * aip + c * aiq;
                a[q][row] = a[row][q];
            }
        }

        a[p][p] = c * c * app - 2.0f * s * c * apq + s * s * aqq;
        a[q][q] = s * s * app + 2.0f * s * c * apq + c * c * aqq;
        a[p][q] = 0.0f;
        a[q][p] = 0.0f;

        for (row = 0U; row < 3U; row++)
        {
            vip = eigenvectors[row][p];
            viq = eigenvectors[row][q];
            eigenvectors[row][p] = c * vip - s * viq;
            eigenvectors[row][q] = s * vip + c * viq;
        }
    }

    for (row = 0U; row < 3U; row++)
    {
        eigenvalues[row] = a[row][row];
        if ((eigenvalues[row] < AK8963_MAG_FIT_MIN_EIGENVALUE) ||
            (eigenvalues[row] > AK8963_MAG_FIT_MAX_EIGENVALUE))
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t ak8963_build_softiron_matrix(const float params[AK8963_MAG_FIT_PARAM_COUNT],
                                            float correction[3][3],
                                            float *radius_ut)
{
    float quadratic[3][3];
    float eigenvalues[3];
    float eigenvectors[3][3];
    float sqrt_lambda[3];
    float axis_radius[3];
    uint8_t row;
    uint8_t col;
    uint8_t axis;

    ak8963_params_to_matrix(params, quadratic);
    if (ak8963_jacobi_eigen_symmetric3(quadratic, eigenvalues, eigenvectors) == 0U)
    {
        return 0U;
    }

    *radius_ut = 0.0f;
    for (axis = 0U; axis < 3U; axis++)
    {
        sqrt_lambda[axis] = sqrtf(eigenvalues[axis]);
        axis_radius[axis] = 1.0f / sqrt_lambda[axis];
        *radius_ut += axis_radius[axis];
    }
    *radius_ut /= 3.0f;

    if (*radius_ut < AK8963_MAG_RADIUS_MIN_UT)
    {
        return 0U;
    }

    for (row = 0U; row < 3U; row++)
    {
        for (col = 0U; col < 3U; col++)
        {
            correction[row][col] = 0.0f;
            for (axis = 0U; axis < 3U; axis++)
            {
                correction[row][col] += (*radius_ut) *
                                        eigenvectors[row][axis] *
                                        sqrt_lambda[axis] *
                                        eigenvectors[col][axis];
            }
        }
    }

    return 1U;
}

static uint8_t ak8963_fit_ellipsoid(uint16_t sample_count,
                                    float params[AK8963_MAG_FIT_PARAM_COUNT],
                                    float *fit_error)
{
    uint8_t iter;
    uint8_t row;
    float damping = AK8963_MAG_FIT_INITIAL_DAMPING;
    float current_error = ak8963_compute_fit_error(params, sample_count);
    float candidate_error;
    float max_delta;

    for (iter = 0U; iter < AK8963_MAG_FIT_MAX_ITERATIONS; iter++)
    {
        ak8963_build_fit_system(params, sample_count, g_mag_fit_matrix, g_mag_fit_vector);
        for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++)
        {
            g_mag_fit_matrix[row][row] += damping;
            g_mag_fit_delta[row] = 0.0f;
        }

        if (ak8963_solve_fit_system(g_mag_fit_matrix, g_mag_fit_vector, g_mag_fit_delta) == 0U)
        {
            return 0U;
        }

        max_delta = 0.0f;
        for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++)
        {
            g_mag_fit_candidate[row] = params[row] + g_mag_fit_delta[row];
            if (fabsf(g_mag_fit_delta[row]) > max_delta)
            {
                max_delta = fabsf(g_mag_fit_delta[row]);
            }
        }

        candidate_error = ak8963_compute_fit_error(g_mag_fit_candidate, sample_count);
        if (candidate_error < current_error)
        {
            for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++)
            {
                params[row] = g_mag_fit_candidate[row];
            }
            current_error = candidate_error;
            damping *= 0.35f;
            if (damping < 1.0e-7f)
            {
                damping = 1.0e-7f;
            }

            if (max_delta < 1.0e-5f)
            {
                break;
            }
        }
        else
        {
            damping *= 10.0f;
            if (damping > 1.0e6f)
            {
                return 0U;
            }
        }
    }

    *fit_error = current_error;
    return 1U;
}
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
void AK8963_EnterFuseROMMode(void)
{
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
int AK8963_Read_Axis(AK8963_raw_Data *raw)
{
    uint8_t frame[7];

    if (raw == 0)
    {
        return -2;
    }

    if (AK8963_CheckDataReady() == 0)
    {
        return -1;
    }

    /* Read HXL..ST2 in one burst so the AK8963 releases the latched frame cleanly. */
    if (I2C_ReadRegs(AK8963_I2C_ADDR7, AK8963_REG_HXL, frame, (uint16_t)sizeof(frame)) != 0)
    {
        return -2;
    }

    raw->mag_x = (int16_t)(((uint16_t)frame[1] << 8U) | (uint16_t)frame[0]);
    raw->mag_y = (int16_t)(((uint16_t)frame[3] << 8U) | (uint16_t)frame[2]);
    raw->mag_z = (int16_t)(((uint16_t)frame[5] << 8U) | (uint16_t)frame[4]);

    if ((frame[6] & 0x08U) != 0U)
    {
        return -3;
    }

    return 0;
}

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
void MPU9250_CalibrateGyro(uint16_t samples, uint16_t delay_ms)
{
    MPU9250_raw_Data raw;
    int32_t gyro_x_sum = 0;
    int32_t gyro_y_sum = 0;
    int32_t gyro_z_sum = 0;
    uint16_t i;

    if (samples == 0U)
    {
        return;
    }

    for (i = 0U; i < samples; i++)
    {
        MPU9250_ReadAxis(&raw);
        gyro_x_sum += raw.gyro_x;
        gyro_y_sum += raw.gyro_y;
        gyro_z_sum += raw.gyro_z;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    g_gyro_bias_dps[0] = ((float)gyro_x_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS;
    g_gyro_bias_dps[1] = ((float)gyro_y_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS;
    g_gyro_bias_dps[2] = ((float)gyro_z_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS;
}

void MPU9250_CalibrateAccel(uint16_t samples, uint16_t delay_ms)
{
    MPU9250_raw_Data raw;
    float ax_sum = 0.0f;
    float ay_sum = 0.0f;
    float az_sum = 0.0f;
    uint16_t i;

    if (samples == 0U)
    {
        return;
    }

    for (i = 0U; i < samples; i++)
    {
        MPU9250_ReadAxis(&raw);
        ax_sum += (float)raw.accel_x / MPU9250_ACCEL_LSB_PER_G;
        ay_sum += (float)raw.accel_y / MPU9250_ACCEL_LSB_PER_G;
        az_sum += (float)raw.accel_z / MPU9250_ACCEL_LSB_PER_G;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    g_accel_bias_g[0] = ax_sum / (float)samples;
    g_accel_bias_g[1] = ay_sum / (float)samples;
    g_accel_bias_g[2] = (az_sum / (float)samples) - 1.0f;
}

void AK8963_CalibrateMag(uint16_t samples, uint16_t delay_ms)
{
    AK8963_raw_Data raw;
    AK8963_Physical_Data mag;
    float min_x = 99999.0f;
    float min_y = 99999.0f;
    float min_z = 99999.0f;
    float max_x = -99999.0f;
    float max_y = -99999.0f;
    float max_z = -99999.0f;
    float radius_x;
    float radius_y;
    float radius_z;
    float seed_center_x;
    float seed_center_y;
    float seed_center_z;
    float radius_avg;
    float fit_params[AK8963_MAG_FIT_PARAM_COUNT];
    float fit_error = 0.0f;
    float fitted_radius_ut = 0.0f;
    float candidate_correction[3][3];
    uint16_t valid_count = 0U;
    uint16_t requested_samples = samples;
    uint16_t not_ready_count = 0U;
    uint16_t read_fail_count = 0U;
    uint16_t overflow_count = 0U;
    uint16_t i;
    uint8_t row;
    uint8_t col;
    int read_ret;

    if (samples == 0U)
    {
        return;
    }

    if (samples > AK8963_MAG_CAL_MAX_SAMPLES)
    {
        samples = AK8963_MAG_CAL_MAX_SAMPLES;
        Debug_Printf("[AK8963] mag calibration samples limited %u->%u\r\n",
                     requested_samples,
                     samples);
    }

    Debug_Print("[AK8963] mag calibration preparing\r\n");
    Debug_Print("[AK8963] get ready to rotate slowly through all 3 axes\r\n");
    Debug_Print("[AK8963] start in 3...\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    Debug_Print("[AK8963] start in 2...\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    Debug_Print("[AK8963] start in 1...\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    Debug_Print("[AK8963] mag calibration begin,  keep moving\r\n");

    for (i = 0U; i < samples; i++)
    {
        read_ret = AK8963_Read_Axis(&raw);
        if (read_ret == 0)
        {
            ak8963_convert_raw_to_ut(&raw, &mag);

#if (APP_MAG_POINT_CLOUD_DEBUG_ENABLE != 0U)
            if ((APP_MAG_POINT_CLOUD_PRINT_DIV == 0U) ||
                ((valid_count % APP_MAG_POINT_CLOUD_PRINT_DIV) == 0U))
            {
                Debug_Printf("[AK8963_RAW_UT] %.2f,%.2f,%.2f\r\n",
                             mag.mag_x_ut,
                             mag.mag_y_ut,
                             mag.mag_z_ut);
            }
#endif

            if (valid_count < AK8963_MAG_CAL_MAX_SAMPLES)
            {
                g_mag_cal_samples[valid_count] = mag;
            }

            if (mag.mag_x_ut < min_x)
            {
                min_x = mag.mag_x_ut;
            }
            if (mag.mag_x_ut > max_x)
            {
                max_x = mag.mag_x_ut;
            }
            if (mag.mag_y_ut < min_y)
            {
                min_y = mag.mag_y_ut;
            }
            if (mag.mag_y_ut > max_y)
            {
                max_y = mag.mag_y_ut;
            }
            if (mag.mag_z_ut < min_z)
            {
                min_z = mag.mag_z_ut;
            }
            if (mag.mag_z_ut > max_z)
            {
                max_z = mag.mag_z_ut;
            }
            valid_count++;
        }
        else if (read_ret == -1)
        {
            not_ready_count++;
        }
        else if (read_ret == -2)
        {
            read_fail_count++;
        }
        else if (read_ret == -3)
        {
            overflow_count++;
        }

        if ((((uint16_t)(i + 1U)) % 50U) == 0U)
        {
            Debug_Printf("[AK8963] mag calibration progress %u/%u valid=%u nr=%u io=%u ov=%u\r\n",
                         (uint16_t)(i + 1U),
                         samples,
                         valid_count,
                         not_ready_count,
                         read_fail_count,
                         overflow_count);
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    if (valid_count < AK8963_MAG_FIT_MIN_VALID_SAMPLES)
    {
        Debug_Printf("[AK8963] calibrate mag rejected valid=%u\r\n", valid_count);
        return;
    }

    radius_x = (max_x - min_x) * 0.5f;
    radius_y = (max_y - min_y) * 0.5f;
    radius_z = (max_z - min_z) * 0.5f;

    seed_center_x = (min_x + max_x) * 0.5f;
    seed_center_y = (min_y + max_y) * 0.5f;
    seed_center_z = (min_z + max_z) * 0.5f;
    radius_avg = (radius_x + radius_y + radius_z) / 3.0f;
    Debug_Printf("[AK8963] raw bounds x=%.1f..%.1f y=%.1f..%.1f z=%.1f..%.1f\r\n",
                 min_x, max_x, min_y, max_y, min_z, max_z);
    Debug_Printf("[AK8963] seed center=%.1f/%.1f/%.1f radius=%.1f/%.1f/%.1f avg=%.1f\r\n",
                 seed_center_x,
                 seed_center_y,
                 seed_center_z,
                 radius_x,
                 radius_y,
                 radius_z,
                 radius_avg);

    if ((radius_x < AK8963_MAG_RADIUS_MIN_UT) ||
        (radius_y < AK8963_MAG_RADIUS_MIN_UT) ||
        (radius_z < AK8963_MAG_RADIUS_MIN_UT))
    {
        Debug_Printf("[AK8963] calibrate mag rejected radius=%.1f/%.1f/%.1f\r\n",
                     radius_x,
                     radius_y,
                     radius_z);
        return;
    }

    fit_params[0] = (min_x + max_x) * 0.5f;
    fit_params[1] = (min_y + max_y) * 0.5f;
    fit_params[2] = (min_z + max_z) * 0.5f;
    fit_params[3] = 1.0f / (radius_x * radius_x);
    fit_params[4] = 0.0f;
    fit_params[5] = 0.0f;
    fit_params[6] = 1.0f / (radius_y * radius_y);
    fit_params[7] = 0.0f;
    fit_params[8] = 1.0f / (radius_z * radius_z);

    if (ak8963_fit_ellipsoid(valid_count, fit_params, &fit_error) == 0U)
    {
        Debug_Print("[AK8963] calibrate mag rejected fit failed\r\n");
        return;
    }

    if (ak8963_build_softiron_matrix(fit_params, candidate_correction, &fitted_radius_ut) == 0U)
    {
        Debug_Print("[AK8963] calibrate mag rejected invalid ellipsoid\r\n");
        return;
    }

    if ((fit_params[0] < min_x) || (fit_params[0] > max_x) ||
        (fit_params[1] < min_y) || (fit_params[1] > max_y) ||
        (fit_params[2] < min_z) || (fit_params[2] > max_z) ||
        (fitted_radius_ut < (radius_avg * 0.5f)) ||
        (fitted_radius_ut > (radius_avg * 1.5f)))
    {
        Debug_Printf("[AK8963] calibrate mag rejected implausible fit center=%.1f/%.1f/%.1f radius=%.1f seed_avg=%.1f\r\n",
                     fit_params[0],
                     fit_params[1],
                     fit_params[2],
                     fitted_radius_ut,
                     radius_avg);
        return;
    }

    g_mag_offset_ut[0] = fit_params[0];
    g_mag_offset_ut[1] = fit_params[1];
    g_mag_offset_ut[2] = fit_params[2];
    for (row = 0U; row < 3U; row++)
    {
        for (col = 0U; col < 3U; col++)
        {
            g_mag_correction[row][col] = candidate_correction[row][col];
        }
    }

    Debug_Printf("[AK8963] calibrate mag ok center=%.1f/%.1f/%.1f radius=%.1f fit=%.4f\r\n",
                 g_mag_offset_ut[0],
                 g_mag_offset_ut[1],
                 g_mag_offset_ut[2],
                 fitted_radius_ut,
                 fit_error);
    Debug_Printf("[AK8963] mag matrix row0=%.3f,%.3f,%.3f\r\n",
                 g_mag_correction[0][0],
                 g_mag_correction[0][1],
                 g_mag_correction[0][2]);
    Debug_Printf("[AK8963] mag matrix row1=%.3f,%.3f,%.3f\r\n",
                 g_mag_correction[1][0],
                 g_mag_correction[1][1],
                 g_mag_correction[1][2]);
    Debug_Printf("[AK8963] mag matrix row2=%.3f,%.3f,%.3f\r\n",
                 g_mag_correction[2][0],
                 g_mag_correction[2][1],
                 g_mag_correction[2][2]);
    Debug_Print("[AK8963] mag calibration done\r\n");
}

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
    AK8963_CalibrateMag(500U, 20U);                                                       // 调用 AK8963 磁力计接口
    Debug_Print("[MPU9250] driver init ok\r\n");                                          // 输出调试日志
    return 1U;                                                                            // 返回函数执行结果
}                                                                                         // 结束代码块

/*=================================================================================姿态解算与融合=================================================================================*/

EulerAngle_t g_euler_acc_mag = {0.0f, 0.0f, 0.0f};
EulerAngle_t g_euler_fused = {0.0f, 0.0f, 0.0f};

static Quaternion_t g_q = {1.0f, 0.0f, 0.0f, 0.0f};
static float g_exInt = 0.0f;
static float g_eyInt = 0.0f;
static float g_ezInt = 0.0f;
static float g_kp = 0.3f;
static float g_ki = 0.0f;
static float g_acc_norm_prev = 1.0f;

static float mpu9250_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float mpu9250_wrap_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float mpu9250_math_yaw_to_heading_deg(float math_yaw_deg)
{
    /*
     * Internal yaw uses the mathematical convention:
     *   east = 0 deg, counter-clockwise positive.
     * Externally expose the navigation convention used by aircraft:
     *   north = 0 deg, east = +90 deg, range = (-180, 180].
     */
    return mpu9250_wrap_angle_deg(90.0f - math_yaw_deg);
}

static void mpu9250_update_euler_from_quaternion(void)
{
    float q0 = g_q.q0;
    float q1 = g_q.q1;
    float q2 = g_q.q2;
    float q3 = g_q.q3;
    float pitch_sin;

    g_euler_fused.roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                                1.0f - 2.0f * (q1 * q1 + q2 * q2));

    pitch_sin = 2.0f * (q0 * q2 - q3 * q1);
    pitch_sin = mpu9250_clampf(pitch_sin, -1.0f, 1.0f);
    g_euler_fused.pitch = asinf(pitch_sin);

    g_euler_fused.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                               1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

static void mpu9250_integrate_quaternion(float gx, float gy, float gz, float dt)
{
    float q0 = g_q.q0;
    float q1 = g_q.q1;
    float q2 = g_q.q2;
    float q3 = g_q.q3;
    float qDot0;
    float qDot1;
    float qDot2;
    float qDot3;
    float norm_q;

    qDot0 = -0.5f * (q1 * gx + q2 * gy + q3 * gz);
    qDot1 =  0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot2 =  0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot3 =  0.5f * (q0 * gz + q1 * gy - q2 * gx);

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    norm_q = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm_q < 1e-6f)
    {
        return;
    }

    g_q.q0 = q0 / norm_q;
    g_q.q1 = q1 / norm_q;
    g_q.q2 = q2 / norm_q;
    g_q.q3 = q3 / norm_q;

    mpu9250_update_euler_from_quaternion();
}

void MPU9250_ComputeEuler_FromAccMag(const MPU9250_Physical_Data *imu,
                                     const AK8963_Physical_Data *mag)
{
    float ax;
    float ay;
    float az;
    float mx;
    float my;
    float mz;
    float norm;
    float roll;
    float pitch;
    float sinRoll;
    float cosRoll;
    float sinPitch;
    float cosPitch;
    float mx2;
    float my2;

    if ((imu == 0) || (mag == 0))
    {
        return;
    }

    ax =  imu->accel_x_g;
    ay = -imu->accel_y_g;
    az =  imu->accel_z_g;

    mx =  mag->mag_x_ut;
    my = -mag->mag_y_ut;
    mz = -mag->mag_z_ut;

    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f)
    {
        return;
    }
    ax /= norm;
    ay /= norm;
    az /= norm;

    roll = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm < 1e-6f)
    {
        g_euler_acc_mag.roll = roll;
        g_euler_acc_mag.pitch = pitch;
        return;
    }
    mx /= norm;
    my /= norm;
    mz /= norm;

    sinRoll = sinf(roll);
    cosRoll = cosf(roll);
    sinPitch = sinf(pitch);
    cosPitch = cosf(pitch);

    mx2 = mx * cosPitch + mz * sinPitch;
    my2 = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;

    g_euler_acc_mag.roll = roll;
    g_euler_acc_mag.pitch = pitch;
    g_euler_acc_mag.yaw = atan2f(-my2, mx2);
}

void MPU9250_GetEulerDeg(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    const float rad2deg = 57.295779513f;

    if (roll_deg != 0)
    {
        *roll_deg = g_euler_acc_mag.roll * rad2deg;
    }
    if (pitch_deg != 0)
    {
        *pitch_deg = g_euler_acc_mag.pitch * rad2deg;
    }
    if (yaw_deg != 0)
    {
        *yaw_deg = mpu9250_math_yaw_to_heading_deg(g_euler_acc_mag.yaw * rad2deg);
    }
}

void MPU9250_MahonyInit(float kp, float ki)
{
    g_kp = kp;
    g_ki = ki;

    g_q.q0 = 1.0f;
    g_q.q1 = 0.0f;
    g_q.q2 = 0.0f;
    g_q.q3 = 0.0f;

    g_exInt = 0.0f;
    g_eyInt = 0.0f;
    g_ezInt = 0.0f;
    g_acc_norm_prev = 1.0f;

    g_euler_acc_mag.roll = 0.0f;
    g_euler_acc_mag.pitch = 0.0f;
    g_euler_acc_mag.yaw = 0.0f;
    g_euler_fused.roll = 0.0f;
    g_euler_fused.pitch = 0.0f;
    g_euler_fused.yaw = 0.0f;
}

void MPU9250_MahonyUpdate(const MPU9250_Physical_Data *imu,
                          const AK8963_Physical_Data *mag,
                          float dt)
{
    const float deg2rad = 0.01745329252f;
    const float mag_min_ut = 15.0f;
    const float mag_max_ut = 100.0f;
    const float int_lim = 0.1f;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
    float norm_acc;
    float norm_mag;
    float acc_error;
    float acc_diff;
    float kp_dynamic;
    float q0;
    float q1;
    float q2;
    float q3;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;
    float hx;
    float hy;
    float hz;
    float bx;
    float bz;
    float wx;
    float wy;
    float wz;
    uint8_t mag_valid;

    if ((imu == 0) || (mag == 0) || (dt <= 0.0f) || (dt > 0.2f))
    {
        return;
    }

    ax =  imu->accel_x_g;
    ay = -imu->accel_y_g;
    az =  imu->accel_z_g;

    gx =  imu->gyro_x_dps * deg2rad;
    gy = -imu->gyro_y_dps * deg2rad;
    gz =  imu->gyro_z_dps * deg2rad;

    mx =  mag->mag_x_ut;
    my = -mag->mag_y_ut;
    mz = -mag->mag_z_ut;

    norm_acc = sqrtf(ax * ax + ay * ay + az * az);
    if (norm_acc < 1e-6f)
    {
        return;
    }

    norm_mag = sqrtf(mx * mx + my * my + mz * mz);
    mag_valid = ((norm_mag > mag_min_ut) && (norm_mag < mag_max_ut)) ? 1U : 0U;

    acc_error = fabsf(norm_acc - 1.0f);
    acc_diff = fabsf(norm_acc - g_acc_norm_prev);
    kp_dynamic = g_kp;
    if ((acc_error < 0.04f) && (acc_diff < 0.02f))
    {
        kp_dynamic = g_kp * 2.0f;
    }
    else if ((acc_error > 0.25f) || (acc_diff > 0.25f))
    {
        kp_dynamic = g_kp * 0.35f;
    }
    g_acc_norm_prev = norm_acc;

    ax /= norm_acc;
    ay /= norm_acc;
    az /= norm_acc;

    if (norm_mag > 1e-6f)
    {
        mx /= norm_mag;
        my /= norm_mag;
        mz /= norm_mag;
    }
    else
    {
        mag_valid = 0U;
    }

    q0 = g_q.q0;
    q1 = g_q.q1;
    q2 = g_q.q2;
    q3 = g_q.q3;

    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    if (mag_valid != 0U)
    {
        hx = 2.0f * mx * (0.5f - q2 * q2 - q3 * q3)
           + 2.0f * my * (q1 * q2 - q0 * q3)
           + 2.0f * mz * (q1 * q3 + q0 * q2);
        hy = 2.0f * mx * (q1 * q2 + q0 * q3)
           + 2.0f * my * (0.5f - q1 * q1 - q3 * q3)
           + 2.0f * mz * (q2 * q3 - q0 * q1);
        hz = 2.0f * mx * (q1 * q3 - q0 * q2)
           + 2.0f * my * (q2 * q3 + q0 * q1)
           + 2.0f * mz * (0.5f - q1 * q1 - q2 * q2);

        bx = sqrtf(hx * hx + hy * hy);
        bz = hz;

        wx = 2.0f * bx * (0.5f - q2 * q2 - q3 * q3)
           + 2.0f * bz * (q1 * q3 - q0 * q2);
        wy = 2.0f * bx * (q1 * q2 - q0 * q3)
           + 2.0f * bz * (q0 * q1 + q2 * q3);
        wz = 2.0f * bx * (q0 * q2 + q1 * q3)
           + 2.0f * bz * (0.5f - q1 * q1 - q2 * q2);

        ex += my * wz - mz * wy;
        ey += mz * wx - mx * wz;
        ez += mx * wy - my * wx;
    }

    if (g_ki > 0.0f)
    {
        g_exInt += ex * g_ki * dt;
        g_eyInt += ey * g_ki * dt;
        g_ezInt += ez * g_ki * dt;

        g_exInt = mpu9250_clampf(g_exInt, -int_lim, int_lim);
        g_eyInt = mpu9250_clampf(g_eyInt, -int_lim, int_lim);
        g_ezInt = mpu9250_clampf(g_ezInt, -int_lim, int_lim);

        gx += g_exInt;
        gy += g_eyInt;
        gz += g_ezInt;
    }
    else
    {
        g_exInt = 0.0f;
        g_eyInt = 0.0f;
        g_ezInt = 0.0f;
    }

    gx += kp_dynamic * ex;
    gy += kp_dynamic * ey;
    gz += kp_dynamic * ez;

    mpu9250_integrate_quaternion(gx, gy, gz, dt);
}

void MPU9250_MahonyUpdateIMU(const MPU9250_Physical_Data *imu, float dt)
{
    const float deg2rad = 0.01745329252f;
    const float int_lim = 0.1f;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float norm;
    float q0;
    float q1;
    float q2;
    float q3;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;

    if ((imu == 0) || (dt <= 0.0f) || (dt > 0.2f))
    {
        return;
    }

    ax =  imu->accel_x_g;
    ay = -imu->accel_y_g;
    az =  imu->accel_z_g;

    gx =  imu->gyro_x_dps * deg2rad;
    gy = -imu->gyro_y_dps * deg2rad;
    gz =  imu->gyro_z_dps * deg2rad;

    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f)
    {
        return;
    }

    ax /= norm;
    ay /= norm;
    az /= norm;

    q0 = g_q.q0;
    q1 = g_q.q1;
    q2 = g_q.q2;
    q3 = g_q.q3;

    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    if (g_ki > 0.0f)
    {
        g_exInt += ex * g_ki * dt;
        g_eyInt += ey * g_ki * dt;
        g_ezInt += ez * g_ki * dt;

        g_exInt = mpu9250_clampf(g_exInt, -int_lim, int_lim);
        g_eyInt = mpu9250_clampf(g_eyInt, -int_lim, int_lim);
        g_ezInt = mpu9250_clampf(g_ezInt, -int_lim, int_lim);

        gx += g_exInt;
        gy += g_eyInt;
        gz += g_ezInt;
    }
    else
    {
        g_exInt = 0.0f;
        g_eyInt = 0.0f;
        g_ezInt = 0.0f;
    }

    gx += g_kp * ex;
    gy += g_kp * ey;
    gz += g_kp * ez;

    mpu9250_integrate_quaternion(gx, gy, gz, dt);
}

void MPU9250_GetEulerFusedDeg(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    const float rad2deg = 57.295779513f;

    if (roll_deg != 0)
    {
        *roll_deg = g_euler_fused.roll * rad2deg;
    }
    if (pitch_deg != 0)
    {
        *pitch_deg = g_euler_fused.pitch * rad2deg;
    }
    if (yaw_deg != 0)
    {
        *yaw_deg = mpu9250_math_yaw_to_heading_deg(g_euler_fused.yaw * rad2deg);
    }
}








