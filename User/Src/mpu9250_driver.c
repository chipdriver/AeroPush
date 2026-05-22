/**
 * @file    mpu9250_driver.c
 * @brief   MPU9250 六轴与 AK8963 磁力计驱动实现。
 */
#include "mpu9250_driver.h" // 提供 MPU9250/AK8963 驱动接口和数据结构
#include "app_config.h" // 提供工程配置宏
#include "app_status.h" // 提供校准灯语状态接口
#include <math.h> // 提供 sqrtf、atan2f、sinf、cosf 等数学函数

#define MPU9250_PWR_MGMT_1_REG 0x6BU // 电源管理 1：复位、睡眠、时钟源
#define MPU9250_PWR_MGMT_2_REG 0x6CU // 电源管理 2：各轴使能控制
#define MPU9250_USER_CTRL_REG 0x6AU // 用户控制：I2C 主机模式等
#define MPU9250_INT_PIN_CFG_REG 0x37U // 中断引脚配置：旁路访问磁力计
#define MPU9250_CONFIG_REG 0x1AU // 配置寄存器：陀螺仪 DLPF
#define MPU9250_SMPLRT_DIV_REG 0x19U // 六轴共用采样率分频寄存器
#define MPU9250_ACCEL_CONFIG_REG 0x1CU // 加速度计量程配置
#define MPU9250_ACCEL_CONFIG2_REG 0x1DU // 加速度计 DLPF 配置
#define MPU9250_GYRO_CONFIG_REG 0x1BU // 陀螺仪量程配置
#define MPU9250_ACCEL_XOUT_H_REG 0x3BU // 加速度计 X 轴高字节
#define MPU9250_ACCEL_YOUT_H_REG 0x3DU // 加速度计 Y 轴高字节
#define MPU9250_ACCEL_ZOUT_H_REG 0x3FU // 加速度计 Z 轴高字节
#define MPU9250_TEMP_OUT_H_REG 0x41U // 温度高字节
#define MPU9250_GYRO_XOUT_H_REG 0x43U // 陀螺仪 X 轴高字节
#define MPU9250_GYRO_YOUT_H_REG 0x45U // 陀螺仪 Y 轴高字节
#define MPU9250_GYRO_ZOUT_H_REG 0x47U // 陀螺仪 Z 轴高字节

#define AK8963_REG_WIA 0x00U // AK8963 设备 ID 寄存器
#define AK8963_REG_ST1 0x02U // AK8963 数据就绪状态寄存器
#define AK8963_REG_HXL 0x03U // 磁力计 X 轴低字节
#define AK8963_REG_HYL 0x05U // 磁力计 Y 轴低字节
#define AK8963_REG_HZL 0x07U // 磁力计 Z 轴低字节
#define AK8963_REG_ST2 0x09U // AK8963 数据溢出状态寄存器
#define AK8963_REG_CNTL1 0x0AU // AK8963 工作模式控制寄存器
#define AK8963_REG_ASAX 0x10U // X 轴灵敏度调整值
#define AK8963_REG_ASAY 0x11U // Y 轴灵敏度调整值
#define AK8963_REG_ASAZ 0x12U // Z 轴灵敏度调整值
#define AK8963_WHO_AM_I_VALUE 0x48U // AK8963 正常设备 ID

#define MPU9250_ACCEL_LSB_PER_G 4096.0f // 加速度计 +-8g 量程下每 g 对应 LSB
#define MPU9250_GYRO_LSB_PER_DPS 32.8f // 陀螺仪 +-1000dps 量程下每 dps 对应 LSB
#define AK8963_16BIT_UT_PER_LSB 0.15f // AK8963 16 位模式下每 LSB 对应微特斯拉
#define AK8963_MAG_RADIUS_MIN_UT 5.0f // 磁场半径有效性下限，过滤异常拟合
#define AK8963_MAG_CAL_MAX_SAMPLES 500U // 磁力计校准最多采样点数
#define AK8963_MAG_FIT_PARAM_COUNT 9U // 椭球拟合参数数量
#define AK8963_MAG_FIT_MAX_ITERATIONS 24U // 椭球拟合最大迭代次数
#define AK8963_MAG_FIT_MIN_VALID_SAMPLES 64U // 椭球拟合最少有效样本数
#define AK8963_MAG_FIT_INITIAL_DAMPING 1.0e-3f // Levenberg-Marquardt 初始阻尼
#define AK8963_MAG_FIT_MIN_EIGENVALUE 1.0e-8f // 软铁矩阵特征值下限
#define AK8963_MAG_FIT_MAX_EIGENVALUE 1.0f // 软铁矩阵特征值上限

static uint8_t g_ak8963_asa[3] = {0U, 0U, 0U}; // 保存 AK8963 Fuse ROM 灵敏度调整值
static float g_ak8963_sensitivity[3] = {1.0f, 1.0f, 1.0f}; // 保存磁力计三轴灵敏度修正系数

static float g_gyro_bias_dps[3] = {0.0f, 0.0f, 0.0f}; // 陀螺仪三轴零偏，单位 dps
static float g_accel_bias_g[3] = {0.0f, 0.0f, 0.0f}; // 加速度计三轴零偏，单位 g
static float g_mag_offset_ut[3] = {0.0f, 0.0f, 0.0f}; // 磁力计硬铁偏移，单位 uT
static float g_mag_correction[3][3] = {{1.0f, 0.0f, 0.0f}, // 软铁修正矩阵第 0 行
                                       {0.0f, 1.0f, 0.0f}, // 软铁修正矩阵第 1 行
                                       {0.0f, 0.0f, 1.0f}}; // 软铁修正矩阵第 2 行
static AK8963_Physical_Data g_mag_cal_samples[AK8963_MAG_CAL_MAX_SAMPLES]; // 磁力计校准采样缓存
static float g_mag_fit_matrix[AK8963_MAG_FIT_PARAM_COUNT][AK8963_MAG_FIT_PARAM_COUNT]; // 椭球拟合法方程矩阵
static float g_mag_fit_vector[AK8963_MAG_FIT_PARAM_COUNT]; // 椭球拟合法方程右端向量
static float g_mag_fit_delta[AK8963_MAG_FIT_PARAM_COUNT]; // 椭球拟合参数修正量
static float g_mag_fit_candidate[AK8963_MAG_FIT_PARAM_COUNT]; // 椭球拟合候选参数

/**
 * @brief 将 AK8963 原始磁力计值转换为微特斯拉。
 * @param raw AK8963 三轴原始值。
 * @param mag_out 输出的磁场物理量。
 * @retval None
 */
static void ak8963_convert_raw_to_ut(const AK8963_raw_Data *raw, AK8963_Physical_Data *mag_out)
{
    if ((raw == 0) || (mag_out == 0)) // 防止空指针访问
    {
        return; // 参数无效，直接返回
    }

    mag_out->mag_x_ut = (float)raw->mag_x * g_ak8963_sensitivity[0] * AK8963_16BIT_UT_PER_LSB; // 转换 X 轴磁场
    mag_out->mag_y_ut = (float)raw->mag_y * g_ak8963_sensitivity[1] * AK8963_16BIT_UT_PER_LSB; // 转换 Y 轴磁场
    mag_out->mag_z_ut = (float)raw->mag_z * g_ak8963_sensitivity[2] * AK8963_16BIT_UT_PER_LSB; // 转换 Z 轴磁场
}

/**
 * @brief 对磁力计物理量应用硬铁和软铁校准。
 * @param mag 待校准的磁场物理量。
 * @retval None
 */
static void ak8963_apply_mag_calibration(AK8963_Physical_Data *mag)
{
    float centered_x; // 去硬铁偏移后的 X 轴磁场
    float centered_y; // 去硬铁偏移后的 Y 轴磁场
    float centered_z; // 去硬铁偏移后的 Z 轴磁场
    float corrected_x; // 软铁修正后的 X 轴磁场
    float corrected_y; // 软铁修正后的 Y 轴磁场
    float corrected_z; // 软铁修正后的 Z 轴磁场

    if (mag == 0) // 防止空指针访问
    {
        return; // 参数无效，直接返回
    }

    centered_x = mag->mag_x_ut - g_mag_offset_ut[0]; // 去除 X 轴硬铁偏移
    centered_y = mag->mag_y_ut - g_mag_offset_ut[1]; // 去除 Y 轴硬铁偏移
    centered_z = mag->mag_z_ut - g_mag_offset_ut[2]; // 去除 Z 轴硬铁偏移

    corrected_x = g_mag_correction[0][0] * centered_x + // 软铁矩阵第 0 行作用到校正向量
                  g_mag_correction[0][1] * centered_y +
                  g_mag_correction[0][2] * centered_z;
    corrected_y = g_mag_correction[1][0] * centered_x + // 软铁矩阵第 1 行作用到校正向量
                  g_mag_correction[1][1] * centered_y +
                  g_mag_correction[1][2] * centered_z;
    corrected_z = g_mag_correction[2][0] * centered_x + // 软铁矩阵第 2 行作用到校正向量
                  g_mag_correction[2][1] * centered_y +
                  g_mag_correction[2][2] * centered_z;

    mag->mag_x_ut = corrected_x; // 写回校准后的 X 轴磁场
    mag->mag_y_ut = corrected_y; // 写回校准后的 Y 轴磁场
    mag->mag_z_ut = corrected_z; // 写回校准后的 Z 轴磁场
}

/**
 * @brief 清空椭球拟合的法方程矩阵和右端向量。
 * @param matrix 法方程矩阵。
 * @param vector 法方程右端向量。
 * @retval None
 */
static void ak8963_zero_fit_system(float matrix[AK8963_MAG_FIT_PARAM_COUNT][AK8963_MAG_FIT_PARAM_COUNT],
                                   float vector[AK8963_MAG_FIT_PARAM_COUNT])
{
    uint8_t row; // 矩阵行索引
    uint8_t col; // 矩阵列索引

    for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 遍历每一行
    {
        vector[row] = 0.0f; // 清空右端向量元素
        for (col = 0U; col < AK8963_MAG_FIT_PARAM_COUNT; col++) // 遍历当前行的每一列
        {
            matrix[row][col] = 0.0f; // 清空矩阵元素
        }
    }
}

/**
 * @brief 计算单个磁力计样本到当前椭球模型的残差。
 * @param params 椭球参数，前 3 个为中心，后 6 个为二次型矩阵。
 * @param sample 单个磁力计校准样本。
 * @retval 残差值，越接近 0 说明样本越贴近椭球面。
 */
static float ak8963_eval_ellipsoid_residual(const float params[AK8963_MAG_FIT_PARAM_COUNT],
                                            const AK8963_Physical_Data *sample)
{
    float dx = sample->mag_x_ut - params[0]; // 样本 X 轴相对椭球中心的偏移
    float dy = sample->mag_y_ut - params[1]; // 样本 Y 轴相对椭球中心的偏移
    float dz = sample->mag_z_ut - params[2]; // 样本 Z 轴相对椭球中心的偏移

    return params[3] * dx * dx + // 二次型 xx 项
           2.0f * params[4] * dx * dy + // 二次型 xy 交叉项
           2.0f * params[5] * dx * dz + // 二次型 xz 交叉项
           params[6] * dy * dy + // 二次型 yy 项
           2.0f * params[7] * dy * dz + // 二次型 yz 交叉项
           params[8] * dz * dz - // 二次型 zz 项
           1.0f; // 椭球标准面为 1
}

/**
 * @brief 计算当前椭球参数对所有样本的平均平方误差。
 * @param params 椭球拟合参数。
 * @param sample_count 有效校准样本数量。
 * @retval 平均平方残差。
 */
static float ak8963_compute_fit_error(const float params[AK8963_MAG_FIT_PARAM_COUNT], uint16_t sample_count)
{
    uint16_t i; // 样本索引
    float residual; // 单个样本残差
    float sum_sq = 0.0f; // 残差平方和

    for (i = 0U; i < sample_count; i++) // 遍历所有校准样本
    {
        residual = ak8963_eval_ellipsoid_residual(params, &g_mag_cal_samples[i]); // 计算当前样本残差
        sum_sq += residual * residual; // 累加残差平方
    }

    return sum_sq / (float)sample_count; // 返回平均平方误差
}

/**
 * @brief 根据当前椭球参数构建一次迭代用的法方程。
 * @param params 当前椭球参数。
 * @param sample_count 有效校准样本数量。
 * @param matrix 输出的法方程矩阵。
 * @param vector 输出的法方程右端向量。
 * @retval None
 */
static void ak8963_build_fit_system(const float params[AK8963_MAG_FIT_PARAM_COUNT],
                                    uint16_t sample_count,
                                    float matrix[AK8963_MAG_FIT_PARAM_COUNT][AK8963_MAG_FIT_PARAM_COUNT],
                                    float vector[AK8963_MAG_FIT_PARAM_COUNT])
{
    uint16_t i; // 样本索引
    uint8_t row; // 法方程行索引
    uint8_t col; // 法方程列索引
    float dx; // 样本 X 轴中心化值
    float dy; // 样本 Y 轴中心化值
    float dz; // 样本 Z 轴中心化值
    float residual; // 当前样本残差
    float jac[AK8963_MAG_FIT_PARAM_COUNT]; // 残差对各参数的雅可比

    ak8963_zero_fit_system(matrix, vector); // 重新构建前先清空方程

    for (i = 0U; i < sample_count; i++) // 遍历全部校准样本
    {
        dx = g_mag_cal_samples[i].mag_x_ut - params[0]; // 计算 X 轴中心化值
        dy = g_mag_cal_samples[i].mag_y_ut - params[1]; // 计算 Y 轴中心化值
        dz = g_mag_cal_samples[i].mag_z_ut - params[2]; // 计算 Z 轴中心化值
        residual = ak8963_eval_ellipsoid_residual(params, &g_mag_cal_samples[i]); // 计算当前残差

        jac[0] = -2.0f * (params[3] * dx + params[4] * dy + params[5] * dz); // 对中心 X 的偏导
        jac[1] = -2.0f * (params[4] * dx + params[6] * dy + params[7] * dz); // 对中心 Y 的偏导
        jac[2] = -2.0f * (params[5] * dx + params[7] * dy + params[8] * dz); // 对中心 Z 的偏导
        jac[3] = dx * dx; // 对 xx 二次项的偏导
        jac[4] = 2.0f * dx * dy; // 对 xy 交叉项的偏导
        jac[5] = 2.0f * dx * dz; // 对 xz 交叉项的偏导
        jac[6] = dy * dy; // 对 yy 二次项的偏导
        jac[7] = 2.0f * dy * dz; // 对 yz 交叉项的偏导
        jac[8] = dz * dz; // 对 zz 二次项的偏导

        for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 累加法方程每一行
        {
            vector[row] -= jac[row] * residual; // 累加右端向量 Jt * (-r)
            for (col = row; col < AK8963_MAG_FIT_PARAM_COUNT; col++) // 只累加上三角
            {
                matrix[row][col] += jac[row] * jac[col]; // 累加 Jt * J
            }
        }
    }

    for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 补齐矩阵下三角
    {
        for (col = 0U; col < row; col++) // 拷贝对称位置
        {
            matrix[row][col] = matrix[col][row]; // 保持法方程矩阵对称
        }
    }
}

/**
 * @brief 使用高斯消元求解椭球拟合法方程。
 * @param matrix 法方程矩阵，求解过程会被修改。
 * @param vector 法方程右端向量，求解过程会被修改。
 * @param solution 输出的参数增量。
 * @retval 1 求解成功；0 矩阵奇异或数值不可用。
 */
static uint8_t ak8963_solve_fit_system(float matrix[AK8963_MAG_FIT_PARAM_COUNT][AK8963_MAG_FIT_PARAM_COUNT],
                                       float vector[AK8963_MAG_FIT_PARAM_COUNT],
                                       float solution[AK8963_MAG_FIT_PARAM_COUNT])
{
    uint8_t pivot; // 当前主元列
    uint8_t row; // 行索引
    uint8_t col; // 列索引
    uint8_t max_row; // 当前列绝对值最大的行
    float max_abs; // 主元候选最大绝对值
    float candidate_abs; // 当前行候选主元绝对值
    float temp; // 行交换临时变量
    float factor; // 消元系数
    float sum; // 回代累加值

    for (pivot = 0U; pivot < AK8963_MAG_FIT_PARAM_COUNT; pivot++) // 逐列选择主元并前向消元
    {
        max_row = pivot; // 默认当前行为主元行
        max_abs = fabsf(matrix[pivot][pivot]); // 读取当前主元绝对值
        for (row = (uint8_t)(pivot + 1U); row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 搜索更稳定的主元行
        {
            candidate_abs = fabsf(matrix[row][pivot]); // 计算候选主元绝对值
            if (candidate_abs > max_abs) // 发现更大的主元
            {
                max_abs = candidate_abs; // 更新最大绝对值
                max_row = row; // 记录主元行
            }
        }

        if (max_abs < 1.0e-12f) // 主元过小会导致数值不稳定
        {
            return 0U; // 法方程不可解
        }

        if (max_row != pivot) // 需要交换主元行
        {
            for (col = pivot; col < AK8963_MAG_FIT_PARAM_COUNT; col++) // 交换矩阵当前行后半部分
            {
                temp = matrix[pivot][col]; // 暂存当前行元素
                matrix[pivot][col] = matrix[max_row][col]; // 将主元行搬到当前位置
                matrix[max_row][col] = temp; // 原当前行放到主元行
            }
            temp = vector[pivot]; // 暂存右端向量元素
            vector[pivot] = vector[max_row]; // 同步交换右端向量
            vector[max_row] = temp; // 完成右端向量交换
        }

        for (row = (uint8_t)(pivot + 1U); row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 消去主元列下方元素
        {
            factor = matrix[row][pivot] / matrix[pivot][pivot]; // 计算当前行消元系数
            matrix[row][pivot] = 0.0f; // 主元列下方置零
            for (col = (uint8_t)(pivot + 1U); col < AK8963_MAG_FIT_PARAM_COUNT; col++) // 更新当前行剩余列
            {
                matrix[row][col] -= factor * matrix[pivot][col]; // 扣除主元行对应比例
            }
            vector[row] -= factor * vector[pivot]; // 同步更新右端向量
        }
    }

    for (row = AK8963_MAG_FIT_PARAM_COUNT; row > 0U; row--) // 从最后一行开始回代
    {
        uint8_t idx = (uint8_t)(row - 1U); // 当前回代行索引
        sum = vector[idx]; // 从右端向量开始累加
        for (col = (uint8_t)(idx + 1U); col < AK8963_MAG_FIT_PARAM_COUNT; col++) // 扣除已求出的未知量
        {
            sum -= matrix[idx][col] * solution[col]; // 更新当前行剩余值
        }

        if (fabsf(matrix[idx][idx]) < 1.0e-12f) // 对角元素过小无法稳定回代
        {
            return 0U; // 回代失败
        }

        solution[idx] = sum / matrix[idx][idx]; // 求出当前未知量
    }

    return 1U; // 法方程求解成功
}

/**
 * @brief 将椭球参数中的二次型部分展开成 3x3 对称矩阵。
 * @param params 椭球拟合参数。
 * @param matrix 输出的二次型矩阵。
 * @retval None
 */
static void ak8963_params_to_matrix(const float params[AK8963_MAG_FIT_PARAM_COUNT], float matrix[3][3])
{
    matrix[0][0] = params[3]; // xx 系数
    matrix[0][1] = params[4]; // xy 系数
    matrix[0][2] = params[5]; // xz 系数
    matrix[1][0] = params[4]; // yx 系数，与 xy 对称
    matrix[1][1] = params[6]; // yy 系数
    matrix[1][2] = params[7]; // yz 系数
    matrix[2][0] = params[5]; // zx 系数，与 xz 对称
    matrix[2][1] = params[7]; // zy 系数，与 yz 对称
    matrix[2][2] = params[8]; // zz 系数
}

/**
 * @brief 对 3x3 对称矩阵做 Jacobi 特征值分解。
 * @param input 输入的 3x3 对称矩阵。
 * @param eigenvalues 输出特征值。
 * @param eigenvectors 输出特征向量矩阵。
 * @retval 1 分解成功且特征值有效；0 特征值异常。
 */
static uint8_t ak8963_jacobi_eigen_symmetric3(const float input[3][3], float eigenvalues[3], float eigenvectors[3][3])
{
    float a[3][3]; // 工作矩阵，避免修改输入矩阵
    uint8_t row; // 行索引
    uint8_t col; // 列索引
    uint8_t iter; // Jacobi 迭代次数
    uint8_t p; // 最大非对角元素所在行
    uint8_t q; // 最大非对角元素所在列
    float max_off_diag; // 当前最大非对角元素绝对值
    float off_diag; // 候选非对角元素绝对值
    float app; // 旋转前 a[p][p]
    float aqq; // 旋转前 a[q][q]
    float apq; // 旋转前 a[p][q]
    float phi; // Jacobi 旋转角
    float c; // 旋转角余弦
    float s; // 旋转角正弦
    float aip; // 当前行 p 列旧值
    float aiq; // 当前行 q 列旧值
    float vip; // 特征向量 p 列旧值
    float viq; // 特征向量 q 列旧值

    for (row = 0U; row < 3U; row++) // 初始化工作矩阵和单位特征向量矩阵
    {
        for (col = 0U; col < 3U; col++) // 遍历 3x3 矩阵元素
        {
            a[row][col] = input[row][col]; // 拷贝输入矩阵
            eigenvectors[row][col] = (row == col) ? 1.0f : 0.0f; // 初始化为单位矩阵
        }
    }

    for (iter = 0U; iter < 18U; iter++) // 固定最多 18 次 Jacobi 旋转
    {
        p = 0U; // 默认检查 a[0][1]
        q = 1U; // 默认检查 a[0][1]
        max_off_diag = fabsf(a[0][1]); // 记录当前最大非对角元素

        off_diag = fabsf(a[0][2]); // 检查 a[0][2]
        if (off_diag > max_off_diag) // 找到更大的非对角元素
        {
            max_off_diag = off_diag; // 更新最大值
            p = 0U; // 更新行索引
            q = 2U; // 更新列索引
        }

        off_diag = fabsf(a[1][2]); // 检查 a[1][2]
        if (off_diag > max_off_diag) // 找到更大的非对角元素
        {
            max_off_diag = off_diag; // 更新最大值
            p = 1U; // 更新行索引
            q = 2U; // 更新列索引
        }

        if (max_off_diag < 1.0e-9f) // 非对角项足够小，认为已经收敛
        {
            break; // 结束 Jacobi 迭代
        }

        app = a[p][p]; // 保存旋转前 p 轴对角项
        aqq = a[q][q]; // 保存旋转前 q 轴对角项
        apq = a[p][q]; // 保存待消去的非对角项
        phi = 0.5f * atan2f(2.0f * apq, aqq - app); // 计算旋转角
        c = cosf(phi); // 计算旋转余弦
        s = sinf(phi); // 计算旋转正弦

        for (row = 0U; row < 3U; row++) // 更新 p/q 以外的相关元素
        {
            if ((row != p) && (row != q)) // 跳过参与旋转的两个主轴
            {
                aip = a[row][p]; // 保存 row,p 旧值
                aiq = a[row][q]; // 保存 row,q 旧值
                a[row][p] = c * aip - s * aiq; // 更新 row,p
                a[p][row] = a[row][p]; // 保持矩阵对称
                a[row][q] = s * aip + c * aiq; // 更新 row,q
                a[q][row] = a[row][q]; // 保持矩阵对称
            }
        }

        a[p][p] = c * c * app - 2.0f * s * c * apq + s * s * aqq; // 更新 p 轴对角项
        a[q][q] = s * s * app + 2.0f * s * c * apq + c * c * aqq; // 更新 q 轴对角项
        a[p][q] = 0.0f; // 消去 p,q 非对角项
        a[q][p] = 0.0f; // 保持对称位置为 0

        for (row = 0U; row < 3U; row++) // 同步累计特征向量旋转
        {
            vip = eigenvectors[row][p]; // 保存特征向量 p 列旧值
            viq = eigenvectors[row][q]; // 保存特征向量 q 列旧值
            eigenvectors[row][p] = c * vip - s * viq; // 更新 p 列特征向量
            eigenvectors[row][q] = s * vip + c * viq; // 更新 q 列特征向量
        }
    }

    for (row = 0U; row < 3U; row++) // 读取对角线作为特征值
    {
        eigenvalues[row] = a[row][row]; // 保存当前轴特征值
        if ((eigenvalues[row] < AK8963_MAG_FIT_MIN_EIGENVALUE) ||
            (eigenvalues[row] > AK8963_MAG_FIT_MAX_EIGENVALUE)) // 特征值过小或过大都表示拟合不可信
        {
            return 0U; // 特征值无效
        }
    }

    return 1U; // 特征值分解成功
}

/**
 * @brief 根据椭球拟合参数生成磁力计软铁修正矩阵。
 * @param params 椭球拟合参数。
 * @param correction 输出的 3x3 软铁修正矩阵。
 * @param radius_ut 输出平均磁场半径，单位 uT。
 * @retval 1 矩阵生成成功；0 椭球参数不可信。
 */
static uint8_t ak8963_build_softiron_matrix(const float params[AK8963_MAG_FIT_PARAM_COUNT],
                                            float correction[3][3],
                                            float *radius_ut)
{
    float quadratic[3][3]; // 椭球二次型矩阵
    float eigenvalues[3]; // 二次型矩阵特征值
    float eigenvectors[3][3]; // 二次型矩阵特征向量
    float sqrt_lambda[3]; // 特征值平方根
    float axis_radius[3]; // 椭球三个主轴半径
    uint8_t row; // 矩阵行索引
    uint8_t col; // 矩阵列索引
    uint8_t axis; // 主轴索引

    ak8963_params_to_matrix(params, quadratic); // 提取椭球二次型矩阵
    if (ak8963_jacobi_eigen_symmetric3(quadratic, eigenvalues, eigenvectors) == 0U) // 分解二次型矩阵
    {
        return 0U; // 特征值异常，拟合结果不可用
    }

    *radius_ut = 0.0f; // 清空平均半径
    for (axis = 0U; axis < 3U; axis++) // 计算三个主轴半径
    {
        sqrt_lambda[axis] = sqrtf(eigenvalues[axis]); // 特征值平方根用于构造修正矩阵
        axis_radius[axis] = 1.0f / sqrt_lambda[axis]; // 椭球主轴半径
        *radius_ut += axis_radius[axis]; // 累加主轴半径
    }
    *radius_ut /= 3.0f; // 取平均半径作为目标球半径

    if (*radius_ut < AK8963_MAG_RADIUS_MIN_UT) // 半径过小通常说明采样或拟合异常
    {
        return 0U; // 拒绝不可信拟合
    }

    for (row = 0U; row < 3U; row++) // 构造软铁修正矩阵每一行
    {
        for (col = 0U; col < 3U; col++) // 构造当前行每一列
        {
            correction[row][col] = 0.0f; // 清空矩阵元素
            for (axis = 0U; axis < 3U; axis++) // 按特征向量基变换回原坐标系
            {
                correction[row][col] += (*radius_ut) *
                                        eigenvectors[row][axis] *
                                        sqrt_lambda[axis] *
                                        eigenvectors[col][axis]; // 累加当前主轴贡献
            }
        }
    }

    return 1U; // 软铁矩阵生成成功
}

/**
 * @brief 使用阻尼最小二乘迭代拟合磁力计椭球。
 * @param sample_count 有效校准样本数量。
 * @param params 输入初值并输出最终椭球参数。
 * @param fit_error 输出最终平均平方残差。
 * @retval 1 拟合成功；0 方程不可解或阻尼发散。
 */
static uint8_t ak8963_fit_ellipsoid(uint16_t sample_count,
                                    float params[AK8963_MAG_FIT_PARAM_COUNT],
                                    float *fit_error)
{
    uint8_t iter; // 迭代次数
    uint8_t row; // 参数索引
    float damping = AK8963_MAG_FIT_INITIAL_DAMPING; // 阻尼系数，控制步长稳定性
    float current_error = ak8963_compute_fit_error(params, sample_count); // 当前参数误差
    float candidate_error; // 候选参数误差
    float max_delta; // 本轮最大参数变化量

    for (iter = 0U; iter < AK8963_MAG_FIT_MAX_ITERATIONS; iter++) // 迭代优化椭球参数
    {
        ak8963_build_fit_system(params, sample_count, g_mag_fit_matrix, g_mag_fit_vector); // 构建当前法方程
        for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 对角线加入阻尼
        {
            g_mag_fit_matrix[row][row] += damping; // 增强数值稳定性
            g_mag_fit_delta[row] = 0.0f; // 清空本轮参数增量
        }

        if (ak8963_solve_fit_system(g_mag_fit_matrix, g_mag_fit_vector, g_mag_fit_delta) == 0U) // 求解参数增量
        {
            return 0U; // 法方程求解失败
        }

        max_delta = 0.0f; // 重新统计本轮最大增量
        for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 生成候选参数
        {
            g_mag_fit_candidate[row] = params[row] + g_mag_fit_delta[row]; // 应用参数增量
            if (fabsf(g_mag_fit_delta[row]) > max_delta) // 更新最大增量
            {
                max_delta = fabsf(g_mag_fit_delta[row]); // 保存最大绝对变化量
            }
        }

        candidate_error = ak8963_compute_fit_error(g_mag_fit_candidate, sample_count); // 计算候选参数误差
        if (candidate_error < current_error) // 候选参数让误差下降
        {
            for (row = 0U; row < AK8963_MAG_FIT_PARAM_COUNT; row++) // 接受候选参数
            {
                params[row] = g_mag_fit_candidate[row]; // 写回当前参数
            }
            current_error = candidate_error; // 更新当前误差
            damping *= 0.35f; // 误差下降，减小阻尼让收敛更快
            if (damping < 1.0e-7f) // 限制最小阻尼
            {
                damping = 1.0e-7f; // 保持基本稳定性
            }

            if (max_delta < 1.0e-5f) // 参数变化已经很小
            {
                break; // 认为拟合已收敛
            }
        }
        else
        {
            damping *= 10.0f; // 误差未下降，增大阻尼缩小步长
            if (damping > 1.0e6f) // 阻尼过大说明拟合难以收敛
            {
                return 0U; // 判定拟合失败
            }
        }
    }

    *fit_error = current_error; // 输出最终拟合误差
    return 1U; // 椭球拟合成功
}
/**
 * @brief 读取并校验 MPU9250 设备 ID。
 * @retval 0 设备正确；-1 读取失败；-2 ID 不匹配。
 */
static int mpu9250_check_device(void)
{
    uint8_t id = 0U; // 保存 WHO_AM_I 读取结果

    if (MPU9250_Driver_ReadWhoAmI(&id) == 0U) // 读取 MPU9250 设备 ID
    {
        Debug_Print("[MPU9250] WHO_AM_I read failed\r\n"); // 输出 ID 读取失败日志
        return -1; // I2C 读取失败
    }

    Debug_Printf("[MPU9250] WHO_AM_I = 0x%02X\r\n", id); // 打印实际读到的设备 ID

    if (id != MPU9250_WHO_AM_I_VALUE) // 检查 ID 是否等于 MPU9250 预期值
    {
        Debug_Print("[MPU9250] WHO_AM_I value error\r\n"); // 输出 ID 不匹配日志
        return -2; // 设备 ID 错误
    }

    Debug_Print("[MPU9250] WHO_AM_I check ok\r\n"); // 输出设备检查成功日志
    return 0; // 设备检查通过
}

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
 * @brief 初始化 AK8963 磁力计。
 *
 * 主要完成旁路访问、设备 ID 检查、灵敏度读取和连续测量模式配置。
 *
 * @retval 1 初始化成功；0 初始化失败。
 */
static uint8_t ak8963_init(void)
{
    mpu_set_ak8963_by_mcu(); // 允许 MCU 通过 MPU9250 旁路访问 AK8963

    if (AK8963_CheckDeviceID() != 0) // 检查 AK8963 设备 ID
    {
        Debug_Print("[AK8963] device ID check failed\r\n"); // 输出设备 ID 检查失败日志
        return 0U; // 磁力计设备检查失败
    }

    Debug_Print("[AK8963] device ID check ok\r\n"); // 输出设备 ID 检查成功日志

    AK8963_EnterPowerDownMode(); // 先进入掉电模式，满足模式切换要求
    AK8963_EnterFuseROMMode(); // 进入 Fuse ROM 模式读取灵敏度调整值
    AK8963_AdjustSensitivity(); // 读取并计算三轴灵敏度修正系数
    AK8963_EnterPowerDownMode(); // 退出 Fuse ROM 前先回到掉电模式
    AK8963_EnterContinuousMeasurementMode(); // 进入 16 位连续测量模式

    if (AK8963_CheckDataReady() == 0) // 确认磁力计是否已有有效数据
    {
        Debug_Print("[AK8963] data not ready\r\n"); // 输出数据未就绪日志
        return 0U; // 磁力计未就绪
    }

    Debug_Print("[AK8963] data ready\r\n"); // 输出磁力计数据就绪日志
    return 1U; // 磁力计初始化成功
}

/**
 * @brief 读取 MPU9250 的 WHO_AM_I 设备 ID。
 * @param id 输出设备 ID。
 * @retval 1 读取成功；0 读取失败或参数无效。
 */
uint8_t MPU9250_Driver_ReadWhoAmI(uint8_t *id)
{
    int ret; // I2C 读取返回值

    if (id == 0) // 检查输出指针
    {
        return 0U; // 参数无效
    }

    ret = I2C_ReadRegData(MPU9250_I2C_ADDR7, MPU9250_REG_WHO_AM_I, id); // 读取 WHO_AM_I 寄存器
    return (ret == 0) ? 1U : 0U; // I2C 成功时返回 1
}

/**
 * @brief 软件复位 MPU9250。
 * @retval None
 */
void MPU9250_SoftReset(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG); // 读取电源管理寄存器

    val &= (uint8_t)~(1U << 7U); // 先清除复位位，保证写入动作明确
    val |= (uint8_t)(1U << 7U); // 置位 DEVICE_RESET 位
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG, val); // 写回触发芯片复位

    vTaskDelay(pdMS_TO_TICKS(100)); // 等待复位完成
}

/**
 * @brief 打印 MPU9250 电源管理寄存器，用于初始化调试确认。
 * @retval None
 */
void MPU9250_Read_PowerMgmt(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG); // 读取 PWR_MGMT_1 当前值

    Debug_Printf("[MPU9250] PWR_MGMT_1 = 0x%02X\r\n", val); // 打印电源管理状态
}

/**
 * @brief 唤醒 MPU9250，并选择 PLL 时钟源。
 * @retval None
 */
void mpu_set_clock_to_auto(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG); // 读取电源管理寄存器

    val &= (uint8_t)~((1U << 6U) | 0x07U); // 清除睡眠位和时钟选择位
    val |= 0x01U; // 选择 X 轴陀螺仪 PLL 作为时钟源
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG, val); // 写回唤醒和时钟配置
}

/**
 * @brief 使能 MPU9250 的三轴加速度计和三轴陀螺仪。
 * @retval None
 */
void mpu_enable_six_axis(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_2_REG); // 读取六轴待机控制寄存器

    val &= (uint8_t)~0x3FU; // 清除六个轴的 standby 位
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_2_REG, val); // 写回六轴使能配置
}

/**
 * @brief 配置陀螺仪 DLPF 为 CFG=3。
 * @retval None
 */
void mpu_set_dlpf_cfg_3(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_CONFIG_REG); // 读取陀螺仪滤波配置

    val = (uint8_t)((val & (uint8_t)~0x07U) | 0x03U); // 设置 DLPF_CFG=3
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_CONFIG_REG, val); // 写回陀螺仪滤波配置
}

/**
 * @brief 设置六轴共用采样率分频，当前约 200Hz。
 * @retval None
 */
void mpu_set_sample_rate_200hz(void)
{
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_SMPLRT_DIV_REG, 0x04U); // 1kHz 基准下分频为 200Hz
}

/**
 * @brief 配置加速度计低通滤波。
 * @retval None
 */
void mpu_set_accel_dlpf(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG2_REG); // 读取加速度计滤波配置

    val &= (uint8_t)~0x07U; // 清除 A_DLPFCFG 位
    val |= 0x03U; // 设置加速度计 DLPF_CFG=3
    val &= (uint8_t)~(1U << 3U); // 关闭加速度计高带宽旁路
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG2_REG, val); // 写回加速度计滤波配置
}

/**
 * @brief 配置陀螺仪量程为 +-1000dps。
 * @retval None
 */
void mpu_set_gyro_config(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_GYRO_CONFIG_REG); // 读取陀螺仪量程配置

    val &= (uint8_t)~0x03U; // 清除自检相关低位
    val &= (uint8_t)~(3U << 3U); // 清除 FS_SEL 量程位
    val |= (uint8_t)(2U << 3U); // 设置 FS_SEL=2，对应 +-1000dps
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_GYRO_CONFIG_REG, val); // 写回陀螺仪量程配置
}

/**
 * @brief 配置加速度计量程为 +-8g。
 * @retval None
 */
void mpu_set_accel_range(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG_REG); // 读取加速度计量程配置

    val &= (uint8_t)~(3U << 3U); // 清除 AFS_SEL 量程位
    val |= (uint8_t)(2U << 3U); // 设置 AFS_SEL=2，对应 +-8g
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG_REG, val); // 写回加速度计量程配置
}

/**
 * @brief 读取 MPU9250 加速度计、陀螺仪和温度原始寄存器。
 * @param raw 传感器原始采样数据结构体。
 * @retval None
 */
void MPU9250_ReadAxis(MPU9250_raw_Data *raw)
{
    uint8_t frame[14]; // ACCEL_XOUT_H 到 GYRO_ZOUT_L 的连续读取缓存

    if (raw == 0) // 检查输出结构体指针
    {
        return; // 参数无效，直接返回
    }

    if (I2C_ReadRegs(MPU9250_I2C_ADDR7, MPU9250_ACCEL_XOUT_H_REG, frame, (uint16_t)sizeof(frame)) != 0) // 一次读取六轴和温度
    {
        raw->accel_x = 0; // 读取失败时清空本帧，避免沿用脏数据
        raw->accel_y = 0;
        raw->accel_z = 0;
        raw->gyro_x = 0;
        raw->gyro_y = 0;
        raw->gyro_z = 0;
        raw->temp = 0;
        return;
    }

    raw->accel_x = (int16_t)(((uint16_t)frame[0] << 8U) | (uint16_t)frame[1]); // 解析加速度计 X 轴
    raw->accel_y = (int16_t)(((uint16_t)frame[2] << 8U) | (uint16_t)frame[3]); // 解析加速度计 Y 轴
    raw->accel_z = (int16_t)(((uint16_t)frame[4] << 8U) | (uint16_t)frame[5]); // 解析加速度计 Z 轴
    raw->temp = (int16_t)(((uint16_t)frame[6] << 8U) | (uint16_t)frame[7]); // 解析温度原始值
    raw->gyro_x = (int16_t)(((uint16_t)frame[8] << 8U) | (uint16_t)frame[9]); // 解析陀螺仪 X 轴
    raw->gyro_y = (int16_t)(((uint16_t)frame[10] << 8U) | (uint16_t)frame[11]); // 解析陀螺仪 Y 轴
    raw->gyro_z = (int16_t)(((uint16_t)frame[12] << 8U) | (uint16_t)frame[13]); // 解析陀螺仪 Z 轴
}

/**
 * @brief 将 MPU9250 原始值转换成 g、dps 和摄氏度物理量。
 * @param raw 传感器原始采样数据结构体。
 * @param physical MPU9250 六轴物理量输出结构体。
 * @retval None
 */
void MPU9250_ConvertToPhysical(const MPU9250_raw_Data *raw, MPU9250_Physical_Data *physical)
{
    if ((raw == 0) || (physical == 0)) // 检查输入和输出指针
    {
        return; // 参数无效，直接返回
    }

    physical->accel_x_g = (float)raw->accel_x / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[0]; // 转换 X 轴加速度并扣除零偏
    physical->accel_y_g = (float)raw->accel_y / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[1]; // 转换 Y 轴加速度并扣除零偏
    physical->accel_z_g = (float)raw->accel_z / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[2]; // 转换 Z 轴加速度并扣除零偏
    physical->gyro_x_dps = (float)raw->gyro_x / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[0]; // 转换 X 轴角速度并扣除零偏
    physical->gyro_y_dps = (float)raw->gyro_y / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[1]; // 转换 Y 轴角速度并扣除零偏
    physical->gyro_z_dps = (float)raw->gyro_z / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[2]; // 转换 Z 轴角速度并扣除零偏
    physical->temp_c = ((float)raw->temp / 333.87f) + 21.0f; // 转换芯片温度，单位摄氏度
}

/**
 * @brief 配置 MPU9250 旁路模式，让 MCU 直接访问 AK8963。
 * @retval None
 */
void mpu_set_ak8963_by_mcu(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_USER_CTRL_REG); // 读取用户控制寄存器

    val &= (uint8_t)~(1U << 5U); // 关闭 MPU9250 内部 I2C 主机模式
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_USER_CTRL_REG, val); // 写回用户控制寄存器

    val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_INT_PIN_CFG_REG); // 读取中断引脚配置寄存器
    val |= (uint8_t)(1U << 1U); // 打开 BYPASS_EN，使外部 MCU 可访问 AK8963
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_INT_PIN_CFG_REG, val); // 写回旁路配置
}

/**
 * @brief 检查 AK8963 磁力计设备 ID。
 * @retval 0 ID 正确；-1 ID 不匹配。
 */
int AK8963_CheckDeviceID(void)
{
    uint8_t device_id = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_WIA); // 读取 WIA 设备 ID

    return (device_id == AK8963_WHO_AM_I_VALUE) ? 0 : -1; // 返回 ID 校验结果
}

/**
 * @brief 让 AK8963 进入掉电模式。
 * @retval None
 */
void AK8963_EnterPowerDownMode(void)
{
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x00U); // 写入 Power-down 模式
    vTaskDelay(pdMS_TO_TICKS(10)); // 等待模式切换完成
}

/**
 * @brief 让 AK8963 进入 Fuse ROM 模式。
 * @retval None
 */
void AK8963_EnterFuseROMMode(void)
{
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x0FU); // 写入 Fuse ROM access 模式
    vTaskDelay(pdMS_TO_TICKS(10)); // 等待模式切换完成
}

/**
 * @brief 读取 AK8963 出厂灵敏度调整值并计算修正系数。
 * @retval None
 */
void AK8963_AdjustSensitivity(void)
{
    g_ak8963_asa[0] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAX); // 读取 X 轴 ASA
    g_ak8963_asa[1] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAY); // 读取 Y 轴 ASA
    g_ak8963_asa[2] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAZ); // 读取 Z 轴 ASA

    g_ak8963_sensitivity[0] = (((float)g_ak8963_asa[0] - 128.0f) * 0.5f / 128.0f) + 1.0f; // 计算 X 轴灵敏度系数
    g_ak8963_sensitivity[1] = (((float)g_ak8963_asa[1] - 128.0f) * 0.5f / 128.0f) + 1.0f; // 计算 Y 轴灵敏度系数
    g_ak8963_sensitivity[2] = (((float)g_ak8963_asa[2] - 128.0f) * 0.5f / 128.0f) + 1.0f; // 计算 Z 轴灵敏度系数
}

/**
 * @brief 让 AK8963 进入 16 位连续测量模式。
 * @retval None
 */
void AK8963_EnterContinuousMeasurementMode(void)
{
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x16U); // 设置 16 位连续测量模式 2
    vTaskDelay(pdMS_TO_TICKS(10)); // 等待模式切换完成
}

/**
 * @brief 检查 AK8963 磁力计数据是否就绪。
 * @retval 1 数据就绪；0 数据未就绪。
 */
int AK8963_CheckDataReady(void)
{
    uint8_t status = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ST1); // 读取 ST1 状态寄存器

    return ((status & 0x01U) != 0U) ? 1 : 0; // 检查 DRDY 位
}

/**
 * @brief 读取 AK8963 三轴磁力计原始值。
 * @param raw 传感器原始采样数据结构体。
 * @retval 0 读取成功；负数表示数据未就绪、参数无效或磁场溢出。
 */
int AK8963_Read_Axis(AK8963_raw_Data *raw)
{
    uint8_t frame[7]; // HXL 到 ST2 的一次性读取缓存

    if (raw == 0) // 检查输出指针
    {
        return -2; // 参数无效
    }

    if (AK8963_CheckDataReady() == 0) // 检查磁力计数据是否就绪
    {
        return -1; // 数据未就绪
    }

    /* 连续读取 HXL..ST2，确保 AK8963 释放本帧锁存数据。 */
    if (I2C_ReadRegs(AK8963_I2C_ADDR7, AK8963_REG_HXL, frame, (uint16_t)sizeof(frame)) != 0) // 读取三轴数据和 ST2
    {
        return -2; // I2C 读取失败
    }

    raw->mag_x = (int16_t)(((uint16_t)frame[1] << 8U) | (uint16_t)frame[0]); // 合成 X 轴原始值
    raw->mag_y = (int16_t)(((uint16_t)frame[3] << 8U) | (uint16_t)frame[2]); // 合成 Y 轴原始值
    raw->mag_z = (int16_t)(((uint16_t)frame[5] << 8U) | (uint16_t)frame[4]); // 合成 Z 轴原始值

    if ((frame[6] & 0x08U) != 0U) // 检查磁力计溢出标志 HOFL
    {
        return -3; // 磁力计数据溢出
    }

    return 0; // 磁力计读取成功
}

/**
 * @brief 将 AK8963 原始磁场值转换并应用校准。
 * @param raw 传感器原始采样数据结构体。
 * @param physical MPU9250 六轴物理量输出结构体。
 * @retval None
 */
void AK8963_Calibrate(const AK8963_raw_Data *raw, AK8963_Physical_Data *physical)
{
    ak8963_convert_raw_to_ut(raw, physical); // 原始值转换为 uT
    ak8963_apply_mag_calibration(physical); // 应用硬铁和软铁校准
}

/**
 * @brief 读取并校准 AK8963 磁力计物理量。
 * @param mag_out 输出磁场物理量，单位 uT。
 * @retval 0 成功；负值表示读取失败原因。
 */
int AK8963_Read_Mag_UT(AK8963_Physical_Data *mag_out)
{
    AK8963_raw_Data raw; // 暂存磁力计原始值
    int ret; // 磁力计读取结果

    if (mag_out == 0) // 检查输出指针
    {
        return -2; // 参数无效
    }

    ret = AK8963_Read_Axis(&raw); // 读取磁力计三轴原始值
    if (ret != 0) // 原始值读取失败
    {
        return ret; // 保留底层错误码
    }

    AK8963_Calibrate(&raw, mag_out); // 转换并校准磁场物理量
    return 0; // 磁力计物理量读取成功
}

/**
 * @brief 静止采样陀螺仪，计算三轴零偏。
 * @param samples 校准采样次数。
 * @param delay_ms 采样间隔毫秒数。
 * @retval None
 */
void MPU9250_CalibrateGyro(uint16_t samples, uint16_t delay_ms)
{
    MPU9250_raw_Data raw; // 暂存 MPU9250 原始值
    int32_t gyro_x_sum = 0; // X 轴陀螺仪原始值累加
    int32_t gyro_y_sum = 0; // Y 轴陀螺仪原始值累加
    int32_t gyro_z_sum = 0; // Z 轴陀螺仪原始值累加
    uint16_t i; // 采样索引

    if (samples == 0U) // 检查采样次数
    {
        AppStatus_SetCalState(APP_CAL_STATE_FAIL); // 采样参数异常，提示标定失败
        return; // 无采样请求，直接返回
    }

    for (i = 0U; i < samples; i++) // 连续采样陀螺仪静止数据
    {
        MPU9250_ReadAxis(&raw); // 读取六轴原始值
        gyro_x_sum += raw.gyro_x; // 累加 X 轴陀螺仪原始值
        gyro_y_sum += raw.gyro_y; // 累加 Y 轴陀螺仪原始值
        gyro_z_sum += raw.gyro_z; // 累加 Z 轴陀螺仪原始值
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // 按指定间隔等待下一次采样
    }

    g_gyro_bias_dps[0] = ((float)gyro_x_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS; // 保存 X 轴零偏 dps
    g_gyro_bias_dps[1] = ((float)gyro_y_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS; // 保存 Y 轴零偏 dps
    g_gyro_bias_dps[2] = ((float)gyro_z_sum / (float)samples) / MPU9250_GYRO_LSB_PER_DPS; // 保存 Z 轴零偏 dps
}

/**
 * @brief 静止水平采样加速度计，计算三轴零偏。
 * @param samples 校准采样次数。
 * @param delay_ms 采样间隔毫秒数。
 * @retval None
 */
void MPU9250_CalibrateAccel(uint16_t samples, uint16_t delay_ms)
{
    MPU9250_raw_Data raw; // 暂存 MPU9250 原始值
    float ax_sum = 0.0f; // X 轴加速度 g 值累加
    float ay_sum = 0.0f; // Y 轴加速度 g 值累加
    float az_sum = 0.0f; // Z 轴加速度 g 值累加
    uint16_t i; // 采样索引

    if (samples == 0U) // 检查采样次数
    {
        AppStatus_SetCalState(APP_CAL_STATE_FAIL); // 采样参数异常，提示标定失败
        return; // 无采样请求，直接返回
    }

    for (i = 0U; i < samples; i++) // 连续采样静止加速度数据
    {
        MPU9250_ReadAxis(&raw); // 读取六轴原始值
        ax_sum += (float)raw.accel_x / MPU9250_ACCEL_LSB_PER_G; // 累加 X 轴加速度 g 值
        ay_sum += (float)raw.accel_y / MPU9250_ACCEL_LSB_PER_G; // 累加 Y 轴加速度 g 值
        az_sum += (float)raw.accel_z / MPU9250_ACCEL_LSB_PER_G; // 累加 Z 轴加速度 g 值
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // 按指定间隔等待下一次采样
    }

    g_accel_bias_g[0] = ax_sum / (float)samples; // 保存 X 轴零偏
    g_accel_bias_g[1] = ay_sum / (float)samples; // 保存 Y 轴零偏
    g_accel_bias_g[2] = (az_sum / (float)samples) - 1.0f; // 保存 Z 轴零偏，扣除静止重力 1g
}

/**
 * @brief 采集磁力计样本并计算硬铁和软铁校准参数。
 * @param samples 校准采样次数。
 * @param delay_ms 采样间隔毫秒数。
 * @retval None
 */
void AK8963_CalibrateMag(uint16_t samples, uint16_t delay_ms)
{
    AK8963_raw_Data raw; // 暂存磁力计原始值
    AK8963_Physical_Data mag; // 暂存未应用硬铁/软铁的磁场物理量
    float min_x = 99999.0f; // X 轴采样最小值
    float min_y = 99999.0f; // Y 轴采样最小值
    float min_z = 99999.0f; // Z 轴采样最小值
    float max_x = -99999.0f; // X 轴采样最大值
    float max_y = -99999.0f; // Y 轴采样最大值
    float max_z = -99999.0f; // Z 轴采样最大值
    float radius_x; // X 轴初始半径估计
    float radius_y; // Y 轴初始半径估计
    float radius_z; // Z 轴初始半径估计
    float seed_center_x; // X 轴初始中心估计
    float seed_center_y; // Y 轴初始中心估计
    float seed_center_z; // Z 轴初始中心估计
    float radius_avg; // 三轴初始半径平均值
    float fit_params[AK8963_MAG_FIT_PARAM_COUNT]; // 椭球拟合参数
    float fit_error = 0.0f; // 椭球拟合残差
    float fitted_radius_ut = 0.0f; // 拟合得到的平均磁场半径
    float candidate_correction[3][3]; // 候选软铁修正矩阵
    uint16_t valid_count = 0U; // 有效磁力计样本数
    uint16_t requested_samples = samples; // 用户请求采样数
    uint16_t not_ready_count = 0U; // 数据未就绪次数
    uint16_t read_fail_count = 0U; // I2C 读取失败次数
    uint16_t overflow_count = 0U; // 磁力计溢出次数
    uint16_t i; // 采样索引
    uint8_t row; // 矩阵行索引
    uint8_t col; // 矩阵列索引
    int read_ret; // 单次磁力计读取结果

    if (samples == 0U) // 检查采样次数
    {
        AppStatus_SetCalState(APP_CAL_STATE_FAIL); // 采样参数异常，提示标定失败
        return; // 无采样请求，直接返回
    }

    if (samples > AK8963_MAG_CAL_MAX_SAMPLES) // 限制采样数不超过缓存容量
    {
        samples = AK8963_MAG_CAL_MAX_SAMPLES; // 截断到最大缓存容量
        Debug_Printf("[AK8963] mag calibration samples limited %u->%u\r\n",
                     requested_samples,
                     samples); // 提示采样数被限制
    }

    Debug_Print("[AK8963] mag calibration preparing\r\n"); // 提示磁力计校准准备开始
    Debug_Print("[AK8963] get ready to rotate slowly through all 3 axes\r\n"); // 提示需要做完整 3D 转动
    Debug_Print("[AK8963] start in 3...\r\n"); // 倒计时 3 秒
    vTaskDelay(pdMS_TO_TICKS(1000)); // 等待用户准备
    Debug_Print("[AK8963] start in 2...\r\n"); // 倒计时 2 秒
    vTaskDelay(pdMS_TO_TICKS(1000)); // 等待用户准备
    Debug_Print("[AK8963] start in 1...\r\n"); // 倒计时 1 秒
    vTaskDelay(pdMS_TO_TICKS(1000)); // 等待用户准备
    AppStatus_SetCalState(APP_CAL_STATE_MAG_ROTATE); // 进入磁力计旋转校准灯语
    Debug_Print("[AK8963] mag calibration begin,  keep moving\r\n"); // 开始采集磁力计点云

    for (i = 0U; i < samples; i++) // 按指定次数采集磁力计样本
    {
        read_ret = AK8963_Read_Axis(&raw); // 读取一帧磁力计原始值
        if (read_ret == 0) // 本帧读取成功
        {
            ak8963_convert_raw_to_ut(&raw, &mag); // 转换为未校准的 uT 物理量

#if (APP_MAG_POINT_CLOUD_DEBUG_ENABLE != 0U) // 可选输出点云，用于离线检查采样覆盖
            if ((APP_MAG_POINT_CLOUD_PRINT_DIV == 0U) ||
                ((valid_count % APP_MAG_POINT_CLOUD_PRINT_DIV) == 0U)) // 按配置分频打印点云
            {
                Debug_Printf("[AK8963_RAW_UT] %.2f,%.2f,%.2f\r\n",
                             mag.mag_x_ut,
                             mag.mag_y_ut,
                             mag.mag_z_ut); // 输出一组三轴磁场点
            }
#endif

            if (valid_count < AK8963_MAG_CAL_MAX_SAMPLES) // 缓存仍有空间
            {
                g_mag_cal_samples[valid_count] = mag; // 保存有效样本供椭球拟合使用
            }

            if (mag.mag_x_ut < min_x) // 更新 X 轴最小值
            {
                min_x = mag.mag_x_ut;
            }
            if (mag.mag_x_ut > max_x) // 更新 X 轴最大值
            {
                max_x = mag.mag_x_ut;
            }
            if (mag.mag_y_ut < min_y) // 更新 Y 轴最小值
            {
                min_y = mag.mag_y_ut;
            }
            if (mag.mag_y_ut > max_y) // 更新 Y 轴最大值
            {
                max_y = mag.mag_y_ut;
            }
            if (mag.mag_z_ut < min_z) // 更新 Z 轴最小值
            {
                min_z = mag.mag_z_ut;
            }
            if (mag.mag_z_ut > max_z) // 更新 Z 轴最大值
            {
                max_z = mag.mag_z_ut;
            }
            valid_count++; // 记录有效样本数
        }
        else if (read_ret == -1) // 磁力计数据未就绪
        {
            not_ready_count++; // 累计未就绪次数
        }
        else if (read_ret == -2) // 磁力计 I2C 读取失败
        {
            read_fail_count++; // 累计读取失败次数
        }
        else if (read_ret == -3) // 磁力计数据溢出
        {
            overflow_count++; // 累计溢出次数
        }

        if ((((uint16_t)(i + 1U)) % 50U) == 0U) // 每 50 次采样输出一次进度
        {
            Debug_Printf("[AK8963] mag calibration progress %u/%u valid=%u nr=%u io=%u ov=%u\r\n",
                         (uint16_t)(i + 1U),
                         samples,
                         valid_count,
                         not_ready_count,
                         read_fail_count,
                         overflow_count); // 输出有效样本和错误统计
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // 按指定间隔等待下一次采样
    }

    if (valid_count < AK8963_MAG_FIT_MIN_VALID_SAMPLES) // 有效样本太少无法稳定拟合
    {
        Debug_Printf("[AK8963] calibrate mag rejected valid=%u\r\n", valid_count); // 输出样本不足原因
        AppStatus_SetCalState(APP_CAL_STATE_FAIL); // 提示磁力计标定失败
        return; // 放弃本次校准
    }

    radius_x = (max_x - min_x) * 0.5f; // 用 min/max 估算 X 轴半径
    radius_y = (max_y - min_y) * 0.5f; // 用 min/max 估算 Y 轴半径
    radius_z = (max_z - min_z) * 0.5f; // 用 min/max 估算 Z 轴半径

    seed_center_x = (min_x + max_x) * 0.5f; // 用 min/max 估算 X 轴中心
    seed_center_y = (min_y + max_y) * 0.5f; // 用 min/max 估算 Y 轴中心
    seed_center_z = (min_z + max_z) * 0.5f; // 用 min/max 估算 Z 轴中心
    radius_avg = (radius_x + radius_y + radius_z) / 3.0f; // 估算平均磁场半径
    Debug_Printf("[AK8963] raw bounds x=%.1f..%.1f y=%.1f..%.1f z=%.1f..%.1f\r\n",
                 min_x, max_x, min_y, max_y, min_z, max_z); // 输出三轴采样范围
    Debug_Printf("[AK8963] seed center=%.1f/%.1f/%.1f radius=%.1f/%.1f/%.1f avg=%.1f\r\n",
                 seed_center_x,
                 seed_center_y,
                 seed_center_z,
                 radius_x,
                 radius_y,
                 radius_z,
                 radius_avg); // 输出椭球拟合初值

    if ((radius_x < AK8963_MAG_RADIUS_MIN_UT) ||
        (radius_y < AK8963_MAG_RADIUS_MIN_UT) ||
        (radius_z < AK8963_MAG_RADIUS_MIN_UT)) // 任意轴覆盖范围过小都说明采样不足
    {
        Debug_Printf("[AK8963] calibrate mag rejected radius=%.1f/%.1f/%.1f\r\n",
                     radius_x,
                     radius_y,
                     radius_z); // 输出被拒绝的三轴半径
        AppStatus_SetCalState(APP_CAL_STATE_FAIL); // 提示磁力计标定失败
        return; // 放弃本次校准
    }

    fit_params[0] = (min_x + max_x) * 0.5f; // 椭球中心 X 初值
    fit_params[1] = (min_y + max_y) * 0.5f; // 椭球中心 Y 初值
    fit_params[2] = (min_z + max_z) * 0.5f; // 椭球中心 Z 初值
    fit_params[3] = 1.0f / (radius_x * radius_x); // xx 二次项初值
    fit_params[4] = 0.0f; // xy 交叉项初值
    fit_params[5] = 0.0f; // xz 交叉项初值
    fit_params[6] = 1.0f / (radius_y * radius_y); // yy 二次项初值
    fit_params[7] = 0.0f; // yz 交叉项初值
    fit_params[8] = 1.0f / (radius_z * radius_z); // zz 二次项初值

    if (ak8963_fit_ellipsoid(valid_count, fit_params, &fit_error) == 0U) // 执行椭球拟合
    {
        Debug_Print("[AK8963] calibrate mag rejected fit failed\r\n"); // 输出拟合失败原因
        AppStatus_SetCalState(APP_CAL_STATE_FAIL); // 提示磁力计标定失败
        return; // 放弃本次校准
    }

    if (ak8963_build_softiron_matrix(fit_params, candidate_correction, &fitted_radius_ut) == 0U) // 生成软铁矩阵
    {
        Debug_Print("[AK8963] calibrate mag rejected invalid ellipsoid\r\n"); // 输出椭球参数无效原因
        AppStatus_SetCalState(APP_CAL_STATE_FAIL); // 提示磁力计标定失败
        return; // 放弃本次校准
    }

    if ((fit_params[0] < min_x) || (fit_params[0] > max_x) ||
        (fit_params[1] < min_y) || (fit_params[1] > max_y) ||
        (fit_params[2] < min_z) || (fit_params[2] > max_z) ||
        (fitted_radius_ut < (radius_avg * 0.5f)) ||
        (fitted_radius_ut > (radius_avg * 1.5f))) // 拟合中心或半径明显偏离采样范围
    {
        Debug_Printf("[AK8963] calibrate mag rejected implausible fit center=%.1f/%.1f/%.1f radius=%.1f seed_avg=%.1f\r\n",
                     fit_params[0],
                     fit_params[1],
                     fit_params[2],
                     fitted_radius_ut,
                     radius_avg); // 输出不可信拟合结果
        AppStatus_SetCalState(APP_CAL_STATE_FAIL); // 提示磁力计标定失败
        return; // 放弃本次校准
    }

    g_mag_offset_ut[0] = fit_params[0]; // 保存 X 轴硬铁偏移
    g_mag_offset_ut[1] = fit_params[1]; // 保存 Y 轴硬铁偏移
    g_mag_offset_ut[2] = fit_params[2]; // 保存 Z 轴硬铁偏移
    for (row = 0U; row < 3U; row++) // 拷贝软铁修正矩阵每一行
    {
        for (col = 0U; col < 3U; col++) // 拷贝当前行每一列
        {
            g_mag_correction[row][col] = candidate_correction[row][col]; // 保存软铁修正矩阵元素
        }
    }

    Debug_Printf("[AK8963] calibrate mag ok center=%.1f/%.1f/%.1f radius=%.1f fit=%.4f\r\n",
                 g_mag_offset_ut[0],
                 g_mag_offset_ut[1],
                 g_mag_offset_ut[2],
                 fitted_radius_ut,
                 fit_error); // 输出最终中心、半径和拟合误差
    Debug_Printf("[AK8963] mag matrix row0=%.3f,%.3f,%.3f\r\n",
                 g_mag_correction[0][0],
                 g_mag_correction[0][1],
                 g_mag_correction[0][2]); // 输出软铁矩阵第 0 行
    Debug_Printf("[AK8963] mag matrix row1=%.3f,%.3f,%.3f\r\n",
                 g_mag_correction[1][0],
                 g_mag_correction[1][1],
                 g_mag_correction[1][2]); // 输出软铁矩阵第 1 行
    Debug_Printf("[AK8963] mag matrix row2=%.3f,%.3f,%.3f\r\n",
                 g_mag_correction[2][0],
                 g_mag_correction[2][1],
                 g_mag_correction[2][2]); // 输出软铁矩阵第 2 行
    Debug_Print("[AK8963] mag calibration done\r\n"); // 输出磁力计校准完成日志
    AppStatus_SetCalState(APP_CAL_STATE_SUCCESS); // 提示标定成功
}

/**
 * @brief 初始化 MPU9250、AK8963，并完成启动阶段校准。
 *
 * 主要做四件事：
 * 1. 初始化软件 I2C；
 * 2. 检查 MPU9250 并配置六轴；
 * 3. 初始化 AK8963 磁力计；
 * 4. 执行陀螺仪、加速度计、磁力计校准。
 *
 * @retval 1 初始化成功；0 初始化失败。
 */
uint8_t MPU9250_Driver_Init(void)
{
    AppStatus_SetCalState(APP_CAL_STATE_SELF_CHECK); // 进入上电自检灯语

    /* 1. 初始化软件 I2C */
    BSP_I2C_Soft_Init(); // 初始化软件 I2C 总线

    /* 2. 检查 MPU9250 并配置六轴 */
    if (mpu9250_check_device() != 0) // 检查 MPU9250 设备 ID
    {
        Debug_Print("[MPU9250] driver init failed\r\n"); // 输出 MPU9250 检查失败日志
        AppStatus_SetCalState(APP_CAL_STATE_FAIL); // 提示初始化失败
        return 0U; // MPU9250 检查失败
    }

    mpu9250_config_six_axis(); // 配置加速度计和陀螺仪参数

    /* 3. 初始化 AK8963 磁力计 */
    if (ak8963_init() == 0U) // 初始化磁力计并读取灵敏度参数
    {
        AppStatus_SetCalState(APP_CAL_STATE_FAIL); // 提示磁力计初始化失败
        return 0U; // 磁力计初始化失败
    }

    /* 4. 启动阶段校准 */
    AppStatus_SetCalState(APP_CAL_STATE_STATIC); // 进入陀螺仪和加速度计静止校准灯语
    MPU9250_CalibrateGyro(1000U, 10U); // 校准陀螺仪零偏
    MPU9250_CalibrateAccel(1000U, 10U); // 校准加速度计零偏
    AK8963_CalibrateMag(500U, 20U); // 校准磁力计硬铁和软铁误差
    Debug_Print("[MPU9250] driver init ok\r\n"); // 输出驱动初始化成功日志

    return 1U; // 驱动初始化成功
}

/*=================================================================================姿态解算与融合=================================================================================*/

// EulerAngle_t g_euler_acc_mag = {0.0f, 0.0f, 0.0f}; // 旧的加速度计+磁力计直接解算角，目前保留为学习/调试参考
EulerAngle_t g_euler_fused = {0.0f, 0.0f, 0.0f}; // Mahony 融合后的欧拉角，内部单位为弧度

static Quaternion_t g_q = {1.0f, 0.0f, 0.0f, 0.0f}; // 当前姿态四元数
static float g_exInt = 0.0f; // Mahony X 轴积分误差
static float g_eyInt = 0.0f; // Mahony Y 轴积分误差
static float g_ezInt = 0.0f; // Mahony Z 轴积分误差
static float g_kp = 0.3f; // Mahony 比例修正增益
static float g_ki = 0.0f; // Mahony 积分修正增益
static float g_acc_norm_prev = 1.0f; // 上一次加速度模长，用于动态调节修正强度

/**
 * @brief 将浮点数限制在指定范围内。
 * @param value 输入值。
 * @param min_value 下限。
 * @param max_value 上限。
 * @retval 限幅后的值。
 */
static float mpu9250_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) // 小于下限
    {
        return min_value; // 返回下限
    }
    if (value > max_value) // 大于上限
    {
        return max_value; // 返回上限
    }
    return value; // 原值已经在范围内
}

/**
 * @brief 将角度归一化到 (-180, 180] 范围。
 * @param angle_deg 输入角度。
 * @retval 归一化后的角度。
 */
static float mpu9250_wrap_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) // 超过正半圈
    {
        angle_deg -= 360.0f; // 向负方向折回一圈
    }
    while (angle_deg <= -180.0f) // 超过负半圈
    {
        angle_deg += 360.0f; // 向正方向折回一圈
    }
    return angle_deg; // 返回归一化角度
}

/**
 * @brief 将数学坐标系 yaw 转为导航航向角。
 * @param math_yaw_deg 数学坐标系 yaw，东为 0 度，逆时针为正。
 * @retval 导航航向角，北为 0 度，东为 +90 度。
 */
static float mpu9250_math_yaw_to_heading_deg(float math_yaw_deg)
{
    /*
     * Internal yaw uses the mathematical convention:
     *   east = 0 deg, counter-clockwise positive.
     * Externally expose the navigation convention used by aircraft:
     *   north = 0 deg, east = +90 deg, range = (-180, 180].
     */
    return mpu9250_wrap_angle_deg(90.0f - math_yaw_deg); // 转换为导航航向角并归一化
}

/**
 * @brief 将当前四元数转换为融合欧拉角。
 * @retval None
 */
static void mpu9250_update_euler_from_quaternion(void)
{
    float q0 = g_q.q0; // 四元数标量分量
    float q1 = g_q.q1; // 四元数 X 分量
    float q2 = g_q.q2; // 四元数 Y 分量
    float q3 = g_q.q3; // 四元数 Z 分量
    float pitch_sin; // pitch 反三角输入值

    g_euler_fused.roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                                1.0f - 2.0f * (q1 * q1 + q2 * q2)); // 由四元数计算 roll

    pitch_sin = 2.0f * (q0 * q2 - q3 * q1); // 由四元数计算 pitch 的正弦值
    pitch_sin = mpu9250_clampf(pitch_sin, -1.0f, 1.0f); // 防止浮点误差导致 asinf 越界
    g_euler_fused.pitch = asinf(pitch_sin); // 由正弦值计算 pitch

    g_euler_fused.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                               1.0f - 2.0f * (q2 * q2 + q3 * q3)); // 由四元数计算 yaw
}

/**
 * @brief mpu9250_integrate_quaternion 函数。
 * @param gx X 轴角速度弧度值。
 * @param gy Y 轴角速度弧度值。
 * @param gz Z 轴角速度弧度值。
 * @param dt 姿态融合采样周期秒数。
 * @retval None
 */
static void mpu9250_integrate_quaternion(float gx, float gy, float gz, float dt)
{
    float q0 = g_q.q0; // 当前四元数标量分量
    float q1 = g_q.q1; // 当前四元数 X 分量
    float q2 = g_q.q2; // 当前四元数 Y 分量
    float q3 = g_q.q3; // 当前四元数 Z 分量
    float qDot0; // 四元数标量分量导数
    float qDot1; // 四元数 X 分量导数
    float qDot2; // 四元数 Y 分量导数
    float qDot3; // 四元数 Z 分量导数
    float norm_q; // 四元数模长

    qDot0 = -0.5f * (q1 * gx + q2 * gy + q3 * gz); // 由角速度计算 q0 导数
    qDot1 =  0.5f * (q0 * gx + q2 * gz - q3 * gy); // 由角速度计算 q1 导数
    qDot2 =  0.5f * (q0 * gy - q1 * gz + q3 * gx); // 由角速度计算 q2 导数
    qDot3 =  0.5f * (q0 * gz + q1 * gy - q2 * gx); // 由角速度计算 q3 导数

    q0 += qDot0 * dt; // 积分更新 q0
    q1 += qDot1 * dt; // 积分更新 q1
    q2 += qDot2 * dt; // 积分更新 q2
    q3 += qDot3 * dt; // 积分更新 q3

    norm_q = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3); // 计算四元数模长
    if (norm_q < 1e-6f) // 四元数异常接近零
    {
        return; // 放弃本次更新
    }

    g_q.q0 = q0 / norm_q; // 归一化 q0
    g_q.q1 = q1 / norm_q; // 归一化 q1
    g_q.q2 = q2 / norm_q; // 归一化 q2
    g_q.q3 = q3 / norm_q; // 归一化 q3

    mpu9250_update_euler_from_quaternion(); // 同步更新融合欧拉角
}

/*
 * 旧的加速度计 + 磁力计直接解算欧拉角路径目前没有参与编译。
 * 运行时使用下面的 Mahony 融合路径：
 *   MPU9250_MahonyUpdate() / MPU9250_MahonyUpdateIMU()
 *   -> mpu9250_update_euler_from_quaternion()
 *   -> MPU9250_GetEulerFusedDeg()
 */

/**
 * @brief 初始化 Mahony 姿态融合四元数和误差积分项。
 * @param kp Mahony 比例修正增益。
 * @param ki Mahony 积分修正增益。
 * @retval None
 */
void MPU9250_MahonyInit(float kp, float ki) // 初始化 Mahony 融合状态
{
    g_kp = kp; // 设置比例修正增益
    g_ki = ki; // 设置积分修正增益

    g_q.q0 = 1.0f; // 初始化单位四元数 q0
    g_q.q1 = 0.0f; // 初始化单位四元数 q1
    g_q.q2 = 0.0f; // 初始化单位四元数 q2
    g_q.q3 = 0.0f; // 初始化单位四元数 q3

    g_exInt = 0.0f; // 清空 X 轴积分误差
    g_eyInt = 0.0f; // 清空 Y 轴积分误差
    g_ezInt = 0.0f; // 清空 Z 轴积分误差
    g_acc_norm_prev = 1.0f; // 初始化上一帧加速度模长

    g_euler_fused.roll = 0.0f; // 清空融合横滚角
    g_euler_fused.pitch = 0.0f; // 清空融合俯仰角
    g_euler_fused.yaw = 0.0f; // 清空融合航向角
}

/**
 * @brief 使用九轴数据执行 Mahony 姿态融合更新。
 * @param imu MPU9250 六轴物理量数据。
 * @param mag AK8963 磁力计物理量数据。
 * @param dt 姿态融合采样周期秒数。
 * @retval None
 */
void MPU9250_MahonyUpdate(const MPU9250_Physical_Data *imu, // 输入加速度计和陀螺仪数据
                          const AK8963_Physical_Data *mag, // 输入磁力计数据
                          float dt) // 输入本次融合周期
{
    const float deg2rad = 0.01745329252f; // 角速度单位从 dps 转 rad/s
    const float mag_min_ut = 15.0f; // 磁场强度有效下限
    const float mag_max_ut = 100.0f; // 磁场强度有效上限
    const float int_lim = 0.1f; // 积分误差限幅
    float ax; // 加速度 X 分量
    float ay; // 加速度 Y 分量
    float az; // 加速度 Z 分量
    float gx; // 角速度 X 分量
    float gy; // 角速度 Y 分量
    float gz; // 角速度 Z 分量
    float mx; // 磁场 X 分量
    float my; // 磁场 Y 分量
    float mz; // 磁场 Z 分量
    float norm_acc; // 加速度向量模长
    float norm_mag; // 磁场向量模长
    float acc_error; // 加速度模长与 1g 的偏差
    float acc_diff; // 当前与上一帧加速度模长变化
    float kp_dynamic; // 动态比例修正增益
    float q0; // 当前四元数 q0
    float q1; // 当前四元数 q1
    float q2; // 当前四元数 q2
    float q3; // 当前四元数 q3
    float vx; // 估计重力方向 X 分量
    float vy; // 估计重力方向 Y 分量
    float vz; // 估计重力方向 Z 分量
    float ex; // 姿态误差 X 分量
    float ey; // 姿态误差 Y 分量
    float ez; // 姿态误差 Z 分量
    float hx; // 磁场参考向量 X 分量
    float hy; // 磁场参考向量 Y 分量
    float hz; // 磁场参考向量 Z 分量
    float bx; // 磁场水平参考分量
    float bz; // 磁场垂直参考分量
    float wx; // 估计磁场方向 X 分量
    float wy; // 估计磁场方向 Y 分量
    float wz; // 估计磁场方向 Z 分量
    uint8_t mag_valid; // 磁力计数据有效标志

    if ((imu == 0) || (mag == 0) || (dt <= 0.0f) || (dt > 0.2f)) // 检查输入指针和周期是否合法
    {
        return; // 输入无效时跳过本次融合
    }

    ax =  imu->accel_x_g; // 取 X 轴加速度
    ay = -imu->accel_y_g; // 取 Y 轴加速度并统一坐标方向
    az =  imu->accel_z_g; // 取 Z 轴加速度

    gx =  imu->gyro_x_dps * deg2rad; // 取 X 轴角速度并转弧度
    gy = -imu->gyro_y_dps * deg2rad; // 取 Y 轴角速度并统一坐标方向
    gz =  imu->gyro_z_dps * deg2rad; // 取 Z 轴角速度并转弧度

    mx =  mag->mag_x_ut; // 取 X 轴磁场
    my = -mag->mag_y_ut; // 取 Y 轴磁场并统一坐标方向
    mz = -mag->mag_z_ut; // 取 Z 轴磁场并统一坐标方向

    norm_acc = sqrtf(ax * ax + ay * ay + az * az); // 计算加速度向量模长
    if (norm_acc < 1e-6f) // 判断加速度是否接近零
    {
        return; // 加速度异常时跳过本次融合
    }

    norm_mag = sqrtf(mx * mx + my * my + mz * mz); // 计算磁场向量模长
    mag_valid = ((norm_mag > mag_min_ut) && (norm_mag < mag_max_ut)) ? 1U : 0U; // 判断磁场强度是否可信

    acc_error = fabsf(norm_acc - 1.0f); // 计算加速度静态偏差
    acc_diff = fabsf(norm_acc - g_acc_norm_prev); // 计算加速度帧间变化
    kp_dynamic = g_kp; // 默认使用配置的比例增益
    if ((acc_error < 0.04f) && (acc_diff < 0.02f)) // 判断当前是否接近静止
    {
        kp_dynamic = g_kp * 2.0f; // 静止时提高加速度修正权重
    }
    else if ((acc_error > 0.25f) || (acc_diff > 0.25f)) // 判断当前是否有明显外加运动
    {
        kp_dynamic = g_kp * 0.35f; // 动态运动时降低加速度修正权重
    }
    g_acc_norm_prev = norm_acc; // 保存本帧加速度模长

    ax /= norm_acc; // 归一化加速度 X 分量
    ay /= norm_acc; // 归一化加速度 Y 分量
    az /= norm_acc; // 归一化加速度 Z 分量

    if (norm_mag > 1e-6f) // 判断磁场向量是否可归一化
    {
        mx /= norm_mag; // 归一化磁场 X 分量
        my /= norm_mag; // 归一化磁场 Y 分量
        mz /= norm_mag; // 归一化磁场 Z 分量
    }
    else // 磁场向量异常
    {
        mag_valid = 0U; // 标记磁力计无效
    }

    q0 = g_q.q0; // 读取当前四元数 q0
    q1 = g_q.q1; // 读取当前四元数 q1
    q2 = g_q.q2; // 读取当前四元数 q2
    q3 = g_q.q3; // 读取当前四元数 q3

    vx = 2.0f * (q1 * q3 - q0 * q2); // 由四元数估计重力 X 分量
    vy = 2.0f * (q0 * q1 + q2 * q3); // 由四元数估计重力 Y 分量
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // 由四元数估计重力 Z 分量

    ex = ay * vz - az * vy; // 计算加速度修正误差 X 分量
    ey = az * vx - ax * vz; // 计算加速度修正误差 Y 分量
    ez = ax * vy - ay * vx; // 计算加速度修正误差 Z 分量

    if (mag_valid != 0U) // 磁力计有效时加入航向修正
    {
        hx = 2.0f * mx * (0.5f - q2 * q2 - q3 * q3) // 计算地磁参考 X 分量
           + 2.0f * my * (q1 * q2 - q0 * q3) // 叠加地磁参考 X 分量
           + 2.0f * mz * (q1 * q3 + q0 * q2); // 完成地磁参考 X 分量
        hy = 2.0f * mx * (q1 * q2 + q0 * q3) // 计算地磁参考 Y 分量
           + 2.0f * my * (0.5f - q1 * q1 - q3 * q3) // 叠加地磁参考 Y 分量
           + 2.0f * mz * (q2 * q3 - q0 * q1); // 完成地磁参考 Y 分量
        hz = 2.0f * mx * (q1 * q3 - q0 * q2) // 计算地磁参考 Z 分量
           + 2.0f * my * (q2 * q3 + q0 * q1) // 叠加地磁参考 Z 分量
           + 2.0f * mz * (0.5f - q1 * q1 - q2 * q2); // 完成地磁参考 Z 分量

        bx = sqrtf(hx * hx + hy * hy); // 计算地磁水平参考量
        bz = hz; // 保存地磁垂直参考量

        wx = 2.0f * bx * (0.5f - q2 * q2 - q3 * q3) // 估计地磁方向 X 分量
           + 2.0f * bz * (q1 * q3 - q0 * q2); // 完成地磁方向 X 分量
        wy = 2.0f * bx * (q1 * q2 - q0 * q3) // 估计地磁方向 Y 分量
           + 2.0f * bz * (q0 * q1 + q2 * q3); // 完成地磁方向 Y 分量
        wz = 2.0f * bx * (q0 * q2 + q1 * q3) // 估计地磁方向 Z 分量
           + 2.0f * bz * (0.5f - q1 * q1 - q2 * q2); // 完成地磁方向 Z 分量

        ex += my * wz - mz * wy; // 叠加磁力计修正误差 X 分量
        ey += mz * wx - mx * wz; // 叠加磁力计修正误差 Y 分量
        ez += mx * wy - my * wx; // 叠加磁力计修正误差 Z 分量
    }

    if (g_ki > 0.0f) // 判断是否启用积分修正
    {
        g_exInt += ex * g_ki * dt; // 累加 X 轴积分误差
        g_eyInt += ey * g_ki * dt; // 累加 Y 轴积分误差
        g_ezInt += ez * g_ki * dt; // 累加 Z 轴积分误差

        g_exInt = mpu9250_clampf(g_exInt, -int_lim, int_lim); // 限制 X 轴积分误差
        g_eyInt = mpu9250_clampf(g_eyInt, -int_lim, int_lim); // 限制 Y 轴积分误差
        g_ezInt = mpu9250_clampf(g_ezInt, -int_lim, int_lim); // 限制 Z 轴积分误差

        gx += g_exInt; // 用积分项修正 X 轴角速度
        gy += g_eyInt; // 用积分项修正 Y 轴角速度
        gz += g_ezInt; // 用积分项修正 Z 轴角速度
    }
    else // 未启用积分修正
    {
        g_exInt = 0.0f; // 清空 X 轴积分误差
        g_eyInt = 0.0f; // 清空 Y 轴积分误差
        g_ezInt = 0.0f; // 清空 Z 轴积分误差
    }

    gx += kp_dynamic * ex; // 用比例项修正 X 轴角速度
    gy += kp_dynamic * ey; // 用比例项修正 Y 轴角速度
    gz += kp_dynamic * ez; // 用比例项修正 Z 轴角速度

    mpu9250_integrate_quaternion(gx, gy, gz, dt); // 积分更新四元数和欧拉角
}

/**
 * @brief 使用六轴 IMU 数据执行 Mahony 姿态融合更新。
 * @param imu MPU9250 六轴物理量数据。
 * @param dt 姿态融合采样周期秒数。
 * @retval None
 */
void MPU9250_MahonyUpdateIMU(const MPU9250_Physical_Data *imu, float dt) // 只用六轴数据更新 Mahony 融合
{
    const float deg2rad = 0.01745329252f; // 角速度单位从 dps 转 rad/s
    const float int_lim = 0.1f; // 积分误差限幅
    float ax; // 加速度 X 分量
    float ay; // 加速度 Y 分量
    float az; // 加速度 Z 分量
    float gx; // 角速度 X 分量
    float gy; // 角速度 Y 分量
    float gz; // 角速度 Z 分量
    float norm; // 加速度向量模长
    float q0; // 当前四元数 q0
    float q1; // 当前四元数 q1
    float q2; // 当前四元数 q2
    float q3; // 当前四元数 q3
    float vx; // 估计重力方向 X 分量
    float vy; // 估计重力方向 Y 分量
    float vz; // 估计重力方向 Z 分量
    float ex; // 姿态误差 X 分量
    float ey; // 姿态误差 Y 分量
    float ez; // 姿态误差 Z 分量

    if ((imu == 0) || (dt <= 0.0f) || (dt > 0.2f)) // 检查输入指针和周期是否合法
    {
        return; // 输入无效时跳过本次融合
    }

    ax =  imu->accel_x_g; // 取 X 轴加速度
    ay = -imu->accel_y_g; // 取 Y 轴加速度并统一坐标方向
    az =  imu->accel_z_g; // 取 Z 轴加速度

    gx =  imu->gyro_x_dps * deg2rad; // 取 X 轴角速度并转弧度
    gy = -imu->gyro_y_dps * deg2rad; // 取 Y 轴角速度并统一坐标方向
    gz =  imu->gyro_z_dps * deg2rad; // 取 Z 轴角速度并转弧度

    norm = sqrtf(ax * ax + ay * ay + az * az); // 计算加速度向量模长
    if (norm < 1e-6f) // 判断加速度是否接近零
    {
        return; // 加速度异常时跳过本次融合
    }

    ax /= norm; // 归一化加速度 X 分量
    ay /= norm; // 归一化加速度 Y 分量
    az /= norm; // 归一化加速度 Z 分量

    q0 = g_q.q0; // 读取当前四元数 q0
    q1 = g_q.q1; // 读取当前四元数 q1
    q2 = g_q.q2; // 读取当前四元数 q2
    q3 = g_q.q3; // 读取当前四元数 q3

    vx = 2.0f * (q1 * q3 - q0 * q2); // 由四元数估计重力 X 分量
    vy = 2.0f * (q0 * q1 + q2 * q3); // 由四元数估计重力 Y 分量
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // 由四元数估计重力 Z 分量

    ex = ay * vz - az * vy; // 计算加速度修正误差 X 分量
    ey = az * vx - ax * vz; // 计算加速度修正误差 Y 分量
    ez = ax * vy - ay * vx; // 计算加速度修正误差 Z 分量

    if (g_ki > 0.0f) // 判断是否启用积分修正
    {
        g_exInt += ex * g_ki * dt; // 累加 X 轴积分误差
        g_eyInt += ey * g_ki * dt; // 累加 Y 轴积分误差
        g_ezInt += ez * g_ki * dt; // 累加 Z 轴积分误差

        g_exInt = mpu9250_clampf(g_exInt, -int_lim, int_lim); // 限制 X 轴积分误差
        g_eyInt = mpu9250_clampf(g_eyInt, -int_lim, int_lim); // 限制 Y 轴积分误差
        g_ezInt = mpu9250_clampf(g_ezInt, -int_lim, int_lim); // 限制 Z 轴积分误差

        gx += g_exInt; // 用积分项修正 X 轴角速度
        gy += g_eyInt; // 用积分项修正 Y 轴角速度
        gz += g_ezInt; // 用积分项修正 Z 轴角速度
    }
    else // 未启用积分修正
    {
        g_exInt = 0.0f; // 清空 X 轴积分误差
        g_eyInt = 0.0f; // 清空 Y 轴积分误差
        g_ezInt = 0.0f; // 清空 Z 轴积分误差
    }

    gx += g_kp * ex; // 用比例项修正 X 轴角速度
    gy += g_kp * ey; // 用比例项修正 Y 轴角速度
    gz += g_kp * ez; // 用比例项修正 Z 轴角速度

    mpu9250_integrate_quaternion(gx, gy, gz, dt); // 积分更新四元数和欧拉角
}

/**
 * @brief 读取 Mahony 融合后的欧拉角，并由弧度转换为角度。
 * @param roll_deg 横滚角角度值。
 * @param pitch_deg 俯仰角角度值。
 * @param yaw_deg 航向角角度值。
 * @retval None
 */
void MPU9250_GetEulerFusedDeg(float *roll_deg, float *pitch_deg, float *yaw_deg) // 读取融合欧拉角角度值
{
    const float rad2deg = 57.295779513f; // 弧度转角度系数

    if (roll_deg != 0) // 调用方需要横滚角
    {
        *roll_deg = g_euler_fused.roll * rad2deg; // 输出横滚角
    }
    if (pitch_deg != 0) // 调用方需要俯仰角
    {
        *pitch_deg = g_euler_fused.pitch * rad2deg; // 输出俯仰角
    }
    if (yaw_deg != 0) // 调用方需要航向角
    {
        *yaw_deg = mpu9250_math_yaw_to_heading_deg(g_euler_fused.yaw * rad2deg); // 输出 0-360 度航向角
    }
}
