/**
 * @file    mpu9250_driver.c
 * @brief   MPU9250 六轴与 AK8963 磁力计驱动实现。
 */
#include "mpu9250_driver.h"

#define MPU9250_PWR_MGMT_1_REG          0x6BU
#define MPU9250_PWR_MGMT_2_REG          0x6CU
#define MPU9250_USER_CTRL_REG           0x6AU
#define MPU9250_INT_PIN_CFG_REG         0x37U
#define MPU9250_CONFIG_REG              0x1AU
#define MPU9250_SMPLRT_DIV_REG          0x19U
#define MPU9250_ACCEL_CONFIG_REG        0x1CU
#define MPU9250_ACCEL_CONFIG2_REG       0x1DU
#define MPU9250_GYRO_CONFIG_REG         0x1BU
#define MPU9250_ACCEL_XOUT_H_REG        0x3BU
#define MPU9250_ACCEL_YOUT_H_REG        0x3DU
#define MPU9250_ACCEL_ZOUT_H_REG        0x3FU
#define MPU9250_TEMP_OUT_H_REG          0x41U
#define MPU9250_GYRO_XOUT_H_REG         0x43U
#define MPU9250_GYRO_YOUT_H_REG         0x45U
#define MPU9250_GYRO_ZOUT_H_REG         0x47U

#define AK8963_REG_WIA                  0x00U
#define AK8963_REG_ST1                  0x02U
#define AK8963_REG_HXL                  0x03U
#define AK8963_REG_HYL                  0x05U
#define AK8963_REG_HZL                  0x07U
#define AK8963_REG_ST2                  0x09U
#define AK8963_REG_CNTL1                0x0AU
#define AK8963_REG_ASAX                 0x10U
#define AK8963_REG_ASAY                 0x11U
#define AK8963_REG_ASAZ                 0x12U
#define AK8963_WHO_AM_I_VALUE           0x48U

#define MPU9250_ACCEL_LSB_PER_G         4096.0f
#define MPU9250_GYRO_LSB_PER_DPS        32.8f
#define AK8963_16BIT_UT_PER_LSB         0.15f
#define AK8963_MAG_RADIUS_MIN_UT        0.001f

static uint8_t g_ak8963_asa[3] = {0U, 0U, 0U};
static float g_ak8963_sensitivity[3] = {1.0f, 1.0f, 1.0f};

static float g_gyro_bias_dps[3] = {0.0f, 0.0f, 0.0f};
static float g_accel_bias_g[3] = {0.0f, 0.0f, 0.0f};
static float g_mag_offset_ut[3] = {0.0f, 0.0f, 0.0f};
static float g_mag_scale[3] = {1.0f, 1.0f, 1.0f};

/**
 * @brief  读取 MPU9250 的 16 位大端有符号寄存器值。
 * @param  reg 高字节寄存器地址。
 * @return 组合后的 16 位有符号值。
 */
static int16_t mpu9250_read_word(uint8_t reg)
{
    uint8_t high_byte = I2C_ReadReg(MPU9250_I2C_ADDR7, reg);
    uint8_t low_byte = I2C_ReadReg(MPU9250_I2C_ADDR7, (uint8_t)(reg + 1U));

    return (int16_t)(((uint16_t)high_byte << 8U) | (uint16_t)low_byte);
}

/**
 * @brief  读取 AK8963 的 16 位小端有符号寄存器值。
 * @param  low_reg 低字节寄存器地址。
 * @return 组合后的 16 位有符号值。
 */
static int16_t ak8963_read_word(uint8_t low_reg)
{
    uint8_t low_byte = I2C_ReadReg(AK8963_I2C_ADDR7, low_reg);
    uint8_t high_byte = I2C_ReadReg(AK8963_I2C_ADDR7, (uint8_t)(low_reg + 1U));

    return (int16_t)(((uint16_t)high_byte << 8U) | (uint16_t)low_byte);
}

/**
 * @brief  将 AK8963 原始值转换为只经过 ASA 灵敏度补偿的 uT 数据。
 * @param  raw 磁力计原始数据。
 * @param  mag_out 输出的基础磁场物理量。
 */
static void ak8963_convert_raw_to_ut(const AK8963_raw_Data *raw, AK8963_Physical_Data *mag_out)
{
    if ((raw == 0) || (mag_out == 0))
    {
        return;
    }

    mag_out->mag_x_ut = (float)raw->mag_x * g_ak8963_sensitivity[0] * AK8963_16BIT_UT_PER_LSB;
    mag_out->mag_y_ut = (float)raw->mag_y * g_ak8963_sensitivity[1] * AK8963_16BIT_UT_PER_LSB;
    mag_out->mag_z_ut = (float)raw->mag_z * g_ak8963_sensitivity[2] * AK8963_16BIT_UT_PER_LSB;
}

/**
 * @brief  应用磁力计硬铁偏移和软铁缩放校准。
 * @param  mag 待校准的磁场物理量，函数会原地更新。
 */
static void ak8963_apply_mag_calibration(AK8963_Physical_Data *mag)
{
    if (mag == 0)
    {
        return;
    }

    mag->mag_x_ut = (mag->mag_x_ut - g_mag_offset_ut[0]) * g_mag_scale[0];
    mag->mag_y_ut = (mag->mag_y_ut - g_mag_offset_ut[1]) * g_mag_scale[1];
    mag->mag_z_ut = (mag->mag_z_ut - g_mag_offset_ut[2]) * g_mag_scale[2];
}

/**
 * @brief  读取并校验 MPU9250 设备 ID。
 * @return 0 表示 ID 正确，负数表示读取失败或 ID 错误。
 */
static int mpu9250_check_device(void)
{
    uint8_t id = 0U;

    if (MPU9250_Driver_ReadWhoAmI(&id) == 0U)
    {
        Debug_Print("[MPU9250] WHO_AM_I read failed\r\n");
        return -1;
    }

    Debug_Printf("[MPU9250] WHO_AM_I = 0x%02X\r\n", id);

    if (id != MPU9250_WHO_AM_I_VALUE)
    {
        Debug_Print("[MPU9250] WHO_AM_I value error\r\n");
        return -2;
    }

    Debug_Print("[MPU9250] WHO_AM_I check ok\r\n");
    return 0;
}

/**
 * @brief  配置 MPU9250 六轴传感器的基础寄存器。
 */
static void mpu9250_config_six_axis(void)
{
    MPU9250_SoftReset();
    mpu_set_clock_to_auto();
    mpu_enable_six_axis();
    mpu_set_dlpf_cfg_3();
    mpu_set_sample_rate_200hz();
    mpu_set_accel_dlpf();
    mpu_set_gyro_config();
    mpu_set_accel_range();
    MPU9250_Read_PowerMgmt();
}

/**
 * @brief  初始化 AK8963 磁力计并进入连续测量模式。
 * @return 1 表示初始化成功，0 表示初始化失败。
 */
static uint8_t ak8963_init(void)
{
    mpu_set_ak8963_by_mcu();

    if (AK8963_CheckDeviceID() != 0)
    {
        Debug_Print("[AK8963] device ID check failed\r\n");
        return 0U;
    }

    Debug_Print("[AK8963] device ID check ok\r\n");

    AK8963_EnterPowerDownMode();
    AK8963_EnterFuseROMMode();
    AK8963_AdjustSensitivity();
    AK8963_EnterPowerDownMode();
    AK8963_EnterContinuousMeasurementMode();

    if (AK8963_CheckDataReady() == 0)
    {
        Debug_Print("[AK8963] data not ready\r\n");
        return 0U;
    }

    Debug_Print("[AK8963] data ready\r\n");
    return 1U;
}

/**
 * @brief  读取 MPU9250 WHO_AM_I 寄存器。
 * @param  id 保存读取结果的指针。
 * @return 1 表示读取成功，0 表示读取失败。
 */
uint8_t MPU9250_Driver_ReadWhoAmI(uint8_t *id)
{
    int ret;

    if (id == 0)
    {
        return 0U;
    }

    ret = I2C_ReadRegData(MPU9250_I2C_ADDR7, MPU9250_REG_WHO_AM_I, id);
    return (ret == 0) ? 1U : 0U;
}

/**
 * @brief  软复位 MPU9250。
 */
void MPU9250_SoftReset(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG);

    val &= (uint8_t)~(1U << 7U);
    val |= (uint8_t)(1U << 7U);
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG, val);

    vTaskDelay(pdMS_TO_TICKS(100));
}

/**
 * @brief  读取并打印 MPU9250 电源管理寄存器。
 */
void MPU9250_Read_PowerMgmt(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG);

    Debug_Printf("[MPU9250] PWR_MGMT_1 = 0x%02X\r\n", val);
}

/**
 * @brief  唤醒 MPU9250 并设置 PLL 时钟源。
 */
void mpu_set_clock_to_auto(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG);

    val &= (uint8_t)~((1U << 6U) | 0x07U);
    val |= 0x01U;
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_1_REG, val);
}

/**
 * @brief  启用 MPU9250 加速度计和陀螺仪六轴。
 */
void mpu_enable_six_axis(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_2_REG);

    val &= (uint8_t)~0x3FU;
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_PWR_MGMT_2_REG, val);
}

/**
 * @brief  配置陀螺仪低通滤波为 DLPF_CFG=3。
 */
void mpu_set_dlpf_cfg_3(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_CONFIG_REG);

    val = (uint8_t)((val & (uint8_t)~0x07U) | 0x03U);
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_CONFIG_REG, val);
}

/**
 * @brief  配置 MPU9250 采样率为 200Hz。
 */
void mpu_set_sample_rate_200hz(void)
{
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_SMPLRT_DIV_REG, 0x04U);
}

/**
 * @brief  配置加速度计低通滤波。
 */
void mpu_set_accel_dlpf(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG2_REG);

    val &= (uint8_t)~0x07U;
    val |= 0x03U;
    val &= (uint8_t)~(1U << 3U);
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG2_REG, val);
}

/**
 * @brief  配置陀螺仪量程为 +/-1000 dps。
 */
void mpu_set_gyro_config(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_GYRO_CONFIG_REG);

    val &= (uint8_t)~0x03U;
    val &= (uint8_t)~(3U << 3U);
    val |= (uint8_t)(2U << 3U);
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_GYRO_CONFIG_REG, val);
}

/**
 * @brief  配置加速度计量程为 +/-8g。
 */
void mpu_set_accel_range(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG_REG);

    val &= (uint8_t)~(3U << 3U);
    val |= (uint8_t)(2U << 3U);
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_ACCEL_CONFIG_REG, val);
}

/**
 * @brief  读取 MPU9250 六轴原始数据。
 * @param  raw 保存六轴原始数据的指针。
 */
void MPU9250_ReadAxis(MPU9250_raw_Data *raw)
{
    if (raw == 0)
    {
        return;
    }

    raw->accel_x = mpu9250_read_word(MPU9250_ACCEL_XOUT_H_REG);
    raw->accel_y = mpu9250_read_word(MPU9250_ACCEL_YOUT_H_REG);
    raw->accel_z = mpu9250_read_word(MPU9250_ACCEL_ZOUT_H_REG);
    raw->gyro_x = mpu9250_read_word(MPU9250_GYRO_XOUT_H_REG);
    raw->gyro_y = mpu9250_read_word(MPU9250_GYRO_YOUT_H_REG);
    raw->gyro_z = mpu9250_read_word(MPU9250_GYRO_ZOUT_H_REG);
    raw->temp = mpu9250_read_word(MPU9250_TEMP_OUT_H_REG);
}

/**
 * @brief  将 MPU9250 原始六轴数据转换为物理量并应用零偏。
 * @param  raw 原始六轴数据。
 * @param  physical 输出的六轴物理量。
 */
void MPU9250_ConvertToPhysical(const MPU9250_raw_Data *raw, MPU9250_Physical_Data *physical)
{
    if ((raw == 0) || (physical == 0))
    {
        return;
    }

    physical->accel_x_g = (float)raw->accel_x / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[0];
    physical->accel_y_g = (float)raw->accel_y / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[1];
    physical->accel_z_g = (float)raw->accel_z / MPU9250_ACCEL_LSB_PER_G - g_accel_bias_g[2];
    physical->gyro_x_dps = (float)raw->gyro_x / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[0];
    physical->gyro_y_dps = (float)raw->gyro_y / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[1];
    physical->gyro_z_dps = (float)raw->gyro_z / MPU9250_GYRO_LSB_PER_DPS - g_gyro_bias_dps[2];
    physical->temp_c = ((float)raw->temp / 333.87f) + 21.0f;
}

/**
 * @brief  配置 MPU9250 BYPASS，让 MCU 直接访问 AK8963。
 */
void mpu_set_ak8963_by_mcu(void)
{
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_USER_CTRL_REG);

    val &= (uint8_t)~(1U << 5U);
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_USER_CTRL_REG, val);

    val = I2C_ReadReg(MPU9250_I2C_ADDR7, MPU9250_INT_PIN_CFG_REG);
    val |= (uint8_t)(1U << 1U);
    I2C_WriteReg(MPU9250_I2C_ADDR7, MPU9250_INT_PIN_CFG_REG, val);
}

/**
 * @brief  检查 AK8963 设备 ID。
 * @return 0 表示 ID 正确，-1 表示 ID 错误。
 */
int AK8963_CheckDeviceID(void)
{
    uint8_t device_id = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_WIA);

    return (device_id == AK8963_WHO_AM_I_VALUE) ? 0 : -1;
}

/**
 * @brief  使 AK8963 进入 Power-down 模式。
 */
void AK8963_EnterPowerDownMode(void)
{
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x00U);
    vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * @brief  使 AK8963 进入 Fuse ROM 模式。
 */
void AK8963_EnterFuseROMMode(void)
{
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x0FU);
    vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * @brief  读取 AK8963 Fuse ROM 中的 ASA 灵敏度补偿系数。
 */
void AK8963_AdjustSensitivity(void)
{
    g_ak8963_asa[0] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAX);
    g_ak8963_asa[1] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAY);
    g_ak8963_asa[2] = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ASAZ);

    g_ak8963_sensitivity[0] = (((float)g_ak8963_asa[0] - 128.0f) * 0.5f / 128.0f) + 1.0f;
    g_ak8963_sensitivity[1] = (((float)g_ak8963_asa[1] - 128.0f) * 0.5f / 128.0f) + 1.0f;
    g_ak8963_sensitivity[2] = (((float)g_ak8963_asa[2] - 128.0f) * 0.5f / 128.0f) + 1.0f;
}

/**
 * @brief  使 AK8963 进入 16-bit 100Hz 连续测量模式。
 */
void AK8963_EnterContinuousMeasurementMode(void)
{
    I2C_WriteReg(AK8963_I2C_ADDR7, AK8963_REG_CNTL1, 0x16U);
    vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * @brief  检查 AK8963 数据准备标志。
 * @return 1 表示数据已准备好，0 表示未准备好。
 */
int AK8963_CheckDataReady(void)
{
    uint8_t status = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ST1);

    return ((status & 0x01U) != 0U) ? 1 : 0;
}

/**
 * @brief  读取 AK8963 三轴原始磁力计数据。
 * @param  raw 保存磁力计原始数据的指针。
 * @return 0 表示成功，-1 表示未就绪，-2 表示读取失败，-3 表示磁场溢出。
 */
int AK8963_Read_Axis(AK8963_raw_Data *raw)
{
    uint8_t st2;

    if (raw == 0)
    {
        return -2;
    }

    if (AK8963_CheckDataReady() == 0)
    {
        return -1;
    }

    raw->mag_x = ak8963_read_word(AK8963_REG_HXL);
    raw->mag_y = ak8963_read_word(AK8963_REG_HYL);
    raw->mag_z = ak8963_read_word(AK8963_REG_HZL);

    st2 = I2C_ReadReg(AK8963_I2C_ADDR7, AK8963_REG_ST2);
    if (st2 == 0xFFU)
    {
        return -2;
    }

    if ((st2 & 0x08U) != 0U)
    {
        return -3;
    }

    return 0;
}

/**
 * @brief  将 AK8963 原始数据转换为最终校准后的 uT 数据。
 * @param  raw 原始磁力计数据。
 * @param  physical 输出的校准后磁场物理量。
 */
void AK8963_Calibrate(const AK8963_raw_Data *raw, AK8963_Physical_Data *physical)
{
    ak8963_convert_raw_to_ut(raw, physical);
    ak8963_apply_mag_calibration(physical);
}

/**
 * @brief  读取一帧 AK8963 磁力计数据并输出校准后的 uT 数据。
 * @param  mag_out 保存磁力计物理量的指针。
 * @return 0 表示成功，其他值沿用 AK8963_Read_Axis 的错误码。
 */
int AK8963_Read_Mag_UT(AK8963_Physical_Data *mag_out)
{
    AK8963_raw_Data raw;
    int ret;

    if (mag_out == 0)
    {
        return -2;
    }

    ret = AK8963_Read_Axis(&raw);
    if (ret != 0)
    {
        return ret;
    }

    AK8963_Calibrate(&raw, mag_out);
    return 0;
}

/**
 * @brief  校准陀螺仪零偏。
 * @param  samples 采样次数。
 * @param  delay_ms 每次采样之间的延时，单位 ms。
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

/**
 * @brief  校准加速度计静态零偏。
 * @param  samples 采样次数。
 * @param  delay_ms 每次采样之间的延时，单位 ms。
 * @note   调用时模块需要水平静止，Z 轴朝上。
 */
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

    g_accel_bias_g[0] = (ax_sum / (float)samples);
    g_accel_bias_g[1] = (ay_sum / (float)samples);
    g_accel_bias_g[2] = (az_sum / (float)samples) - 1.0f;
}

/**
 * @brief  校准 AK8963 磁力计硬铁偏移和软铁缩放。
 * @param  samples 采样次数。
 * @param  delay_ms 每次采样之间的延时，单位 ms。
 * @note   调用期间需要持续旋转模块，尽量覆盖完整三维方向。
 */
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
    float radius_avg;
    uint16_t valid_count = 0U;
    uint16_t i;

    if (samples == 0U)
    {
        return;
    }

    g_mag_offset_ut[0] = 0.0f;
    g_mag_offset_ut[1] = 0.0f;
    g_mag_offset_ut[2] = 0.0f;
    g_mag_scale[0] = 1.0f;
    g_mag_scale[1] = 1.0f;
    g_mag_scale[2] = 1.0f;

    for (i = 0U; i < samples; i++)
    {
        if (AK8963_Read_Axis(&raw) == 0)
        {
            ak8963_convert_raw_to_ut(&raw, &mag);

            if (mag.mag_x_ut < min_x) { min_x = mag.mag_x_ut; }
            if (mag.mag_x_ut > max_x) { max_x = mag.mag_x_ut; }
            if (mag.mag_y_ut < min_y) { min_y = mag.mag_y_ut; }
            if (mag.mag_y_ut > max_y) { max_y = mag.mag_y_ut; }
            if (mag.mag_z_ut < min_z) { min_z = mag.mag_z_ut; }
            if (mag.mag_z_ut > max_z) { max_z = mag.mag_z_ut; }
            valid_count++;
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    if (valid_count == 0U)
    {
        return;
    }

    g_mag_offset_ut[0] = (min_x + max_x) * 0.5f;
    g_mag_offset_ut[1] = (min_y + max_y) * 0.5f;
    g_mag_offset_ut[2] = (min_z + max_z) * 0.5f;

    radius_x = (max_x - min_x) * 0.5f;
    radius_y = (max_y - min_y) * 0.5f;
    radius_z = (max_z - min_z) * 0.5f;

    if ((radius_x < AK8963_MAG_RADIUS_MIN_UT) ||
        (radius_y < AK8963_MAG_RADIUS_MIN_UT) ||
        (radius_z < AK8963_MAG_RADIUS_MIN_UT))
    {
        g_mag_scale[0] = 1.0f;
        g_mag_scale[1] = 1.0f;
        g_mag_scale[2] = 1.0f;
        return;
    }

    radius_avg = (radius_x + radius_y + radius_z) / 3.0f;
    g_mag_scale[0] = radius_avg / radius_x;
    g_mag_scale[1] = radius_avg / radius_y;
    g_mag_scale[2] = radius_avg / radius_z;
}

/**
 * @brief  初始化 MPU9250 六轴和 AK8963 磁力计。
 * @return 1 表示初始化成功，0 表示初始化失败。
 */
uint8_t MPU9250_Driver_Init(void)
{
    Debug_Print("[MPU9250] driver init start\r\n");

    BSP_I2C_Soft_Init();
    Debug_Print("[MPU9250] soft i2c init ok\r\n");

    if (mpu9250_check_device() != 0)
    {
        Debug_Print("[MPU9250] driver init failed\r\n");
        return 0U;
    }

    mpu9250_config_six_axis();

    if (ak8963_init() == 0U)
    {
        return 0U;
    }

    MPU9250_CalibrateGyro(1000U, 10U);
    MPU9250_CalibrateAccel(1000U, 10U);
    Debug_Print("[AK8963] calibrate mag start\r\n");
    AK8963_CalibrateMag(500U, 20U);

    Debug_Print("[MPU9250] driver init ok\r\n");
    return 1U;
}
