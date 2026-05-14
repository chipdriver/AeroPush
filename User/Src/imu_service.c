/**
 * @file    imu_service.c
 * @brief   IMU 服务层实现，负责把底层 MPU9250/AK8963 驱动封装给任务层使用。
 */
#include "imu_service.h"
#include "app_config.h"
#include "debug_log.h"

/**
 * @brief  初始化 IMU 服务。
 * @return 1 表示初始化成功，0 表示初始化失败。
 */
uint8_t ImuService_Init(void)
{
    return MPU9250_Driver_Init();
}

/**
 * @brief  构造一帧模拟姿态数据。
 * @param  attitude 保存模拟姿态数据的指针。
 */
void ImuService_BuildSimAttitude(AttitudeData_t *attitude)
{
    static float angle = 0.0f;

    if (attitude == 0)
    {
        return;
    }

    angle += APP_SIM_ATTITUDE_STEP_DEG;
    if (angle >= APP_SIM_ATTITUDE_MAX_DEG)
    {
        angle = 0.0f;
    }

    memset(attitude, 0, sizeof(AttitudeData_t));
    attitude->roll_deg = angle;
    attitude->pitch_deg = angle + 1.0f;
    attitude->yaw_deg = angle + 2.0f;
    attitude->timestamp_ms = xTaskGetTickCount();
    attitude->valid = 1;
}

/**
 * @brief  读取 MPU9250 六轴原始数据。
 * @param  raw 保存六轴原始数据的指针。
 */
void ImuService_ReadRaw(MPU9250_raw_Data *raw)
{
    if (raw == 0)
    {
        return;
    }

    MPU9250_ReadAxis(raw);
}

/**
 * @brief  读取 MPU9250 六轴和 AK8963 磁力计物理量。
 * @param  phys 保存 MPU9250 六轴物理量的指针。
 * @param  mag 保存 AK8963 磁力计物理量的指针。
 * @return 1 表示读取成功，0 表示读取失败。
 */
uint8_t ImuService_ReadPhys(MPU9250_Physical_Data *phys, AK8963_Physical_Data *mag)
{
    MPU9250_raw_Data raw;

    if ((phys == 0) || (mag == 0))
    {
        return 0U;
    }

    MPU9250_ReadAxis(&raw);
    MPU9250_ConvertToPhysical(&raw, phys);

    if (AK8963_Read_Mag_UT(mag) != 0)
    {
        return 0U;
    }

    return 1U;
}
