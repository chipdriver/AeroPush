/**
 * @file    imu_service.c
 * @brief   IMU 服务层实现，负责把底层 MPU9250/AK8963 驱动封装给任务层使用。
 */
#include "imu_service.h" // 提供 IMU 服务层接口
#include "app_config.h" // 提供模拟姿态配置
#include "debug_log.h" // 提供调试日志接口

/**
 * @brief 初始化 IMU 服务并调用底层 MPU9250 驱动。
 * @retval 1 初始化成功；0 初始化失败。
 */
uint8_t ImuService_Init(void) // 初始化 IMU 服务
{
    return MPU9250_Driver_Init(); // 返回底层驱动初始化结果
}

/**
 * @brief 构造一组模拟姿态数据。
 * @param attitude 输出姿态数据。
 * @retval None
 */
void ImuService_BuildSimAttitude(AttitudeData_t *attitude) // 构造模拟姿态
{
    static float angle = 0.0f; // 模拟角度递增值

    if (attitude == 0) // 检查输出指针
    {
        return; // 输出指针为空时不处理
    }

    angle += APP_SIM_ATTITUDE_STEP_DEG; // 按配置步进增加角度
    if (angle >= APP_SIM_ATTITUDE_MAX_DEG) // 角度超过模拟上限
    {
        angle = 0.0f; // 回到模拟起点
    }

    memset(attitude, 0, sizeof(AttitudeData_t)); // 清空输出姿态结构体
    attitude->roll_deg = angle; // 写入模拟横滚角
    attitude->pitch_deg = angle + 1.0f; // 写入模拟俯仰角
    attitude->yaw_deg = angle + 2.0f; // 写入模拟航向角
    attitude->timestamp_ms = xTaskGetTickCount(); // 写入当前 tick 时间戳
    attitude->valid = 1; // 标记模拟姿态有效
}

/**
 * @brief 读取 MPU9250 六轴原始数据。
 * @param raw 输出传感器原始采样数据。
 * @retval None
 */
void ImuService_ReadRaw(MPU9250_raw_Data *raw) // 读取 IMU 原始值
{
    if (raw == 0) // 检查输出指针
    {
        return; // 输出指针为空时不处理
    }

    MPU9250_ReadAxis(raw); // 读取 MPU9250 原始寄存器数据
}

/**
 * @brief 读取 MPU9250 六轴和 AK8963 磁力计物理量。
 * @param phys 输出 MPU9250 六轴物理量。
 * @param mag 输出 AK8963 磁力计物理量。
 * @retval IMU_SERVICE_READ_9AXIS_OK、IMU_SERVICE_READ_6AXIS_OK 或 IMU_SERVICE_READ_FAIL。
 */
uint8_t ImuService_ReadPhys(MPU9250_Physical_Data *phys, AK8963_Physical_Data *mag) // 读取 IMU 物理量
{
    MPU9250_raw_Data raw; // MPU9250 原始采样缓存

    if ((phys == 0) || (mag == 0)) // 检查输出指针
    {
        return IMU_SERVICE_READ_FAIL; // 输出指针无效
    }

    MPU9250_ReadAxis(&raw); // 读取六轴原始数据
    MPU9250_ConvertToPhysical(&raw, phys); // 转换为 g、dps 和摄氏度

    if (AK8963_Read_Mag_UT(mag) != 0) // 读取磁力计失败
    {
        return IMU_SERVICE_READ_6AXIS_OK; // 返回六轴数据有效
    }

    return IMU_SERVICE_READ_9AXIS_OK; // 返回九轴数据有效
}
