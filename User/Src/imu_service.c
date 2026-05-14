/**
 * @file    imu_service.c
 * @brief   IMU 服务层实现，负责把底层 MPU9250/AK8963 驱动封装给任务层使用。
 */
#include "imu_service.h"                                                                  // 包含所需头文件
#include "app_config.h"                                                                   // 包含所需头文件
#include "debug_log.h"                                                                    // 包含所需头文件

/**
 * @brief  初始化 IMU 服务。
 * @param  None
 * @return 1 表示初始化成功，0 表示初始化失败。
 */
uint8_t ImuService_Init(void)                                                             // 定义局部变量
{                                                                                         // 进入代码块
    return MPU9250_Driver_Init();                                                         // 返回函数执行结果
}                                                                                         // 结束代码块

/**
 * @brief  构造一帧模拟姿态数据。
 * @param  attitude 保存模拟姿态数据的指针。
 * @retval None
 */
void ImuService_BuildSimAttitude(AttitudeData_t *attitude)                                // 说明当前代码行
{                                                                                         // 进入代码块
    static float angle = 0.0f;                                                            // 定义本文件内部静态变量

    if (attitude == 0)                                                                    // 判断条件是否成立
    {                                                                                     // 进入代码块
        return;                                                                           // 返回函数执行结果
    }                                                                                     // 结束代码块

    angle += APP_SIM_ATTITUDE_STEP_DEG;                                                   // 累加或更新当前变量
    if (angle >= APP_SIM_ATTITUDE_MAX_DEG)                                                // 判断条件是否成立
    {                                                                                     // 进入代码块
        angle = 0.0f;                                                                     // 给变量或寄存器写入新值
    }                                                                                     // 结束代码块

    memset(attitude, 0, sizeof(AttitudeData_t));                                          // 清空结构体或缓冲区内容
    attitude->roll_deg = angle;                                                           // 给变量或寄存器写入新值
    attitude->pitch_deg = angle + 1.0f;                                                   // 给变量或寄存器写入新值
    attitude->yaw_deg = angle + 2.0f;                                                     // 给变量或寄存器写入新值
    attitude->timestamp_ms = xTaskGetTickCount();                                         // 给变量或寄存器写入新值
    attitude->valid = 1;                                                                  // 给变量或寄存器写入新值
}                                                                                         // 结束代码块

/**
 * @brief  读取 MPU9250 六轴原始数据。
 * @param  raw 保存六轴原始数据的指针。
 * @retval None
 */
void ImuService_ReadRaw(MPU9250_raw_Data *raw)                                            // 说明当前代码行
{                                                                                         // 进入代码块
    if (raw == 0)                                                                         // 判断条件是否成立
    {                                                                                     // 进入代码块
        return;                                                                           // 返回函数执行结果
    }                                                                                     // 结束代码块

    MPU9250_ReadAxis(raw);                                                                // 调用 MPU9250 驱动接口
}                                                                                         // 结束代码块

/**
 * @brief  读取 MPU9250 六轴和 AK8963 磁力计物理量。
 * @param  phys 保存 MPU9250 六轴物理量的指针。
 * @param  mag 保存 AK8963 磁力计物理量的指针。
 * @return 1 表示读取成功，0 表示读取失败。
 */
uint8_t ImuService_ReadPhys(MPU9250_Physical_Data *phys, AK8963_Physical_Data *mag)       // 定义局部变量
{                                                                                         // 进入代码块
    MPU9250_raw_Data raw;                                                                 // 定义局部变量

    if ((phys == 0) || (mag == 0))                                                        // 判断条件是否成立
    {                                                                                     // 进入代码块
        return 0U;                                                                        // 返回函数执行结果
    }                                                                                     // 结束代码块

    MPU9250_ReadAxis(&raw);                                                               // 调用 MPU9250 驱动接口
    MPU9250_ConvertToPhysical(&raw, phys);                                                // 调用 MPU9250 驱动接口

    if (AK8963_Read_Mag_UT(mag) != 0)                                                     // 判断条件是否成立
    {                                                                                     // 进入代码块
        return 0U;                                                                        // 返回函数执行结果
    }                                                                                     // 结束代码块

    return 1U;                                                                            // 返回函数执行结果
}                                                                                         // 结束代码块
