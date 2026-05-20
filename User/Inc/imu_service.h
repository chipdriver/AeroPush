#ifndef __IMU_SERVICE_H__ // 防止头文件重复包含
#define __IMU_SERVICE_H__

#include <stdint.h> // 提供固定宽度整数类型
#include <string.h> // 提供 memset
#include "FreeRTOS.h" // 提供 FreeRTOS 基础类型
#include "task.h" // 提供 tick 计数接口
#include "app_types.h" // 提供姿态数据结构
#include "mpu9250_driver.h" // 提供 MPU9250/AK8963 驱动接口

#define IMU_SERVICE_READ_FAIL 0U // IMU 读取失败
#define IMU_SERVICE_READ_9AXIS_OK 1U // 六轴和磁力计读取成功
#define IMU_SERVICE_READ_6AXIS_OK 2U // 只有六轴读取成功

/**
 * @brief 初始化 IMU 服务并调用底层 MPU9250 驱动。
 * @retval 1 初始化成功；0 初始化失败。
 */
uint8_t ImuService_Init(void); // 初始化 IMU 服务

/**
 * @brief 构造一组模拟姿态数据。
 * @param attitude 输出姿态数据。
 * @retval None
 */
void ImuService_BuildSimAttitude(AttitudeData_t *attitude); // 构造模拟姿态

/**
 * @brief 读取 MPU9250 六轴原始数据。
 * @param raw 输出传感器原始采样数据。
 * @retval None
 */
void ImuService_ReadRaw(MPU9250_raw_Data *raw); // 读取 IMU 原始值

/**
 * @brief 读取 MPU9250 六轴和 AK8963 磁力计物理量。
 * @param phys 输出 MPU9250 六轴物理量。
 * @param mag 输出 AK8963 磁力计物理量。
 * @retval IMU_SERVICE_READ_9AXIS_OK、IMU_SERVICE_READ_6AXIS_OK 或 IMU_SERVICE_READ_FAIL。
 */
uint8_t ImuService_ReadPhys(MPU9250_Physical_Data *phys, AK8963_Physical_Data *mag); // 读取 IMU 物理量

#endif // __IMU_SERVICE_H__
