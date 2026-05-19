#ifndef __IMU_SERVICE_H__ // 检查 __IMU_SERVICE_H__ 是否未定义，防止头文件重复包含
#define __IMU_SERVICE_H__ // 定义 __IMU_SERVICE_H__ 变量

#include <stdint.h> // 引入 stdint.h 提供的接口、宏和类型定义
#include <string.h> // 引入 string.h 提供的接口、宏和类型定义
#include "FreeRTOS.h" // 引入 FreeRTOS.h 提供的接口、宏和类型定义
#include "task.h" // 引入 task.h 提供的接口、宏和类型定义
#include "app_types.h" // 引入 app_types.h 提供的接口、宏和类型定义
#include "mpu9250_driver.h" // 引入 mpu9250_driver.h 提供的接口、宏和类型定义

#define IMU_SERVICE_READ_FAIL 0U // 定义 IMU_SERVICE_READ_FAIL 变量为 0U
#define IMU_SERVICE_READ_9AXIS_OK 1U // 定义 IMU_SERVICE_READ_9AXIS_OK 变量为 1U
#define IMU_SERVICE_READ_6AXIS_OK 2U // 定义 IMU_SERVICE_READ_6AXIS_OK 变量为 2U

/**
 * @brief 初始化 IMU 服务并调用底层 MPU9250 驱动。
 * @retval 函数执行结果或计算得到的返回值。
 */
uint8_t ImuService_Init(void); // 声明ImuService_Init 函数签名：初始化 IMU 服务并调用底层 MPU9250 驱动
/**
 * @brief ImuService_BuildSimAttitude 函数。
 * @param attitude 姿态数据结构体。
 * @retval None
 */
void ImuService_BuildSimAttitude(AttitudeData_t *attitude); // 声明ImuService_BuildSimAttitude 函数签名：ImuService_BuildSimAttitude 函数
/**
 * @brief 读取 MPU9250 六轴原始数据。
 * @param raw 传感器原始采样数据结构体。
 * @retval None
 */
void ImuService_ReadRaw(MPU9250_raw_Data *raw); // 声明ImuService_ReadRaw 函数签名：读取 MPU9250 六轴原始数据
/**
 * @brief 读取 MPU9250 六轴和 AK8963 磁力计物理量。
 * @param phys MPU9250 六轴物理量数据。
 * @param mag AK8963 磁力计物理量数据。
 * @retval 函数执行结果或计算得到的返回值。
 */
uint8_t ImuService_ReadPhys(MPU9250_Physical_Data *phys, AK8963_Physical_Data *mag); // 声明ImuService_ReadPhys 函数签名：读取 MPU9250 六轴和 AK8963 磁力计物理量

#endif // 结束当前条件编译或头文件保护范围
