#ifndef __IMU_SERVICE_H__ /* 防止 imu_service.h 被重复包含 */
#define __IMU_SERVICE_H__ /* 定义 IMU 服务头文件保护宏 */

#include <stdint.h>         /* 引入 uint8_t 等标准整数类型 */
#include <string.h>         /* 引入 memset 函数声明 */
#include "FreeRTOS.h"       /* 引入 FreeRTOS 基础定义 */
#include "task.h"           /* 引入 xTaskGetTickCount 等任务接口 */
#include "app_types.h"      /* 引入项目公共数据结构 */
#include "mpu9250_driver.h" /* 引入 MPU9250/AK8963 驱动数据类型 */

uint8_t ImuService_Init(void);                                                       /* 声明 IMU 服务初始化函数 */
void ImuService_BuildSimAttitude(AttitudeData_t *attitude);                          /* 声明模拟姿态数据构造函数 */
void ImuService_ReadRaw(MPU9250_raw_Data *raw);                                      /* 声明 MPU9250 六轴原始数据读取函数 */
uint8_t ImuService_ReadPhys(MPU9250_Physical_Data *phys, AK8963_Physical_Data *mag); /* 声明 IMU 物理量读取函数 */

#endif /* 结束 IMU 服务头文件保护宏 */
