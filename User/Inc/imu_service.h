#ifndef __IMU_SERVICE_H__                               // 防止 imu_service.h 被重复包含
#define __IMU_SERVICE_H__                               // 定义 IMU 服务头文件保护宏

/* header file inclusion */                             // 头文件包含区域
#include "app_types.h"                                  // 引入姿态数据类型

#include "FreeRTOS.h"                                   // 引入 FreeRTOS 基础定义
#include "task.h"                                       // 引入 xTaskGetTickCount 等任务接口

#include <string.h>                                     // 引入 memset 函数声明
#include <stdint.h>                                     // 引入 uint8_t 等标准整数类型

/* function declaration */                              // 函数声明区域

/**
 * @brief  初始化 IMU 服务。
 * @param  None
 * @retval 1 表示初始化成功，0 表示初始化失败。
 */
uint8_t ImuService_Init(void);                          // 声明 IMU 服务初始化函数

/**
 * @brief  构造一帧模拟姿态数据。
 * @param  attitude 用于保存模拟姿态数据的结构体指针。
 * @retval None
 */
void ImuService_BuildSimAttitude(AttitudeData_t *attitude); // 声明模拟姿态数据构造函数


#endif /* __IMU_SERVICE_H__ */                          // 结束 IMU 服务头文件保护宏
