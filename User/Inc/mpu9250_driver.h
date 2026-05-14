/**
 * @file    mpu9250_driver.h
 * @brief   MPU9250 驱动接口声明。
 */
#ifndef __MPU9250_DRIVER_H__                       /* 防止 mpu9250_driver.h 被重复包含 */
#define __MPU9250_DRIVER_H__                       /* 定义 MPU9250 驱动头文件保护宏 */

#include <stdint.h>                                /* 引入 uint8_t、int16_t 等标准整数类型 */
#include "bsp_i2c_soft.h"                          /* 引入软件 I2C 底层读写接口 */
#include "debug_log.h"                             /* 引入调试日志输出接口 */
#include "FreeRTOS.h"                              /* 引入 FreeRTOS 基础定义 */
#include "task.h"                                  /* 引入 vTaskDelay 和 pdMS_TO_TICKS */

#define MPU9250_I2C_ADDR        0x68U              /* 定义 MPU9250 7 位 I2C 地址，AD0 接 GND 时为 0x68 */
#define MPU9250_I2C_ADDR7       MPU9250_I2C_ADDR   /* 兼容复用库里的 MPU9250_I2C_ADDR7 命名 */
#define MPU9250_REG_WHO_AM_I    0x75U              /* 定义 MPU9250 WHO_AM_I 寄存器地址 */
#define MPU9250_WHO_AM_I_VALUE  0x71U              /* 定义 MPU9250 正常 WHO_AM_I 返回值 */

/* 存放 MPU9250 原始数据 */
typedef struct
{
    int16_t accel_x;                               /* 加速度计 X 轴原始值 */
    int16_t accel_y;                               /* 加速度计 Y 轴原始值 */
    int16_t accel_z;                               /* 加速度计 Z 轴原始值 */
    int16_t gyro_x;                                /* 陀螺仪 X 轴原始值 */
    int16_t gyro_y;                                /* 陀螺仪 Y 轴原始值 */
    int16_t gyro_z;                                /* 陀螺仪 Z 轴原始值 */
    int16_t temp;                                  /* 温度传感器原始值 */
} MPU9250_raw_Data;

/* 存放转换后的物理量 */
typedef struct
{
    float accel_x_g;                               /* 加速度计 X 轴物理量，单位 g */
    float accel_y_g;                               /* 加速度计 Y 轴物理量，单位 g */
    float accel_z_g;                               /* 加速度计 Z 轴物理量，单位 g */
    float gyro_x_dps;                              /* 陀螺仪 X 轴物理量，单位 dps */
    float gyro_y_dps;                              /* 陀螺仪 Y 轴物理量，单位 dps */
    float gyro_z_dps;                              /* 陀螺仪 Z 轴物理量，单位 dps */
    float temp_c;                                  /* 温度物理量，单位摄氏度 */
} MPU9250_Physical_Data;

/**
 * @brief  初始化 MPU9250 驱动。
 * @param  None
 * @retval 1 表示初始化成功，0 表示初始化失败。
 */
uint8_t MPU9250_Driver_Init(void);                 /* 声明 MPU9250 驱动初始化函数 */

/**
 * @brief  读取 MPU9250 WHO_AM_I 寄存器。
 * @param  id 用于保存 WHO_AM_I 读取结果的指针。
 * @retval 1 表示读取成功，0 表示读取失败。
 */
uint8_t MPU9250_Driver_ReadWhoAmI(uint8_t *id);    /* 声明读取 WHO_AM_I 寄存器的函数 */

/**
 * @brief  软复位。
 * @param  None
 * @return None
 */
void MPU9250_SoftReset(void);                      /* 声明 MPU9250 软复位函数 */

/**
 * @brief  读取 PWR_MGMT_1 寄存器。
 * @param  None
 * @return None
 */
void MPU9250_Read_PowerMgmt(void);                 /* 声明读取电源管理寄存器函数 */

/**
 * @brief  唤醒以及设置时钟源。
 * @param  None
 * @return None
 */
void mpu_set_clock_to_auto(void);                  /* 声明设置时钟源函数 */

/**
 * @brief  使能六轴传感器。
 * @param  None
 * @return None
 */
void mpu_enable_six_axis(void);                    /* 声明六轴使能函数 */

/**
 * @brief  配置 DLPF。
 * @param  None
 * @return None
 */
void mpu_set_dlpf_cfg_3(void);                     /* 声明陀螺仪 DLPF 配置函数 */

/**
 * @brief  配置采样率。
 * @param  None
 * @return None
 */
void mpu_set_sample_rate_200hz(void);              /* 声明采样率配置函数 */

/**
 * @brief  配置加速度计的 DLPF。
 * @param  None
 * @return None
 */
void mpu_set_accel_dlpf(void);                     /* 声明加速度计 DLPF 配置函数 */

/**
 * @brief  配置陀螺仪的量程。
 * @param  None
 * @return None
 */
void mpu_set_gyro_config(void);                    /* 声明陀螺仪量程配置函数 */

/**
 * @brief  配置加速度计的量程。
 * @param  None
 * @return None
 */
void mpu_set_accel_range(void);                    /* 声明加速度计量程配置函数 */

/**
 * @brief  读取 MPU9250 六轴传感器的原始数据。
 * @param  raw 用于保存六轴原始数据的结构体指针。
 * @return None
 */
void MPU9250_ReadAxis(MPU9250_raw_Data *raw);      /* 声明六轴原始数据读取函数 */

/****************************************************//**
 * @brief  将 MPU9250 原始数据转换成物理量。
 * @param  raw 指向原始数据结构体的指针。
 * @param  physical 用于保存转换后物理量的结构体指针。
 * @return None
 *******************************************************/
void MPU9250_ConvertToPhysical(const MPU9250_raw_Data * raw, MPU9250_Physical_Data * physical);

#endif                                             /* 结束 MPU9250 驱动头文件保护 */
