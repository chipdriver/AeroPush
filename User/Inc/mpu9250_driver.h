#ifndef __MPU9250_DRIVER_H__ /* 防止 mpu9250_driver.h 被重复包含 */
#define __MPU9250_DRIVER_H__ /* 定义 MPU9250 驱动头文件保护宏 */

#include <stdint.h>       /* 引入 uint8_t、int16_t 等标准整数类型 */
#include "bsp_i2c_soft.h" /* 引入软件 I2C 底层读写接口 */
#include "debug_log.h"    /* 引入调试日志输出接口 */
#include "FreeRTOS.h"     /* 引入 FreeRTOS 基础定义 */
#include "task.h"         /* 引入 vTaskDelay 和 pdMS_TO_TICKS */

#define MPU9250_I2C_ADDR 0x68U             /* 定义 MPU9250 7 位 I2C 地址 */
#define MPU9250_I2C_ADDR7 MPU9250_I2C_ADDR /* 定义 MPU9250 7 位 I2C 地址兼容别名 */
#define AK8963_I2C_ADDR7 0x0CU             /* 定义 AK8963 7 位 I2C 地址 */
#define MPU9250_REG_WHO_AM_I 0x75U         /* 定义 MPU9250 WHO_AM_I 寄存器地址 */
#define MPU9250_WHO_AM_I_VALUE 0x71U       /* 定义 MPU9250 WHO_AM_I 正常返回值 */

typedef struct /* 定义 MPU9250 六轴原始数据结构体 */
{
    int16_t accel_x; /* 保存加速度计 X 轴原始值 */
    int16_t accel_y; /* 保存加速度计 Y 轴原始值 */
    int16_t accel_z; /* 保存加速度计 Z 轴原始值 */
    int16_t gyro_x;  /* 保存陀螺仪 X 轴原始值 */
    int16_t gyro_y;  /* 保存陀螺仪 Y 轴原始值 */
    int16_t gyro_z;  /* 保存陀螺仪 Z 轴原始值 */
    int16_t temp;    /* 保存温度传感器原始值 */
} MPU9250_raw_Data;  /* 声明 MPU9250 六轴原始数据类型 */

typedef struct /* 定义 MPU9250 六轴物理量结构体 */
{
    float accel_x_g;     /* 保存加速度计 X 轴物理量，单位 g */
    float accel_y_g;     /* 保存加速度计 Y 轴物理量，单位 g */
    float accel_z_g;     /* 保存加速度计 Z 轴物理量，单位 g */
    float gyro_x_dps;    /* 保存陀螺仪 X 轴物理量，单位 dps */
    float gyro_y_dps;    /* 保存陀螺仪 Y 轴物理量，单位 dps */
    float gyro_z_dps;    /* 保存陀螺仪 Z 轴物理量，单位 dps */
    float temp_c;        /* 保存温度物理量，单位摄氏度 */
} MPU9250_Physical_Data; /* 声明 MPU9250 六轴物理量类型 */

typedef struct /* 定义 AK8963 磁力计原始数据结构体 */
{
    int16_t mag_x; /* 保存磁力计 X 轴原始值 */
    int16_t mag_y; /* 保存磁力计 Y 轴原始值 */
    int16_t mag_z; /* 保存磁力计 Z 轴原始值 */
} AK8963_raw_Data; /* 声明 AK8963 磁力计原始数据类型 */

typedef struct /* 定义 AK8963 磁力计物理量结构体 */
{
    float mag_x_ut;     /* 保存磁力计 X 轴物理量，单位 uT */
    float mag_y_ut;     /* 保存磁力计 Y 轴物理量，单位 uT */
    float mag_z_ut;     /* 保存磁力计 Z 轴物理量，单位 uT */
} AK8963_Physical_Data; /* 声明 AK8963 磁力计物理量类型 */

uint8_t MPU9250_Driver_Init(void);                                                            /* 声明 MPU9250/AK8963 驱动初始化函数 */
uint8_t MPU9250_Driver_ReadWhoAmI(uint8_t *id);                                               /* 声明 MPU9250 WHO_AM_I 读取函数 */
void MPU9250_SoftReset(void);                                                                 /* 声明 MPU9250 软复位函数 */
void MPU9250_Read_PowerMgmt(void);                                                            /* 声明 MPU9250 电源管理寄存器打印函数 */
void mpu_set_clock_to_auto(void);                                                             /* 声明 MPU9250 唤醒和时钟源配置函数 */
void mpu_enable_six_axis(void);                                                               /* 声明 MPU9250 六轴使能函数 */
void mpu_set_dlpf_cfg_3(void);                                                                /* 声明 MPU9250 陀螺仪 DLPF 配置函数 */
void mpu_set_sample_rate_200hz(void);                                                         /* 声明 MPU9250 采样率配置函数 */
void mpu_set_accel_dlpf(void);                                                                /* 声明 MPU9250 加速度计 DLPF 配置函数 */
void mpu_set_gyro_config(void);                                                               /* 声明 MPU9250 陀螺仪量程配置函数 */
void mpu_set_accel_range(void);                                                               /* 声明 MPU9250 加速度计量程配置函数 */
void MPU9250_ReadAxis(MPU9250_raw_Data *raw);                                                 /* 声明 MPU9250 六轴原始数据读取函数 */
void MPU9250_ConvertToPhysical(const MPU9250_raw_Data *raw, MPU9250_Physical_Data *physical); /* 声明 MPU9250 六轴物理量转换函数 */
void mpu_set_ak8963_by_mcu(void);                                                             /* 声明 MPU9250 BYPASS 配置函数 */
int AK8963_CheckDeviceID(void);                                                               /* 声明 AK8963 设备 ID 检查函数 */
void AK8963_EnterPowerDownMode(void);                                                         /* 声明 AK8963 Power-down 模式配置函数 */
void AK8963_EnterFuseROMMode(void);                                                           /* 声明 AK8963 Fuse ROM 模式配置函数 */
void AK8963_AdjustSensitivity(void);                                                          /* 声明 AK8963 ASA 灵敏度读取函数 */
void AK8963_EnterContinuousMeasurementMode(void);                                             /* 声明 AK8963 连续测量模式配置函数 */
int AK8963_CheckDataReady(void);                                                              /* 声明 AK8963 数据就绪检查函数 */
int AK8963_Read_Axis(AK8963_raw_Data *raw);                                                   /* 声明 AK8963 三轴原始数据读取函数 */
void AK8963_Calibrate(const AK8963_raw_Data *raw, AK8963_Physical_Data *physical);            /* 声明 AK8963 磁力计校准应用函数 */
int AK8963_Read_Mag_UT(AK8963_Physical_Data *mag_out);                                        /* 声明 AK8963 校准后磁场物理量读取函数 */
void MPU9250_CalibrateGyro(uint16_t samples, uint16_t delay_ms);                              /* 声明 MPU9250 陀螺仪零偏校准函数 */
void MPU9250_CalibrateAccel(uint16_t samples, uint16_t delay_ms);                             /* 声明 MPU9250 加速度计零偏校准函数 */
void AK8963_CalibrateMag(uint16_t samples, uint16_t delay_ms);                                /* 声明 AK8963 硬铁和软铁校准函数 */

#endif /* 结束 MPU9250 驱动头文件保护宏 */
