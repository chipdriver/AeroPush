/**
 * @file    mpu9250_driver.c
 * @brief   MPU9250 驱动实现。
 */
#include "mpu9250_driver.h"                        /* 引入 MPU9250 驱动模块头文件 */

/**********************************************************************************
 *
 * 检查 I2C 驱动编程是否正常工作
 *
 *********************************************************************************/

/**
 * @brief  读取 MPU9250 WHO_AM_I 寄存器。
 * @param  id 用于保存 WHO_AM_I 读取结果的指针。
 * @retval 1 表示读取成功，0 表示读取失败。
 */
uint8_t MPU9250_Driver_ReadWhoAmI(uint8_t *id)     /* 定义 MPU9250 WHO_AM_I 读取函数 */
{                                                  /* 函数体开始 */
    int ret;                                       /* 定义 I2C 读取返回值变量 */

    if (id == 0)                                   /* 判断输出指针是否为空 */
    {                                              /* 空指针保护分支开始 */
        return 0U;                                 /* 指针无效时返回读取失败 */
    }                                              /* 空指针保护分支结束 */

    ret = I2C_ReadRegData(MPU9250_I2C_ADDR7,       /* 使用 MPU9250 7 位 I2C 地址读取 */
                          MPU9250_REG_WHO_AM_I,   /* 指定读取 WHO_AM_I 寄存器 0x75 */
                          id);                     /* 将读取结果保存到调用者提供的缓存 */

    return (ret == 0) ? 1U : 0U;                   /* I2C 返回 0 表示读取成功，否则表示失败 */
}                                                  /* 函数体结束 */

/**
 * @brief  检查 MPU9250 WHO_AM_I 是否匹配预期值。
 * @param  None
 * @retval 0 表示设备存在且 ID 正确，-1 表示读取失败，-2 表示 ID 不匹配。
 */
static int MPU9250_CheckDevice(void)               /* 定义 MPU9250 设备检查函数 */
{                                                  /* 函数体开始 */
    uint8_t id = 0U;                               /* 定义 WHO_AM_I 读取缓存并清零 */

    if (MPU9250_Driver_ReadWhoAmI(&id) == 0U)      /* 调用 WHO_AM_I 读取函数并判断是否失败 */
    {                                              /* 读取失败分支开始 */
        Debug_Print("[MPU9250] WHO_AM_I read failed\r\n"); /* 打印 WHO_AM_I 读取失败日志 */
        return -1;                                 /* 返回 I2C 读取失败错误码 */
    }                                              /* 读取失败分支结束 */

    Debug_Printf("[MPU9250] WHO_AM_I = 0x%02X\r\n", id); /* 打印实际读取到的 WHO_AM_I 值 */

    if (id == MPU9250_WHO_AM_I_VALUE)              /* 判断 WHO_AM_I 是否等于 MPU9250 标准值 0x71 */
    {                                              /* ID 匹配分支开始 */
        Debug_Print("[MPU9250] WHO_AM_I check ok\r\n"); /* 打印 WHO_AM_I 校验成功日志 */
        return 0;                                  /* 返回设备检查成功 */
    }                                              /* ID 匹配分支结束 */

    Debug_Print("[MPU9250] WHO_AM_I value error\r\n"); /* 打印 WHO_AM_I 值不匹配日志 */
    return -2;                                     /* 返回 ID 不匹配错误码 */
}                                                  /* 函数体结束 */

/**
 * @brief 软复位。
 * @param None
 * @return None
 */
void MPU9250_SoftReset(void)                       /* 定义 MPU9250 软复位函数 */
{                                                  /* 函数体开始 */
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0x6BU); /* 读取寄存器 0x6B 的值 */

    val &= (uint8_t)~(1U << 7U);                   /* 清除 bit7（DEVICE_RESET 位） */

    val |= (uint8_t)(1U << 7U);                    /* 设置 bit7（DEVICE_RESET 位）为 1，触发软复位 */

    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x6BU, val);   /* 向 PWR_MGMT_1 寄存器写入新的值，触发软复位 */

    vTaskDelay(pdMS_TO_TICKS(100));                /* 等待 100ms，确保复位完成 */
}                                                  /* 函数体结束 */

/**
 * @brief 读取 PWR_MGMT_1 寄存器，并判断 bit6：SLEEP 位。
 * @return None
 */
void MPU9250_Read_PowerMgmt(void)                  /* 定义读取 PWR_MGMT_1 寄存器函数 */
{                                                  /* 函数体开始 */
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0x6BU); /* 读取 PWR_MGMT_1 寄存器 */

    Debug_Printf("[MPU9250] PWR_MGMT_1 = 0x%02X\r\n", val); /* bit6 读到 0 就是没有在睡眠 */
}                                                  /* 函数体结束 */

/**
 * @brief 唤醒以及设置时钟源。
 * @param None
 * @return None
 */
void mpu_set_clock_to_auto(void)                   /* 定义唤醒以及设置时钟源函数 */
{                                                  /* 函数体开始 */
    uint8_t reg_val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0x6BU); /* 读取寄存器 0x6B 的当前值 */

    reg_val &= (uint8_t)~((1U << 6U) | 0x07U);     /* 清空 bit6（SLEEP 位）和 bit0-2（时钟源选择位） */

    reg_val |= 0x01U;                              /* 设置 bit0-2 为 001，选择 PLL 作为时钟源 */

    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x6BU, reg_val); /* 向寄存器 0x6B 写入修改后的值 */
}                                                  /* 函数体结束 */

/**
 * @brief 使能六轴传感器。
 * @param None
 * @return None
 */
void mpu_enable_six_axis(void)                     /* 定义使能六轴传感器函数 */
{                                                  /* 函数体开始 */
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0x6CU); /* 读取寄存器 0x6C 的值 */

    val &= (uint8_t)~0x3FU;                        /* 清除 bit5~0（六个 DIS 位），启用六轴 */

    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x6CU, val);   /* 写回寄存器 0x6C */
}                                                  /* 函数体结束 */

/**
 * @brief 配置 DLPF（低通滤波）。
 * @param None
 * @return None
 */
void mpu_set_dlpf_cfg_3(void)                      /* 定义配置 DLPF 函数 */
{                                                  /* 函数体开始 */
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0x1AU); /* 读取寄存器 0x1A 的值 */

    val = (uint8_t)((val & (uint8_t)~0x07U) | 0x03U); /* 清除 DLPF_CFG 位，然后设置为 011 */

    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x1AU, val);   /* 写回寄存器 0x1A */
}                                                  /* 函数体结束 */

/**
 * @brief 配置采样率。
 * @param None
 * @return None
 */
void mpu_set_sample_rate_200hz(void)               /* 定义配置采样率函数 */
{                                                  /* 函数体开始 */
    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x19U, 0x04U); /* 设置采样率寄存器 SMPLRT_DIV 为 4，采样率=1000/(1+4)=200Hz */
}                                                  /* 函数体结束 */

/**
 * @brief 配置加速度计的 DLPF。
 * @param None
 * @return None
 */
void mpu_set_accel_dlpf(void)                      /* 定义配置加速度计 DLPF 函数 */
{                                                  /* 函数体开始 */
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0x1DU); /* 读取 ACCEL_CONFIG2 */

    val &= (uint8_t)~0x07U;                        /* 清除 A_DLPFCFG[2:0] */
    val |= 0x03U;                                  /* 设置 A_DLPFCFG = 3，加速度计带宽 41Hz 左右 */

    val &= (uint8_t)~(1U << 3U);                   /* accel_fchoice_b = 0，启用滤波器 */

    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x1DU, val);   /* 写回 ACCEL_CONFIG2 */
}                                                  /* 函数体结束 */

/**
 * @brief 配置陀螺仪的量程。
 * @param None
 * @return None
 */
void mpu_set_gyro_config(void)                     /* 定义配置陀螺仪量程函数 */
{                                                  /* 函数体开始 */
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0x1BU); /* 读取寄存器 0x1B 的值 */

    val &= (uint8_t)~0x03U;                        /* Fchoice_b = 00，清除 bit1:0 */
    val &= (uint8_t)~(3U << 3U);                   /* 清除量程选择位 */
    val |= (uint8_t)(2U << 3U);                    /* GYRO_FS_SEL = 10，选择 +/-1000 dps */

    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x1BU, val);   /* 写回寄存器 0x1B */
}                                                  /* 函数体结束 */

/**
 * @brief 配置加速度计的量程。
 * @param None
 * @return None
 */
void mpu_set_accel_range(void)                     /* 定义配置加速度计量程函数 */
{                                                  /* 函数体开始 */
    uint8_t val = I2C_ReadReg(MPU9250_I2C_ADDR7, 0x1CU); /* 读取寄存器 0x1C 的值 */

    val &= (uint8_t)~(3U << 3U);                   /* 清除量程选择位 */
    val |= (uint8_t)(2U << 3U);                    /* ACCEL_FS_SEL = 10，选择 +/-8g */

    I2C_WriteReg(MPU9250_I2C_ADDR7, 0x1CU, val);   /* 写回寄存器 0x1C */
}                                                  /* 函数体结束 */

/**
 * @brief 读取 MPU9250 的 16 位寄存器值。
 * @param reg 寄存器地址。
 * @return 读取到的 16 位有符号整数。
 */
static int16_t mpu_read_word(uint8_t reg)          /* 定义读取 16 位寄存器值函数 */
{                                                  /* 函数体开始 */
    uint8_t high_byte = I2C_ReadReg(MPU9250_I2C_ADDR7, reg); /* 读取高字节 */
    uint8_t low_byte = I2C_ReadReg(MPU9250_I2C_ADDR7, (uint8_t)(reg + 1U)); /* 读取低字节 */

    return (int16_t)(((uint16_t)high_byte << 8U) | (uint16_t)low_byte); /* 组合并返回 16 位原始值 */
}                                                  /* 函数体结束 */

/**
 * @brief 读取 MPU9250 六轴传感器的原始数据。
 * @param raw 用于保存六轴原始数据的结构体指针。
 * @return None
 */
void MPU9250_ReadAxis(MPU9250_raw_Data *raw)       /* 定义读取六轴原始数据函数 */
{                                                  /* 函数体开始 */
    if (raw == 0)                                  /* 判断输出结构体指针是否为空 */
    {                                              /* 空指针保护分支开始 */
        return;                                    /* 指针无效时直接返回 */
    }                                              /* 空指针保护分支结束 */

    raw->accel_x = mpu_read_word(0x3BU);           /* 读取加速度 X 轴原始值 */
    raw->accel_y = mpu_read_word(0x3DU);           /* 读取加速度 Y 轴原始值 */
    raw->accel_z = mpu_read_word(0x3FU);           /* 读取加速度 Z 轴原始值 */

    raw->gyro_x = mpu_read_word(0x43U);            /* 读取陀螺仪 X 轴原始值 */
    raw->gyro_y = mpu_read_word(0x45U);            /* 读取陀螺仪 Y 轴原始值 */
    raw->gyro_z = mpu_read_word(0x47U);            /* 读取陀螺仪 Z 轴原始值 */

    raw->temp = mpu_read_word(0x41U);              /* 读取温度原始值 */
}                                                  /* 函数体结束 */

/**
 * @brief  初始化 MPU9250。
 * @param  None
 * @retval 1 表示初始化成功，0 表示初始化失败。
 */
uint8_t MPU9250_Driver_Init(void)                  /* 定义 MPU9250 驱动初始化函数 */
{                                                  /* 函数体开始 */
    int ret = 0;                                   /* 定义设备检查返回值并初始化为 0 */
    MPU9250_raw_Data raw;                          /* 定义六轴原始数据测试变量 */

    Debug_Print("[MPU9250] driver init start\r\n"); /* 打印 MPU9250 驱动初始化开始日志 */

    BSP_I2C_Soft_Init();                           /* 初始化 PB6/PB7 软件 I2C 总线 */
    Debug_Print("[MPU9250] soft i2c init ok\r\n"); /* 打印软件 I2C 初始化完成日志 */

    ret = MPU9250_CheckDevice();                   /* 读取并校验 MPU9250 WHO_AM_I 寄存器 */
    if (ret != 0)                                  /* 判断 WHO_AM_I 校验是否失败 */
    {                                              /* 初始化失败分支开始 */
        Debug_Print("[MPU9250] driver init failed\r\n"); /* 打印 MPU9250 驱动初始化失败日志 */
        return 0U;                                 /* 返回 0，表示 MPU9250 初始化失败 */
    }                                              /* 初始化失败分支结束 */

    MPU9250_SoftReset();                           /* 软复位 MPU9250 */
    mpu_set_clock_to_auto();                       /* 唤醒 MPU9250 并设置时钟源 */
    mpu_enable_six_axis();                         /* 启用六轴传感器 */
    mpu_set_dlpf_cfg_3();                          /* 配置陀螺仪 DLPF */
    mpu_set_sample_rate_200hz();                   /* 配置采样率 */
    mpu_set_accel_dlpf();                          /* 配置加速度计 DLPF */
    mpu_set_gyro_config();                         /* 配置陀螺仪量程 */
    mpu_set_accel_range();                         /* 配置加速度计量程 */
    MPU9250_Read_PowerMgmt();                      /* 打印 PWR_MGMT_1，确认是否退出睡眠 */

    MPU9250_ReadAxis(&raw);                        /* 读取一次六轴原始数据用于测试输出 */
    Debug_Printf("[MPU9250] raw ax=%d ay=%d az=%d gx=%d gy=%d gz=%d temp=%d\r\n", /* 打印六轴原始数据 */
                 raw.accel_x,                      /* 输出加速度 X 轴 */
                 raw.accel_y,                      /* 输出加速度 Y 轴 */
                 raw.accel_z,                      /* 输出加速度 Z 轴 */
                 raw.gyro_x,                       /* 输出陀螺仪 X 轴 */
                 raw.gyro_y,                       /* 输出陀螺仪 Y 轴 */
                 raw.gyro_z,                       /* 输出陀螺仪 Z 轴 */
                 raw.temp);                        /* 输出温度原始值 */

    Debug_Print("[MPU9250] driver init ok\r\n");   /* 打印 MPU9250 驱动初始化成功日志 */
    return 1U;                                     /* 返回 1，表示 MPU9250 初始化成功 */
}                                                  /* 函数体结束 */
