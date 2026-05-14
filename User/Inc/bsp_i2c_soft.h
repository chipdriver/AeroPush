/**
 * @file    bsp_i2c_soft.h
 * @brief   软件模拟 I2C 驱动接口声明，当前用于 PB6/PB7 连接 MPU9250。
 */
#ifndef __BSP_I2C_SOFT_H__                 /* 防止头文件被重复包含 */
#define __BSP_I2C_SOFT_H__                 /* 定义头文件保护宏 */

#include <stdint.h>                        /* 引入 uint8_t、uint16_t 等标准整数类型 */
#include "stm32f4xx.h"                     /* 引入 STM32F4 标准外设库核心定义 */

#ifdef __cplusplus                         /* 兼容 C++ 编译环境 */
extern "C" {                               /* 使用 C 链接方式导出接口 */
#endif                                     /* 结束 C++ 兼容判断 */

/* MPU9250 wiring: SCL -> PB6, SDA -> PB7, AD0 -> GND, NCS -> 3.3V. */ /* 记录当前硬件接线 */
#define I2C_GPIO_PORT       GPIOB          /* 软件 I2C 所在 GPIO 端口 */
#define I2C_GPIO_CLK        RCC_AHB1Periph_GPIOB /* GPIOB 的 AHB1 外设时钟 */
#define I2C_SCL_PIN         GPIO_Pin_6     /* SCL 时钟线使用 PB6 */
#define I2C_SDA_PIN         GPIO_Pin_7     /* SDA 数据线使用 PB7 */
#define I2C_MPU9250_ADDR    0x68U          /* AD0 接 GND 时 MPU9250 的 7 位地址 */

#define I2C_ACK             0U             /* 读字节后发送 ACK 的参数值 */
#define I2C_NACK_BIT        1U             /* 读字节后发送 NACK 的参数值 */

#define SCL_H()             (I2C_GPIO_PORT->BSRR = (uint32_t)I2C_SCL_PIN) /* 释放/拉高 SCL */
#define SCL_L()             (I2C_GPIO_PORT->BSRR = ((uint32_t)I2C_SCL_PIN << 16U)) /* 拉低 SCL */
#define SCL_READ()          ((I2C_GPIO_PORT->IDR & I2C_SCL_PIN) ? 1U : 0U) /* 读取 SCL 电平 */

#define SDA_H()             (I2C_GPIO_PORT->BSRR = (uint32_t)I2C_SDA_PIN) /* 释放/拉高 SDA */
#define SDA_L()             (I2C_GPIO_PORT->BSRR = ((uint32_t)I2C_SDA_PIN << 16U)) /* 拉低 SDA */
#define SDA_READ()          ((I2C_GPIO_PORT->IDR & I2C_SDA_PIN) ? 1U : 0U) /* 读取 SDA 电平 */

/**
 * @brief  初始化软件 I2C GPIO。
 * @param  None
 * @retval None
 */
void    BSP_I2C_Soft_Init(void);           /* 初始化 PB6/PB7 为开漏上拉输出 */

/**
 * @brief  尝试恢复被从机拉住的数据总线。
 * @param  None
 * @retval 0 表示恢复成功，负数表示恢复失败。
 */
int     BSP_I2C_Soft_RecoverBus(void);     /* 通过额外 SCL 脉冲释放 SDA */

/**
 * @brief  探测指定 7 位地址的 I2C 设备是否应答。
 * @param  dev7 7 位 I2C 设备地址，不包含读写位。
 * @retval 0 表示收到 ACK，负数表示无应答。
 */
int     I2C_CheckDevice(uint8_t dev7);     /* 发送地址帧检查设备是否存在 */

/**
 * @brief  产生 I2C 起始条件。
 * @param  None
 * @retval None
 */
void    I2C_Start(void);                   /* SCL 高电平期间 SDA 从高到低 */

/**
 * @brief  产生 I2C 停止条件。
 * @param  None
 * @retval None
 */
void    I2C_Stop(void);                    /* SCL 高电平期间 SDA 从低到高 */

/**
 * @brief  等待从机 ACK。
 * @param  None
 * @retval 0 表示 ACK，1 表示 NACK。
 */
uint8_t I2C_WaitAck(void);                 /* 读取第 9 个时钟周期的 SDA 状态 */

/**
 * @brief  主机发送 ACK。
 * @param  None
 * @retval None
 */
void    I2C_SendACK(void);                 /* 主机继续读取时发送 ACK */

/**
 * @brief  主机发送 NACK。
 * @param  None
 * @retval None
 */
void    I2C_NACK(void);                    /* 主机结束读取时发送 NACK */

/**
 * @brief  主机发送 NACK，作为 I2C_NACK 的语义化别名。
 * @param  None
 * @retval None
 */
void    I2C_SendNACK(void);                /* 保留更直观的 NACK 函数名 */

/**
 * @brief  发送 1 个字节。
 * @param  data 要发送的字节。
 * @retval None
 */
void    I2C_SendByte(uint8_t data);        /* 按 MSB 到 LSB 发送 8 位数据 */

/**
 * @brief  读取 1 个字节。
 * @param  send_ack 读后应答控制，I2C_ACK 发送 ACK，I2C_NACK_BIT 发送 NACK。
 * @retval 读取到的字节。
 */
uint8_t I2C_ReadByte(uint8_t send_ack);    /* 按 MSB 到 LSB 读取 8 位数据 */

/**
 * @brief  向设备寄存器写入 1 个字节。
 * @param  dev7 7 位 I2C 设备地址。
 * @param  reg  寄存器地址。
 * @param  val  要写入的数据。
 * @retval 0 表示成功，负数表示失败。
 */
int     I2C_WriteReg(uint8_t dev7, uint8_t reg, uint8_t val); /* 写单字节寄存器 */

/**
 * @brief  从设备寄存器读取 1 个字节。
 * @param  dev7 7 位 I2C 设备地址。
 * @param  reg  寄存器地址。
 * @retval 读取到的数据；通信失败时返回 0xFF。
 */
uint8_t I2C_ReadReg(uint8_t dev7, uint8_t reg); /* 兼容旧接口的单字节读取 */

/**
 * @brief  从设备寄存器读取 1 个字节并返回状态码。
 * @param  dev7 7 位 I2C 设备地址。
 * @param  reg  寄存器地址。
 * @param  val  保存读取结果的指针。
 * @retval 0 表示成功，负数表示失败。
 */
int     I2C_ReadRegData(uint8_t dev7, uint8_t reg, uint8_t *val); /* 带状态码的单字节读取 */

/**
 * @brief  向连续寄存器写入多个字节。
 * @param  dev7 7 位 I2C 设备地址。
 * @param  reg  起始寄存器地址。
 * @param  buf  待写入数据缓冲区。
 * @param  len  待写入字节数。
 * @retval 0 表示成功，负数表示失败。
 */
int     I2C_WriteRegs(uint8_t dev7, uint8_t reg, const uint8_t *buf, uint16_t len); /* 多字节写寄存器 */

/**
 * @brief  从连续寄存器读取多个字节。
 * @param  dev7 7 位 I2C 设备地址。
 * @param  reg  起始寄存器地址。
 * @param  buf  保存读取结果的缓冲区。
 * @param  len  待读取字节数。
 * @retval 0 表示成功，负数表示失败。
 */
int     I2C_ReadRegs(uint8_t dev7, uint8_t reg, uint8_t *buf, uint16_t len); /* 多字节读寄存器 */

#ifdef __cplusplus                         /* 兼容 C++ 编译环境 */
}                                          /* 结束 C 链接声明区域 */
#endif                                     /* 结束 C++ 兼容判断 */

#endif /* __BSP_I2C_SOFT_H__ */            /* 结束头文件保护 */
