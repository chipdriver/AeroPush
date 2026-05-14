#ifndef __BSP_I2C_SOFT_H__                                  /* 防止 bsp_i2c_soft.h 被重复包含 */
#define __BSP_I2C_SOFT_H__                                  /* 定义软件 I2C 头文件保护宏 */

#include <stdint.h>                                         /* 引入标准整数类型 */
#include "stm32f4xx.h"                                      /* 引入 STM32F4 基础定义 */

#ifdef __cplusplus                                          /* 判断是否为 C++ 编译环境 */
extern "C" {                                                /* 使用 C 链接方式导出接口 */
#endif                                                      /* 结束 C++ 编译环境判断 */

#define I2C_GPIO_PORT       GPIOB                           /* 定义软件 I2C 使用的 GPIO 端口 */
#define I2C_GPIO_CLK        RCC_AHB1Periph_GPIOB            /* 定义软件 I2C GPIO 时钟 */
#define I2C_SCL_PIN         GPIO_Pin_6                      /* 定义软件 I2C SCL 引脚 */
#define I2C_SDA_PIN         GPIO_Pin_7                      /* 定义软件 I2C SDA 引脚 */
#define I2C_MPU9250_ADDR    0x68U                           /* 定义 MPU9250 7 位 I2C 地址 */

#define I2C_ACK             0U                              /* 定义读字节后发送 ACK 的参数 */
#define I2C_NACK_BIT        1U                              /* 定义读字节后发送 NACK 的参数 */

#define SCL_H()             (I2C_GPIO_PORT->BSRR = (uint32_t)I2C_SCL_PIN)                 /* 释放或拉高 SCL */
#define SCL_L()             (I2C_GPIO_PORT->BSRR = ((uint32_t)I2C_SCL_PIN << 16U))         /* 拉低 SCL */
#define SCL_READ()          ((I2C_GPIO_PORT->IDR & I2C_SCL_PIN) ? 1U : 0U)                 /* 读取 SCL 电平 */
#define SDA_H()             (I2C_GPIO_PORT->BSRR = (uint32_t)I2C_SDA_PIN)                 /* 释放或拉高 SDA */
#define SDA_L()             (I2C_GPIO_PORT->BSRR = ((uint32_t)I2C_SDA_PIN << 16U))         /* 拉低 SDA */
#define SDA_READ()          ((I2C_GPIO_PORT->IDR & I2C_SDA_PIN) ? 1U : 0U)                 /* 读取 SDA 电平 */

void BSP_I2C_Soft_Init(void);                              /* 声明软件 I2C GPIO 初始化函数 */
int BSP_I2C_Soft_RecoverBus(void);                         /* 声明软件 I2C 总线恢复函数 */
int I2C_CheckDevice(uint8_t dev7);                         /* 声明 I2C 设备探测函数 */
void I2C_Start(void);                                      /* 声明 I2C 起始条件函数 */
void I2C_Stop(void);                                       /* 声明 I2C 停止条件函数 */
uint8_t I2C_WaitAck(void);                                 /* 声明等待从机 ACK 函数 */
void I2C_SendACK(void);                                    /* 声明主机发送 ACK 函数 */
void I2C_NACK(void);                                       /* 声明主机发送 NACK 函数 */
void I2C_SendNACK(void);                                   /* 声明主机发送 NACK 语义化别名函数 */
void I2C_SendByte(uint8_t data);                           /* 声明 I2C 单字节发送函数 */
uint8_t I2C_ReadByte(uint8_t send_ack);                    /* 声明 I2C 单字节读取函数 */
int I2C_WriteReg(uint8_t dev7, uint8_t reg, uint8_t val);  /* 声明 I2C 单寄存器写入函数 */
uint8_t I2C_ReadReg(uint8_t dev7, uint8_t reg);            /* 声明 I2C 单寄存器读取函数 */
int I2C_ReadRegData(uint8_t dev7, uint8_t reg, uint8_t *val); /* 声明带状态码的 I2C 单寄存器读取函数 */
int I2C_WriteRegs(uint8_t dev7, uint8_t reg, const uint8_t *buf, uint16_t len); /* 声明 I2C 多字节写入函数 */
int I2C_ReadRegs(uint8_t dev7, uint8_t reg, uint8_t *buf, uint16_t len);        /* 声明 I2C 多字节读取函数 */

#ifdef __cplusplus                                          /* 判断是否为 C++ 编译环境 */
}                                                           /* 结束 C 链接导出区域 */
#endif                                                      /* 结束 C++ 编译环境判断 */

#endif                                                      /* 结束软件 I2C 头文件保护宏 */
