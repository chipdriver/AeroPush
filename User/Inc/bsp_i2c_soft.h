#ifndef __BSP_I2C_SOFT_H__ // 防止头文件重复包含
#define __BSP_I2C_SOFT_H__

#include <stdint.h> // 提供固定宽度整数类型
#include "stm32f4xx.h" // 提供 STM32F4 外设寄存器定义

#ifdef __cplusplus
extern "C"
{
#endif

#define I2C_GPIO_PORT GPIOB // 软件 I2C 所在 GPIO 端口
#define I2C_GPIO_CLK RCC_AHB1Periph_GPIOB // 软件 I2C GPIO 时钟
#define I2C_SCL_PIN GPIO_Pin_6 // 软件 I2C SCL 引脚
#define I2C_SDA_PIN GPIO_Pin_7 // 软件 I2C SDA 引脚
#define I2C_MPU9250_ADDR 0x68U // MPU9250 7 位 I2C 地址

#define I2C_ACK 0U // I2C ACK 位
#define I2C_NACK_BIT 1U // I2C NACK 位

#define SCL_H() (I2C_GPIO_PORT->BSRR = (uint32_t)I2C_SCL_PIN) // 拉高或释放 SCL
#define SCL_L() (I2C_GPIO_PORT->BSRR = ((uint32_t)I2C_SCL_PIN << 16U)) // 拉低 SCL
#define SCL_READ() ((I2C_GPIO_PORT->IDR & I2C_SCL_PIN) ? 1U : 0U) // 读取 SCL 电平
#define SDA_H() (I2C_GPIO_PORT->BSRR = (uint32_t)I2C_SDA_PIN) // 拉高或释放 SDA
#define SDA_L() (I2C_GPIO_PORT->BSRR = ((uint32_t)I2C_SDA_PIN << 16U)) // 拉低 SDA
#define SDA_READ() ((I2C_GPIO_PORT->IDR & I2C_SDA_PIN) ? 1U : 0U) // 读取 SDA 电平

/**
 * @brief 初始化软件 I2C 引脚并恢复总线。
 * @retval None
 */
void BSP_I2C_Soft_Init(void); // 初始化软件 I2C

/**
 * @brief 尝试释放被从机占用的 I2C 总线。
 * @retval 0 总线空闲；-1 总线仍异常。
 */
int BSP_I2C_Soft_RecoverBus(void); // 恢复 I2C 总线

/**
 * @brief 探测 7 位 I2C 设备地址是否有应答。
 * @param dev7 7 位 I2C 设备地址。
 * @retval 0 有应答；-1 无应答。
 */
int I2C_CheckDevice(uint8_t dev7); // 检查 I2C 设备

/**
 * @brief 发送 I2C 起始条件。
 * @retval None
 */
void I2C_Start(void); // 发送 START

/**
 * @brief 发送 I2C 停止条件。
 * @retval None
 */
void I2C_Stop(void); // 发送 STOP

/**
 * @brief 等待从机 ACK。
 * @retval 0 ACK；1 NACK。
 */
uint8_t I2C_WaitAck(void); // 等待 ACK

/**
 * @brief 主机发送 ACK。
 * @retval None
 */
void I2C_SendACK(void); // 发送 ACK

/**
 * @brief 主机发送 NACK。
 * @retval None
 */
void I2C_NACK(void); // 发送 NACK

/**
 * @brief 主机发送 NACK。
 * @retval None
 */
void I2C_SendNACK(void); // 发送 NACK 兼容接口

/**
 * @brief 发送 1 个 I2C 字节。
 * @param data 待发送字节。
 * @retval None
 */
void I2C_SendByte(uint8_t data); // 发送字节

/**
 * @brief 读取 1 个 I2C 字节。
 * @param send_ack 读完后发送 ACK 或 NACK。
 * @retval 读取到的字节。
 */
uint8_t I2C_ReadByte(uint8_t send_ack); // 读取字节

/**
 * @brief 写 1 个设备寄存器。
 * @param dev7 7 位 I2C 设备地址。
 * @param reg 寄存器地址。
 * @param val 写入值。
 * @retval 0 成功；负数表示失败阶段。
 */
int I2C_WriteReg(uint8_t dev7, uint8_t reg, uint8_t val); // 写单个寄存器

/**
 * @brief 读 1 个设备寄存器。
 * @param dev7 7 位 I2C 设备地址。
 * @param reg 寄存器地址。
 * @retval 寄存器值，失败时为默认值。
 */
uint8_t I2C_ReadReg(uint8_t dev7, uint8_t reg); // 读单个寄存器

/**
 * @brief 读 1 个设备寄存器并返回状态。
 * @param dev7 7 位 I2C 设备地址。
 * @param reg 寄存器地址。
 * @param val 输出寄存器值。
 * @retval 0 成功；负数表示失败阶段。
 */
int I2C_ReadRegData(uint8_t dev7, uint8_t reg, uint8_t *val); // 读单个寄存器并返回错误码

/**
 * @brief 从指定寄存器开始连续写入多个字节。
 * @param dev7 7 位 I2C 设备地址。
 * @param reg 起始寄存器地址。
 * @param buf 待写入缓冲区。
 * @param len 写入字节数。
 * @retval 0 成功；负数表示失败阶段。
 */
int I2C_WriteRegs(uint8_t dev7, uint8_t reg, const uint8_t *buf, uint16_t len); // 连续写寄存器

/**
 * @brief 从指定寄存器开始连续读取多个字节。
 * @param dev7 7 位 I2C 设备地址。
 * @param reg 起始寄存器地址。
 * @param buf 接收缓冲区。
 * @param len 读取字节数。
 * @retval 0 成功；负数表示失败阶段。
 */
int I2C_ReadRegs(uint8_t dev7, uint8_t reg, uint8_t *buf, uint16_t len); // 连续读寄存器

#ifdef __cplusplus
}
#endif

#endif // __BSP_I2C_SOFT_H__
