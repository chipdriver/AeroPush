#ifndef __BSP_I2C_SOFT_H__ // 检查 __BSP_I2C_SOFT_H__ 是否未定义，防止头文件重复包含
#define __BSP_I2C_SOFT_H__ // 定义 __BSP_I2C_SOFT_H__ 变量

#include <stdint.h> // 引入 stdint.h 提供的接口、宏和类型定义
#include "stm32f4xx.h" // 引入 stm32f4xx.h 提供的接口、宏和类型定义

#ifdef __cplusplus // 根据芯片型号或编译选项选择参与编译的代码
extern "C" // 执行 extern "C"，完成当前上下文中的具体处理
{ // 进入当前代码块
#endif // 结束当前条件编译或头文件保护范围

#define I2C_GPIO_PORT GPIOB // 定义 I2C_GPIO_PORT 变量为 GPIOB
#define I2C_GPIO_CLK RCC_AHB1Periph_GPIOB // 定义 I2C_GPIO_CLK 变量为 RCC_AHB1Periph_GPIOB
#define I2C_SCL_PIN GPIO_Pin_6 // 定义 I2C_SCL_PIN 变量为 GPIO_Pin_6
#define I2C_SDA_PIN GPIO_Pin_7 // 定义 I2C_SDA_PIN 变量为 GPIO_Pin_7
#define I2C_MPU9250_ADDR 0x68U // 定义 I2C_MPU9250_ADDR 变量为 0x68U

#define I2C_ACK 0U // 定义 I2C_ACK 变量为 0U
#define I2C_NACK_BIT 1U // 定义 I2C_NACK_BIT 变量为 1U

#define SCL_H() (I2C_GPIO_PORT->BSRR = (uint32_t)I2C_SCL_PIN) // 定义 SCL_H 变量
#define SCL_L() (I2C_GPIO_PORT->BSRR = ((uint32_t)I2C_SCL_PIN << 16U)) // 定义 SCL_L 变量
#define SCL_READ() ((I2C_GPIO_PORT->IDR & I2C_SCL_PIN) ? 1U : 0U) // 定义 SCL_READ 变量
#define SDA_H() (I2C_GPIO_PORT->BSRR = (uint32_t)I2C_SDA_PIN) // 定义 SDA_H 变量
#define SDA_L() (I2C_GPIO_PORT->BSRR = ((uint32_t)I2C_SDA_PIN << 16U)) // 定义 SDA_L 变量
#define SDA_READ() ((I2C_GPIO_PORT->IDR & I2C_SDA_PIN) ? 1U : 0U) // 定义 SDA_READ 变量

    /**
     * @brief BSP_I2C_Soft_Init 函数。
     * @retval None
     */
    void BSP_I2C_Soft_Init(void); // 声明BSP_I2C_Soft_Init 函数签名：BSP_I2C_Soft_Init 函数
    /**
     * @brief BSP_I2C_Soft_RecoverBus 函数。
     * @retval 函数执行结果或计算得到的返回值。
     */
    int BSP_I2C_Soft_RecoverBus(void); // 声明BSP_I2C_Soft_RecoverBus 函数签名：BSP_I2C_Soft_RecoverBus 函数
    /**
     * @brief I2C_CheckDevice 函数。
     * @param dev7 dev7 变量。
     * @retval 函数执行结果或计算得到的返回值。
     */
    int I2C_CheckDevice(uint8_t dev7); // 声明I2C_CheckDevice 函数签名：I2C_CheckDevice 函数
    /**
     * @brief I2C_Start 函数。
     * @retval None
     */
    void I2C_Start(void); // 声明I2C_Start 函数签名：I2C_Start 函数
    /**
     * @brief I2C_Stop 函数。
     * @retval None
     */
    void I2C_Stop(void); // 声明I2C_Stop 函数签名：I2C_Stop 函数
    /**
     * @brief I2C_WaitAck 函数。
     * @retval 函数执行结果或计算得到的返回值。
     */
    uint8_t I2C_WaitAck(void); // 声明I2C_WaitAck 函数签名：I2C_WaitAck 函数
    /**
     * @brief I2C_SendACK 函数。
     * @retval None
     */
    void I2C_SendACK(void); // 声明I2C_SendACK 函数签名：I2C_SendACK 函数
    /**
     * @brief I2C_NACK 函数。
     * @retval None
     */
    void I2C_NACK(void); // 声明I2C_NACK 函数签名：I2C_NACK 函数
    /**
     * @brief I2C_SendNACK 函数。
     * @retval None
     */
    void I2C_SendNACK(void); // 声明I2C_SendNACK 函数签名：I2C_SendNACK 函数
    /**
     * @brief I2C_SendByte 函数。
     * @param data data 变量。
     * @retval None
     */
    void I2C_SendByte(uint8_t data); // 声明I2C_SendByte 函数签名：I2C_SendByte 函数
    /**
     * @brief I2C_ReadByte 函数。
     * @param send_ack send_ack 变量。
     * @retval 函数执行结果或计算得到的返回值。
     */
    uint8_t I2C_ReadByte(uint8_t send_ack); // 声明I2C_ReadByte 函数签名：I2C_ReadByte 函数
    /**
     * @brief I2C_WriteReg 函数。
     * @param dev7 dev7 变量。
     * @param reg reg 变量。
     * @param val 寄存器临时读写值。
     * @retval 函数执行结果或计算得到的返回值。
     */
    int I2C_WriteReg(uint8_t dev7, uint8_t reg, uint8_t val); // 声明I2C_WriteReg 函数签名：I2C_WriteReg 函数
    /**
     * @brief I2C_ReadReg 函数。
     * @param dev7 dev7 变量。
     * @param reg reg 变量。
     * @retval 函数执行结果或计算得到的返回值。
     */
    uint8_t I2C_ReadReg(uint8_t dev7, uint8_t reg); // 声明I2C_ReadReg 函数签名：I2C_ReadReg 函数
    /**
     * @brief I2C_ReadRegData 函数。
     * @param dev7 dev7 变量。
     * @param reg reg 变量。
     * @param val 寄存器临时读写值。
     * @retval 函数执行结果或计算得到的返回值。
     */
    int I2C_ReadRegData(uint8_t dev7, uint8_t reg, uint8_t *val); // 声明I2C_ReadRegData 函数签名：I2C_ReadRegData 函数
    /**
     * @brief I2C_WriteRegs 函数。
     * @param dev7 dev7 变量。
     * @param reg reg 变量。
     * @param buf buf 变量。
     * @param len len 变量。
     * @retval 函数执行结果或计算得到的返回值。
     */
    int I2C_WriteRegs(uint8_t dev7, uint8_t reg, const uint8_t *buf, uint16_t len); // 声明I2C_WriteRegs 函数签名：I2C_WriteRegs 函数
    /**
     * @brief I2C_ReadRegs 函数。
     * @param dev7 dev7 变量。
     * @param reg reg 变量。
     * @param buf buf 变量。
     * @param len len 变量。
     * @retval 函数执行结果或计算得到的返回值。
     */
    int I2C_ReadRegs(uint8_t dev7, uint8_t reg, uint8_t *buf, uint16_t len); // 声明I2C_ReadRegs 函数签名：I2C_ReadRegs 函数

#ifdef __cplusplus // 根据芯片型号或编译选项选择参与编译的代码
} // 结束当前代码块
#endif // 结束当前条件编译或头文件保护范围

#endif // 结束当前条件编译或头文件保护范围
