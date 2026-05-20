#ifndef __BSP_DEBUG_UART_H__ // 防止头文件重复包含
#define __BSP_DEBUG_UART_H__

#include "stm32f4xx.h" // 提供 STM32F4 外设定义
#include "stm32f4xx_conf.h" // 提供标准外设库配置
#include "stm32f4xx_usart.h" // 提供 USART 外设接口

#define USART6_Port GPIOA // USART6 使用的 GPIO 端口
#define USART6_TX_Pin GPIO_Pin_11 // USART6 TX 引脚
#define USART6_RX_Pin GPIO_Pin_12 // USART6 RX 引脚

/**
 * @brief 初始化底层调试串口外设。
 * @retval None
 */
void BSP_DebugUart_Init(void); // 初始化调试串口

/**
 * @brief 通过调试串口发送单个字符。
 * @param ch 待发送字符。
 * @retval None
 */
void BSP_DebugUart_SendChar(char ch); // 发送单个字符

/**
 * @brief 通过调试串口发送字符串。
 * @param str 以 '\0' 结尾的字符串。
 * @retval None
 */
void BSP_DebugUart_SendString(const char *str); // 发送字符串

#endif // __BSP_DEBUG_UART_H__
