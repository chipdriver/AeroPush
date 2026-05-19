#ifndef __BSP_DEBUG_UART_H__ // 检查 __BSP_DEBUG_UART_H__ 是否未定义，防止头文件重复包含
#define __BSP_DEBUG_UART_H__ // 定义 __BSP_DEBUG_UART_H__ 变量

#include "stm32f4xx.h" // 引入 stm32f4xx.h 提供的接口、宏和类型定义
#include "stm32f4xx_conf.h" // 引入 stm32f4xx_conf.h 提供的接口、宏和类型定义
#include "stm32f4xx_usart.h" // 引入 stm32f4xx_usart.h 提供的接口、宏和类型定义

#define USART6_Port GPIOA // 定义 USART6_Port 变量为 GPIOA
#define USART6_TX_Pin GPIO_Pin_11 // 定义 USART6_TX_Pin 变量为 GPIO_Pin_11
#define USART6_RX_Pin GPIO_Pin_12 // 定义 USART6_RX_Pin 变量为 GPIO_Pin_12

/**
 * @brief 初始化底层调试串口外设。
 * @retval None
 */
void BSP_DebugUart_Init(void); // 声明BSP_DebugUart_Init 函数签名：初始化底层调试串口外设
/**
 * @brief 通过调试串口发送单个字符。
 * @param ch ch 变量。
 * @retval None
 */
void BSP_DebugUart_SendChar(char ch); // 声明BSP_DebugUart_SendChar 函数签名：通过调试串口发送单个字符
/**
 * @brief 通过调试串口发送字符串。
 * @param str str 变量。
 * @retval None
 */
void BSP_DebugUart_SendString(const char *str); // 声明BSP_DebugUart_SendString 函数签名：通过调试串口发送字符串

#endif // 结束当前条件编译或头文件保护范围
