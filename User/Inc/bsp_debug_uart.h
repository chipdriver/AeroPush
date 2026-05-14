#ifndef __BSP_DEBUG_UART_H__ /* 防止 bsp_debug_uart.h 被重复包含 */
#define __BSP_DEBUG_UART_H__ /* 定义调试串口头文件保护宏 */

#include "stm32f4xx.h"       /* 引入 STM32F4 基础定义 */
#include "stm32f4xx_conf.h"  /* 引入标准外设库配置 */
#include "stm32f4xx_usart.h" /* 引入 USART 标准外设接口 */

#define USART6_Port GPIOA         /* 定义 USART6 使用的 GPIO 端口 */
#define USART6_TX_Pin GPIO_Pin_11 /* 定义 USART6 TX 引脚 */
#define USART6_RX_Pin GPIO_Pin_12 /* 定义 USART6 RX 引脚 */

void BSP_DebugUart_Init(void);                  /* 声明调试串口初始化函数 */
void BSP_DebugUart_SendChar(char ch);           /* 声明调试串口单字符发送函数 */
void BSP_DebugUart_SendString(const char *str); /* 声明调试串口字符串发送函数 */

#endif /* 结束调试串口头文件保护宏 */
