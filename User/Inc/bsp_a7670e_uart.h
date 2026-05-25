#ifndef __BSP_A7670E_UART_H__ // 防止头文件重复包含
#define __BSP_A7670E_UART_H__

#include "stm32f4xx.h" // 提供 STM32F4 外设定义
#include "stm32f4xx_conf.h" // 提供标准外设库配置
#include "stm32f4xx_usart.h" // 提供 USART 外设接口

#define A7670E_UART_PORT GPIOA // A7670E 串口使用 GPIOA
#define A7670E_UART_TX_PIN GPIO_Pin_9 // PA9 作为 USART1_TX
#define A7670E_UART_RX_PIN GPIO_Pin_10 // PA10 作为 USART1_RX

/**
 * @brief 初始化 A7670E 使用的 USART1 引脚和串口外设。
 * @retval None
 */
void BSP_A7670E_Uart_Init(void); // 初始化 A7670E 串口

#endif // __BSP_A7670E_UART_H__
