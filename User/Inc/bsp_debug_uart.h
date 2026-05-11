#ifndef __BSP_DEBUG_UART_H__
#define __BSP_DEBUG_UART_H__

/* headler file inclution*/
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_conf.h"

/* macro definition*/
#define USART6_Port GPIOA
#define USART6_TX_Pin  GPIO_Pin_11
#define USART6_RX_Pin  GPIO_Pin_12

/* function declaration */
void BSP_DebugUart_Init(void);
void BSP_DebugUart_SendChar(char ch);
void BSP_DebugUart_SendString(const char *str);

#endif /* __BSP_DEBUG_UART_H__ */