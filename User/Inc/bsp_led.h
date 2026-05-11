#ifndef __BSP_LED_H__
#define __BSP_LED_H__

/* headler file inclution*/
#include "stm32f4xx.h"

/* macro definition */
#define LED_RED_Port GPIOA
#define LED_RED_Pin  GPIO_Pin_1

#define LED_GREEN_Port GPIOA
#define LED_GREEN_Pin  GPIO_Pin_2

/* function declaration */
void BSP_LED_Init(void);    //LED 初始化
void BSP_LED_On(void);      //LED 打开
void BSP_LED_Off(void);     //LED 关闭
void BSP_LED_Toggle(void);  //LED 翻转

#endif /* __BSP_LED_H__ */