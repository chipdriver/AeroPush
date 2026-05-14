#ifndef __BSP_LED_H__                            /* 防止 bsp_led.h 被重复包含 */
#define __BSP_LED_H__                            /* 定义 LED BSP 头文件保护宏 */

#include "stm32f4xx.h"                           /* 引入 STM32F4 基础定义 */

#define LED_RED_Port    GPIOA                    /* 定义红色 LED GPIO 端口 */
#define LED_RED_Pin     GPIO_Pin_1               /* 定义红色 LED GPIO 引脚 */
#define LED_GREEN_Port  GPIOA                    /* 定义绿色 LED GPIO 端口 */
#define LED_GREEN_Pin   GPIO_Pin_2               /* 定义绿色 LED GPIO 引脚 */

void BSP_LED_Init(void);                         /* 声明 LED GPIO 初始化函数 */
void BSP_LED_On(void);                           /* 声明 LED 打开函数 */
void BSP_LED_Off(void);                          /* 声明 LED 关闭函数 */
void BSP_LED_Toggle(void);                       /* 声明 LED 翻转函数 */

#endif                                           /* 结束 LED BSP 头文件保护宏 */
