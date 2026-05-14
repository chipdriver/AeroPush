#ifndef __BSP_LED_H__                         // 防止 bsp_led.h 被重复包含
#define __BSP_LED_H__                         // 定义 LED BSP 头文件保护宏

/* headler file inclution */                  // 头文件包含区域
#include "stm32f4xx.h"                        // 引入 STM32F4 基础定义和 GPIO 类型

/* macro definition */                        // LED 引脚宏定义区域
#define LED_RED_Port GPIOA                    // 定义红色 LED 所在 GPIO 端口
#define LED_RED_Pin  GPIO_Pin_1               // 定义红色 LED 所在 GPIO 引脚

#define LED_GREEN_Port GPIOA                  // 定义绿色 LED 所在 GPIO 端口
#define LED_GREEN_Pin  GPIO_Pin_2             // 定义绿色 LED 所在 GPIO 引脚

/* function declaration */                    // 函数声明区域

/**
 * @brief  初始化板载 LED GPIO。
 * @param  None
 * @retval None
 */
void BSP_LED_Init(void);                      // 声明 LED 初始化函数

/**
 * @brief  打开板载 LED。
 * @param  None
 * @retval None
 */
void BSP_LED_On(void);                        // 声明 LED 打开函数

/**
 * @brief  关闭板载 LED。
 * @param  None
 * @retval None
 */
void BSP_LED_Off(void);                       // 声明 LED 关闭函数

/**
 * @brief  翻转板载 LED 当前状态。
 * @param  None
 * @retval None
 */
void BSP_LED_Toggle(void);                    // 声明 LED 翻转函数

#endif /* __BSP_LED_H__ */                    // 结束 LED BSP 头文件保护宏
