#ifndef __BSP_LED_H__ // 防止头文件重复包含
#define __BSP_LED_H__

#include "stm32f4xx.h" // 提供 STM32F4 GPIO 定义

#define LED_RED_Port GPIOA // 红灯 GPIO 端口
#define LED_RED_Pin GPIO_Pin_1 // 红灯 GPIO 引脚
#define LED_GREEN_Port GPIOA // 绿灯 GPIO 端口
#define LED_GREEN_Pin GPIO_Pin_2 // 绿灯 GPIO 引脚

/**
 * @brief 初始化 LED GPIO。
 * @retval None
 */
void BSP_LED_Init(void); // 初始化 LED

/**
 * @brief 打开 LED。
 * @retval None
 */
void BSP_LED_On(void); // 打开 LED

/**
 * @brief 关闭 LED。
 * @retval None
 */
void BSP_LED_Off(void); // 关闭 LED

/**
 * @brief 翻转 LED。
 * @retval None
 */
void BSP_LED_Toggle(void); // 翻转 LED

#endif // __BSP_LED_H__
