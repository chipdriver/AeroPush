#ifndef __BSP_LED_H__ // 检查 __BSP_LED_H__ 是否未定义，防止头文件重复包含
#define __BSP_LED_H__ // 定义 __BSP_LED_H__ 变量

#include "stm32f4xx.h" // 引入 stm32f4xx.h 提供的接口、宏和类型定义

#define LED_RED_Port GPIOA // 定义 LED_RED_Port 变量为 GPIOA
#define LED_RED_Pin GPIO_Pin_1 // 定义 LED_RED_Pin 变量为 GPIO_Pin_1
#define LED_GREEN_Port GPIOA // 定义 LED_GREEN_Port 变量为 GPIOA
#define LED_GREEN_Pin GPIO_Pin_2 // 定义 LED_GREEN_Pin 变量为 GPIO_Pin_2

/**
 * @brief BSP_LED_Init 函数。
 * @retval None
 */
void BSP_LED_Init(void); // 声明BSP_LED_Init 函数签名：BSP_LED_Init 函数
/**
 * @brief BSP_LED_On 函数。
 * @retval None
 */
void BSP_LED_On(void); // 声明BSP_LED_On 函数签名：BSP_LED_On 函数
/**
 * @brief BSP_LED_Off 函数。
 * @retval None
 */
void BSP_LED_Off(void); // 声明BSP_LED_Off 函数签名：BSP_LED_Off 函数
/**
 * @brief BSP_LED_Toggle 函数。
 * @retval None
 */
void BSP_LED_Toggle(void); // 声明BSP_LED_Toggle 函数签名：BSP_LED_Toggle 函数

#endif // 结束当前条件编译或头文件保护范围
