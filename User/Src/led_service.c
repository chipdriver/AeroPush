#include "led_service.h" // 引入 led_service.h 提供的接口、宏和类型定义

/**
 * @brief 初始化 LED 服务。
 * @retval None
 */
void LedService_Init(void) // 定义LedService_Init 函数签名：初始化 LED 服务
{ // 进入当前代码块
    BSP_LED_Init(); // 调用BSP_LED_Init 函数
} // 结束当前代码块

/**
 * @brief LedService_On 函数。
 * @retval None
 */
void LedService_On(void) // 定义LedService_On 函数签名：LedService_On 函数
{ // 进入当前代码块
    BSP_LED_On(); // 调用BSP_LED_On 函数
} // 结束当前代码块

/**
 * @brief LedService_Off 函数。
 * @retval None
 */
void LedService_Off(void) // 定义LedService_Off 函数签名：LedService_Off 函数
{ // 进入当前代码块
    BSP_LED_Off(); // 调用BSP_LED_Off 函数
} // 结束当前代码块

/**
 * @brief 翻转 LED 当前亮灭状态。
 * @retval None
 */
void LedService_Toggle(void) // 定义LedService_Toggle 函数签名：翻转 LED 当前亮灭状态
{ // 进入当前代码块
    BSP_LED_Toggle(); // 调用BSP_LED_Toggle 函数
} // 结束当前代码块
