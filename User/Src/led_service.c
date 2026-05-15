#include "led_service.h" // 引入 LED 服务接口

/**
 * @brief  初始化 LED 服务。
 * @param  None
 * @retval None
 */
void LedService_Init(void) // 定义 LED 服务初始化函数
{                          // 进入代码块
    BSP_LED_Init();        // 调用 LED BSP 初始化底层 GPIO
} // 结束代码块

/**
 * @brief  打开 LED。
 * @param  None
 * @retval None
 */
void LedService_On(void) // 定义 LED 服务打开函数
{                        // 进入代码块
    BSP_LED_On();        // 调用 LED BSP 打开 LED
} // 结束代码块

/**
 * @brief  关闭 LED。
 * @param  None
 * @retval None
 */
void LedService_Off(void) // 定义 LED 服务关闭函数
{                         // 进入代码块
    BSP_LED_Off();        // 调用 LED BSP 关闭 LED
} // 结束代码块

/**
 * @brief  翻转 LED 当前状态。
 * @param  None
 * @retval None
 */
void LedService_Toggle(void) // 定义 LED 服务翻转函数
{                            // 进入代码块
    BSP_LED_Toggle();        // 调用 LED BSP 翻转 LED 状态
} // 结束代码块
