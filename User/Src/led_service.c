#include "led_service.h" // 提供 LED 服务层接口

/**
 * @brief 初始化 LED 服务。
 * @retval None
 */
void LedService_Init(void) // 初始化 LED 服务
{
    BSP_LED_Init(); // 初始化底层 LED GPIO
}

/**
 * @brief 打开 LED。
 * @retval None
 */
void LedService_On(void) // 打开 LED
{
    BSP_LED_On(); // 调用底层 LED 打开接口
}

/**
 * @brief 关闭 LED。
 * @retval None
 */
void LedService_Off(void) // 关闭 LED
{
    BSP_LED_Off(); // 调用底层 LED 关闭接口
}

/**
 * @brief 翻转 LED 当前亮灭状态。
 * @retval None
 */
void LedService_Toggle(void) // 翻转 LED
{
    BSP_LED_Toggle(); // 调用底层 LED 翻转接口
}
