#ifndef __LED_SERVICE_H__ // 防止头文件重复包含
#define __LED_SERVICE_H__

#include "bsp_led.h" // 提供底层 LED 控制接口

/**
 * @brief 初始化 LED 服务。
 * @retval None
 */
void LedService_Init(void); // 初始化 LED 服务

/**
 * @brief 打开 LED。
 * @retval None
 */
void LedService_On(void); // 打开 LED

/**
 * @brief 关闭 LED。
 * @retval None
 */
void LedService_Off(void); // 关闭 LED

/**
 * @brief 翻转 LED 当前亮灭状态。
 * @retval None
 */
void LedService_Toggle(void); // 翻转 LED

#endif // __LED_SERVICE_H__
