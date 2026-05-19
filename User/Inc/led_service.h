#ifndef __LED_SERVICE_H__ // 检查 __LED_SERVICE_H__ 是否未定义，防止头文件重复包含
#define __LED_SERVICE_H__ // 定义 __LED_SERVICE_H__ 变量

#include "bsp_led.h" // 引入 bsp_led.h 提供的接口、宏和类型定义

/**
 * @brief 初始化 LED 服务。
 * @retval None
 */
void LedService_Init(void); // 声明LedService_Init 函数签名：初始化 LED 服务
/**
 * @brief LedService_On 函数。
 * @retval None
 */
void LedService_On(void); // 声明LedService_On 函数签名：LedService_On 函数
/**
 * @brief LedService_Off 函数。
 * @retval None
 */
void LedService_Off(void); // 声明LedService_Off 函数签名：LedService_Off 函数
/**
 * @brief 翻转 LED 当前亮灭状态。
 * @retval None
 */
void LedService_Toggle(void); // 声明LedService_Toggle 函数签名：翻转 LED 当前亮灭状态

#endif // 结束当前条件编译或头文件保护范围
