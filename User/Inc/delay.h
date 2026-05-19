#ifndef __DELAY_H__ // 检查 __DELAY_H__ 是否未定义，防止头文件重复包含
#define __DELAY_H__ // 定义 __DELAY_H__ 变量

#include "stm32f4xx.h" // 引入 stm32f4xx.h 提供的接口、宏和类型定义

/**
 * @brief Delay_Init 函数。
 * @retval None
 */
void Delay_Init(void); // 声明Delay_Init 函数签名：Delay_Init 函数
/**
 * @brief Delay_ms 函数。
 * @param ms ms 变量。
 * @retval None
 */
void Delay_ms(uint32_t ms); // 声明Delay_ms 函数签名：Delay_ms 函数
/**
 * @brief TimingDelay_Decrement 函数。
 * @retval None
 */
void TimingDelay_Decrement(void); // 声明TimingDelay_Decrement 函数签名：TimingDelay_Decrement 函数

#endif // 结束当前条件编译或头文件保护范围
