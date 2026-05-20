#ifndef __DELAY_H__ // 防止头文件重复包含
#define __DELAY_H__

#include "stm32f4xx.h" // 提供 SysTick 和系统时钟接口

/**
 * @brief 初始化 SysTick 为 1ms 节拍。
 * @retval None
 */
void Delay_Init(void); // 初始化延时模块

/**
 * @brief 阻塞延时指定毫秒数。
 * @param ms 延时毫秒数。
 * @retval None
 */
void Delay_ms(uint32_t ms); // 毫秒延时

/**
 * @brief 在 SysTick 中断中递减延时计数。
 * @retval None
 */
void TimingDelay_Decrement(void); // 递减延时计数

#endif // __DELAY_H__
