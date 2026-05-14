#ifndef __DELAY_H__ /* 防止 delay.h 被重复包含 */
#define __DELAY_H__ /* 定义延时模块头文件保护宏 */

#include "stm32f4xx.h" /* 引入 STM32F4 基础定义 */

void Delay_Init(void);            /* 声明 SysTick 延时初始化函数 */
void Delay_ms(uint32_t ms);       /* 声明毫秒级阻塞延时函数 */
void TimingDelay_Decrement(void); /* 声明延时计数递减函数 */

#endif /* 结束延时模块头文件保护宏 */
