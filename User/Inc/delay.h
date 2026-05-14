#ifndef __DELAY_H__                         // 防止 delay.h 被重复包含
#define __DELAY_H__                         // 定义延时模块头文件保护宏

/*	header				file		*/      // 头文件包含区域
#include "stm32f4xx.h"                      // 引入 STM32F4 基础定义和 uint32_t 类型

/*	function declaration	*/              // 函数声明区域

/**
 * @brief  初始化 SysTick 延时基准。
 * @param  None
 * @retval None
 */
void Delay_Init(void);	                    // 声明延时初始化函数，用于初始化 SysTick

/**
 * @brief  毫秒级阻塞延时。
 * @param  ms 要延时的毫秒数。
 * @retval None
 */
void Delay_ms(uint32_t ms);                 // 声明毫秒延时函数

/**
 * @brief  延时计数递减函数。
 * @param  None
 * @retval None
 */
void TimingDelay_Decrement(void);           // 声明延时计数递减函数

#endif /*__DELAY_H__#*/                     // 结束延时模块头文件保护宏
