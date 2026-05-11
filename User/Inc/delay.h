#ifndef __DELAY_H__
#define __DELAY_H__

/*	header				file		*/
#include "stm32f4xx.h"

/*	function declaration	*/
void Delay_Init(void);	            //生命延时初始化函数：用于初始化SysTick
void Delay_ms(uint32_t ms);         //毫秒延时函数
void TimingDelay_Decrement(void);   //延时计数递减函数

#endif /*__DELAY_H__#*/