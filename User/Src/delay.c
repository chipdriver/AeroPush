#include "delay.h"                              // 引入延时模块接口

static __IO uint32_t TimingDelay = 0;           // 用来保存当前剩余的延时时间，初始值 0 表示默认没有延时任务

/**
 * @brief   初始化 SysTick，配置为 1ms 中断一次。
 * @param   None
 * @return  None
 */
void Delay_Init(void)                           // 定义 SysTick 延时初始化函数
{                                               // Delay_Init 函数体开始
    SysTick_Config(SystemCoreClock / 1000);     // 配置 SysTick 每 1ms 产生一次中断
}                                               // Delay_Init 函数体结束

/**
 * @brief   毫秒级阻塞延时函数。
 * @param   ms 要延时的毫秒数。
 * @return  None
 */
void Delay_ms(uint32_t ms)                      // 定义毫秒延时函数，参数 ms 表示延时多少 ms
{                                               // Delay_ms 函数体开始
    TimingDelay = ms ;                          // 把要延时的毫秒数赋值给全局计数变量
    while(TimingDelay != 0)                     // 只要 TimingDelay 不等于 0，就一直在这里等待
    {                                           // 阻塞等待循环开始

    }                                           // 阻塞等待循环结束
}                                               // Delay_ms 函数体结束

/**
 * @brief   延时计数递减函数。
 * @note    需要在 SysTick 中断服务函数中调用。
 * @param   None
 * @return  None
 */
void TimingDelay_Decrement(void)                // 定义延时计数递减函数
{                                               // TimingDelay_Decrement 函数体开始
    if(TimingDelay != 0)                        // 如果当前 TimingDelay 不等于 0，说明还在延时中
    {                                           // 延时计数递减分支开始
        TimingDelay--;                          // 将 TimingDelay 减 1，表示又过去了 1ms
    }                                           // 延时计数递减分支结束
}                                               // TimingDelay_Decrement 函数体结束
