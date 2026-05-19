#include "delay.h" // 引入 delay.h 提供的接口、宏和类型定义

static __IO uint32_t TimingDelay = 0; // 执行 static __IO uint32_t TimingDelay = 0;，完成当前上下文中的具体处理

/**
 * @brief Delay_Init 函数。
 * @retval None
 */
void Delay_Init(void) // 定义Delay_Init 函数签名：Delay_Init 函数
{ // 进入当前代码块
    SysTick_Config(SystemCoreClock / 1000); // 调用SysTick_Config 函数，参数为 SystemCoreClock / 1000
} // 结束当前代码块

/**
 * @brief Delay_ms 函数。
 * @param ms ms 变量。
 * @retval None
 */
void Delay_ms(uint32_t ms) // 定义Delay_ms 函数签名：Delay_ms 函数
{ // 进入当前代码块
    TimingDelay = ms; // 把 ms 变量 写入 TimingDelay 变量
    while (TimingDelay != 0) // 当 TimingDelay != 0 成立时持续执行循环体
    { // 进入当前代码块

    } // 结束当前代码块
} // 结束当前代码块

/**
 * @brief TimingDelay_Decrement 函数。
 * @retval None
 */
void TimingDelay_Decrement(void) // 定义TimingDelay_Decrement 函数签名：TimingDelay_Decrement 函数
{ // 进入当前代码块
    if (TimingDelay != 0) // 判断 TimingDelay != 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        TimingDelay--; // 执行 TimingDelay--;，完成当前上下文中的具体处理
    } // 结束当前代码块
} // 结束当前代码块
