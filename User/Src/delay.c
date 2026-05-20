#include "delay.h" // 提供 SysTick 延时接口

static __IO uint32_t TimingDelay = 0; // 毫秒延时倒计数

/**
 * @brief 初始化 SysTick 为 1ms 节拍。
 * @retval None
 */
void Delay_Init(void) // 初始化阻塞延时模块
{
    SysTick_Config(SystemCoreClock / 1000); // 配置 SysTick 每 1ms 触发一次
}

/**
 * @brief 阻塞延时指定毫秒数。
 * @param ms 延时毫秒数。
 * @retval None
 */
void Delay_ms(uint32_t ms) // 毫秒阻塞延时
{
    TimingDelay = ms; // 设置延时倒计数
    while (TimingDelay != 0) // 等待 SysTick 中断递减到 0
    {
    }
}

/**
 * @brief 在 SysTick 中断中递减延时计数。
 * @retval None
 */
void TimingDelay_Decrement(void) // 递减延时计数
{
    if (TimingDelay != 0) // 当前仍处于延时中
    {
        TimingDelay--; // 递减 1ms
    }
}
