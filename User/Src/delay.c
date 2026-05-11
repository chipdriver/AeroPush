#include "delay.h"

static __IO uint32_t TimingDelay = 0;   //用来保存当前剩余的延时时间；初始值：0，表示默认没有延时任务

/**
 * @brief   初始化SysTick，配置为1ms中断一次
 * @param   无
 * @return  无
 */
void Delay_Init(void)
{
    SysTick_Config(SystemCoreClock / 1000);
}

/**
 * @brief   毫秒延时函数
 * @param   ms :要延时的毫秒数
 * @return  无
 */
void Delay_ms(uint32_t ms)  //定义毫秒延时函数，参数ms表示延时多少ms
{
    TimingDelay = ms ;      //把要延时的毫秒数赋值给全局计算变量
    while(TimingDelay != 0) //只要TimingDelat 不等于0 =，就一直在这里等待
    {

    }
}

/**
 * @brief   延时计数递减函数
 * @note    需要在Systick 中断服务函数中调用
 * @param   无
 * @return  无
 */
void TimingDelay_Decrement(void)
{
    if(TimingDelay != 0)    //如果当前TimingDelat 不等于0，说明还在延时中
    {
        TimingDelay--;      //将TimingDelay减1，表示又过去了1ms
    }
}