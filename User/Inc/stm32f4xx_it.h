#ifndef __STM32F4xx_IT_H /* 防止 stm32f4xx_it.h 被重复包含 */
#define __STM32F4xx_IT_H /* 定义中断处理头文件保护宏 */

#ifdef __cplusplus /* 判断是否为 C++ 编译环境 */
extern "C"
{      /* 使用 C 链接方式导出接口 */
#endif /* 结束 C++ 编译环境判断 */

#include "stm32f4xx.h" /* 引入 STM32F4 基础定义 */

    void NMI_Handler(void);        /* 声明 NMI 异常中断处理函数 */
    void HardFault_Handler(void);  /* 声明 HardFault 异常中断处理函数 */
    void MemManage_Handler(void);  /* 声明 MemManage 异常中断处理函数 */
    void BusFault_Handler(void);   /* 声明 BusFault 异常中断处理函数 */
    void UsageFault_Handler(void); /* 声明 UsageFault 异常中断处理函数 */
    void DebugMon_Handler(void);   /* 声明 DebugMon 异常中断处理函数 */

#ifdef __cplusplus /* 判断是否为 C++ 编译环境 */
} /* 结束 C 链接导出区域 */
#endif /* 结束 C++ 编译环境判断 */

#endif /* 结束中断处理头文件保护宏 */
