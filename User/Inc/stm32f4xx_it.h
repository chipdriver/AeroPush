#ifndef __STM32F4xx_IT_H // 防止头文件重复包含
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx.h" // 提供 Cortex-M 和 STM32F4 中断类型

/**
 * @brief 不可屏蔽中断处理函数。
 * @retval None
 */
void NMI_Handler(void); // NMI 异常入口

/**
 * @brief 硬件错误中断处理函数。
 * @retval None
 */
void HardFault_Handler(void); // HardFault 异常入口

/**
 * @brief 内存管理错误中断处理函数。
 * @retval None
 */
void MemManage_Handler(void); // MemManage 异常入口

/**
 * @brief 总线错误中断处理函数。
 * @retval None
 */
void BusFault_Handler(void); // BusFault 异常入口

/**
 * @brief 用法错误中断处理函数。
 * @retval None
 */
void UsageFault_Handler(void); // UsageFault 异常入口

/**
 * @brief 调试监视中断处理函数。
 * @retval None
 */
void DebugMon_Handler(void); // DebugMon 异常入口

/**
 * @brief USART1 全局中断处理函数。
 * @retval None
 */
void USART1_IRQHandler(void); // USART1 外设中断入口

#ifdef __cplusplus
}
#endif

#endif // __STM32F4xx_IT_H
