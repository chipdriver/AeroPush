/**
 ******************************************************************************
 * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c
 * @author  MCD Application Team
 * @version V1.8.1
 * @date    27-January-2022
 * @brief   Cortex-M4 异常中断处理函数模板。
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "stm32f4xx_it.h" // 提供中断处理函数声明
#include "main.h" // 提供工程主头文件
#include "delay.h" // 提供 SysTick 延时计数接口
#include "bsp_a7670e_uart.h" // 提供 A7670E USART1 中断处理接口

/**
 * @brief 不可屏蔽中断处理函数。
 * @retval None
 */
void NMI_Handler(void) // 处理 NMI 异常
{
}

/**
 * @brief 硬件错误中断处理函数。
 * @retval None
 */
void HardFault_Handler(void) // 处理硬件错误异常
{
    while (1) // 停在错误现场等待调试
    {
    }
}

/**
 * @brief 内存管理错误中断处理函数。
 * @retval None
 */
void MemManage_Handler(void) // 处理内存管理异常
{
    while (1) // 停在错误现场等待调试
    {
    }
}

/**
 * @brief 总线错误中断处理函数。
 * @retval None
 */
void BusFault_Handler(void) // 处理总线错误异常
{
    while (1) // 停在错误现场等待调试
    {
    }
}

/**
 * @brief 用法错误中断处理函数。
 * @retval None
 */
void UsageFault_Handler(void) // 处理用法错误异常
{
    while (1) // 停在错误现场等待调试
    {
    }
}

/**
 * @brief 调试监视中断处理函数。
 * @retval None
 */
void DebugMon_Handler(void) // 处理调试监视异常
{
}

/*
 * SVC、PendSV 和 SysTick 当前由 FreeRTOS 接管。
 * 如需启用裸机 SysTick 延时，需要在 FreeRTOS 配置之外重新确认中断归属。
 */

/**
 * @brief USART1 全局中断处理函数。
 * @retval None
 */
void USART1_IRQHandler(void) // 处理 A7670E 使用的 USART1 中断
{
    BSP_A7670E_Uart_IRQHandler(); // 转交 BSP 层处理 USART1 接收数据
}
