/**
******************************************************************************
* @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c
* @author  MCD Application Team
* @version V1.8.1
* @date    27-January-2022
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and
*          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h" // 引入 stm32f4xx_it.h 提供的接口、宏和类型定义
#include "main.h" // 引入 main.h 提供的接口、宏和类型定义
#include "delay.h" // 引入 delay.h 提供的接口、宏和类型定义
/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief NMI_Handler 函数。
 * @retval None
 */
void NMI_Handler(void) // 定义NMI_Handler 函数签名：NMI_Handler 函数
{ // 进入当前代码块
} // 结束当前代码块

/**
 * @brief HardFault_Handler 函数。
 * @retval None
 */
void HardFault_Handler(void) // 定义HardFault_Handler 函数签名：HardFault_Handler 函数
{ // 进入当前代码块
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) // 当 1 成立时持续执行循环体
    { // 进入当前代码块
    } // 结束当前代码块
} // 结束当前代码块

/**
 * @brief MemManage_Handler 函数。
 * @retval None
 */
void MemManage_Handler(void) // 定义MemManage_Handler 函数签名：MemManage_Handler 函数
{ // 进入当前代码块
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1) // 当 1 成立时持续执行循环体
    { // 进入当前代码块
    } // 结束当前代码块
} // 结束当前代码块

/**
 * @brief BusFault_Handler 函数。
 * @retval None
 */
void BusFault_Handler(void) // 定义BusFault_Handler 函数签名：BusFault_Handler 函数
{ // 进入当前代码块
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1) // 当 1 成立时持续执行循环体
    { // 进入当前代码块
    } // 结束当前代码块
} // 结束当前代码块

/**
 * @brief UsageFault_Handler 函数。
 * @retval None
 */
void UsageFault_Handler(void) // 定义UsageFault_Handler 函数签名：UsageFault_Handler 函数
{ // 进入当前代码块
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1) // 当 1 成立时持续执行循环体
    { // 进入当前代码块
    } // 结束当前代码块
} // 结束当前代码块

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
/*void SVC_Handler(void)
{ // 进入当前代码块
}*/ // 执行 }*/，完成当前上下文中的具体处理

/**
 * @brief DebugMon_Handler 函数。
 * @retval None
 */
void DebugMon_Handler(void) // 定义DebugMon_Handler 函数签名：DebugMon_Handler 函数
{ // 进入当前代码块
} // 结束当前代码块

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
/*void PendSV_Handler(void)
{ // 进入当前代码块
} // 结束当前代码块
*/
/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
/*void SysTick_Handler(void)
{ // 进入当前代码块
    TimingDelay_Decrement(); // 调用TimingDelay_Decrement 函数
}*/ // 执行 }*/，完成当前上下文中的具体处理

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
/*void PPP_IRQHandler(void)
{ // 进入当前代码块
}*/ // 执行 }*/，完成当前上下文中的具体处理

/**
 * @}
 */
