/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_conf.h  
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   Library configuration file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_CONF_H // 检查 __STM32F4xx_CONF_H 是否未定义，防止头文件重复包含
#define __STM32F4xx_CONF_H // 定义 __STM32F4xx_CONF_H 变量

/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
#include "stm32f4xx_adc.h" // 引入 stm32f4xx_adc.h 提供的接口、宏和类型定义
#include "stm32f4xx_crc.h" // 引入 stm32f4xx_crc.h 提供的接口、宏和类型定义
#include "stm32f4xx_dbgmcu.h" // 引入 stm32f4xx_dbgmcu.h 提供的接口、宏和类型定义
#include "stm32f4xx_dma.h" // 引入 stm32f4xx_dma.h 提供的接口、宏和类型定义
#include "stm32f4xx_exti.h" // 引入 stm32f4xx_exti.h 提供的接口、宏和类型定义
#include "stm32f4xx_flash.h" // 引入 stm32f4xx_flash.h 提供的接口、宏和类型定义
#include "stm32f4xx_gpio.h" // 引入 stm32f4xx_gpio.h 提供的接口、宏和类型定义
#include "stm32f4xx_i2c.h" // 引入 stm32f4xx_i2c.h 提供的接口、宏和类型定义
#include "stm32f4xx_iwdg.h" // 引入 stm32f4xx_iwdg.h 提供的接口、宏和类型定义
#include "stm32f4xx_pwr.h" // 引入 stm32f4xx_pwr.h 提供的接口、宏和类型定义
#include "stm32f4xx_rcc.h" // 引入 stm32f4xx_rcc.h 提供的接口、宏和类型定义
#include "stm32f4xx_rtc.h" // 引入 stm32f4xx_rtc.h 提供的接口、宏和类型定义
#include "stm32f4xx_sdio.h" // 引入 stm32f4xx_sdio.h 提供的接口、宏和类型定义
#include "stm32f4xx_spi.h" // 引入 stm32f4xx_spi.h 提供的接口、宏和类型定义
#include "stm32f4xx_syscfg.h" // 引入 stm32f4xx_syscfg.h 提供的接口、宏和类型定义
#include "stm32f4xx_tim.h" // 引入 stm32f4xx_tim.h 提供的接口、宏和类型定义
#include "stm32f4xx_usart.h" // 引入 stm32f4xx_usart.h 提供的接口、宏和类型定义
#include "stm32f4xx_wwdg.h" // 引入 stm32f4xx_wwdg.h 提供的接口、宏和类型定义
#include "misc.h" // 引入 misc.h 提供的接口、宏和类型定义

#if defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx) // 根据芯片型号或编译选项选择参与编译的代码
#include "stm32f4xx_cryp.h" // 引入 stm32f4xx_cryp.h 提供的接口、宏和类型定义
#include "stm32f4xx_hash.h" // 引入 stm32f4xx_hash.h 提供的接口、宏和类型定义
#include "stm32f4xx_rng.h" // 引入 stm32f4xx_rng.h 提供的接口、宏和类型定义
#include "stm32f4xx_can.h" // 引入 stm32f4xx_can.h 提供的接口、宏和类型定义
#include "stm32f4xx_dac.h" // 引入 stm32f4xx_dac.h 提供的接口、宏和类型定义
#include "stm32f4xx_dcmi.h" // 引入 stm32f4xx_dcmi.h 提供的接口、宏和类型定义
#include "stm32f4xx_dma2d.h" // 引入 stm32f4xx_dma2d.h 提供的接口、宏和类型定义
#include "stm32f4xx_fmc.h" // 引入 stm32f4xx_fmc.h 提供的接口、宏和类型定义
#include "stm32f4xx_ltdc.h" // 引入 stm32f4xx_ltdc.h 提供的接口、宏和类型定义
#include "stm32f4xx_sai.h" // 引入 stm32f4xx_sai.h 提供的接口、宏和类型定义
#endif // 结束当前条件编译或头文件保护范围

#if defined(STM32F427_437xx) // 根据芯片型号或编译选项选择参与编译的代码
#include "stm32f4xx_cryp.h" // 引入 stm32f4xx_cryp.h 提供的接口、宏和类型定义
#include "stm32f4xx_hash.h" // 引入 stm32f4xx_hash.h 提供的接口、宏和类型定义
#include "stm32f4xx_rng.h" // 引入 stm32f4xx_rng.h 提供的接口、宏和类型定义
#include "stm32f4xx_can.h" // 引入 stm32f4xx_can.h 提供的接口、宏和类型定义
#include "stm32f4xx_dac.h" // 引入 stm32f4xx_dac.h 提供的接口、宏和类型定义
#include "stm32f4xx_dcmi.h" // 引入 stm32f4xx_dcmi.h 提供的接口、宏和类型定义
#include "stm32f4xx_dma2d.h" // 引入 stm32f4xx_dma2d.h 提供的接口、宏和类型定义
#include "stm32f4xx_fmc.h" // 引入 stm32f4xx_fmc.h 提供的接口、宏和类型定义
#include "stm32f4xx_sai.h" // 引入 stm32f4xx_sai.h 提供的接口、宏和类型定义
#endif // 结束当前条件编译或头文件保护范围

#if defined(STM32F40_41xxx) // 根据芯片型号或编译选项选择参与编译的代码
#include "stm32f4xx_cryp.h" // 引入 stm32f4xx_cryp.h 提供的接口、宏和类型定义
#include "stm32f4xx_hash.h" // 引入 stm32f4xx_hash.h 提供的接口、宏和类型定义
#include "stm32f4xx_rng.h" // 引入 stm32f4xx_rng.h 提供的接口、宏和类型定义
#include "stm32f4xx_can.h" // 引入 stm32f4xx_can.h 提供的接口、宏和类型定义
#include "stm32f4xx_dac.h" // 引入 stm32f4xx_dac.h 提供的接口、宏和类型定义
#include "stm32f4xx_dcmi.h" // 引入 stm32f4xx_dcmi.h 提供的接口、宏和类型定义
#include "stm32f4xx_fsmc.h" // 引入 stm32f4xx_fsmc.h 提供的接口、宏和类型定义
#endif // 结束当前条件编译或头文件保护范围

#if defined(STM32F410xx) // 根据芯片型号或编译选项选择参与编译的代码
#include "stm32f4xx_rng.h" // 引入 stm32f4xx_rng.h 提供的接口、宏和类型定义
#include "stm32f4xx_dac.h" // 引入 stm32f4xx_dac.h 提供的接口、宏和类型定义
#endif // 结束当前条件编译或头文件保护范围

#if defined(STM32F411xE) // 根据芯片型号或编译选项选择参与编译的代码
#include "stm32f4xx_flash_ramfunc.h" // 引入 stm32f4xx_flash_ramfunc.h 提供的接口、宏和类型定义
#endif // 结束当前条件编译或头文件保护范围

#if defined(STM32F446xx) || defined(STM32F469_479xx) // 根据芯片型号或编译选项选择参与编译的代码
#include "stm32f4xx_qspi.h" // 引入 stm32f4xx_qspi.h 提供的接口、宏和类型定义
#endif // 结束当前条件编译或头文件保护范围

#if defined(STM32F410xx) || defined(STM32F446xx) // 根据芯片型号或编译选项选择参与编译的代码
#include "stm32f4xx_fmpi2c.h" // 引入 stm32f4xx_fmpi2c.h 提供的接口、宏和类型定义
#endif // 结束当前条件编译或头文件保护范围

#if defined(STM32F446xx) // 根据芯片型号或编译选项选择参与编译的代码
#include "stm32f4xx_spdifrx.h" // 引入 stm32f4xx_spdifrx.h 提供的接口、宏和类型定义
#include "stm32f4xx_cec.h" // 引入 stm32f4xx_cec.h 提供的接口、宏和类型定义
#endif // 结束当前条件编译或头文件保护范围

#if defined(STM32F469_479xx) // 根据芯片型号或编译选项选择参与编译的代码
#include "stm32f4xx_dsi.h" // 引入 stm32f4xx_dsi.h 提供的接口、宏和类型定义
#endif // 结束当前条件编译或头文件保护范围

#if defined(STM32F410xx) // 根据芯片型号或编译选项选择参与编译的代码
#include "stm32f4xx_lptim.h" // 引入 stm32f4xx_lptim.h 提供的接口、宏和类型定义
#endif // 结束当前条件编译或头文件保护范围

#if defined(STM32F412xG) // 根据芯片型号或编译选项选择参与编译的代码
#include "stm32f4xx_rng.h" // 引入 stm32f4xx_rng.h 提供的接口、宏和类型定义
#include "stm32f4xx_can.h" // 引入 stm32f4xx_can.h 提供的接口、宏和类型定义
#include "stm32f4xx_qspi.h" // 引入 stm32f4xx_qspi.h 提供的接口、宏和类型定义
#include "stm32f4xx_rng.h" // 引入 stm32f4xx_rng.h 提供的接口、宏和类型定义
#include "stm32f4xx_fsmc.h" // 引入 stm32f4xx_fsmc.h 提供的接口、宏和类型定义
#include "stm32f4xx_dfsdm.h" // 引入 stm32f4xx_dfsdm.h 提供的接口、宏和类型定义
#endif // 结束当前条件编译或头文件保护范围

#if defined(STM32F413_423xx) // 根据芯片型号或编译选项选择参与编译的代码
#include "stm32f4xx_cryp.h" // 引入 stm32f4xx_cryp.h 提供的接口、宏和类型定义
#include "stm32f4xx_fmpi2c.h" // 引入 stm32f4xx_fmpi2c.h 提供的接口、宏和类型定义
#include "stm32f4xx_rng.h" // 引入 stm32f4xx_rng.h 提供的接口、宏和类型定义
#include "stm32f4xx_can.h" // 引入 stm32f4xx_can.h 提供的接口、宏和类型定义
#include "stm32f4xx_qspi.h" // 引入 stm32f4xx_qspi.h 提供的接口、宏和类型定义
#include "stm32f4xx_rng.h" // 引入 stm32f4xx_rng.h 提供的接口、宏和类型定义
#include "stm32f4xx_fsmc.h" // 引入 stm32f4xx_fsmc.h 提供的接口、宏和类型定义
#include "stm32f4xx_dfsdm.h" // 引入 stm32f4xx_dfsdm.h 提供的接口、宏和类型定义
#endif // 结束当前条件编译或头文件保护范围

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* If an external clock source is used, then the value of the following define 
   should be set to the value of the external clock source, else, if no external // 执行 should be set to the value of the external clock source, else, if no external，完成当前上下文中的具体处理
   clock is used, keep this define commented */ // 执行 clock is used, keep this define commented */，完成当前上下文中的具体处理
/*#define I2S_EXTERNAL_CLOCK_VAL   12288000 */ /* Value of the external clock in Hz */


/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */ // 执行 Standard Peripheral Library drivers code */，完成当前上下文中的具体处理
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT // 根据芯片型号或编译选项选择参与编译的代码

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *   which reports the name of the source file and the source
  *   line number of the call that failed. 
  *   If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__)) // 定义 assert_param 变量
/* Exported functions ------------------------------------------------------- */
  /**
   * @brief assert_failed 函数。
   * @param file file 变量。
   * @param line line 变量。
   * @retval None
   */
  void assert_failed(uint8_t* file, uint32_t line); // 声明assert_failed 函数签名：assert_failed 函数
#else // 根据芯片型号或编译选项选择参与编译的代码
  #define assert_param(expr) ((void)0) // 定义 assert_param 变量
#endif // 结束当前条件编译或头文件保护范围

#endif // 结束当前条件编译或头文件保护范围

