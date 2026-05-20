/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_conf.h
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   STM32F4 标准外设库配置头文件。
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

#ifndef __STM32F4xx_CONF_H // 防止头文件重复包含
#define __STM32F4xx_CONF_H

/* 当前工程启用的标准外设库头文件。 */
#include "stm32f4xx_adc.h" // ADC 外设库
#include "stm32f4xx_crc.h" // CRC 外设库
#include "stm32f4xx_dbgmcu.h" // 调试 MCU 外设库
#include "stm32f4xx_dma.h" // DMA 外设库
#include "stm32f4xx_exti.h" // 外部中断外设库
#include "stm32f4xx_flash.h" // Flash 外设库
#include "stm32f4xx_gpio.h" // GPIO 外设库
#include "stm32f4xx_i2c.h" // I2C 外设库
#include "stm32f4xx_iwdg.h" // 独立看门狗外设库
#include "stm32f4xx_pwr.h" // 电源控制外设库
#include "stm32f4xx_rcc.h" // 时钟控制外设库
#include "stm32f4xx_rtc.h" // RTC 外设库
#include "stm32f4xx_sdio.h" // SDIO 外设库
#include "stm32f4xx_spi.h" // SPI 外设库
#include "stm32f4xx_syscfg.h" // 系统配置外设库
#include "stm32f4xx_tim.h" // 定时器外设库
#include "stm32f4xx_usart.h" // USART 外设库
#include "stm32f4xx_wwdg.h" // 窗口看门狗外设库
#include "misc.h" // NVIC 和系统辅助接口

#if defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx) // 对应芯片系列启用扩展外设
#include "stm32f4xx_cryp.h" // 加密外设库
#include "stm32f4xx_hash.h" // HASH 外设库
#include "stm32f4xx_rng.h" // 随机数外设库
#include "stm32f4xx_can.h" // CAN 外设库
#include "stm32f4xx_dac.h" // DAC 外设库
#include "stm32f4xx_dcmi.h" // DCMI 外设库
#include "stm32f4xx_dma2d.h" // DMA2D 外设库
#include "stm32f4xx_fmc.h" // FMC 外设库
#include "stm32f4xx_ltdc.h" // LTDC 外设库
#include "stm32f4xx_sai.h" // SAI 外设库
#endif

#if defined(STM32F427_437xx) // 对应芯片系列启用扩展外设
#include "stm32f4xx_cryp.h" // 加密外设库
#include "stm32f4xx_hash.h" // HASH 外设库
#include "stm32f4xx_rng.h" // 随机数外设库
#include "stm32f4xx_can.h" // CAN 外设库
#include "stm32f4xx_dac.h" // DAC 外设库
#include "stm32f4xx_dcmi.h" // DCMI 外设库
#include "stm32f4xx_dma2d.h" // DMA2D 外设库
#include "stm32f4xx_fmc.h" // FMC 外设库
#include "stm32f4xx_sai.h" // SAI 外设库
#endif

#if defined(STM32F40_41xxx) // 对应芯片系列启用扩展外设
#include "stm32f4xx_cryp.h" // 加密外设库
#include "stm32f4xx_hash.h" // HASH 外设库
#include "stm32f4xx_rng.h" // 随机数外设库
#include "stm32f4xx_can.h" // CAN 外设库
#include "stm32f4xx_dac.h" // DAC 外设库
#include "stm32f4xx_dcmi.h" // DCMI 外设库
#include "stm32f4xx_fsmc.h" // FSMC 外设库
#endif

#if defined(STM32F410xx) // STM32F410 扩展外设
#include "stm32f4xx_rng.h" // 随机数外设库
#include "stm32f4xx_dac.h" // DAC 外设库
#endif

#if defined(STM32F411xE) // STM32F411 Flash RAM 函数支持
#include "stm32f4xx_flash_ramfunc.h" // Flash RAM 函数接口
#endif

#if defined(STM32F446xx) || defined(STM32F469_479xx) // QSPI 外设支持
#include "stm32f4xx_qspi.h" // QSPI 外设库
#endif

#if defined(STM32F410xx) || defined(STM32F446xx) // FMPI2C 外设支持
#include "stm32f4xx_fmpi2c.h" // FMPI2C 外设库
#endif

#if defined(STM32F446xx) // STM32F446 音频相关外设支持
#include "stm32f4xx_spdifrx.h" // SPDIFRX 外设库
#include "stm32f4xx_cec.h" // CEC 外设库
#endif

#if defined(STM32F469_479xx) // STM32F469/479 DSI 支持
#include "stm32f4xx_dsi.h" // DSI 外设库
#endif

#if defined(STM32F410xx) // STM32F410 低功耗定时器支持
#include "stm32f4xx_lptim.h" // LPTIM 外设库
#endif

#if defined(STM32F412xG) // STM32F412 扩展外设支持
#include "stm32f4xx_rng.h" // 随机数外设库
#include "stm32f4xx_can.h" // CAN 外设库
#include "stm32f4xx_qspi.h" // QSPI 外设库
#include "stm32f4xx_rng.h" // 随机数外设库
#include "stm32f4xx_fsmc.h" // FSMC 外设库
#include "stm32f4xx_dfsdm.h" // DFSDM 外设库
#endif

#if defined(STM32F413_423xx) // STM32F413/423 扩展外设支持
#include "stm32f4xx_cryp.h" // 加密外设库
#include "stm32f4xx_fmpi2c.h" // FMPI2C 外设库
#include "stm32f4xx_rng.h" // 随机数外设库
#include "stm32f4xx_can.h" // CAN 外设库
#include "stm32f4xx_qspi.h" // QSPI 外设库
#include "stm32f4xx_rng.h" // 随机数外设库
#include "stm32f4xx_fsmc.h" // FSMC 外设库
#include "stm32f4xx_dfsdm.h" // DFSDM 外设库
#endif

/* 如需 I2S 外部时钟，在这里定义外部时钟频率。 */
/*#define I2S_EXTERNAL_CLOCK_VAL   12288000 */ /* Value of the external clock in Hz */

/* 如需启用标准外设库参数检查，打开下面的 USE_FULL_ASSERT。 */
/* #define USE_FULL_ASSERT    1 */

#ifdef USE_FULL_ASSERT // 启用参数检查时展开 assert_param
/**
  * @brief  检查标准外设库函数参数。
  * @param  expr 待检查表达式。
  * @retval None
  */
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__)) // 参数错误时调用 assert_failed

/**
 * @brief 参数检查失败回调。
 * @param file 触发断言的文件名。
 * @param line 触发断言的行号。
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line); // 参数断言失败处理
#else
#define assert_param(expr) ((void)0) // 未启用参数检查时忽略表达式
#endif

#endif // __STM32F4xx_CONF_H
