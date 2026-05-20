#ifndef __DEBUG_LOG_H__ // 防止头文件重复包含
#define __DEBUG_LOG_H__

#include <stdarg.h> // 提供可变参数类型
#include <stdint.h> // 提供固定宽度整数类型
#include <stdio.h> // 提供 vsnprintf
#include <string.h> // 提供字符串处理接口
#include "FreeRTOS.h" // 提供 FreeRTOS 基础类型
#include "bsp_debug_uart.h" // 提供底层调试串口发送接口
#include "freertos_objects.h" // 提供串口日志互斥锁
#include "semphr.h" // 提供互斥锁接口

/**
 * @brief 通过调试串口输出字符串。
 * @param str 待输出字符串。
 * @retval None
 */
void Debug_Print(const char *str); // 输出调试字符串

/**
 * @brief 格式化并通过调试串口输出调试信息。
 * @param fmt printf 风格格式字符串。
 * @retval None
 */
void Debug_Printf(const char *fmt, ...); // 输出格式化调试日志

#endif // __DEBUG_LOG_H__
