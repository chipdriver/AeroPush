#ifndef __DEBUG_LOG_H__ // 检查 __DEBUG_LOG_H__ 是否未定义，防止头文件重复包含
#define __DEBUG_LOG_H__ // 定义 __DEBUG_LOG_H__ 变量

#include <stdarg.h> // 引入 stdarg.h 提供的接口、宏和类型定义
#include <stdint.h> // 引入 stdint.h 提供的接口、宏和类型定义
#include <stdio.h> // 引入 stdio.h 提供的接口、宏和类型定义
#include <string.h> // 引入 string.h 提供的接口、宏和类型定义
#include "FreeRTOS.h" // 引入 FreeRTOS.h 提供的接口、宏和类型定义
#include "bsp_debug_uart.h" // 引入 bsp_debug_uart.h 提供的接口、宏和类型定义
#include "freertos_objects.h" // 引入 freertos_objects.h 提供的接口、宏和类型定义
#include "semphr.h" // 引入 semphr.h 提供的接口、宏和类型定义

/**
 * @brief 通过调试串口输出字符串。
 * @param str str 变量。
 * @retval None
 */
void Debug_Print(const char *str); // 声明Debug_Print 函数签名：通过调试串口输出字符串
/**
 * @brief 格式化并通过调试串口输出调试信息。
 * @param fmt fmt 变量。
 * @retval None
 */
void Debug_Printf(const char *fmt, ...); // 声明Debug_Printf 函数签名：格式化并通过调试串口输出调试信息

#endif // 结束当前条件编译或头文件保护范围
