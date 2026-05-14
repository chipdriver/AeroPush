#ifndef __DEBUG_LOG_H__                         // 判断调试日志头文件是否未被包含
#define __DEBUG_LOG_H__                         // 定义头文件保护宏

/* header file inclusion */                     // 头文件包含区域
#include <stdarg.h>                             // 提供 va_list 等可变参数定义
#include <stdint.h>                             // 提供标准整数类型定义
#include <stdio.h>                              // 提供 vsnprintf 函数声明
#include <string.h>                             // 提供字符串处理函数声明
#include "FreeRTOS.h"                           // 提供 FreeRTOS 基础类型和宏
#include "semphr.h"                             // 提供信号量和互斥锁接口
#include "bsp_debug_uart.h"                     // 提供底层调试串口发送接口
#include "freertos_objects.h"                   // 提供串口日志互斥锁声明

/* function declaration */                      // 函数声明区域

/**
 * @brief  通过调试串口发送普通字符串日志。
 * @param  str 指向待发送字符串的指针。
 * @retval None
 */
void Debug_Print(const char *str);              // 声明普通字符串日志输出函数

/**
 * @brief  通过调试串口发送格式化日志。
 * @param  fmt 格式化字符串。
 * @param  ... 与格式化字符串匹配的可变参数。
 * @retval None
 */
void Debug_Printf(const char *fmt, ...);        // 声明格式化日志输出函数

#endif /* __DEBUG_LOG_H__ */                    // 结束头文件保护宏
