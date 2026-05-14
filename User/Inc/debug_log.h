#ifndef __DEBUG_LOG_H__                          /* 防止 debug_log.h 被重复包含 */
#define __DEBUG_LOG_H__                          /* 定义调试日志头文件保护宏 */

#include <stdarg.h>                              /* 引入可变参数接口 */
#include <stdint.h>                              /* 引入标准整数类型 */
#include <stdio.h>                               /* 引入 vsnprintf 声明 */
#include <string.h>                              /* 引入字符串处理接口 */
#include "FreeRTOS.h"                            /* 引入 FreeRTOS 基础定义 */
#include "bsp_debug_uart.h"                      /* 引入底层调试串口发送接口 */
#include "freertos_objects.h"                    /* 引入串口日志互斥锁声明 */
#include "semphr.h"                              /* 引入 FreeRTOS 信号量接口 */

void Debug_Print(const char *str);               /* 声明普通字符串日志输出函数 */
void Debug_Printf(const char *fmt, ...);         /* 声明格式化日志输出函数 */

#endif                                           /* 结束调试日志头文件保护宏 */
