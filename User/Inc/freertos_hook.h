#ifndef __FREERTOS_HOOK_H__                      /* 防止 freertos_hook.h 被重复包含 */
#define __FREERTOS_HOOK_H__                      /* 定义 FreeRTOS 钩子头文件保护宏 */

#include "FreeRTOS.h"                            /* 引入 FreeRTOS 基础定义 */
#include "task.h"                                /* 引入 FreeRTOS 任务接口 */

void vApplicationMallocFailedHook(void);         /* 声明内存分配失败钩子函数 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName); /* 声明任务栈溢出钩子函数 */

#endif                                           /* 结束 FreeRTOS 钩子头文件保护宏 */
