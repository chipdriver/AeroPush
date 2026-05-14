#ifndef __FREERTOS_HOOK_H__                              // 防止 freertos_hook.h 被重复包含
#define __FREERTOS_HOOK_H__                              // 定义 FreeRTOS 钩子头文件保护宏

/* headler file */                                       // 头文件包含区域
#include "FreeRTOS.h"                                    // 引入 FreeRTOS 基础定义
#include "task.h"                                        // 引入 FreeRTOS 任务接口和 TaskHandle_t 类型
/* function */                                           // 函数声明区域

/**
 * @brief  FreeRTOS 内存分配失败钩子函数。
 * @param  None
 * @retval None
 */
void vApplicationMallocFailedHook(void);                 // 声明内存分配失败钩子函数

/**
 * @brief  FreeRTOS 任务栈溢出钩子函数。
 * @param  xTask 发生栈溢出的任务句柄。
 * @param  pcTaskName 发生栈溢出的任务名。
 * @retval None
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName); // 声明任务栈溢出钩子函数

#endif  /*__FREERTOS_HOOK_H__*/                         // 结束 FreeRTOS 钩子头文件保护宏
