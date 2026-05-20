#ifndef __FREERTOS_HOOK_H__ // 防止头文件重复包含
#define __FREERTOS_HOOK_H__

#include "FreeRTOS.h" // 提供 FreeRTOS 基础类型
#include "task.h" // 提供任务句柄类型

/**
 * @brief FreeRTOS 内存申请失败钩子。
 * @retval None
 */
void vApplicationMallocFailedHook(void); // 处理堆内存申请失败

/**
 * @brief FreeRTOS 任务栈溢出钩子。
 * @param xTask 发生栈溢出的任务句柄。
 * @param pcTaskName 发生栈溢出的任务名。
 * @retval None
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName); // 处理任务栈溢出

#endif // __FREERTOS_HOOK_H__
