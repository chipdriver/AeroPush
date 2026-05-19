#ifndef __FREERTOS_HOOK_H__ // 检查 __FREERTOS_HOOK_H__ 是否未定义，防止头文件重复包含
#define __FREERTOS_HOOK_H__ // 定义 __FREERTOS_HOOK_H__ 变量

#include "FreeRTOS.h" // 引入 FreeRTOS.h 提供的接口、宏和类型定义
#include "task.h" // 引入 task.h 提供的接口、宏和类型定义

/**
 * @brief vApplicationMallocFailedHook 函数。
 * @retval None
 */
void vApplicationMallocFailedHook(void); // 声明vApplicationMallocFailedHook 函数签名：vApplicationMallocFailedHook 函数
/**
 * @brief vApplicationStackOverflowHook 函数。
 * @param xTask xTask 变量。
 * @param pcTaskName pcTaskName 变量。
 * @retval None
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName); // 声明vApplicationStackOverflowHook 函数签名：vApplicationStackOverflowHook 函数

#endif // 结束当前条件编译或头文件保护范围
