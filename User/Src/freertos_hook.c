#include "freertos_hook.h" // 引入 freertos_hook.h 提供的接口、宏和类型定义
#include "debug_log.h" // 引入 debug_log.h 提供的接口、宏和类型定义

/**
 * @brief vApplicationMallocFailedHook 函数。
 * @retval None
 */
void vApplicationMallocFailedHook(void) // 定义vApplicationMallocFailedHook 函数签名：vApplicationMallocFailedHook 函数
{ // 进入当前代码块
    taskDISABLE_INTERRUPTS(); // 调用taskDISABLE_INTERRUPTS 函数
    while (1) // 当 1 成立时持续执行循环体
    { // 进入当前代码块
    } // 结束当前代码块
} // 结束当前代码块

/**
 * @brief vApplicationStackOverflowHook 函数。
 * @param xTask xTask 变量。
 * @param pcTaskName pcTaskName 变量。
 * @retval None
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) // 定义vApplicationStackOverflowHook 函数签名：vApplicationStackOverflowHook 函数
{ // 进入当前代码块
    (void)xTask; // 标记 xTask 变量 当前未使用，避免编译器告警
    if (pcTaskName != 0) // 判断 pcTaskName != 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        Debug_Printf("[FreeRTOS] stack overflow task=%s\r\n", pcTaskName); // 调用格式化并通过调试串口输出调试信息，参数为 "[FreeRTOS] stack overflow task=%s\r\n", pcTaskName
    } // 结束当前代码块

    taskDISABLE_INTERRUPTS(); // 调用taskDISABLE_INTERRUPTS 函数
    while (1) // 当 1 成立时持续执行循环体
    { // 进入当前代码块
    } // 结束当前代码块
} // 结束当前代码块
