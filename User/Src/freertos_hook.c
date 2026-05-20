#include "freertos_hook.h" // 提供 FreeRTOS 钩子函数声明
#include "debug_log.h" // 提供调试日志输出接口

/**
 * @brief FreeRTOS 内存申请失败钩子。
 * @retval None
 */
void vApplicationMallocFailedHook(void) // 处理堆内存申请失败
{
    taskDISABLE_INTERRUPTS(); // 关闭中断，停止系统继续运行
    while (1) // 停在现场等待调试
    {
    }
}

/**
 * @brief FreeRTOS 任务栈溢出钩子。
 * @param xTask 发生栈溢出的任务句柄。
 * @param pcTaskName 发生栈溢出的任务名。
 * @retval None
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) // 处理任务栈溢出
{
    (void)xTask; // 当前只打印任务名

    if (pcTaskName != 0) // 任务名有效
    {
        Debug_Printf("[FreeRTOS] stack overflow task=%s\r\n", pcTaskName); // 输出栈溢出任务名
    }

    taskDISABLE_INTERRUPTS(); // 关闭中断，停止系统继续运行
    while (1) // 停在现场等待调试
    {
    }
}
