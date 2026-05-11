#include "freertos_hook.h"
/**
 * @brief  内存分配失败钩子函数
 * @note   当 FreeRTOS 堆内存不足时会进入这里
 * @param  无
 * @return 无
 */
void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();   // 关闭中断，防止继续运行
    while (1)
    {
    }
}

/**
 * @brief  任务栈溢出钩子函数
 * @note   当某个任务发生栈溢出时会进入这里
 * @param  xTask: 发生栈溢出的任务句柄
 * @param  pcTaskName: 发生栈溢出的任务名
 * @return 无
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;                // 防止编译器警告：参数未使用
    (void)pcTaskName;           // 防止编译器警告：参数未使用

    taskDISABLE_INTERRUPTS();   // 关闭中断
    while (1)
    {
    }
}