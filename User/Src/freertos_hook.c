#include "freertos_hook.h" // 引入 FreeRTOS 钩子函数声明

/**
 * @brief  内存分配失败钩子函数
 * @note   当 FreeRTOS 堆内存不足时会进入这里
 * @param  None
 * @return None
 */
void vApplicationMallocFailedHook(void) // 定义 FreeRTOS 内存分配失败钩子函数
{                                       // vApplicationMallocFailedHook 函数体开始
    taskDISABLE_INTERRUPTS();           // 关闭中断，防止系统继续运行造成更大错误
    while (1)                           // 进入死循环，方便调试定位内存不足问题
    {                                   // 死循环开始
    } // 死循环结束
} // vApplicationMallocFailedHook 函数体结束

/**
 * @brief  任务栈溢出钩子函数
 * @note   当某个任务发生栈溢出时会进入这里
 * @param  xTask: 发生栈溢出的任务句柄
 * @param  pcTaskName: 发生栈溢出的任务名
 * @return None
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) // 定义 FreeRTOS 任务栈溢出钩子函数
{                                                                        // vApplicationStackOverflowHook 函数体开始
    (void)xTask;                                                         // 防止编译器警告：参数未使用
    (void)pcTaskName;                                                    // 防止编译器警告：参数未使用

    taskDISABLE_INTERRUPTS(); // 关闭中断，防止系统继续运行造成更大错误
    while (1)                 // 进入死循环，方便调试定位栈溢出问题
    {                         // 死循环开始
    } // 死循环结束
} // vApplicationStackOverflowHook 函数体结束
