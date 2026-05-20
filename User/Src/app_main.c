#include "app_main.h" // 提供应用层主入口声明
#include "app_tasks.h" // 提供应用任务创建接口
#include "FreeRTOS.h" // 提供 FreeRTOS 基础定义
#include "task.h" // 提供 FreeRTOS 任务调度接口

/**
 * @brief 应用层主入口，创建 RTOS 对象和任务并启动调度器。
 * @retval None
 */
void APP_Main(void) // 启动应用层
{
    APP_TasksCreate(); // 创建应用任务

    FreeRTOS_ObjectsCreate(); // 创建队列、互斥锁和事件组

    vTaskStartScheduler(); // 启动 FreeRTOS 调度器

    while (1) // 调度器异常返回时停留在这里
        ; // 等待调试
}
