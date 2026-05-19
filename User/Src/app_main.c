/* header file inclusion */ // 头文件包含区域
#include "app_main.h" // 引入 app_main.h 提供的接口、宏和类型定义
#include "app_tasks.h" // 引入 app_tasks.h 提供的接口、宏和类型定义
#include "FreeRTOS.h" // 引入 FreeRTOS.h 提供的接口、宏和类型定义
#include "task.h" // 引入 task.h 提供的接口、宏和类型定义

/**
 * @brief 应用层主入口，创建 RTOS 对象和任务并启动调度器。
 * @retval None
 */
void APP_Main(void) // 定义APP_Main 函数签名：应用层主入口，创建 RTOS 对象和任务并启动调度器
{ // 进入当前代码块
    FreeRTOS_ObjectsCreate(); // 调用创建系统使用的 FreeRTOS 队列、互斥量和事件组

    APP_TasksCreate(); // 调用创建应用层所有 FreeRTOS 任务

    vTaskStartScheduler(); // 启动 FreeRTOS 任务调度器

    while (1) // 当 1 成立时持续执行循环体
        ; // 执行 ;，完成当前上下文中的具体处理
} // 结束当前代码块
