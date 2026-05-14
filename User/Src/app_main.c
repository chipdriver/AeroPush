/* header file inclusion */              // 头文件包含区域
#include "app_main.h"                    // 引入应用层主入口接口
#include "app_tasks.h"                   // 引入应用任务创建接口
#include "FreeRTOS.h"                    // 引入 FreeRTOS 基础定义
#include "task.h"                        // 引入任务调度器启动接口

/**
 * @brief  应用层主入口。
 * @note   main.c 完成芯片基础初始化后，调用本函数创建 RTOS 对象和任务。
 * @param  None
 * @retval None
 */
void APP_Main(void)                      // 定义应用层主入口函数
{                                        // APP_Main 函数体开始
    FreeRTOS_ObjectsCreate();            // 先创建队列、互斥锁、事件组等 FreeRTOS 对象

    APP_TasksCreate();                   // 创建所有 RTOS 任务

    vTaskStartScheduler();               // 启动 RTOS 任务调度器，让系统开始运行任务

    while(1);                            // 正常情况下不会运行到这里，若运行到这里说明调度器启动失败
}                                        // APP_Main 函数体结束
