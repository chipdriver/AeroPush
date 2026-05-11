/* header file inclusion */
#include "app_main.h"
#include "app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief 应用层主入口
 * @note main.c完成芯片基础初始化后，调用本函数启动 RTOS 应用
 * @param 无
 * @param 无
 */
void APP_Main(void)
{
    FreeRTOS_ObjectsCreate();       //先创建队列,互斥锁等 FreeRTOS 对象

    APP_TasksCreate();              //创建所有RTOS任务

    vTaskStartScheduler();          //启动RTOS任务调度器，让系统开始运行任务。

    while(1);                       //正常情况下不会运行到这里
}
