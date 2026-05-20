#ifndef __APP_MAIN_H // 防止头文件重复包含
#define __APP_MAIN_H

#include "freertos_objects.h" // 提供系统 FreeRTOS 对象创建接口

/**
 * @brief 应用层主入口，创建 RTOS 对象和任务并启动调度器。
 * @retval None
 */
void APP_Main(void); // 启动应用层

#endif // __APP_MAIN_H
