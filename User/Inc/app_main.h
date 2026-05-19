#ifndef __APP_MAIN_H // 检查 __APP_MAIN_H 是否未定义，防止头文件重复包含
#define __APP_MAIN_H // 定义 __APP_MAIN_H 变量

#include "freertos_objects.h" // 引入 freertos_objects.h 提供的接口、宏和类型定义

/**
 * @brief 应用层主入口，创建 RTOS 对象和任务并启动调度器。
 * @retval None
 */
void APP_Main(void); // 声明APP_Main 函数签名：应用层主入口，创建 RTOS 对象和任务并启动调度器

#endif // 结束当前条件编译或头文件保护范围
