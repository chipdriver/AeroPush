#include "app_main.h" // 引入 app_main.h 提供的接口、宏和类型定义
#include "main.h" // 引入 main.h 提供的接口、宏和类型定义

/**
 * @brief main 函数。
 * @retval 函数执行结果或计算得到的返回值。
 */
int main(void) // 定义main 函数签名：main 函数
{ // 进入当前代码块
    SystemInit(); // 调用SystemInit 函数

    APP_Main(); // 调用应用层主入口，创建 RTOS 对象和任务并启动调度器

    while (1) // 当 1 成立时持续执行循环体
    { // 进入当前代码块

    } // 结束当前代码块
} // 结束当前代码块
