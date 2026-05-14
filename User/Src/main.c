#include "app_main.h" // 引入应用层主入口接口
#include "main.h"     // 引入主函数相关头文件

/**
 * @brief  程序主入口。
 * @param  None
 * @retval int 当前嵌入式程序正常不返回。
 */
int main(void)    // 定义 C 程序入口函数
{                 // main 函数体开始
    SystemInit(); // 初始化系统时钟和底层 CMSIS 系统配置

    APP_Main(); // 进入应用层主入口

    while (1) // 防御性死循环，正常情况下不会运行到这里
    {         // 死循环开始

    } // 死循环结束
} // main 函数体结束
