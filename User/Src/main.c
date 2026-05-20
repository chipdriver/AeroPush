#include "app_main.h" // 提供应用层主入口
#include "main.h" // 提供工程主头文件

/**
 * @brief 程序入口。
 * @retval 不返回。
 */
int main(void) // 主函数入口
{
    SystemInit(); // 初始化系统时钟和底层启动配置

    APP_Main(); // 进入应用层并启动 FreeRTOS

    while (1) // APP_Main 正常不会返回
    {
    }
}
