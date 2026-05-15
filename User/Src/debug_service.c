#include "debug_service.h" // 引入调试服务接口

/**
 * @brief  初始化调试服务。
 * @param  None
 * @retval None
 */
void DebugService_Init(void) // 定义调试服务初始化函数
{                            // 进入代码块
    BSP_DebugUart_Init();    // 调用调试串口 BSP 初始化 USART6
} // 结束代码块
