#include "debug_service.h" // 引入 debug_service.h 提供的接口、宏和类型定义

/**
 * @brief DebugService_Init 函数。
 * @retval None
 */
void DebugService_Init(void) // 定义DebugService_Init 函数签名：DebugService_Init 函数
{ // 进入当前代码块
    BSP_DebugUart_Init(); // 调用初始化底层调试串口外设
} // 结束当前代码块
