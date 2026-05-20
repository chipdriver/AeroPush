#include "debug_service.h" // 提供调试服务接口

/**
 * @brief 初始化调试服务。
 * @retval None
 */
void DebugService_Init(void) // 初始化调试串口服务
{
    BSP_DebugUart_Init(); // 初始化底层调试串口
}
