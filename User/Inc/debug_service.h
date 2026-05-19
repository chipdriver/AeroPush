#ifndef __DEBUG_SERVICE_H__ // 检查 __DEBUG_SERVICE_H__ 是否未定义，防止头文件重复包含
#define __DEBUG_SERVICE_H__ // 定义 __DEBUG_SERVICE_H__ 变量

#include "bsp_debug_uart.h" // 引入 bsp_debug_uart.h 提供的接口、宏和类型定义

/**
 * @brief DebugService_Init 函数。
 * @retval None
 */
void DebugService_Init(void); // 声明DebugService_Init 函数签名：DebugService_Init 函数

#endif // 结束当前条件编译或头文件保护范围
