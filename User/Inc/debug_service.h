#ifndef __DEBUG_SERVICE_H__ // 防止头文件重复包含
#define __DEBUG_SERVICE_H__

#include "bsp_debug_uart.h" // 提供底层调试串口接口

/**
 * @brief 初始化调试服务。
 * @retval None
 */
void DebugService_Init(void); // 初始化调试串口服务

#endif // __DEBUG_SERVICE_H__
