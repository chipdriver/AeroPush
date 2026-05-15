#ifndef __DEBUG_SERVICE_H__ /* 防止 debug_service.h 被重复包含 */
#define __DEBUG_SERVICE_H__ /* 定义调试服务头文件保护宏 */

#include "bsp_debug_uart.h" /* 引入调试串口 BSP 底层接口 */

void DebugService_Init(void); /* 声明调试服务初始化函数 */

#endif /* 结束调试服务头文件保护宏 */
