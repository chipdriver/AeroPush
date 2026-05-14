#ifndef __APP_MAIN_H /* 防止 app_main.h 被重复包含 */
#define __APP_MAIN_H /* 定义应用主入口头文件保护宏 */

#include "freertos_objects.h" /* 引入 FreeRTOS 对象创建接口 */

void APP_Main(void); /* 声明应用层主入口函数 */

#endif /* 结束应用主入口头文件保护宏 */
