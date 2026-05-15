#ifndef __LED_SERVICE_H__ /* 防止 led_service.h 被重复包含 */
#define __LED_SERVICE_H__ /* 定义 LED 服务头文件保护宏 */

#include "bsp_led.h" /* 引入 LED BSP 底层接口 */

void LedService_Init(void);   /* 声明 LED 服务初始化函数 */
void LedService_On(void);     /* 声明 LED 服务打开函数 */
void LedService_Off(void);    /* 声明 LED 服务关闭函数 */
void LedService_Toggle(void); /* 声明 LED 服务翻转函数 */

#endif /* 结束 LED 服务头文件保护宏 */
