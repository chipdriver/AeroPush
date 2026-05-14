#ifndef __APP_MAIN_H                         // 防止 app_main.h 被重复包含
#define __APP_MAIN_H                         // 定义应用主入口头文件保护宏

/* header file inclusion */                  // 头文件包含区域
#include "freertos_objects.h"                // 引入 FreeRTOS 对象创建接口

/* functon declaration */                    // 函数声明区域

/**
 * @brief  应用层主入口。
 * @param  None
 * @retval None
 */
void APP_Main(void);                         // 声明应用层主入口函数

#endif /* __APP_MAIN_H */                    // 结束应用主入口头文件保护宏
