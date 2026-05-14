#ifndef __APP_STATUS_H__                     /* 防止 app_status.h 被重复包含 */
#define __APP_STATUS_H__                     /* 定义系统状态头文件保护宏 */

#include <stdint.h>                          /* 引入 uint8_t 等标准整数类型 */
#include "FreeRTOS.h"                        /* 引入 FreeRTOS 基础定义 */
#include "event_groups.h"                    /* 引入 FreeRTOS 事件组接口 */
#include "freertos_objects.h"                /* 引入系统事件组句柄声明 */

#define APP_STATUS_IMU_READY       (1 << 0)  /* 定义 IMU 已就绪状态位 */
#define APP_STATUS_GNSS_READY      (1 << 1)  /* 定义 GNSS 功能已就绪状态位 */
#define APP_STATUS_GNSS_FIX        (1 << 2)  /* 定义 GNSS 已定位状态位 */
#define APP_STATUS_NET_READY       (1 << 3)  /* 定义网络已就绪状态位 */
#define APP_STATUS_MQTT_READY      (1 << 4)  /* 定义 MQTT 已就绪状态位 */
#define APP_STATUS_IMU_ERROR       (1 << 8)  /* 定义 IMU 异常状态位 */
#define APP_STATUS_MODEM_ERROR     (1 << 9)  /* 定义 Modem 异常状态位 */
#define APP_STATUS_MQTT_ERROR      (1 << 10) /* 定义 MQTT 异常状态位 */

void AppStatus_Set(EventBits_t status_bits);      /* 声明设置系统状态位函数 */
void AppStatus_Clear(EventBits_t status_bits);    /* 声明清除系统状态位函数 */
uint8_t AppStatus_IsSet(EventBits_t status_bits); /* 声明检查系统状态位函数 */
EventBits_t AppStatus_GetAll(void);               /* 声明获取全部系统状态位函数 */

#endif                                            /* 结束系统状态头文件保护宏 */
