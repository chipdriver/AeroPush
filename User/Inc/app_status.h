#ifndef __APP_STATUS_H__
#define __APP_STATUS_H__

/* header file inclusion */
#include "FreeRTOS.h"   // 事件组的依赖文件
#include "event_groups.h" // 事件组
#include "freertos_objects.h"                      // 引入 gSystemEventGroup 系统事件组句柄

/* macro definitions */
#define APP_STATUS_IMU_READY       (1 << 0)        // IMU 初始化完成 / 姿态服务可用
#define APP_STATUS_GNSS_READY      (1 << 1)        // GNSS 功能已打开
#define APP_STATUS_GNSS_FIX        (1 << 2)        // GNSS 已定位
#define APP_STATUS_NET_READY       (1 << 3)        // 4G 网络已就绪
#define APP_STATUS_MQTT_READY      (1 << 4)        // MQTT 已连接

#define APP_STATUS_IMU_ERROR       (1 << 8)        // IMU 异常状态
#define APP_STATUS_MODEM_ERROR     (1 << 9)        // A7670E / Modem 异常状态
#define APP_STATUS_MQTT_ERROR      (1 << 10)       // MQTT 异常状态

/* function declarations */
void AppStatus_Set(EventBits_t status_bits);       // 声明设置状态位函数
void AppStatus_Clear(EventBits_t status_bits);     // 声明清除状态位函数
uint8_t AppStatus_IsSet(EventBits_t status_bits);  // 声明判断状态位是否已设置函数
EventBits_t AppStatus_GetAll(void);                // 声明获取全部状态位函数

#endif /* __APP_STATUS_H__ */