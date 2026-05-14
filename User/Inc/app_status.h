#ifndef __APP_STATUS_H__                                  // 防止 app_status.h 被重复包含
#define __APP_STATUS_H__                                  // 定义系统状态头文件保护宏

/* header file inclusion */                                // 头文件包含区域
#include "FreeRTOS.h"                                      // 引入 FreeRTOS 基础定义
#include "event_groups.h"                                  // 引入 FreeRTOS 事件组接口
#include "freertos_objects.h"                              // 引入 gSystemEventGroup 系统事件组句柄

/* macro definitions */                                   // 系统状态位宏定义区域
#define APP_STATUS_IMU_READY       (1 << 0)               // IMU 初始化完成 / 姿态服务可用
#define APP_STATUS_GNSS_READY      (1 << 1)               // GNSS 功能已打开
#define APP_STATUS_GNSS_FIX        (1 << 2)               // GNSS 已定位
#define APP_STATUS_NET_READY       (1 << 3)               // 4G 网络已就绪
#define APP_STATUS_MQTT_READY      (1 << 4)               // MQTT 已连接

#define APP_STATUS_IMU_ERROR       (1 << 8)               // IMU 异常状态
#define APP_STATUS_MODEM_ERROR     (1 << 9)               // A7670E / Modem 异常状态
#define APP_STATUS_MQTT_ERROR      (1 << 10)              // MQTT 异常状态

/* function declarations */                                // 函数声明区域

/**
 * @brief  设置系统状态位。
 * @param  status_bits 要设置的状态位，可以是单个位或多个位的组合。
 * @retval None
 */
void AppStatus_Set(EventBits_t status_bits);              // 声明设置状态位函数

/**
 * @brief  清除系统状态位。
 * @param  status_bits 要清除的状态位，可以是单个位或多个位的组合。
 * @retval None
 */
void AppStatus_Clear(EventBits_t status_bits);            // 声明清除状态位函数

/**
 * @brief  检查系统状态位是否已经设置。
 * @param  status_bits 要检查的状态位，可以是单个位或多个位的组合。
 * @retval 1 表示已设置，0 表示未设置。
 */
uint8_t AppStatus_IsSet(EventBits_t status_bits);         // 声明判断状态位是否已设置函数

/**
 * @brief  获取当前全部系统状态位。
 * @param  None
 * @retval 当前所有状态位的值。
 */
EventBits_t AppStatus_GetAll(void);                       // 声明获取全部状态位函数

#endif /* __APP_STATUS_H__ */                             // 结束系统状态头文件保护宏
