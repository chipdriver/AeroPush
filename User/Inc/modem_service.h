#ifndef __MODEM_SERVICE_H__ // 防止头文件重复包含
#define __MODEM_SERVICE_H__

#include <string.h> // 提供 memset
#include "FreeRTOS.h" // 提供 FreeRTOS 基础类型
#include "app_types.h" // 提供 GNSS 和 MQTT 数据结构
#include "debug_log.h" // 提供调试日志接口
#include "task.h" // 提供 tick 计数接口

/**
 * @brief 构造模拟 GNSS 定位数据。
 * @param gnss 输出 GNSS 定位数据。
 * @retval None
 */
void ModemService_BuildSimGnss(GnssData_t *gnss); // 构造模拟 GNSS

/**
 * @brief 处理一条待发布 MQTT 消息。
 * @param msg 待发布 MQTT 消息。
 * @retval None
 */
void ModemService_Publish(const MqttPublishMsg_t *msg); // 处理 MQTT 发布

#endif // __MODEM_SERVICE_H__
