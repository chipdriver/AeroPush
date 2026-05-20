#ifndef __TELEMETRY_SERVICE_H__ // 防止头文件重复包含
#define __TELEMETRY_SERVICE_H__

#include <stdio.h> // 提供 snprintf
#include <string.h> // 提供 memset 和 strlen
#include "app_types.h" // 提供姿态、GNSS 和 MQTT 数据结构

/**
 * @brief 根据姿态和 GNSS 数据构造 MQTT 遥测消息。
 * @param att 姿态数据。
 * @param gnss GNSS 定位数据。
 * @param msg 输出 MQTT 发布消息。
 * @retval None
 */
void Telemetry_BuildMqttMsg(const AttitudeData_t *att, const GnssData_t *gnss, MqttPublishMsg_t *msg); // 构造遥测 MQTT 消息

#endif // __TELEMETRY_SERVICE_H__
