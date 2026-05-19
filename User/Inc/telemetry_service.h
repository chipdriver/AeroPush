#ifndef __TELEMETRY_SERVICE_H__ // 检查 __TELEMETRY_SERVICE_H__ 是否未定义，防止头文件重复包含
#define __TELEMETRY_SERVICE_H__ // 定义 __TELEMETRY_SERVICE_H__ 变量

#include <stdio.h> // 引入 stdio.h 提供的接口、宏和类型定义
#include <string.h> // 引入 string.h 提供的接口、宏和类型定义
#include "app_types.h" // 引入 app_types.h 提供的接口、宏和类型定义

/**
 * @brief 根据姿态和 GNSS 数据构造 MQTT 遥测消息。
 * @param att att 变量。
 * @param gnss GNSS 定位数据结构体。
 * @param msg msg 变量。
 * @retval None
 */
void Telemetry_BuildMqttMsg(const AttitudeData_t *att, const GnssData_t *gnss, MqttPublishMsg_t *msg); // 声明Telemetry_BuildMqttMsg 函数签名：根据姿态和 GNSS 数据构造 MQTT 遥测消息

#endif // 结束当前条件编译或头文件保护范围
