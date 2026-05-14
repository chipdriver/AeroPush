#ifndef __TELEMETRY_SERVICE_H__ /* 防止 telemetry_service.h 被重复包含 */
#define __TELEMETRY_SERVICE_H__ /* 定义遥测服务头文件保护宏 */

#include <stdio.h>     /* 引入 snprintf 等格式化接口 */
#include <string.h>    /* 引入 memset 等字符串处理接口 */
#include "app_types.h" /* 引入姿态、GNSS、MQTT 消息类型 */

void Telemetry_BuildMqttMsg(const AttitudeData_t *att, const GnssData_t *gnss, MqttPublishMsg_t *msg); /* 声明 MQTT 遥测消息组装函数 */

#endif /* 结束遥测服务头文件保护宏 */
