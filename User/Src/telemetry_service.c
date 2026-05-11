#include "telemetry_service.h"     // 引入遥测服务模块接口声明
#include "app_config.h"            // 引入全局配置，例如 MQTT topic


void Telemetry_BuildMqttMsg(const AttitudeData_t *att,const GnssData_t *gnss,MqttPublishMsg_t *msg) // 将姿态数据和 GNSS 数据打包成 MQTT 消息
{
    if( att == 0 || gnss == 0 || msg == 0)       // 检查输入指针是否为空，避免访问非法地址
        return;                                  // 任意输入为空时直接返回，不继续组包

        memset(msg,0,sizeof(MqttPublishMsg_t));  // 清空 MQTT 消息结构体，避免残留旧数据

        strcpy(msg->topic,APP_MQTT_TOPIC);       // 使用全局配置中的 MQTT topic

        snprintf(msg->payload,                   // 将遥测数据格式化为 JSON 字符串并写入 payload
                MQTT_PAYLOAD_MAX_LEN,            // 限制写入长度，避免 payload 缓冲区溢出
            "{\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"lat\":%.6f,\"lon\":%.6f}", // JSON 字符串格式模板
                att->roll_deg,                   // 写入 roll 角度，保留 1 位小数
                att->pitch_deg,                  // 写入 pitch 角度，保留 1 位小数
                att->yaw_deg,                    // 写入 yaw 角度，保留 1 位小数
                gnss->latitude,                  // 写入纬度，保留 6 位小数
                gnss->longitude);                // 写入经度，保留 6 位小数

        msg->payload_len = strlen(msg->payload); // 记录 payload 实际字符串长度

}                                               // Telemetry_BuildMqttMsg 函数结束
