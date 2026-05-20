#include "telemetry_service.h" // 提供遥测组包接口
#include "app_config.h" // 提供 MQTT 主题配置

/**
 * @brief 根据姿态和 GNSS 数据构造 MQTT 遥测消息。
 * @param att 姿态数据。
 * @param gnss GNSS 定位数据。
 * @param msg 输出 MQTT 发布消息。
 * @retval None
 */
void Telemetry_BuildMqttMsg(const AttitudeData_t *att, const GnssData_t *gnss, MqttPublishMsg_t *msg) // 构造遥测 MQTT 消息
{
    if (att == 0 || gnss == 0 || msg == 0) // 检查输入输出指针
    {
        return; // 参数无效时不组包
    }

    memset(msg, 0, sizeof(MqttPublishMsg_t)); // 清空输出消息结构体

    strcpy(msg->topic, APP_MQTT_TOPIC); // 写入 MQTT 主题

    snprintf(msg->payload, // 写入 JSON 负载
             MQTT_PAYLOAD_MAX_LEN, // 限制负载最大长度
             "{\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"lat\":%.6f,\"lon\":%.6f}", // 遥测 JSON 格式
             att->roll_deg, // 写入横滚角
             att->pitch_deg, // 写入俯仰角
             att->yaw_deg, // 写入航向角
             gnss->latitude, // 写入纬度
             gnss->longitude); // 写入经度

    msg->payload_len = strlen(msg->payload); // 保存负载实际长度
}
