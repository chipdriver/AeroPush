#include "telemetry_service.h" // 引入 telemetry_service.h 提供的接口、宏和类型定义
#include "app_config.h" // 引入 app_config.h 提供的接口、宏和类型定义

/**
 * @brief 根据姿态和 GNSS 数据构造 MQTT 遥测消息。
 * @param att att 变量。
 * @param gnss GNSS 定位数据结构体。
 * @param msg msg 变量。
 * @retval None
 */
void Telemetry_BuildMqttMsg(const AttitudeData_t *att, const GnssData_t *gnss, MqttPublishMsg_t *msg) // 定义Telemetry_BuildMqttMsg 函数签名：根据姿态和 GNSS 数据构造 MQTT 遥测消息
{ // 进入当前代码块
    if (att == 0 || gnss == 0 || msg == 0) // 判断 att == 0 || gnss == 0 || msg == 0 是否成立，以选择后续执行路径
        return; // 当前条件不满足继续处理，直接返回调用者

    memset(msg, 0, sizeof(MqttPublishMsg_t)); // 按指定字节值填充目标内存区域，参数为 msg, 0, sizeof(MqttPublishMsg_t)

    strcpy(msg->topic, APP_MQTT_TOPIC); // 调用strcpy 函数，参数为 msg->topic, APP_MQTT_TOPIC

    snprintf(msg->payload, // 把格式化后的文本写入字符缓冲区，参数为 msg->payload,
             MQTT_PAYLOAD_MAX_LEN, // 继续传入 MQTT_PAYLOAD_MAX_LEN 变量，作为当前多行调用或初始化列表的一项
             "{\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"lat\":%.6f,\"lon\":%.6f}", // 继续传入 "{\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"lat\":%.6f,\"lon\":%.6f}" 字段值，作为当前多行调用或初始化列表的一项
             att->roll_deg, // 继续传入 att->roll_deg 字段值，作为当前多行调用或初始化列表的一项
             att->pitch_deg, // 继续传入 att->pitch_deg 字段值，作为当前多行调用或初始化列表的一项
             att->yaw_deg, // 继续传入 att->yaw_deg 字段值，作为当前多行调用或初始化列表的一项
             gnss->latitude, // 继续传入 gnss->latitude 字段值，作为当前多行调用或初始化列表的一项
             gnss->longitude); // 执行 gnss->longitude);，完成当前上下文中的具体处理

    msg->payload_len = strlen(msg->payload); // 把 strlen(msg->payload) 字段值 写入 msg->payload_len 字段值

} // 结束当前代码块
