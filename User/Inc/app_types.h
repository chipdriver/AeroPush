#ifndef __APP_TYPES_H__ // 防止头文件重复包含
#define __APP_TYPES_H__

#include <stdint.h> // 提供固定宽度整数类型

#define MQTT_TOPIC_MAX_LEN 128 // MQTT 主题字符串最大长度
#define MQTT_PAYLOAD_MAX_LEN 512 // MQTT 负载字符串最大长度

typedef struct // 姿态数据
{
    float roll_deg; // 横滚角，单位度
    float pitch_deg; // 俯仰角，单位度
    float yaw_deg; // 航向角，单位度
    uint32_t timestamp_ms; // 数据时间戳，单位 tick/ms
    uint8_t valid; // 数据有效标志
} AttitudeData_t;

typedef struct // GNSS 定位数据
{
    double latitude; // 纬度
    double longitude; // 经度
    float altitude_m; // 高度，单位 m
    float speed_mps; // 速度，单位 m/s
    uint8_t gps_num; // 可见或使用的卫星数量
    uint8_t fix_valid; // 定位有效标志
    uint32_t timestamp_ms; // 数据时间戳，单位 tick/ms
} GnssData_t;

typedef struct // 遥测汇总数据
{
    AttitudeData_t attitude; // 姿态数据
    GnssData_t gnss; // GNSS 数据
    uint8_t battery_pct; // 电池电量百分比
    uint32_t timestamp_ms; // 遥测时间戳，单位 tick/ms
} TelemetryData_t;

typedef struct // MQTT 发布消息
{
    char topic[MQTT_TOPIC_MAX_LEN]; // MQTT 主题
    char payload[MQTT_PAYLOAD_MAX_LEN]; // MQTT 负载
    uint16_t payload_len; // MQTT 负载长度
} MqttPublishMsg_t;

#endif // __APP_TYPES_H__
