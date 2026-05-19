#ifndef __APP_TYPES_H__ // 检查 __APP_TYPES_H__ 是否未定义，防止头文件重复包含
#define __APP_TYPES_H__ // 定义 __APP_TYPES_H__ 变量

#include <stdint.h> // 引入 stdint.h 提供的接口、宏和类型定义

#define MQTT_TOPIC_MAX_LEN 128 // 定义 MQTT_TOPIC_MAX_LEN 变量为 128
#define MQTT_PAYLOAD_MAX_LEN 512 // 定义 MQTT_PAYLOAD_MAX_LEN 变量为 512

typedef struct // 开始定义结构体类型，用于集中保存相关数据字段
{ // 进入当前代码块
    float roll_deg; // 声明 横滚角角度值，供后续计算、状态保存或模块间传递使用
    float pitch_deg; // 声明 俯仰角角度值，供后续计算、状态保存或模块间传递使用
    float yaw_deg; // 声明 航向角角度值，供后续计算、状态保存或模块间传递使用
    uint32_t timestamp_ms; // 声明 timestamp_ms 变量，供后续计算、状态保存或模块间传递使用
    uint8_t valid; // 声明 valid 变量，供后续计算、状态保存或模块间传递使用
} AttitudeData_t; // 结束结构体定义，并声明结构体类型名 AttitudeData_t

typedef struct // 开始定义结构体类型，用于集中保存相关数据字段
{ // 进入当前代码块
    double latitude; // 声明 latitude 变量，供后续计算、状态保存或模块间传递使用
    double longitude; // 声明 longitude 变量，供后续计算、状态保存或模块间传递使用
    float altitude_m; // 声明 altitude_m 变量，供后续计算、状态保存或模块间传递使用
    float speed_mps; // 声明 speed_mps 变量，供后续计算、状态保存或模块间传递使用
    uint8_t gps_num; // 声明 gps_num 变量，供后续计算、状态保存或模块间传递使用
    uint8_t fix_valid; // 声明 fix_valid 变量，供后续计算、状态保存或模块间传递使用
    uint32_t timestamp_ms; // 声明 timestamp_ms 变量，供后续计算、状态保存或模块间传递使用
} GnssData_t; // 结束结构体定义，并声明结构体类型名 GnssData_t

typedef struct // 开始定义结构体类型，用于集中保存相关数据字段
{ // 进入当前代码块
    AttitudeData_t attitude; // 声明 姿态数据结构体，供后续计算、状态保存或模块间传递使用
    GnssData_t gnss; // 声明 GNSS 定位数据结构体，供后续计算、状态保存或模块间传递使用
    uint8_t battery_pct; // 声明 battery_pct 变量，供后续计算、状态保存或模块间传递使用
    uint32_t timestamp_ms; // 声明 timestamp_ms 变量，供后续计算、状态保存或模块间传递使用
} TelemetryData_t; // 结束结构体定义，并声明结构体类型名 TelemetryData_t

typedef struct // 开始定义结构体类型，用于集中保存相关数据字段
{ // 进入当前代码块
    char topic[MQTT_TOPIC_MAX_LEN]; // 声明 topic 变量，供后续计算、状态保存或模块间传递使用
    char payload[MQTT_PAYLOAD_MAX_LEN]; // 声明 payload 变量，供后续计算、状态保存或模块间传递使用
    uint16_t payload_len; // 声明 payload_len 变量，供后续计算、状态保存或模块间传递使用
} MqttPublishMsg_t; // 结束结构体定义，并声明结构体类型名 MqttPublishMsg_t

#endif // 结束当前条件编译或头文件保护范围
