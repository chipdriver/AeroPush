#ifndef __APP_TYPES_H__ /* 防止 app_types.h 被重复包含 */
#define __APP_TYPES_H__ /* 定义应用数据类型头文件保护宏 */

#include <stdint.h> /* 引入标准整数类型 */

#define MQTT_TOPIC_MAX_LEN 128   /* 定义 MQTT topic 最大长度 */
#define MQTT_PAYLOAD_MAX_LEN 512 /* 定义 MQTT payload 最大长度 */

typedef struct /* 定义姿态数据结构体 */
{
    float roll_deg;        /* 保存横滚角，单位 deg */
    float pitch_deg;       /* 保存俯仰角，单位 deg */
    float yaw_deg;         /* 保存航向角，单位 deg */
    uint32_t timestamp_ms; /* 保存姿态时间戳，单位 ms */
    uint8_t valid;         /* 保存姿态有效标志，1 有效，0 无效 */
} AttitudeData_t;          /* 声明姿态数据类型 */

typedef struct /* 定义 GNSS 定位数据结构体 */
{
    double latitude;       /* 保存纬度 */
    double longitude;      /* 保存经度 */
    float altitude_m;      /* 保存海拔高度，单位 m */
    float speed_mps;       /* 保存地面速度，单位 m/s */
    uint8_t gps_num;       /* 保存可用卫星数量 */
    uint8_t fix_valid;     /* 保存定位有效标志，1 有效，0 无效 */
    uint32_t timestamp_ms; /* 保存 GNSS 时间戳，单位 ms */
} GnssData_t;              /* 声明 GNSS 数据类型 */

typedef struct /* 定义遥测数据结构体 */
{
    AttitudeData_t attitude; /* 保存姿态数据 */
    GnssData_t gnss;         /* 保存 GNSS 数据 */
    uint8_t battery_pct;     /* 保存电池电量百分比 */
    uint32_t timestamp_ms;   /* 保存遥测时间戳，单位 ms */
} TelemetryData_t;           /* 声明遥测数据类型 */

typedef struct /* 定义 MQTT 发布消息结构体 */
{
    char topic[MQTT_TOPIC_MAX_LEN];     /* 保存 MQTT topic 字符串 */
    char payload[MQTT_PAYLOAD_MAX_LEN]; /* 保存 MQTT payload 字符串 */
    uint16_t payload_len;               /* 保存 MQTT payload 实际长度 */
} MqttPublishMsg_t;                     /* 声明 MQTT 发布消息类型 */

#endif /* 结束应用数据类型头文件保护宏 */
