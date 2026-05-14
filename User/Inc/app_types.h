#ifndef __APP_TYPES_H__                              // 防止 app_types.h 被重复包含
#define __APP_TYPES_H__                              // 定义应用数据类型头文件保护宏
/* headler file inclution */                         // 头文件包含区域
#include <stdint.h>                                  // 引入标准整数类型

/* macro definition */                               // 宏定义区域
#define MQTT_TOPIC_MAX_LEN 128                       // 定义 MQTT topic 字符串最大长度为 128 字节
#define MQTT_PAYLOAD_MAX_LEN 512                     // 定义 MQTT payload 字符串最大长度为 512 字节
/* stricture */                                      // 结构体定义区域

/* 姿态数据结构体 */
typedef struct                                      // 定义姿态数据结构体
{                                                   // AttitudeData_t 结构体开始
    float roll_deg;                                 // 翻滚角，单位：度
    float pitch_deg;                                // 俯仰角，单位：度
    float yaw_deg;                                  // 偏航角，单位：度

    uint32_t timestamp_ms;                          // 时间戳，单位：ms
    uint8_t valid;                                  // 数据有效标志，1 表示有效，0 表示无效
}AttitudeData_t;                                    // 将结构体类型命名为 AttitudeData_t

/* GNSS 定位数据结构体 */
typedef struct                                      // 定义 GNSS 定位数据结构体
{                                                   // GnssData_t 结构体开始
    double latitude;                                // 纬度，使用 double 提高精度
    double longitude;                               // 经度，使用 double 提高精度

    float altitude_m;                               // 海拔高度，单位：米
    float speed_mps;                                // 地面速度，单位：米每秒

    uint8_t gps_num;                                // 当前可用卫星数量
    uint8_t fix_valid;                              // 定位有效标志，1 表示已定位，0 表示未定位
    
    uint32_t timestamp_ms;                          // GNSS 数据时间戳，单位：毫秒
}GnssData_t;                                        // 将结构体类型命名为 GnssData_t

/* 遥测数据结构体 */
typedef struct                                      // 定义遥测数据结构体
{                                                   // TelemetryData_t 结构体开始
    AttitudeData_t attitude;                        // 姿态数据
    GnssData_t gnss;                                // GNSS 定位数据

    uint8_t battery_pct;                            // 电池电量百分比
    uint32_t timestamp_ms;                          // 遥测数据时间戳，单位：毫秒
}TelemetryData_t;                                   // 将结构体类型命名为 TelemetryData_t

/* MQTT 发布消息结构体*/
typedef struct                                      // 定义 MQTT 发布消息结构体
{                                                   // MqttPublishMsg_t 结构体开始
    char topic[MQTT_TOPIC_MAX_LEN];                 // MQTT 主题字符串缓冲区
    char payload[MQTT_PAYLOAD_MAX_LEN];             // MQTT 负载数据字符串缓冲区
    uint16_t payload_len;                           // MQTT 负载数据实际长度
}MqttPublishMsg_t;                                  // 将结构体类型命名为 MqttPublishMsg_t




/* function declaration */                          // 函数声明区域，当前暂无函数声明

#endif /* __APP_TYPES_H__ */                        // 结束应用数据类型头文件保护宏
