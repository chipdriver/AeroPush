#ifndef __APP_TYPES_H__
#define __APP_TYPES_H__
/* headler file inclution */
#include <stdint.h>

/* macro definition */
#define MQTT_TOPIC_MAX_LEN 128  //定义 MQTT topic 字符串最大长度为 128 字节
#define MQTT_PAYLOAD_MAX_LEN 512    //定义 MQTT payload 字符串最大长度为 512 字节
/* stricture */

/* 姿态数据结构体 */
typedef struct 
{
    float roll_deg;     //翻滚角,单位:度
    float pitch_deg;    //俯仰角,单位,度
    float yaw_deg;      //翻滚角,单位,度

    uint32_t timestamp_ms;  //时间戳,单位:ms
    uint8_t valid;  //数据有效标志,1 表示有效,0 表示无效
}AttitudeData_t;    //  将结构体类型命名为 AttitudeData_t

/* GNSS 定位数据结构体 */
typedef struct
{
    double latitude;    //纬度,使用 double 提高精度
    double longitude;   //经度,使用 double 提高精度

    float altitude_m;   //海拔高度:单位:米
    float speed_mps;    //地面速度,单位:米每秒

    uint8_t gps_num;    //当前可用卫星数量
    uint8_t fix_valid;  //定位有效标志, 1 表示已定位, 0 表示未定位
    
    uint32_t timestamp_ms;                      // GNSS 数据时间戳，单位：毫秒
}GnssData_t;            //  将结构体类型命名为 GnssData_t

/* 遥测数据结构体 */
typedef struct 
{
    AttitudeData_t attitude;        //姿态数据
    GnssData_t gnss;                //GNSS 定位数据

    uint8_t battery_pct;            //电池电量百分比
    uint32_t timestamp_ms;          //遥测数据时间戳,单位:毫秒
}TelemetryData_t;       //  将结构体类型命名为 TelemetryData_t

/* MQTT 发布消息结构体*/
typedef struct 
{
    char topic[MQTT_TOPIC_MAX_LEN]; //MQTT 主题字符串缓冲区
    char payload[MQTT_PAYLOAD_MAX_LEN]; //MQTT 负载数据字符串缓冲区
    uint16_t payload_len;       //MQTT 负载数据实际长度
}MqttPublishMsg_t;          //  将结构体类型命名为 MqttPublishMsg_t




/* function declaration*/

#endif /* __APP_TYPES_H__ */