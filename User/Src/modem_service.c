#include "modem_service.h"                              // 引入通信服务模块接口声明

void ModemService_BuildSimGnss(GnssData_t *gnss)         // 构造模拟 GNSS 定位数据
{
    static uint32_t gnss_count = 0;                      // 定义 GNSS 计数变量，用于模拟经纬度逐渐变化

    if(gnss == NULL)                                     // 检查输出参数是否为空，避免访问非法地址
        return;                                          // 参数为空时直接返回，不生成 GNSS 数据
    gnss_count++;                                        // GNSS 计数值加 1，用于生成不断变化的模拟定位数据

    memset(gnss,0,sizeof(GnssData_t));                   // 清空 GNSS 结构体，避免残留旧数据

    gnss->latitude = 36.000000 + gnss_count * 0.000001;  // 设置模拟纬度，每次调用都微小增加
    gnss->longitude = 120.000000 + gnss_count * 0.000001; // 设置模拟经度，每次调用都微小增加

    gnss->altitude_m = 30.0f;                            // 设置模拟海拔高度为 30 米
    gnss->speed_mps = 5.0f;                              // 设置模拟地面速度为 5 米每秒
    
    gnss->gps_num = 12;                                  // 设置模拟可用卫星数量为 12
    gnss->fix_valid = 1;                                 // 设置定位有效标志为 1，表示定位有效

    gnss->timestamp_ms = xTaskGetTickCount();            // 获取当前系统 tick，作为 GNSS 数据时间戳
}

void ModemService_Publish(const MqttPublishMsg_t *msg)   // 模拟 MQTT 发布动作
{
    if(msg == NULL)                                      // 检查 MQTT 消息指针是否为空
        return;                                          // 消息为空时直接返回，不执行发布打印
    
    Debug_Printf("[ModemTask] publish topic=%s payload=%s\r\n",msg->topic,msg->payload); // 打印模拟发布的主题和负载内容
}
