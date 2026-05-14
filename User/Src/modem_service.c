#include "modem_service.h"                              // 引入通信服务模块接口声明
#include "app_config.h"                                 // 引入模拟 GNSS 配置参数

/**
 * @brief  构造一帧模拟 GNSS 定位数据。
 * @param  gnss 用于保存模拟 GNSS 数据的结构体指针。
 * @retval None
 */
void ModemService_BuildSimGnss(GnssData_t *gnss)         // 构造模拟 GNSS 定位数据
{                                                         // ModemService_BuildSimGnss 函数体开始
    static uint32_t gnss_count = 0;                      // 定义 GNSS 计数变量，用于模拟经纬度逐渐变化

    if(gnss == NULL)                                     // 检查输出参数是否为空，避免访问非法地址
        return;                                          // 参数为空时直接返回，不生成 GNSS 数据
    gnss_count++;                                        // GNSS 计数值加 1，用于生成不断变化的模拟定位数据

    memset(gnss,0,sizeof(GnssData_t));                   // 清空 GNSS 结构体，避免残留旧数据

    gnss->latitude = APP_SIM_GNSS_BASE_LAT + gnss_count * APP_SIM_GNSS_STEP;  // 使用配置中的初始纬度和步长生成模拟纬度
    gnss->longitude = APP_SIM_GNSS_BASE_LON + gnss_count * APP_SIM_GNSS_STEP; // 使用配置中的初始经度和步长生成模拟经度

    gnss->altitude_m = APP_SIM_GNSS_ALTITUDE_M;          // 使用配置中的模拟高度
    gnss->speed_mps = APP_SIM_GNSS_SPEED_MPS;            // 使用配置中的模拟速度
    
    gnss->gps_num = APP_SIM_GNSS_NUM;                    // 使用配置中的模拟卫星数量
    gnss->fix_valid = 1;                                 // 设置定位有效标志为 1，表示定位有效

    gnss->timestamp_ms = xTaskGetTickCount();            // 获取当前系统 tick，作为 GNSS 数据时间戳
}                                                         // ModemService_BuildSimGnss 函数体结束

/**
 * @brief  模拟发布 MQTT 消息。
 * @param  msg 指向待发布 MQTT 消息的结构体指针。
 * @retval None
 */
void ModemService_Publish(const MqttPublishMsg_t *msg)   // 模拟 MQTT 发布动作
{                                                         // ModemService_Publish 函数体开始
    if(msg == NULL)                                      // 检查 MQTT 消息指针是否为空
        return;                                          // 消息为空时直接返回，不执行发布打印
    
    // Debug_Printf("[ModemTask] publish topic=%s payload=%s\r\n",msg->topic,msg->payload); // 打印模拟发布的主题和负载内容
}                                                         // ModemService_Publish 函数体结束
