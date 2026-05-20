#include "modem_service.h" // 提供通信服务接口
#include "app_config.h" // 提供模拟 GNSS 配置

/**
 * @brief 构造模拟 GNSS 定位数据。
 * @param gnss 输出 GNSS 定位数据。
 * @retval None
 */
void ModemService_BuildSimGnss(GnssData_t *gnss) // 构造模拟 GNSS 数据
{
    static uint32_t gnss_count = 0; // 模拟轨迹步进计数

    if (gnss == NULL) // 检查输出指针
    {
        return; // 输出指针为空时不处理
    }

    gnss_count++; // 推进模拟定位点

    memset(gnss, 0, sizeof(GnssData_t)); // 清空 GNSS 数据结构体

    gnss->latitude = APP_SIM_GNSS_BASE_LAT + gnss_count * APP_SIM_GNSS_STEP; // 生成模拟纬度
    gnss->longitude = APP_SIM_GNSS_BASE_LON + gnss_count * APP_SIM_GNSS_STEP; // 生成模拟经度

    gnss->altitude_m = APP_SIM_GNSS_ALTITUDE_M; // 写入模拟高度
    gnss->speed_mps = APP_SIM_GNSS_SPEED_MPS; // 写入模拟速度

    gnss->gps_num = APP_SIM_GNSS_NUM; // 写入模拟卫星数量
    gnss->fix_valid = 1; // 标记模拟定位有效

    gnss->timestamp_ms = xTaskGetTickCount(); // 写入当前 tick 时间戳
}

/**
 * @brief 处理一条待发布 MQTT 消息。
 * @param msg 待发布 MQTT 消息。
 * @retval None
 */
void ModemService_Publish(const MqttPublishMsg_t *msg) // 处理 MQTT 发布请求
{
    if (msg == NULL) // 检查输入指针
    {
        return; // 输入指针为空时不处理
    }

    // Debug_Printf("[ModemTask] publish topic=%s payload=%s\r\n", msg->topic, msg->payload); // 调试阶段查看发布内容
}
