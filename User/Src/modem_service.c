#include "modem_service.h" // 引入 modem_service.h 提供的接口、宏和类型定义
#include "app_config.h" // 引入 app_config.h 提供的接口、宏和类型定义

/**
 * @brief 构造模拟 GNSS 定位数据。
 * @param gnss GNSS 定位数据结构体。
 * @retval None
 */
void ModemService_BuildSimGnss(GnssData_t *gnss) // 定义ModemService_BuildSimGnss 函数签名：构造模拟 GNSS 定位数据
{ // 进入当前代码块
    static uint32_t gnss_count = 0; // 定义 gnss_count 变量，初始值设置为 0

    if (gnss == NULL) // 判断 gnss == NULL 是否成立，以选择后续执行路径
        return; // 当前条件不满足继续处理，直接返回调用者
    gnss_count++; // 将 gnss_count 变量 自增 1，用于推进计数或索引

    memset(gnss, 0, sizeof(GnssData_t)); // 按指定字节值填充目标内存区域，参数为 gnss, 0, sizeof(GnssData_t)

    gnss->latitude = APP_SIM_GNSS_BASE_LAT + gnss_count * APP_SIM_GNSS_STEP; // 把 APP_SIM_GNSS_BASE_LAT + gnss_count * APP_SIM_GNSS_STEP 的计算结果 写入 gnss->latitude 字段值
    gnss->longitude = APP_SIM_GNSS_BASE_LON + gnss_count * APP_SIM_GNSS_STEP; // 把 APP_SIM_GNSS_BASE_LON + gnss_count * APP_SIM_GNSS_STEP 的计算结果 写入 gnss->longitude 字段值

    gnss->altitude_m = APP_SIM_GNSS_ALTITUDE_M; // 把 APP_SIM_GNSS_ALTITUDE_M 应用配置项 写入 gnss->altitude_m 字段值
    gnss->speed_mps = APP_SIM_GNSS_SPEED_MPS; // 把 APP_SIM_GNSS_SPEED_MPS 应用配置项 写入 gnss->speed_mps 字段值

    gnss->gps_num = APP_SIM_GNSS_NUM; // 把 APP_SIM_GNSS_NUM 应用配置项 写入 gnss->gps_num 字段值
    gnss->fix_valid = 1; // 把 1 写入 gnss->fix_valid 字段值

    gnss->timestamp_ms = xTaskGetTickCount(); // 把 当前 FreeRTOS tick 计数 写入 gnss->timestamp_ms 字段值
} // 结束当前代码块

/**
 * @brief 处理一条待发布 MQTT 消息。
 * @param msg msg 变量。
 * @retval None
 */
void ModemService_Publish(const MqttPublishMsg_t *msg) // 定义ModemService_Publish 函数签名：处理一条待发布 MQTT 消息
{ // 进入当前代码块
    if (msg == NULL) // 判断 msg == NULL 是否成立，以选择后续执行路径
        return; // 当前条件不满足继续处理，直接返回调用者

    // Debug_Printf("[ModemTask] publish topic=%s payload=%s\r\n",msg->topic,msg->payload); // 打印模拟发布的主题和负载内容
} // 结束当前代码块
