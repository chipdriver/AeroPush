#ifndef __MODEM_SERVICE_H__ // 防止头文件重复包含
#define __MODEM_SERVICE_H__

#include <string.h> // 提供 memset
#include "FreeRTOS.h" // 提供 FreeRTOS 基础类型
#include "app_types.h" // 提供 GNSS 和 MQTT 数据结构
#include "debug_log.h" // 提供调试日志接口
#include "task.h" // 提供 tick 计数接口

/**
 * @brief 构造模拟 GNSS 定位数据。
 * @param gnss 输出 GNSS 定位数据。
 * @retval None
 */
void ModemService_BuildSimGnss(GnssData_t *gnss); // 构造模拟 GNSS

/**
 * @brief 打开 A7670E GNSS 电源。
 * @retval 1U 表示 GNSS 上电命令返回成功，0U 表示失败。
 */
uint8_t ModemService_GnssInit(void); // 初始化 A7670E GNSS 电源

/**
 * @brief 读取并解析 A7670E 的真实 GNSS 定位数据。
 * @param gnss 输出 GNSS 定位数据。
 * @retval 1U 表示定位有效，0U 表示未定位或解析失败。
 */
uint8_t ModemService_ReadGnss(GnssData_t *gnss); // 读取真实 GNSS 数据

/**
 * @brief 初始化 A7670E 4G 数据网络。
 * @retval 1U 表示 4G 网络初始化成功。
 * @retval 0U 表示 4G 网络初始化失败。
 */
uint8_t ModemService_NetInit(void); // 初始化 A7670E 4G 网络

/**
 * @brief 初始化并连接 A7670E MQTT 客户端。
 * @retval 1U 表示 MQTT 连接服务器成功。
 * @retval 0U 表示 MQTT 初始化或连接失败。
 */
uint8_t ModemService_MqttInit(void); // 初始化并连接 MQTT 服务器

/**
 * @brief 处理一条待发布 MQTT 消息。
 * @param msg 待发布 MQTT 消息。
 * @retval None
 */
void ModemService_Publish(const MqttPublishMsg_t *msg); // 处理 MQTT 发布

#endif // __MODEM_SERVICE_H__
