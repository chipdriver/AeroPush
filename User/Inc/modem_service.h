#ifndef __MODEM_SERVICE_H__ // 检查 __MODEM_SERVICE_H__ 是否未定义，防止头文件重复包含
#define __MODEM_SERVICE_H__ // 定义 __MODEM_SERVICE_H__ 变量

#include <string.h> // 引入 string.h 提供的接口、宏和类型定义
#include "FreeRTOS.h" // 引入 FreeRTOS.h 提供的接口、宏和类型定义
#include "app_types.h" // 引入 app_types.h 提供的接口、宏和类型定义
#include "debug_log.h" // 引入 debug_log.h 提供的接口、宏和类型定义
#include "task.h" // 引入 task.h 提供的接口、宏和类型定义

/**
 * @brief 构造模拟 GNSS 定位数据。
 * @param gnss GNSS 定位数据结构体。
 * @retval None
 */
void ModemService_BuildSimGnss(GnssData_t *gnss); // 声明ModemService_BuildSimGnss 函数签名：构造模拟 GNSS 定位数据
/**
 * @brief 处理一条待发布 MQTT 消息。
 * @param msg msg 变量。
 * @retval None
 */
void ModemService_Publish(const MqttPublishMsg_t *msg); // 声明ModemService_Publish 函数签名：处理一条待发布 MQTT 消息

#endif // 结束当前条件编译或头文件保护范围
