#ifndef __MODEM_SERVICE_H__ /* 防止 modem_service.h 被重复包含 */
#define __MODEM_SERVICE_H__ /* 定义通信服务头文件保护宏 */

#include <string.h>    /* 引入 memset 等内存处理接口 */
#include "FreeRTOS.h"  /* 引入 FreeRTOS 基础定义 */
#include "app_types.h" /* 引入 GNSS 和 MQTT 消息类型 */
#include "debug_log.h" /* 引入调试日志接口 */
#include "task.h"      /* 引入 FreeRTOS 任务时间接口 */

void ModemService_BuildSimGnss(GnssData_t *gnss);       /* 声明模拟 GNSS 数据构造函数 */
void ModemService_Publish(const MqttPublishMsg_t *msg); /* 声明模拟 MQTT 发布函数 */

#endif /* 结束通信服务头文件保护宏 */
