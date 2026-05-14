#ifndef __MODEM_SERVICE_H__                         // 防止 modem_service.h 被重复包含
#define __MODEM_SERVICE_H__                         // 定义头文件保护宏

/* header file inclusion */                         // 头文件包含区域
#include "app_types.h"                              // 引入 GNSS 数据和 MQTT 发布消息等项目数据类型
#include "debug_log.h"                              // 引入线程安全日志打印函数

#include "FreeRTOS.h"                               // 引入 FreeRTOS 基础类型和宏定义
#include "task.h"                                   // 引入 xTaskGetTickCount 等任务相关函数

#include <string.h>                                 // 引入 memset 等内存和字符串处理函数声明
/* macro function */                                // 宏定义区域，当前暂无本模块专用宏

/* function declaration */                          // 函数声明区域

/**
 * @brief  构造一帧模拟 GNSS 定位数据。
 * @param  gnss 用于保存模拟 GNSS 数据的结构体指针。
 * @retval None
 */
void ModemService_BuildSimGnss(GnssData_t *gnss);   // 构造一帧模拟 GNSS 定位数据

/**
 * @brief  模拟发布 MQTT 消息。
 * @param  msg 指向待发布 MQTT 消息的结构体指针。
 * @retval None
 */
void ModemService_Publish(const MqttPublishMsg_t *msg); // 模拟发布 MQTT 消息并通过调试串口打印

#endif /* __MODEM_SERVICE_H__ */                    // 结束头文件保护宏
