#ifndef __TELEMETRY_SERVICE_H__        // 防止 telemetry_service.h 被重复包含
#define __TELEMETRY_SERVICE_H__        // 定义头文件保护宏

/* header file inclusion */            // 头文件包含区域
#include "app_types.h"                 // 引入姿态、GNSS、MQTT 消息等项目数据类型
#include <stdio.h>                     // 引入 snprintf 等格式化输出函数声明
#include <string.h>                    // 引入 memset、strcpy、strlen 等字符串处理函数声明
/* macro definitions */                // 宏定义区域，当前暂无本模块专用宏

/* function declaration */             // 函数声明区域
void Telemetry_BuildMqttMsg(const AttitudeData_t *att,const GnssData_t *gnss,MqttPublishMsg_t *msg); // 根据姿态和 GNSS 数据组装 MQTT 发布消息
#endif /* __TELEMETRY_SERVICE_H__ */   // 结束头文件保护宏
