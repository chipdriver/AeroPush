#ifndef app_tasks_h                                      // 防止 app_tasks.h 被重复包含
#define app_tasks_h                                      // 定义应用任务头文件保护宏
/* header file inclusion */                              // 头文件包含区域
#include "bsp_led.h"                                     // 引入 LED BSP 接口
#include "bsp_debug_uart.h"                              // 引入调试串口 BSP 接口
#include <stdio.h>                                       // 引入标准输出库，用于 snprintf 等函数
#include <string.h>                                      // 引入字符串处理库，用于 memset、strcpy、strlen 等函数
#include "freertos_objects.h"                            // 引入队列、互斥锁和项目数据类型
#include  "debug_log.h"                                  // 引入调试日志输出接口
#include "FreeRTOS.h"                                    // 引入 FreeRTOS 基础定义
#include "task.h"                                        // 引入 FreeRTOS 任务接口
#include "telemetry_service.h"                           // 引入遥测服务接口
#include "modem_service.h"                               // 引入通信服务接口
#include "imu_service.h"                                 // 引入 IMU 服务接口
#include "app_config.h"                                  // 引入应用配置参数
#include "app_status.h"                                  // 引入系统状态管理接口
/* function declaration */                               // 函数声明区域

/**
 * @brief  创建应用层所有 FreeRTOS 任务。
 * @param  None
 * @retval None
 */
void APP_TasksCreate(void);                              // 声明创建所有 RTOS 任务函数
#endif  /* app_tasks_h */                                // 结束应用任务头文件保护宏
