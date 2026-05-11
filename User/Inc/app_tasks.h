#ifndef app_tasks_h
#define app_tasks_h
/*header file inclusion*/
#include "bsp_led.h"
#include "bsp_debug_uart.h"
#include <stdio.h>      //引入标准输出输出库,用于 snprintf 等函数
#include <string.h>     //引入字符串处理库,用于 memset,strcpy,strlen 等函数
#include "freertos_objects.h"   //引入队列,互斥锁和项目数据类型
#include  "debug_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "telemetry_service.h"
#include "modem_service.h"
#include "imu_service.h"

/*function declaration*/
void APP_TasksCreate(void);  //创建所有RTOS任务
#endif  /* app_tasks_h */
