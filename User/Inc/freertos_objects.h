#ifndef __FREERTOS_OBJECTS_H__
#define __FREERTOS_OBJECTS_H__
/* headler file incution */
#include "FreeRTOS.h"   //队列和互斥锁的依赖文件
#include "queue.h"  //队列
#include "semphr.h" //互斥锁
#include "app_types.h"  //类型文件

/* macro definition */
extern QueueHandle_t qAttitude;
extern QueueHandle_t qGnss;
extern QueueHandle_t qMqttPublish;
extern SemaphoreHandle_t mutexUart6log;
/* function declaration */
 void FreeRTOS_ObjectsCreate(void);
#endif /* __FREERTOS_OBJECTS_H__ */