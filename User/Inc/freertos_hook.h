#ifndef __FREERTOS_HOOK_H__
#define __FREERTOS_HOOK_H__

/*headler file  */
#include "FreeRTOS.h"
#include "task.h"
/*function */
void vApplicationMallocFailedHook(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);

#endif  /*__FREERTOS_HOOK_H__*/