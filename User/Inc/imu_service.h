#ifndef __IMU_SERVICE_H__
#define __IMU_SERVICE_H__

/* header file inclusion */
#include "app_types.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* function declaration */
void ImuService_BuildSimAttitude(AttitudeData_t *attitude);

#endif /* __IMU_SERVICE_H__ */
