#include "imu_service.h"
#include "app_config.h"
#include "mpu9250_driver.h"
#include "debug_log.h"

uint8_t ImuService_Init(void)
{
    uint8_t ret;

    Debug_Print("[ImuService] init start\r\n");

    ret = MPU9250_Driver_Init();
    if(ret == 0)
    {
        Debug_Print("[ImuService] init failed\r\n");
        return 0;
    }

    Debug_Print("[ImuService] init ok\r\n");
    return 1;
}

void ImuService_BuildSimAttitude(AttitudeData_t *attitude)
{
    static float angle = 0.0f;

    if(attitude == NULL)
        return;

    angle += APP_SIM_ATTITUDE_STEP_DEG;

    if(angle >= APP_SIM_ATTITUDE_MAX_DEG)
    {
        angle = 0.0f;
    }

    memset(attitude,0,sizeof(AttitudeData_t));

    attitude->roll_deg = angle;
    attitude->pitch_deg = angle + 1.0f;
    attitude->yaw_deg = angle + 2.0f;
    attitude->timestamp_ms = xTaskGetTickCount();
    attitude->valid = 1;
}
