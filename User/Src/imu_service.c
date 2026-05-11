#include "imu_service.h"

void ImuService_BuildSimAttitude(AttitudeData_t *attitude)
{
    static float angle = 0.0f;

    if(attitude == NULL)
        return;

    angle += 1.0f;

    if(angle >= 360.0f)
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
