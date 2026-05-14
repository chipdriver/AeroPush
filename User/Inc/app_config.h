#ifndef __APP_CONFIG_H__                                      // 防止 app_config.h 被重复包含
#define __APP_CONFIG_H__                                      // 定义头文件保护宏

#define APP_TASK_INIT_STACK_SIZE          256                 // InitTask 栈大小
#define APP_TASK_IMU_STACK_SIZE           256                 // ImuTask 栈大小
#define APP_TASK_MODEM_STACK_SIZE         768                 // ModemTask 栈大小
#define APP_TASK_TELEMETRY_STACK_SIZE     768                 // TelemetryTask 栈大小
#define APP_TASK_LED_STACK_SIZE           128                 // LedTask 栈大小

#define APP_TASK_INIT_PRIORITY            4                   // InitTask 优先级
#define APP_TASK_IMU_PRIORITY             3                   // ImuTask 优先级
#define APP_TASK_MODEM_PRIORITY           2                   // ModemTask 优先级
#define APP_TASK_TELEMETRY_PRIORITY       2                   // TelemetryTask 优先级
#define APP_TASK_LED_PRIORITY             1                   // LedTask 优先级

#define APP_IMU_TASK_PERIOD_MS            20                   // ImuTask 周期，单位 ms
#define APP_MODEM_TASK_PERIOD_MS          20                  // ModemTask 周期，单位 ms
#define APP_TELEMETRY_TASK_PERIOD_MS      1000                // TelemetryTask 周期，单位 ms
#define APP_LED_TASK_PERIOD_MS            500                 // LedTask 周期，单位 ms

#define APP_MQTT_TOPIC                    "aeropush/telemetry" // MQTT 遥测 topic

#define APP_SIM_GNSS_BASE_LAT             36.000000           // 模拟 GNSS 初始纬度
#define APP_SIM_GNSS_BASE_LON             120.000000          // 模拟 GNSS 初始经度
#define APP_SIM_GNSS_STEP                 0.000001            // 模拟 GNSS 变化步长
#define APP_SIM_GNSS_ALTITUDE_M           30.0f               // 模拟高度，单位 m
#define APP_SIM_GNSS_SPEED_MPS            5.0f                // 模拟速度，单位 m/s
#define APP_SIM_GNSS_NUM                  12                  // 模拟卫星数量

#define APP_SIM_ATTITUDE_STEP_DEG         1.0f                // 模拟姿态角变化步长
#define APP_SIM_ATTITUDE_MAX_DEG          360.0f              // 模拟姿态角最大值

#endif                                                        // 结束头文件保护宏
