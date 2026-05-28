#ifndef __APP_CONFIG_H__ // 防止头文件重复包含
#define __APP_CONFIG_H__

#define APP_ENABLE_IMU 0U // 0 关闭 IMU，1 启用 IMU

#define APP_TASK_INIT_STACK_SIZE 1024 // 初始化任务栈大小
#define APP_TASK_IMU_STACK_SIZE 256 // IMU 任务栈大小
#define APP_TASK_MODEM_STACK_SIZE 768 // 通信任务栈大小
#define APP_TASK_TELEMETRY_STACK_SIZE 768 // 遥测任务栈大小
#define APP_TASK_LED_STACK_SIZE 128 // LED 任务栈大小

#define APP_TASK_INIT_PRIORITY 4 // 初始化任务优先级
#define APP_TASK_IMU_PRIORITY 3 // IMU 任务优先级
#define APP_TASK_MODEM_PRIORITY 2 // 通信任务优先级
#define APP_TASK_TELEMETRY_PRIORITY 2 // 遥测任务优先级
#define APP_TASK_LED_PRIORITY 1 // LED 任务优先级

#define APP_IMU_TASK_PERIOD_MS 20 // IMU 任务周期，单位 ms
#define APP_MODEM_TASK_PERIOD_MS 20 // 通信任务周期，单位 ms
#define APP_GNSS_QUERY_PERIOD_MS 1000U // A7670E GNSS 查询周期，单位 ms
#define APP_GNSS_SOURCE_REAL 1U // GNSS 数据来源：真实 A7670E GNSS
#define APP_GNSS_SOURCE_SIM 2U // GNSS 数据来源：软件模拟 GNSS
#define APP_GNSS_SOURCE_MODE APP_GNSS_SOURCE_SIM // 当前阶段默认使用模拟 GNSS，方便测试 4G 和 MQTT
#define APP_TELEMETRY_TASK_PERIOD_MS 500 // 遥测任务周期，单位 ms
#define APP_LED_TASK_PERIOD_MS 500 // LED 正常闪烁周期，单位 ms

#define APP_MQTT_TOPIC "aeropush/telemetry" // 遥测 MQTT 主题

#define APP_SIM_GNSS_BASE_LAT 36.000000 // 模拟 GNSS 起始纬度
#define APP_SIM_GNSS_BASE_LON 120.000000 // 模拟 GNSS 起始经度
#define APP_SIM_GNSS_STEP 0.000001 // 模拟 GNSS 每次移动步长
#define APP_SIM_GNSS_ALTITUDE_M 30.0f // 模拟 GNSS 高度，单位 m
#define APP_SIM_GNSS_SPEED_MPS 5.0f // 模拟 GNSS 速度，单位 m/s
#define APP_SIM_GNSS_NUM 12 // 模拟 GNSS 卫星数量

#define APP_SIM_ATTITUDE_STEP_DEG 1.0f // 模拟姿态角步进，单位度
#define APP_SIM_ATTITUDE_MAX_DEG 360.0f // 模拟姿态角回绕上限，单位度

#define APP_MAG_POINT_CLOUD_DEBUG_ENABLE 1U // 磁力计校准点云调试输出开关
#define APP_MAG_POINT_CLOUD_PRINT_DIV 10U // 磁力计点云打印分频

#endif // __APP_CONFIG_H__
