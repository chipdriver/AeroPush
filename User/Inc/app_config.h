#ifndef __APP_CONFIG_H__ // 防止头文件重复包含
#define __APP_CONFIG_H__

#define APP_ENABLE_IMU 1U // 0 关闭 IMU，1 启用 IMU
#define APP_IMU_SOURCE_REAL 1U // IMU 数据来源：真实 MPU9250
#define APP_IMU_SOURCE_SIM 2U // IMU 数据来源：软件模拟姿态
#define APP_IMU_SOURCE_MODE APP_IMU_SOURCE_SIM // 当前阶段默认使用模拟 IMU 姿态

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
#define APP_A7670E_APN "ctnet" // A7670E 使用的 APN，当前电信卡使用 ctnet
#define APP_NET_INIT_RETRY_PERIOD_MS 5000U // 4G 网络初始化失败后的重试间隔，单位 ms
#define APP_NET_REGISTER_RETRY_COUNT 30U // LTE 数据域注册状态最大查询次数，每次间隔约 1 秒
#define APP_TELEMETRY_TASK_PERIOD_MS 500 // 遥测任务周期，单位 ms
#define APP_LED_TASK_PERIOD_MS 500 // LED 正常闪烁周期，单位 ms

#define APP_MQTT_TOPIC "aeropush/telemetry" // 遥测 MQTT 主题
#define APP_MQTT_BROKER_ADDR "tcp://broker.emqx.io:1883" // MQTT 服务器地址和端口
#define APP_MQTT_CLIENT_ID "aeropush_client" // MQTT 客户端 ID
// #define APP_MQTT_USERNAME "admin" // MQTT 用户名
// #define APP_MQTT_PASSWORD "public" // MQTT 密码
#define APP_MQTT_KEEPALIVE_SEC 60U // MQTT keepalive 时间，单位秒
#define APP_MQTT_CLEAN_SESSION 1U // MQTT clean session 标志，1 表示清理旧会话
#define APP_MQTT_INIT_RETRY_PERIOD_MS 5000U // MQTT 初始化失败后的重试间隔，单位 ms

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
