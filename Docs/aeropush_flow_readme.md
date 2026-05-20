# AeroPush 当前工程流程图

为了避免一张大图被缩小到看不清，流程图拆成了四张：

1. `aeropush_flow_overview_clear.svg`：整体总览
2. `aeropush_flow_init_clear.svg`：InitTask 初始化流程
3. `aeropush_flow_imu_clear.svg`：IMU 姿态角获取链路
4. `aeropush_flow_telemetry_clear.svg`：Telemetry / Modem 流程

其中 IMU 链路已经接通，可以输出 `roll / pitch / yaw`；GNSS 与 MQTT 当前仍是模拟或占位。
