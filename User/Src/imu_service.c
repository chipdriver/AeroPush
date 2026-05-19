/**
 * @file    imu_service.c
 * @brief   IMU 服务层实现，负责把底层 MPU9250/AK8963 驱动封装给任务层使用。
 */
#include "imu_service.h" // 引入 imu_service.h 提供的接口、宏和类型定义
#include "app_config.h" // 引入 app_config.h 提供的接口、宏和类型定义
#include "debug_log.h" // 引入 debug_log.h 提供的接口、宏和类型定义

/**
 * @brief 初始化 IMU 服务并调用底层 MPU9250 驱动。
 * @retval 函数执行结果或计算得到的返回值。
 */
uint8_t ImuService_Init(void) // 定义ImuService_Init 函数签名：初始化 IMU 服务并调用底层 MPU9250 驱动
{ // 进入当前代码块
    return MPU9250_Driver_Init(); // 将 MPU9250_Driver_Init() 返回给调用者
} // 结束当前代码块

/**
 * @brief ImuService_BuildSimAttitude 函数。
 * @param attitude 姿态数据结构体。
 * @retval None
 */
void ImuService_BuildSimAttitude(AttitudeData_t *attitude) // 定义ImuService_BuildSimAttitude 函数签名：ImuService_BuildSimAttitude 函数
{ // 进入当前代码块
    static float angle = 0.0f; // 定义 angle 变量，初始值设置为 0.0f 字段值

    if (attitude == 0) // 判断 attitude == 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    angle += APP_SIM_ATTITUDE_STEP_DEG; // 把 APP_SIM_ATTITUDE_STEP_DEG 应用配置项 累加到 angle 变量
    if (angle >= APP_SIM_ATTITUDE_MAX_DEG) // 判断 angle >= APP_SIM_ATTITUDE_MAX_DEG 是否成立，以选择后续执行路径
    { // 进入当前代码块
        angle = 0.0f; // 把 0.0f 字段值 写入 angle 变量
    } // 结束当前代码块

    memset(attitude, 0, sizeof(AttitudeData_t)); // 按指定字节值填充目标内存区域，参数为 attitude, 0, sizeof(AttitudeData_t)
    attitude->roll_deg = angle; // 把 angle 变量 写入 attitude->roll_deg 字段值
    attitude->pitch_deg = angle + 1.0f; // 把 angle + 1.0f 字段值 写入 attitude->pitch_deg 字段值
    attitude->yaw_deg = angle + 2.0f; // 把 angle + 2.0f 字段值 写入 attitude->yaw_deg 字段值
    attitude->timestamp_ms = xTaskGetTickCount(); // 把 当前 FreeRTOS tick 计数 写入 attitude->timestamp_ms 字段值
    attitude->valid = 1; // 把 1 写入 attitude->valid 字段值
} // 结束当前代码块

/**
 * @brief 读取 MPU9250 六轴原始数据。
 * @param raw 传感器原始采样数据结构体。
 * @retval None
 */
void ImuService_ReadRaw(MPU9250_raw_Data *raw) // 定义ImuService_ReadRaw 函数签名：读取 MPU9250 六轴原始数据
{ // 进入当前代码块
    if (raw == 0) // 判断 raw == 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    MPU9250_ReadAxis(raw); // 调用读取 MPU9250 加速度计、陀螺仪和温度原始寄存器，参数为 raw
} // 结束当前代码块

/**
 * @brief 读取 MPU9250 六轴和 AK8963 磁力计物理量。
 * @param phys MPU9250 六轴物理量数据。
 * @param mag AK8963 磁力计物理量数据。
 * @retval 函数执行结果或计算得到的返回值。
 */
uint8_t ImuService_ReadPhys(MPU9250_Physical_Data *phys, AK8963_Physical_Data *mag) // 定义ImuService_ReadPhys 函数签名：读取 MPU9250 六轴和 AK8963 磁力计物理量
{ // 进入当前代码块
    MPU9250_raw_Data raw; // 声明 传感器原始采样数据结构体，供后续计算、状态保存或模块间传递使用

    if ((phys == 0) || (mag == 0)) // 判断 (phys == 0) || (mag == 0) 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return IMU_SERVICE_READ_FAIL; // 将 IMU_SERVICE_READ_FAIL 变量 返回给调用者
    } // 结束当前代码块

    MPU9250_ReadAxis(&raw); // 调用读取 MPU9250 加速度计、陀螺仪和温度原始寄存器，参数为 &raw
    MPU9250_ConvertToPhysical(&raw, phys); // 调用将 MPU9250 原始值转换成 g、dps 和摄氏度物理量，参数为 &raw, phys

    if (AK8963_Read_Mag_UT(mag) != 0) // 判断 AK8963_Read_Mag_UT(mag) != 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return IMU_SERVICE_READ_6AXIS_OK; // 将 IMU_SERVICE_READ_6AXIS_OK 变量 返回给调用者
    } // 结束当前代码块

    return IMU_SERVICE_READ_9AXIS_OK; // 将 IMU_SERVICE_READ_9AXIS_OK 变量 返回给调用者
} // 结束当前代码块
