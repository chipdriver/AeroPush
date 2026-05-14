#include "imu_service.h"                                   // 引入 IMU 服务模块接口
#include "app_config.h"                                    // 引入应用配置参数
#include "mpu9250_driver.h"                                // 引入 MPU9250 驱动接口
#include "debug_log.h"                                     // 引入调试日志输出接口

/**
 * @brief  初始化 IMU 服务。
 * @param  None
 * @retval 1 表示初始化成功，0 表示初始化失败。
 */
uint8_t ImuService_Init(void)                              // 定义 IMU 服务初始化函数
{                                                          // ImuService_Init 函数体开始
    uint8_t ret;                                           // 保存 MPU9250 驱动初始化返回值

    Debug_Print("[ImuService] init start\r\n");            // 打印 IMU 服务初始化开始日志

    ret = MPU9250_Driver_Init();                           // 调用 MPU9250 驱动初始化函数
    if(ret == 0)                                           // 判断 MPU9250 驱动初始化是否失败
    {                                                      // 初始化失败分支开始
        Debug_Print("[ImuService] init failed\r\n");       // 打印 IMU 服务初始化失败日志
        return 0;                                          // 返回 0 表示初始化失败
    }                                                      // 初始化失败分支结束

    Debug_Print("[ImuService] init ok\r\n");               // 打印 IMU 服务初始化成功日志
    return 1;                                              // 返回 1 表示初始化成功
}                                                          // ImuService_Init 函数体结束

/**
 * @brief  构造一帧模拟姿态数据。
 * @param  attitude 用于保存模拟姿态数据的结构体指针。
 * @retval None
 */
void ImuService_BuildSimAttitude(AttitudeData_t *attitude) // 定义模拟姿态数据构造函数
{                                                          // ImuService_BuildSimAttitude 函数体开始
    static float angle = 0.0f;                             // 保存模拟姿态角，函数多次调用时持续累加

    if(attitude == NULL)                                   // 判断输出参数是否为空
        return;                                            // 参数为空时直接返回

    angle += APP_SIM_ATTITUDE_STEP_DEG;                    // 按配置步长增加模拟姿态角

    if(angle >= APP_SIM_ATTITUDE_MAX_DEG)                  // 判断模拟角度是否达到最大值
    {                                                      // 角度回绕分支开始
        angle = 0.0f;                                      // 达到最大值后从 0 重新开始
    }                                                      // 角度回绕分支结束

    memset(attitude,0,sizeof(AttitudeData_t));             // 清空姿态结构体，避免残留旧数据

    attitude->roll_deg = angle;                            // 写入模拟 roll 角
    attitude->pitch_deg = angle + 1.0f;                    // 写入模拟 pitch 角
    attitude->yaw_deg = angle + 2.0f;                      // 写入模拟 yaw 角
    attitude->timestamp_ms = xTaskGetTickCount();          // 写入当前系统 tick 作为时间戳
    attitude->valid = 1;                                   // 设置姿态数据有效标志
}                                                          // ImuService_BuildSimAttitude 函数体结束

void ImuService_ReadRaw(MPU9250_raw_Data *raw)
{
    if(raw == NULL)                                   // 判断输出参数是否为空
        return;                                            // 参数为空时直接返回

    MPU9250_ReadAxis(raw);                           // 调用 MPU9250 读取六轴原始数据函数

    return;                                              // 返回 1 表示读取成功
}

/**
 * @brief 读取 MPU9250 物理量数据。
 * @param phys 用于保存物理量数据的结构体指针。
 * @retval 1 表示读取成功，0 表示读取失败。
 */
uint8_t ImuService_ReadPhys(MPU9250_Physical_Data *phys)
{
    MPU9250_raw_Data raw;                         // 定义一个 MPU9250 原始数据变量

    if(phys == NULL)                                   // 判断输出参数是否为空
        return 0;                                            // 参数为空时直接返回读取失败

    MPU9250_ReadAxis(&raw);                           // 调用 MPU9250 读取六轴原始数据函数

    MPU9250_ConvertToPhysical(&raw,phys);            // 将原始数据转换成物理量并保存到输出参数指向的结构体中

    return 1;                                              // 返回 1 表示读取成功
}