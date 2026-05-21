#ifndef __APP_STATUS_H__ // 防止头文件重复包含
#define __APP_STATUS_H__

#include <stdint.h> // 提供固定宽度整数类型
#include "FreeRTOS.h" // 提供 FreeRTOS 基础类型
#include "event_groups.h" // 提供事件组类型和接口
#include "freertos_objects.h" // 提供系统事件组对象

#define APP_STATUS_IMU_READY (1 << 0) // IMU 已就绪
#define APP_STATUS_GNSS_READY (1 << 1) // GNSS 服务已就绪
#define APP_STATUS_GNSS_FIX (1 << 2) // GNSS 已定位
#define APP_STATUS_NET_READY (1 << 3) // 网络已就绪
#define APP_STATUS_MQTT_READY (1 << 4) // MQTT 已就绪
#define APP_STATUS_IMU_ERROR (1 << 8) // IMU 错误
#define APP_STATUS_MODEM_ERROR (1 << 9) // 通信模块错误
#define APP_STATUS_MQTT_ERROR (1 << 10) // MQTT 错误

typedef enum // 启动标定和运行阶段的用户提示状态
{
    APP_CAL_STATE_IDLE = 0, // 非标定状态
    APP_CAL_STATE_SELF_CHECK, // 上电自检
    APP_CAL_STATE_STATIC, // 陀螺仪和加速度计静止校准
    APP_CAL_STATE_MAG_ROTATE, // 磁力计旋转校准
    APP_CAL_STATE_SUCCESS, // 标定成功
    APP_CAL_STATE_FAIL // 标定失败
} AppCalState_t;

/**
 * @brief 置位指定系统状态标志。
 * @param status_bits 系统状态位掩码。
 * @retval None
 */
void AppStatus_Set(EventBits_t status_bits); // 设置系统状态位

/**
 * @brief 清除指定系统状态标志。
 * @param status_bits 系统状态位掩码。
 * @retval None
 */
void AppStatus_Clear(EventBits_t status_bits); // 清除系统状态位

/**
 * @brief 判断指定系统状态标志是否已置位。
 * @param status_bits 系统状态位掩码。
 * @retval 1 已置位；0 未置位。
 */
uint8_t AppStatus_IsSet(EventBits_t status_bits); // 查询系统状态位

/**
 * @brief 读取当前所有系统状态标志。
 * @retval 当前事件组状态位。
 */
EventBits_t AppStatus_GetAll(void); // 读取全部系统状态位

/**
 * @brief 设置当前校准灯语状态。
 * @param state 校准灯语状态。
 * @retval None
 */
void AppStatus_SetCalState(AppCalState_t state); // 设置校准灯语状态

/**
 * @brief 读取当前校准灯语状态。
 * @retval 当前校准灯语状态。
 */
AppCalState_t AppStatus_GetCalState(void); // 读取校准灯语状态

#endif // __APP_STATUS_H__
