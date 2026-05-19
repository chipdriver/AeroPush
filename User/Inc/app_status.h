#ifndef __APP_STATUS_H__ // 检查 __APP_STATUS_H__ 是否未定义，防止头文件重复包含
#define __APP_STATUS_H__ // 定义 __APP_STATUS_H__ 变量

#include <stdint.h> // 引入 stdint.h 提供的接口、宏和类型定义
#include "FreeRTOS.h" // 引入 FreeRTOS.h 提供的接口、宏和类型定义
#include "event_groups.h" // 引入 event_groups.h 提供的接口、宏和类型定义
#include "freertos_objects.h" // 引入 freertos_objects.h 提供的接口、宏和类型定义

#define APP_STATUS_IMU_READY (1 << 0) // 定义 APP_STATUS_IMU_READY 系统状态位为 (1 << 0)
#define APP_STATUS_GNSS_READY (1 << 1) // 定义 APP_STATUS_GNSS_READY 系统状态位为 (1 << 1)
#define APP_STATUS_GNSS_FIX (1 << 2) // 定义 APP_STATUS_GNSS_FIX 系统状态位为 (1 << 2)
#define APP_STATUS_NET_READY (1 << 3) // 定义 APP_STATUS_NET_READY 系统状态位为 (1 << 3)
#define APP_STATUS_MQTT_READY (1 << 4) // 定义 APP_STATUS_MQTT_READY 系统状态位为 (1 << 4)
#define APP_STATUS_IMU_ERROR (1 << 8) // 定义 APP_STATUS_IMU_ERROR 系统状态位为 (1 << 8)
#define APP_STATUS_MODEM_ERROR (1 << 9) // 定义 APP_STATUS_MODEM_ERROR 系统状态位为 (1 << 9)
#define APP_STATUS_MQTT_ERROR (1 << 10) // 定义 APP_STATUS_MQTT_ERROR 系统状态位为 (1 << 10)

/**
 * @brief 置位指定系统状态标志。
 * @param status_bits 系统状态位掩码。
 * @retval None
 */
void AppStatus_Set(EventBits_t status_bits); // 声明AppStatus_Set 函数签名：置位指定系统状态标志
/**
 * @brief 清除指定系统状态标志。
 * @param status_bits 系统状态位掩码。
 * @retval None
 */
void AppStatus_Clear(EventBits_t status_bits); // 声明AppStatus_Clear 函数签名：清除指定系统状态标志
/**
 * @brief 判断指定系统状态标志是否已置位。
 * @param status_bits 系统状态位掩码。
 * @retval 函数执行结果或计算得到的返回值。
 */
uint8_t AppStatus_IsSet(EventBits_t status_bits); // 声明AppStatus_IsSet 函数签名：判断指定系统状态标志是否已置位
/**
 * @brief 读取当前所有系统状态标志。
 * @retval 函数执行结果或计算得到的返回值。
 */
EventBits_t AppStatus_GetAll(void); // 声明AppStatus_GetAll 函数签名：读取当前所有系统状态标志

#endif // 结束当前条件编译或头文件保护范围
