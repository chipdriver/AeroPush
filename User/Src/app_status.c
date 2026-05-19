#include "app_status.h" // 引入 app_status.h 提供的接口、宏和类型定义

/**
 * @brief 置位指定系统状态标志。
 * @param status_bits 系统状态位掩码。
 * @retval None
 */
void AppStatus_Set(EventBits_t status_bits) // 定义AppStatus_Set 函数签名：置位指定系统状态标志
{ // 进入当前代码块
    if (gSystemEventGroup == NULL) // 判断 gSystemEventGroup == NULL 是否成立，以选择后续执行路径
        return; // 当前条件不满足继续处理，直接返回调用者

    xEventGroupSetBits(gSystemEventGroup, status_bits); // 调用xEventGroupSetBits 函数，参数为 gSystemEventGroup, status_bits

} // 结束当前代码块

/**
 * @brief 清除指定系统状态标志。
 * @param status_bits 系统状态位掩码。
 * @retval None
 */
void AppStatus_Clear(EventBits_t status_bits) // 定义AppStatus_Clear 函数签名：清除指定系统状态标志
{ // 进入当前代码块
    if (gSystemEventGroup == NULL) // 判断 gSystemEventGroup == NULL 是否成立，以选择后续执行路径
        return; // 当前条件不满足继续处理，直接返回调用者

    xEventGroupClearBits(gSystemEventGroup, status_bits); // 调用xEventGroupClearBits 函数，参数为 gSystemEventGroup, status_bits
} // 结束当前代码块

/**
 * @brief 判断指定系统状态标志是否已置位。
 * @param status_bits 系统状态位掩码。
 * @retval 函数执行结果或计算得到的返回值。
 */
uint8_t AppStatus_IsSet(EventBits_t status_bits) // 定义AppStatus_IsSet 函数签名：判断指定系统状态标志是否已置位
{ // 进入当前代码块
    if (gSystemEventGroup == NULL) // 判断 gSystemEventGroup == NULL 是否成立，以选择后续执行路径
        return 0; // 将 0 返回给调用者

    EventBits_t current_bits = xEventGroupGetBits(gSystemEventGroup); // 定义 current_bits 变量，初始值设置为 xEventGroupGetBits(gSystemEventGroup)

    if (current_bits & status_bits) // 判断 current_bits & status_bits 是否成立，以选择后续执行路径
        return 1; // 将 1 返回给调用者
    else // 处理前面判断条件不成立时的备用逻辑
        return 0; // 将 0 返回给调用者
} // 结束当前代码块

/**
 * @brief 读取当前所有系统状态标志。
 * @retval 函数执行结果或计算得到的返回值。
 */
EventBits_t AppStatus_GetAll(void) // 定义AppStatus_GetAll 函数签名：读取当前所有系统状态标志
{ // 进入当前代码块
    if (gSystemEventGroup == NULL) // 判断 gSystemEventGroup == NULL 是否成立，以选择后续执行路径
        return 0; // 将 0 返回给调用者

    return xEventGroupGetBits(gSystemEventGroup); // 将 xEventGroupGetBits(gSystemEventGroup) 返回给调用者
} // 结束当前代码块
