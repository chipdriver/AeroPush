#include "app_status.h" // 提供系统状态位操作接口

static volatile AppCalState_t g_app_cal_state = APP_CAL_STATE_IDLE; // 当前校准灯语状态

/**
 * @brief 置位指定系统状态标志。
 * @param status_bits 系统状态位掩码。
 * @retval None
 */
void AppStatus_Set(EventBits_t status_bits) // 设置系统状态位
{
    if (gSystemEventGroup == NULL) // 检查系统事件组是否已创建
    {
        return; // 事件组未创建时不处理
    }

    xEventGroupSetBits(gSystemEventGroup, status_bits); // 置位指定状态位
}

/**
 * @brief 清除指定系统状态标志。
 * @param status_bits 系统状态位掩码。
 * @retval None
 */
void AppStatus_Clear(EventBits_t status_bits) // 清除系统状态位
{
    if (gSystemEventGroup == NULL) // 检查系统事件组是否已创建
    {
        return; // 事件组未创建时不处理
    }

    xEventGroupClearBits(gSystemEventGroup, status_bits); // 清除指定状态位
}

/**
 * @brief 判断指定系统状态标志是否已置位。
 * @param status_bits 系统状态位掩码。
 * @retval 1 已置位；0 未置位或事件组未创建。
 */
uint8_t AppStatus_IsSet(EventBits_t status_bits) // 查询系统状态位
{
    EventBits_t current_bits; // 当前系统状态位快照

    if (gSystemEventGroup == NULL) // 检查系统事件组是否已创建
    {
        return 0; // 事件组未创建时认为未置位
    }

    current_bits = xEventGroupGetBits(gSystemEventGroup); // 读取当前状态位

    if (current_bits & status_bits) // 指定状态位存在
    {
        return 1; // 返回已置位
    }
    else // 指定状态位不存在
    {
        return 0; // 返回未置位
    }
}

/**
 * @brief 读取当前所有系统状态标志。
 * @retval 当前事件组状态位，事件组未创建时返回 0。
 */
EventBits_t AppStatus_GetAll(void) // 读取全部系统状态位
{
    if (gSystemEventGroup == NULL) // 检查系统事件组是否已创建
    {
        return 0; // 事件组未创建时返回空状态
    }

    return xEventGroupGetBits(gSystemEventGroup); // 返回全部状态位
}

/**
 * @brief 设置当前校准灯语状态。
 * @param state 校准灯语状态。
 * @retval None
 */
void AppStatus_SetCalState(AppCalState_t state) // 设置校准灯语状态
{
    g_app_cal_state = state; // 保存当前校准状态
}

/**
 * @brief 读取当前校准灯语状态。
 * @retval 当前校准灯语状态。
 */
AppCalState_t AppStatus_GetCalState(void) // 读取校准灯语状态
{
    return g_app_cal_state; // 返回当前校准状态
}
