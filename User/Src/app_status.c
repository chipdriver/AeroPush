#include "app_status.h"

/**
  * @brief  设置状态位
  * @param  status_bits 要设置的状态位，可以是单个位或多个位的组合
  * @retval 无
  */
 void AppStatus_Set(EventBits_t status_bits)
 {
    if(gSystemEventGroup == NULL)
        return; // 如果事件组未创建，直接返回

    xEventGroupSetBits(gSystemEventGroup,status_bits); // 设置指定的状态位
    
 }

/**
  * @brief  清除状态位
  * @param  status_bits 要清除的状态位，可以是单个位或多个位的组合
  * @retval 无
  */
 void AppStatus_Clear(EventBits_t status_bits)
 {
    if(gSystemEventGroup == NULL)
        return; // 如果事件组未创建，直接返回

    xEventGroupClearBits(gSystemEventGroup, status_bits); // 清除指定的状态位
 }

 /**
  * @brief  检查状态位是否设置
  * @param  status_bits 要检查的状态位，可以是单个位或多个位的组合
  * @retval 1表示设置，0表示未设置
  */
 uint8_t AppStatus_IsSet(EventBits_t status_bits)
 {
    if(gSystemEventGroup == NULL)
        return 0; // 如果事件组未创建，直接返回未设置

    EventBits_t current_bits = xEventGroupGetBits(gSystemEventGroup); // 获取当前状态位

   if( current_bits & status_bits )
        return 1; // 如果指定的状态位被设置，返回 1
    else
        return 0; // 否则返回 0
 }

/**
  * @brief  获取所有状态位
  * @param  无
  * @retval 当前所有状态位的值
  */
 EventBits_t AppStatus_GetAll(void)
 {
    if(gSystemEventGroup == NULL)
        return 0; // 如果事件组未创建，直接返回 0

    return xEventGroupGetBits(gSystemEventGroup); // 返回当前所有状态位
 }

 