#include "app_status.h"                                      // 引入系统状态管理模块接口

/**
  * @brief  设置系统状态位。
  * @param  status_bits 要设置的状态位，可以是单个位或多个位的组合。
  * @retval None
  */
 void AppStatus_Set(EventBits_t status_bits)                  // 定义系统状态位置位函数
 {                                                            // AppStatus_Set 函数体开始
    if(gSystemEventGroup == NULL)                             // 判断系统事件组是否已经创建
        return;                                               // 如果事件组未创建，直接返回

    xEventGroupSetBits(gSystemEventGroup,status_bits);        // 设置指定的系统状态位
    
 }                                                            // AppStatus_Set 函数体结束

/**
  * @brief  清除系统状态位。
  * @param  status_bits 要清除的状态位，可以是单个位或多个位的组合。
  * @retval None
  */
 void AppStatus_Clear(EventBits_t status_bits)                // 定义系统状态位清除函数
 {                                                            // AppStatus_Clear 函数体开始
    if(gSystemEventGroup == NULL)                             // 判断系统事件组是否已经创建
        return;                                               // 如果事件组未创建，直接返回

    xEventGroupClearBits(gSystemEventGroup, status_bits);     // 清除指定的系统状态位
 }                                                            // AppStatus_Clear 函数体结束

 /**
  * @brief  检查系统状态位是否已经设置。
  * @param  status_bits 要检查的状态位，可以是单个位或多个位的组合。
  * @retval 1 表示已设置，0 表示未设置。
  */
 uint8_t AppStatus_IsSet(EventBits_t status_bits)             // 定义系统状态位查询函数
 {                                                            // AppStatus_IsSet 函数体开始
    if(gSystemEventGroup == NULL)                             // 判断系统事件组是否已经创建
        return 0;                                             // 如果事件组未创建，直接返回未设置

    EventBits_t current_bits = xEventGroupGetBits(gSystemEventGroup); // 获取当前全部系统状态位

   if( current_bits & status_bits )                           // 判断目标状态位是否存在于当前状态位中
        return 1;                                             // 如果指定的状态位被设置，返回 1
    else
        return 0;                                             // 否则返回 0
 }                                                            // AppStatus_IsSet 函数体结束

/**
  * @brief  获取当前全部系统状态位。
  * @param  None
  * @retval 当前所有状态位的值。
  */
 EventBits_t AppStatus_GetAll(void)                           // 定义获取全部系统状态位函数
 {                                                            // AppStatus_GetAll 函数体开始
    if(gSystemEventGroup == NULL)                             // 判断系统事件组是否已经创建
        return 0;                                             // 如果事件组未创建，直接返回 0

    return xEventGroupGetBits(gSystemEventGroup);             // 返回当前所有系统状态位
 }                                                            // AppStatus_GetAll 函数体结束

