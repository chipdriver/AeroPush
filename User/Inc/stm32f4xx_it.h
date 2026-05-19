#ifndef __STM32F4xx_IT_H // 检查 __STM32F4xx_IT_H 是否未定义，防止头文件重复包含
#define __STM32F4xx_IT_H // 定义 __STM32F4xx_IT_H 变量

#ifdef __cplusplus // 根据芯片型号或编译选项选择参与编译的代码
extern "C" // 执行 extern "C"，完成当前上下文中的具体处理
{ // 进入当前代码块
#endif // 结束当前条件编译或头文件保护范围

#include "stm32f4xx.h" // 引入 stm32f4xx.h 提供的接口、宏和类型定义

    /**
     * @brief NMI_Handler 函数。
     * @retval None
     */
    void NMI_Handler(void); // 声明NMI_Handler 函数签名：NMI_Handler 函数
    /**
     * @brief HardFault_Handler 函数。
     * @retval None
     */
    void HardFault_Handler(void); // 声明HardFault_Handler 函数签名：HardFault_Handler 函数
    /**
     * @brief MemManage_Handler 函数。
     * @retval None
     */
    void MemManage_Handler(void); // 声明MemManage_Handler 函数签名：MemManage_Handler 函数
    /**
     * @brief BusFault_Handler 函数。
     * @retval None
     */
    void BusFault_Handler(void); // 声明BusFault_Handler 函数签名：BusFault_Handler 函数
    /**
     * @brief UsageFault_Handler 函数。
     * @retval None
     */
    void UsageFault_Handler(void); // 声明UsageFault_Handler 函数签名：UsageFault_Handler 函数
    /**
     * @brief DebugMon_Handler 函数。
     * @retval None
     */
    void DebugMon_Handler(void); // 声明DebugMon_Handler 函数签名：DebugMon_Handler 函数

#ifdef __cplusplus // 根据芯片型号或编译选项选择参与编译的代码
} // 结束当前代码块
#endif // 结束当前条件编译或头文件保护范围

#endif // 结束当前条件编译或头文件保护范围
