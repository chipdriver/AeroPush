#include "bsp_led.h" // 引入 bsp_led.h 提供的接口、宏和类型定义

/**
 * @brief BSP_LED_Init 函数。
 * @retval None
 */
void BSP_LED_Init(void) // 定义BSP_LED_Init 函数签名：BSP_LED_Init 函数
{ // 进入当前代码块
    GPIO_InitTypeDef GPIO_InitStructure; // 执行 GPIO_InitTypeDef GPIO_InitStructure;，完成当前上下文中的具体处理

    // 1.使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 调用RCC_AHB1PeriphClockCmd 函数，参数为 RCC_AHB1Periph_GPIOA, ENABLE

    // 2.构建结构体
    GPIO_InitStructure.GPIO_Pin = LED_RED_Pin | LED_GREEN_Pin; // 把 LED_RED_Pin | LED_GREEN_Pin 写入 GPIO_InitStructure.GPIO_Pin 字段值
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // 把 GPIO_Mode_OUT 变量 写入 GPIO_InitStructure.GPIO_Mode 字段值
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 把 GPIO_OType_PP 变量 写入 GPIO_InitStructure.GPIO_OType 字段值
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 把 GPIO_Speed_50MHz 变量 写入 GPIO_InitStructure.GPIO_Speed 字段值
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 把 GPIO_PuPd_NOPULL 变量 写入 GPIO_InitStructure.GPIO_PuPd 字段值

    GPIO_Init(LED_RED_Port, &GPIO_InitStructure); // 调用GPIO_Init 函数，参数为 LED_RED_Port, &GPIO_InitStructure
    GPIO_Init(LED_GREEN_Port, &GPIO_InitStructure); // 调用GPIO_Init 函数，参数为 LED_GREEN_Port, &GPIO_InitStructure

    // 3.设置 LED 初始状态
    BSP_LED_Off(); // 调用BSP_LED_Off 函数

} // 结束当前代码块

/**
 * @brief BSP_LED_On 函数。
 * @retval None
 */
void BSP_LED_On(void) // 定义BSP_LED_On 函数签名：BSP_LED_On 函数
{ // 进入当前代码块
    GPIO_SetBits(LED_RED_Port, LED_RED_Pin); // 调用GPIO_SetBits 函数，参数为 LED_RED_Port, LED_RED_Pin
    GPIO_WriteBit(LED_GREEN_Port, LED_GREEN_Pin, Bit_SET); // 调用GPIO_WriteBit 函数，参数为 LED_GREEN_Port, LED_GREEN_Pin, Bit_SET
} // 结束当前代码块

/**
 * @brief BSP_LED_Off 函数。
 * @retval None
 */
void BSP_LED_Off(void) // 定义BSP_LED_Off 函数签名：BSP_LED_Off 函数
{ // 进入当前代码块
    // GPIO_SetBits(LED_Port,LED_Pin);  方式1
    GPIO_WriteBit(LED_RED_Port, LED_RED_Pin, Bit_RESET); // 调用GPIO_WriteBit 函数，参数为 LED_RED_Port, LED_RED_Pin, Bit_RESET
    GPIO_ResetBits(LED_GREEN_Port, LED_GREEN_Pin); // 调用GPIO_ResetBits 函数，参数为 LED_GREEN_Port, LED_GREEN_Pin
} // 结束当前代码块

/**
 * @brief BSP_LED_Toggle 函数。
 * @retval None
 */
void BSP_LED_Toggle(void) // 定义BSP_LED_Toggle 函数签名：BSP_LED_Toggle 函数
{ // 进入当前代码块
    LED_RED_Port->ODR ^= LED_RED_Pin; // 执行 LED_RED_Port->ODR ^= LED_RED_Pin;，完成当前上下文中的具体处理
    LED_GREEN_Port->ODR ^= LED_GREEN_Pin; // 执行 LED_GREEN_Port->ODR ^= LED_GREEN_Pin;，完成当前上下文中的具体处理
} // 结束当前代码块
