#include "bsp_debug_uart.h" // 引入 bsp_debug_uart.h 提供的接口、宏和类型定义

/**
 * @brief 初始化底层调试串口外设。
 * @retval None
 */
void BSP_DebugUart_Init(void) // 定义BSP_DebugUart_Init 函数签名：初始化底层调试串口外设
{ // 进入当前代码块
    // 初始化结构体
    GPIO_InitTypeDef GPIO_InitStructure; // 执行 GPIO_InitTypeDef GPIO_InitStructure;，完成当前上下文中的具体处理
    USART_InitTypeDef USART_InitStructure; // 执行 USART_InitTypeDef USART_InitStructure;，完成当前上下文中的具体处理
    // 1.使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 调用RCC_AHB1PeriphClockCmd 函数，参数为 RCC_AHB1Periph_GPIOA, ENABLE

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); // 调用RCC_APB2PeriphClockCmd 函数，参数为 RCC_APB2Periph_USART6, ENABLE

    // 2.先配置PA11 / PA12 为USART6 复用功能
    GPIO_PinAFConfig(USART6_Port, GPIO_PinSource11, GPIO_AF_USART6); // 调用GPIO_PinAFConfig 函数，参数为 USART6_Port, GPIO_PinSource11, GPIO_AF_USART6
    GPIO_PinAFConfig(USART6_Port, GPIO_PinSource12, GPIO_AF_USART6); // 调用GPIO_PinAFConfig 函数，参数为 USART6_Port, GPIO_PinSource12, GPIO_AF_USART6

    // 3.配置IO参数
    GPIO_InitStructure.GPIO_Pin = USART6_TX_Pin | USART6_RX_Pin; // 把 USART6_TX_Pin | USART6_RX_Pin 写入 GPIO_InitStructure.GPIO_Pin 字段值
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // 把 GPIO_Mode_AF 变量 写入 GPIO_InitStructure.GPIO_Mode 字段值
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 把 GPIO_OType_PP 变量 写入 GPIO_InitStructure.GPIO_OType 字段值
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 把 GPIO_PuPd_NOPULL 变量 写入 GPIO_InitStructure.GPIO_PuPd 字段值
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 把 GPIO_Speed_50MHz 变量 写入 GPIO_InitStructure.GPIO_Speed 字段值
    GPIO_Init(USART6_Port, &GPIO_InitStructure); // 调用GPIO_Init 函数，参数为 USART6_Port, &GPIO_InitStructure

    // 4.配置USART6参数
    USART_InitStructure.USART_BaudRate = 115200; // 把 115200 写入 USART_InitStructure.USART_BaudRate 字段值
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 把 USART_WordLength_8b 变量 写入 USART_InitStructure.USART_WordLength 字段值
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // 把 USART_StopBits_1 变量 写入 USART_InitStructure.USART_StopBits 字段值
    USART_InitStructure.USART_Parity = USART_Parity_No; // 把 USART_Parity_No 变量 写入 USART_InitStructure.USART_Parity 字段值
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 把 USART_Mode_Rx | USART_Mode_Tx 写入 USART_InitStructure.USART_Mode 字段值
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 把 USART_HardwareFlowControl_None 变量 写入 USART_InitStructure.USART_HardwareFlowControl 字段值

    USART_Init(USART6, &USART_InitStructure); // 调用USART_Init 函数，参数为 USART6, &USART_InitStructure

    // 5.使能USART6
    USART_Cmd(USART6, ENABLE); // 调用USART_Cmd 函数，参数为 USART6, ENABLE

} // 结束当前代码块

/**
 * @brief 通过调试串口发送单个字符。
 * @param ch ch 变量。
 * @retval None
 */
void BSP_DebugUart_SendChar(char ch) // 定义BSP_DebugUart_SendChar 函数签名：通过调试串口发送单个字符
{ // 进入当前代码块
    while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET) // 当 USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET 成立时持续执行循环体
        ; // 执行 ;，完成当前上下文中的具体处理

    USART_SendData(USART6, (uint16_t)ch); // 调用USART_SendData 函数，参数为 USART6, (uint16_t)ch
} // 结束当前代码块

/**
 * @brief 通过调试串口发送字符串。
 * @param str str 变量。
 * @retval None
 */
void BSP_DebugUart_SendString(const char *str) // 定义BSP_DebugUart_SendString 函数签名：通过调试串口发送字符串
{ // 进入当前代码块
    while (*str != '\0') // 当 *str != '\0' 成立时持续执行循环体
    { // 进入当前代码块
        BSP_DebugUart_SendChar(*str++); // 调用通过调试串口发送单个字符，参数为 *str++
    } // 结束当前代码块
} // 结束当前代码块
