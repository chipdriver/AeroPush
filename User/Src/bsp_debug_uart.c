#include "bsp_debug_uart.h" // 引入调试串口 BSP 接口

/**
 * @brief  初始化 USART6 调试串口。
 * @param  None
 * @retval None
 */
void BSP_DebugUart_Init(void) // 定义 USART6 调试串口初始化函数
{                             // BSP_DebugUart_Init 函数体开始
    // 初始化结构体
    GPIO_InitTypeDef GPIO_InitStructure;   // 定义 GPIO 初始化结构体
    USART_InitTypeDef USART_InitStructure; // 定义 USART 初始化结构体
    // 1.使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能 GPIOA 外设时钟

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); // 使能 USART6 外设时钟

    // 2.先配置PA11 / PA12 为USART6 复用功能
    GPIO_PinAFConfig(USART6_Port, GPIO_PinSource11, GPIO_AF_USART6); // 将 PA11 复用为 USART6_TX
    GPIO_PinAFConfig(USART6_Port, GPIO_PinSource12, GPIO_AF_USART6); // 将 PA12 复用为 USART6_RX

    // 3.配置IO参数
    GPIO_InitStructure.GPIO_Pin = USART6_TX_Pin | USART6_RX_Pin; // 选择 USART6 TX 和 RX 引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                 // 配置为复用功能模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               // 配置为推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;             // 配置为无上下拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;            // 配置 GPIO 速度为 50MHz
    GPIO_Init(USART6_Port, &GPIO_InitStructure);                 // 按上述参数初始化 USART6 引脚

    // 4.配置USART6参数
    USART_InitStructure.USART_BaudRate = 115200;                                    // 配置 USART6 波特率为 115200
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 配置 8 位数据长度
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // 配置 1 个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // 配置无校验位
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 // 同时开启接收和发送功能
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 配置无硬件流控制

    USART_Init(USART6, &USART_InitStructure); // 按上述参数初始化 USART6

    // 5.使能USART6
    USART_Cmd(USART6, ENABLE); // 使能 USART6

} // BSP_DebugUart_Init 函数体结束

/**
 * @brief  通过 USART6 发送一个字符。
 * @param  ch 要发送的字符。
 * @retval None
 */
void BSP_DebugUart_SendChar(char ch) // 定义 USART6 单字符发送函数
{                                    // BSP_DebugUart_SendChar 函数体开始
    while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET)                          // 开始循环执行
        ; // 通过检查状态寄存器 SR 里的 TXE 标志位，来判断发送数据寄存器 DR 是否为空。

    USART_SendData(USART6, (uint16_t)ch); // 将字符写入 USART6 数据寄存器并发送
} // BSP_DebugUart_SendChar 函数体结束

/**
 * @brief  通过 USART6 发送字符串。
 * @param  str 指向待发送字符串的指针。
 * @retval None
 */
void BSP_DebugUart_SendString(const char *str) // 定义 USART6 字符串发送函数
{                                              // BSP_DebugUart_SendString 函数体开始
    while (*str != '\0')                       // 循环直到遇到字符串结束符
    {                                          // 字符串发送循环开始
        BSP_DebugUart_SendChar(*str++);        // 发送当前字符并让指针指向下一个字符
    } // 字符串发送循环结束
} // BSP_DebugUart_SendString 函数体结束
