#include "bsp_debug_uart.h" // 提供调试串口 GPIO/USART 配置和发送接口

/**
 * @brief 初始化底层调试串口外设。
 * @retval None
 */
void BSP_DebugUart_Init(void) // 初始化 USART6 调试串口
{
    GPIO_InitTypeDef GPIO_InitStructure; // GPIO 初始化结构体
    USART_InitTypeDef USART_InitStructure; // USART 初始化结构体

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能 GPIOA 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); // 使能 USART6 时钟

    GPIO_PinAFConfig(USART6_Port, GPIO_PinSource11, GPIO_AF_USART6); // PA11 复用为 USART6 TX
    GPIO_PinAFConfig(USART6_Port, GPIO_PinSource12, GPIO_AF_USART6); // PA12 复用为 USART6 RX

    GPIO_InitStructure.GPIO_Pin = USART6_TX_Pin | USART6_RX_Pin; // 配置 USART6 TX/RX 引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // 配置为复用功能模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 配置为推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不使用内部上下拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 配置 GPIO 翻转速度
    GPIO_Init(USART6_Port, &GPIO_InitStructure); // 应用 GPIO 配置

    USART_InitStructure.USART_BaudRate = 115200; // 设置波特率 115200
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 设置 8 位数据位
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // 设置 1 位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; // 关闭奇偶校验
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 使能收发模式
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 关闭硬件流控

    USART_Init(USART6, &USART_InitStructure); // 应用 USART6 配置

    USART_Cmd(USART6, ENABLE); // 使能 USART6
} 

/**
 * @brief 通过调试串口发送单个字符。
 * @param ch 待发送字符。
 * @retval None
 */
void BSP_DebugUart_SendChar(char ch) // 发送一个调试字符
{
    while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET) // 等待发送数据寄存器为空
        ; // 串口硬件尚未准备好

    USART_SendData(USART6, (uint16_t)ch); // 写入待发送字符
}

/**
 * @brief 通过调试串口发送字符串。
 * @param str 以 '\0' 结尾的字符串。
 * @retval None
 */
void BSP_DebugUart_SendString(const char *str) // 发送调试字符串
{
    while (*str != '\0') // 遍历字符串直到结束符
    {
        BSP_DebugUart_SendChar(*str++); // 逐字符发送
    }
}
