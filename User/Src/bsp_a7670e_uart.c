#include "bsp_a7670e_uart.h" // 提供 A7670E 串口 GPIO/USART 配置接口

/**
 * @brief 初始化 A7670E 使用的 USART1 引脚和串口外设。
 * @retval None
 */
void BSP_A7670E_Uart_Init(void) // 初始化 USART1，PA9 发送、PA10 接收
{
    GPIO_InitTypeDef GPIO_InitStructure; // GPIO 初始化结构体
    USART_InitTypeDef USART_InitStructure; // USART 初始化结构体

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能 GPIOA 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // 使能 USART1 时钟

    GPIO_PinAFConfig(A7670E_UART_PORT, GPIO_PinSource9, GPIO_AF_USART1); // PA9 复用为 USART1_TX
    GPIO_PinAFConfig(A7670E_UART_PORT, GPIO_PinSource10, GPIO_AF_USART1); // PA10 复用为 USART1_RX

    GPIO_InitStructure.GPIO_Pin = A7670E_UART_TX_PIN | A7670E_UART_RX_PIN; // 配置 PA9/PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // 配置为复用功能模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 配置为推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉保持串口空闲高电平
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 配置 GPIO 翻转速度
    GPIO_Init(A7670E_UART_PORT, &GPIO_InitStructure); // 应用 GPIO 配置

    USART_InitStructure.USART_BaudRate = 115200; // 设置 A7670E 默认调试波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 设置 8 位数据位
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // 设置 1 位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; // 关闭奇偶校验
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 使能收发模式
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 关闭硬件流控
    USART_Init(USART1, &USART_InitStructure); // 应用 USART1 配置

    USART_Cmd(USART1, ENABLE); // 使能 USART1
}
