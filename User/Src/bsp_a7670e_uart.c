#include "bsp_a7670e_uart.h" // 提供 A7670E 串口 GPIO/USART 配置接口
#include <stddef.h> // 提供 NULL 定义

static volatile uint8_t s_a7670e_rx_buf[A7670E_UART_RX_BUF_SIZE]; // A7670E 接收环形缓冲区
static volatile uint16_t s_a7670e_rx_write = 0U; // 环形缓冲区写指针，由中断更新
static volatile uint16_t s_a7670e_rx_read = 0U; // 环形缓冲区读指针，由任务更新
static volatile uint8_t s_a7670e_rx_overflow = 0U; // 接收溢出标志

/**
 * @brief 计算 A7670E 接收环形缓冲区的下一个下标。
 * @param index 当前下标。
 * @retval 回绕后的下一个下标。
 */
static uint16_t BSP_A7670E_RxNextIndex(uint16_t index) // 计算环形缓冲区下一个位置
{
    index++; // 移动到下一个缓冲区位置

    if (index >= A7670E_UART_RX_BUF_SIZE) // 到达缓冲区末尾后需要回绕
    {
        index = 0U; // 回到缓冲区起点
    }

    return index; // 返回计算后的下标
}

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

    BSP_A7670E_Uart_RxClear(); // 初始化时清空接收环形缓冲区

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 开启 USART1 接收非空中断

    NVIC_SetPriority(USART1_IRQn, 6U); // 设置 USART1 中断优先级低于 FreeRTOS 系统调用中断
    NVIC_EnableIRQ(USART1_IRQn); // 使能 USART1 中断通道

    USART_Cmd(USART1, ENABLE); // 使能 USART1
}

/**
 * @brief 通过 USART1 向 A7670E 发送 1 个字节。
 * @param data 待发送字节。
 * @retval None
 */
void BSP_A7670E_Uart_SendByte(uint8_t data) // 发送一个 A7670E 串口字节
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) // 等待发送数据寄存器为空
    {
    } // 串口硬件尚未准备好时保持等待

    USART_SendData(USART1, data); // 写入待发送字节到 USART1
}

/**
 * @brief 从 A7670E 接收环形缓冲区读取 1 个字节。
 * @param data 接收字节输出指针。
 * @retval 1U 表示收到字节，0U 表示无数据或参数无效。
 */
uint8_t BSP_A7670E_Uart_ReceiveByte(uint8_t *data) // 从环形缓冲区读取一个 A7670E 字节
{
    if (data == NULL) // 检查输出指针是否有效
    {
        return 0U; // 空指针不读取数据
    }

    if (s_a7670e_rx_read == s_a7670e_rx_write) // 读写指针相同表示缓冲区为空
    {
        return 0U; // 当前没有可读数据
    }

    *data = s_a7670e_rx_buf[s_a7670e_rx_read]; // 取出读指针位置的数据

    s_a7670e_rx_read = BSP_A7670E_RxNextIndex(s_a7670e_rx_read); // 读指针前移一格

    return 1U; // 成功读取 1 个字节
}

/**
 * @brief 通过 USART1 向 A7670E 发送字符串。
 * @param str 以 '\0' 结尾的字符串。
 * @retval None
 */
void BSP_A7670E_Uart_SendString(const char *str) // 发送 A7670E 串口字符串
{
    if (str == NULL) // 检查字符串指针是否有效
    {
        return; // 空指针不发送
    }

    while (*str != '\0') // 遍历字符串直到结束符
    {
        BSP_A7670E_Uart_SendByte((uint8_t)(*str)); // 逐字节发送当前字符
        str++; // 移动到下一个字符
    }
}

/**
 * @brief 查询 A7670E 接收环形缓冲区中可读字节数。
 * @retval 当前可读字节数。
 */
uint16_t BSP_A7670E_Uart_RxAvailable(void) // 查询环形缓冲区已有字节数
{
    uint16_t write_index; // 写指针快照
    uint16_t read_index; // 读指针快照

    write_index = s_a7670e_rx_write; // 读取当前写指针
    read_index = s_a7670e_rx_read; // 读取当前读指针

    if (write_index >= read_index) // 未回绕时写指针在读指针后方
    {
        return (uint16_t)(write_index - read_index); // 直接相减得到可读数量
    }

    return (uint16_t)(A7670E_UART_RX_BUF_SIZE - read_index + write_index); // 回绕后分段计算可读数量
}

/**
 * @brief 清空 A7670E 接收环形缓冲区。
 * @retval None
 */
void BSP_A7670E_Uart_RxClear(void) // 清空环形缓冲区
{
    s_a7670e_rx_read = 0U; // 读指针归零
    s_a7670e_rx_write = 0U; // 写指针归零
    s_a7670e_rx_overflow = 0U; // 清除溢出标志
}

/**
 * @brief 处理 A7670E USART1 接收中断。
 * @retval None
 */
void BSP_A7670E_Uart_IRQHandler(void) // A7670E USART1 接收中断处理函数
{
    uint8_t data; // 保存本次收到的字节
    uint16_t next_write; // 保存写指针的下一个位置

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // 判断是否为 RXNE 接收非空中断
    {
        data = (uint8_t)USART_ReceiveData(USART1); // 读取 DR 并清除 RXNE 标志

        next_write = BSP_A7670E_RxNextIndex(s_a7670e_rx_write); // 计算写入后的下一个位置

        if (next_write != s_a7670e_rx_read) // 下一个写位置未追上读指针表示未满
        {
            s_a7670e_rx_buf[s_a7670e_rx_write] = data; // 写入当前收到的字节
            s_a7670e_rx_write = next_write; // 更新写指针
        }
        else // 缓冲区已满
        {
            s_a7670e_rx_overflow = 1U; // 标记溢出并丢弃当前字节
        }
    }
}
