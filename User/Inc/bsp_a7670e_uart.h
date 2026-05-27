#ifndef __BSP_A7670E_UART_H__ // 防止头文件重复包含
#define __BSP_A7670E_UART_H__

#include "stm32f4xx.h" // 提供 STM32F4 外设定义
#include "stm32f4xx_conf.h" // 提供标准外设库配置
#include "stm32f4xx_usart.h" // 提供 USART 外设接口

#define A7670E_UART_PORT GPIOA // A7670E 串口使用 GPIOA
#define A7670E_UART_TX_PIN GPIO_Pin_9 // PA9 作为 USART1_TX
#define A7670E_UART_RX_PIN GPIO_Pin_10 // PA10 作为 USART1_RX
#define A7670E_UART_RX_BUF_SIZE 1024U // A7670E 接收环形缓冲区大小

/**
 * @brief 初始化 A7670E 使用的 USART1 引脚和串口外设。
 * @retval None
 */
void BSP_A7670E_Uart_Init(void); // 初始化 A7670E 串口

/**
 * @brief 通过 USART1 向 A7670E 发送 1 个字节。
 * @param data 待发送字节。
 * @retval None
 */
void BSP_A7670E_Uart_SendByte(uint8_t data); // 发送 1 个字节到 A7670E

/**
 * @brief 从 A7670E 接收环形缓冲区读取 1 个字节。
 * @param data 接收字节输出指针。
 * @retval 1U 表示收到字节，0U 表示无数据或参数无效。
 */
uint8_t BSP_A7670E_Uart_ReceiveByte(uint8_t *data); // 从环形缓冲区读取 1 个字节

/**
 * @brief 通过 USART1 向 A7670E 发送字符串。
 * @param str 以 '\0' 结尾的字符串。
 * @retval None
 */
void BSP_A7670E_Uart_SendString(const char *str); // 发送字符串到 A7670E

/**
 * @brief 查询 A7670E 接收环形缓冲区中可读字节数。
 * @retval 当前可读字节数。
 */
uint16_t BSP_A7670E_Uart_RxAvailable(void); // 查询环形缓冲区已有字节数

/**
 * @brief 读取 A7670E 接收环形缓冲区溢出标志。
 * @retval 1U 表示发生过接收溢出，0U 表示未溢出。
 */
uint8_t BSP_A7670E_Uart_GetOverflow(void); // 读取接收溢出标志

/**
 * @brief 清空 A7670E 接收环形缓冲区。
 * @retval None
 */
void BSP_A7670E_Uart_RxClear(void); // 清空环形缓冲区

/**
 * @brief 处理 A7670E USART1 接收中断。
 * @retval None
 */
void BSP_A7670E_Uart_IRQHandler(void); // 由 USART1_IRQHandler 调用

#endif // __BSP_A7670E_UART_H__
