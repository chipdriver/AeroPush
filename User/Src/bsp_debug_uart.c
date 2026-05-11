#include "bsp_debug_uart.h"

/**
 * @brief USART6 Init
 * @param 无
 * @retval 无
 */
void BSP_DebugUart_Init(void)
{
    //初始化结构体
    GPIO_InitTypeDef GPIO_InitStructure;    //IO口结构体
    USART_InitTypeDef USART_InitStructure;  //USART结构体
    //1.使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOA

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART

    //2.先配置PA11 / PA12 为USART6 复用功能
    GPIO_PinAFConfig(USART6_Port,GPIO_PinSource11,GPIO_AF_USART6);
    GPIO_PinAFConfig(USART6_Port,GPIO_PinSource12,GPIO_AF_USART6);

    //3.配置IO参数
    GPIO_InitStructure.GPIO_Pin = USART6_TX_Pin | USART6_RX_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    //复用功能模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    //无上下拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //50MHZ输出速度
    GPIO_Init(USART6_Port,&GPIO_InitStructure);

    //4.配置USART6参数
    USART_InitStructure.USART_BaudRate = 115200;    //USART6 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //字长
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  //停止位
    USART_InitStructure.USART_Parity    =   USART_Parity_No;    //无校验位
    USART_InitStructure.USART_Mode  =   USART_Mode_Rx | USART_Mode_Tx;  //同时开启接收和发送功能
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件流控制

    USART_Init(USART6,&USART_InitStructure);

    //5.使能USART6
    USART_Cmd(USART6,ENABLE);

}

/**
 * @brief 发送一个字符
 * @param ch:要发送的字符
 * @retval 无
 */
void BSP_DebugUart_SendChar(char ch)
{
    while(USART_GetFlagStatus(USART6,USART_FLAG_TXE) == RESET); //通过检查状态寄存器 SR 里的 TXE 标志位，来判断发送数据寄存器 DR 是否为空。

    USART_SendData(USART6,(uint16_t)ch);
}

/**
 * @brief 发送字符串
 * @param 
 * @retval
 */
void BSP_DebugUart_SendString(const char *str)
{
    while(*str != '\0')
    {
        BSP_DebugUart_SendChar(*str++);
    }
}