#include "bsp_led.h"                                      // 引入 LED BSP 接口

/**
  * @brief  初始化板载 LED GPIO。
  * @param  None
  * @retval None
  */
 void BSP_LED_Init(void)                                // 定义 LED 初始化函数
 {                                                      // BSP_LED_Init 函数体开始
   GPIO_InitTypeDef GPIO_InitStructure;                 // 定义 GPIO 初始化结构体
   
   //1.使能时钟
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); // 使能 GPIOA 外设时钟

   //2.构建结构体
   GPIO_InitStructure.GPIO_Pin     =   LED_RED_Pin | LED_GREEN_Pin; // 选择红灯和绿灯引脚
   GPIO_InitStructure.GPIO_Mode    =   GPIO_Mode_OUT;               // 配置为普通输出模式
   GPIO_InitStructure.GPIO_OType   =   GPIO_OType_PP;               // 配置为推挽输出
   GPIO_InitStructure.GPIO_Speed   =   GPIO_Speed_50MHz;            // 配置 GPIO 速度为 50MHz
   GPIO_InitStructure.GPIO_PuPd    =   GPIO_PuPd_NOPULL;            // 配置为无上下拉

   GPIO_Init(LED_RED_Port,&GPIO_InitStructure);          // 初始化红灯所在 GPIO 端口
	GPIO_Init(LED_GREEN_Port,&GPIO_InitStructure);        // 初始化绿灯所在 GPIO 端口
   
   //3.设置 LED 初始状态
   BSP_LED_Off();                                        // 初始化完成后默认关闭 LED

 }                                                       // BSP_LED_Init 函数体结束

 /**
  * @brief  打开板载 LED。
  * @param  None
  * @retval None
  */
 void BSP_LED_On(void)                                  // 定义 LED 打开函数
 {                                                      // BSP_LED_On 函数体开始
    GPIO_SetBits(LED_RED_Port,LED_RED_Pin);             // 将红灯引脚置位
    GPIO_WriteBit(LED_GREEN_Port,LED_GREEN_Pin,Bit_SET); // 将绿灯引脚置位
 }                                                      // BSP_LED_On 函数体结束


 /**
  * @brief  关闭板载 LED。
  * @param  None
  * @retval None
  */
 void BSP_LED_Off(void)                                 // 定义 LED 关闭函数
 {                                                      // BSP_LED_Off 函数体开始
    //GPIO_SetBits(LED_Port,LED_Pin);  方式1
    GPIO_WriteBit(LED_RED_Port,LED_RED_Pin,Bit_RESET);  // 将红灯引脚复位
    GPIO_ResetBits(LED_GREEN_Port,LED_GREEN_Pin);       // 将绿灯引脚复位
 }                                                      // BSP_LED_Off 函数体结束

 /**
  * @brief  翻转板载 LED 当前状态。
  * @param  None
  * @retval None
  */
 void BSP_LED_Toggle(void)                              // 定义 LED 翻转函数
 {                                                      // BSP_LED_Toggle 函数体开始
    LED_RED_Port->ODR ^= LED_RED_Pin;                   // 通过异或翻转红灯输出状态
    LED_GREEN_Port->ODR ^= LED_GREEN_Pin;               // 通过异或翻转绿灯输出状态
 }                                                      // BSP_LED_Toggle 函数体结束
