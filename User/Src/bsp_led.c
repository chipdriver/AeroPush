#include "bsp_led.h"

/**
  * @brief  LED 初始化
  * @param  无
  * @retval 无
  */
 void BSP_LED_Init(void)
 {
   GPIO_InitTypeDef GPIO_InitStructure;    //引脚初始化结构体
   
   //1.使能时钟
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);    //使能GPIOC外设时钟

   //2.构建结构体
   GPIO_InitStructure.GPIO_Pin     =   LED_RED_Pin | LED_GREEN_Pin;
   GPIO_InitStructure.GPIO_Mode    =   GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType   =   GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed   =   GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_PuPd    =   GPIO_PuPd_NOPULL;

   GPIO_Init(LED_RED_Port,&GPIO_InitStructure);
	GPIO_Init(LED_GREEN_Port,&GPIO_InitStructure);
   
   //3.设置 LED 初始状态
   BSP_LED_Off();

 }

 /**
  * @brief  LED 打开
  * @param  无
  * @retval 无
  */
 void BSP_LED_On(void)
 {
    GPIO_SetBits(LED_RED_Port,LED_RED_Pin);
    GPIO_WriteBit(LED_GREEN_Port,LED_GREEN_Pin,Bit_SET);
 }


 /**
  * @brief  LED 关闭
  * @param  无
  * @retval 无
  */
 void BSP_LED_Off(void)
 {
    //GPIO_SetBits(LED_Port,LED_Pin);  方式1
    GPIO_WriteBit(LED_RED_Port,LED_RED_Pin,Bit_RESET);    //方式2
    GPIO_ResetBits(LED_GREEN_Port,LED_GREEN_Pin);
 }

 /**
  * @brief  LED 状态翻转
  * @param  无
  * @retval 无
  */
 void BSP_LED_Toggle(void)
 {
    LED_RED_Port->ODR ^= LED_RED_Pin;
    LED_GREEN_Port->ODR ^= LED_GREEN_Pin;
 }