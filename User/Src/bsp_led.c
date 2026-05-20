#include "bsp_led.h" // 提供 LED GPIO 宏和底层控制接口

/**
 * @brief 初始化红绿 LED 对应 GPIO。
 * @retval None
 */
void BSP_LED_Init(void) // 初始化 LED GPIO
{
    GPIO_InitTypeDef GPIO_InitStructure; // GPIO 初始化结构体

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能 GPIOA 时钟

    GPIO_InitStructure.GPIO_Pin = LED_RED_Pin | LED_GREEN_Pin; // 配置红灯和绿灯引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // 配置为普通输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 配置为推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 配置 GPIO 翻转速度
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不使用内部上下拉

    GPIO_Init(LED_RED_Port, &GPIO_InitStructure); // 初始化红灯 GPIO
    GPIO_Init(LED_GREEN_Port, &GPIO_InitStructure); // 初始化绿灯 GPIO

    BSP_LED_Off(); // 默认关闭 LED
}

/**
 * @brief 打开红绿 LED。
 * @retval None
 */
void BSP_LED_On(void) // 打开 LED
{
    GPIO_SetBits(LED_RED_Port, LED_RED_Pin); // 置位红灯引脚
    GPIO_WriteBit(LED_GREEN_Port, LED_GREEN_Pin, Bit_SET); // 置位绿灯引脚
}

/**
 * @brief 关闭红绿 LED。
 * @retval None
 */
void BSP_LED_Off(void) // 关闭 LED
{
    GPIO_WriteBit(LED_RED_Port, LED_RED_Pin, Bit_RESET); // 复位红灯引脚
    GPIO_ResetBits(LED_GREEN_Port, LED_GREEN_Pin); // 复位绿灯引脚
}

/**
 * @brief 翻转红绿 LED。
 * @retval None
 */
void BSP_LED_Toggle(void) // 翻转 LED 状态
{
    LED_RED_Port->ODR ^= LED_RED_Pin; // 翻转红灯输出位
    LED_GREEN_Port->ODR ^= LED_GREEN_Pin; // 翻转绿灯输出位
}
