/**
 * @file    bsp_i2c_soft.c
 * @brief   软件模拟 I2C 驱动实现，当前使用 PB6/PB7 连接 MPU9250。
 */
#include "bsp_i2c_soft.h" // 引入 bsp_i2c_soft.h 提供的接口、宏和类型定义

#ifndef I2C_SOFT_DELAY_COUNT // 检查 I2C_SOFT_DELAY_COUNT 是否未定义，防止头文件重复包含
#define I2C_SOFT_DELAY_COUNT 100U // 定义 I2C_SOFT_DELAY_COUNT 变量为 100U
#endif // 结束当前条件编译或头文件保护范围

/**
 * @brief I2C_Delay 函数。
 * @retval None
 */
static void I2C_Delay(void) // 定义I2C_Delay 函数签名：I2C_Delay 函数
{ // 进入当前代码块
    volatile uint32_t i; // 声明 循环索引，供后续计算、状态保存或模块间传递使用

    for (i = 0U; i < I2C_SOFT_DELAY_COUNT; i++) // 按照 i = 0U; i < I2C_SOFT_DELAY_COUNT; i++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        __NOP(); // 调用__NOP 函数
    } // 结束当前代码块
} // 结束当前代码块

/**
 * @brief BSP_I2C_Soft_Init 函数。
 * @retval None
 */
void BSP_I2C_Soft_Init(void) // 定义BSP_I2C_Soft_Init 函数签名：BSP_I2C_Soft_Init 函数
{ // 进入当前代码块
    GPIO_InitTypeDef GPIO_InitStructure; // 执行 GPIO_InitTypeDef GPIO_InitStructure;，完成当前上下文中的具体处理

    RCC_AHB1PeriphClockCmd(I2C_GPIO_CLK, ENABLE); // 调用RCC_AHB1PeriphClockCmd 函数，参数为 I2C_GPIO_CLK, ENABLE

    SCL_H(); // 调用SCL_H 函数
    SDA_H(); // 调用SDA_H 函数

    GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN; // 把 I2C_SCL_PIN | I2C_SDA_PIN 写入 GPIO_InitStructure.GPIO_Pin 字段值
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // 把 GPIO_Mode_OUT 变量 写入 GPIO_InitStructure.GPIO_Mode 字段值
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // 把 GPIO_OType_OD 变量 写入 GPIO_InitStructure.GPIO_OType 字段值
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 把 GPIO_Speed_50MHz 变量 写入 GPIO_InitStructure.GPIO_Speed 字段值
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 把 GPIO_PuPd_UP 变量 写入 GPIO_InitStructure.GPIO_PuPd 字段值
    GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure); // 调用GPIO_Init 函数，参数为 I2C_GPIO_PORT, &GPIO_InitStructure

    SCL_H(); // 调用SCL_H 函数
    SDA_H(); // 调用SDA_H 函数
    I2C_Delay(); // 调用I2C_Delay 函数

    (void)BSP_I2C_Soft_RecoverBus(); // 执行 (void)BSP_I2C_Soft_RecoverBus();，完成当前上下文中的具体处理
} // 结束当前代码块

/**
 * @brief BSP_I2C_Soft_RecoverBus 函数。
 * @retval 函数执行结果或计算得到的返回值。
 */
int BSP_I2C_Soft_RecoverBus(void) // 定义BSP_I2C_Soft_RecoverBus 函数签名：BSP_I2C_Soft_RecoverBus 函数
{ // 进入当前代码块
    uint8_t i; // 声明 循环索引，供后续计算、状态保存或模块间传递使用

    SDA_H(); // 调用SDA_H 函数
    SCL_H(); // 调用SCL_H 函数
    I2C_Delay(); // 调用I2C_Delay 函数

    if (SDA_READ() != 0U) // 判断 SDA_READ() != 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return 0; // 将 0 返回给调用者
    } // 结束当前代码块

    for (i = 0U; i < 9U; i++) // 按照 i = 0U; i < 9U; i++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        SCL_L(); // 调用SCL_L 函数
        I2C_Delay(); // 调用I2C_Delay 函数
        SCL_H(); // 调用SCL_H 函数
        I2C_Delay(); // 调用I2C_Delay 函数

        if (SDA_READ() != 0U) // 判断 SDA_READ() != 0U 是否成立，以选择后续执行路径
        { // 进入当前代码块
            break; // 结束当前循环或分支处理
        } // 结束当前代码块
    } // 结束当前代码块

    I2C_Stop(); // 调用I2C_Stop 函数

    return ((SDA_READ() != 0U) && (SCL_READ() != 0U)) ? 0 : -1; // 将 ((SDA_READ() != 0U) && (SCL_READ() != 0U)) ? 0 : -1 的计算结果 返回给调用者
} // 结束当前代码块

/**
 * @brief I2C_Start 函数。
 * @retval None
 */
void I2C_Start(void) // 定义I2C_Start 函数签名：I2C_Start 函数
{ // 进入当前代码块
    SDA_H(); // 调用SDA_H 函数
    SCL_H(); // 调用SCL_H 函数
    I2C_Delay(); // 调用I2C_Delay 函数
    SDA_L(); // 调用SDA_L 函数
    I2C_Delay(); // 调用I2C_Delay 函数
    SCL_L(); // 调用SCL_L 函数
    I2C_Delay(); // 调用I2C_Delay 函数
} // 结束当前代码块

/**
 * @brief I2C_Stop 函数。
 * @retval None
 */
void I2C_Stop(void) // 定义I2C_Stop 函数签名：I2C_Stop 函数
{ // 进入当前代码块
    SCL_L(); // 调用SCL_L 函数
    SDA_L(); // 调用SDA_L 函数
    I2C_Delay(); // 调用I2C_Delay 函数
    SCL_H(); // 调用SCL_H 函数
    I2C_Delay(); // 调用I2C_Delay 函数
    SDA_H(); // 调用SDA_H 函数
    I2C_Delay(); // 调用I2C_Delay 函数
} // 结束当前代码块

/**
 * @brief I2C_WaitAck 函数。
 * @retval 函数执行结果或计算得到的返回值。
 */
uint8_t I2C_WaitAck(void) // 定义I2C_WaitAck 函数签名：I2C_WaitAck 函数
{ // 进入当前代码块
    uint8_t nack; // 声明 nack 变量，供后续计算、状态保存或模块间传递使用

    SDA_H(); // 调用SDA_H 函数
    I2C_Delay(); // 调用I2C_Delay 函数
    SCL_H(); // 调用SCL_H 函数
    I2C_Delay(); // 调用I2C_Delay 函数

    nack = (SDA_READ() != 0U) ? 1U : 0U; // 把 (SDA_READ() != 0U) ? 1U : 0U 写入 nack 变量

    SCL_L(); // 调用SCL_L 函数
    I2C_Delay(); // 调用I2C_Delay 函数

    return nack; // 将 nack 变量 返回给调用者
} // 结束当前代码块

/**
 * @brief I2C_SendACK 函数。
 * @retval None
 */
void I2C_SendACK(void) // 定义I2C_SendACK 函数签名：I2C_SendACK 函数
{ // 进入当前代码块
    SCL_L(); // 调用SCL_L 函数
    SDA_L(); // 调用SDA_L 函数
    I2C_Delay(); // 调用I2C_Delay 函数
    SCL_H(); // 调用SCL_H 函数
    I2C_Delay(); // 调用I2C_Delay 函数
    SCL_L(); // 调用SCL_L 函数
    I2C_Delay(); // 调用I2C_Delay 函数
    SDA_H(); // 调用SDA_H 函数
} // 结束当前代码块

/**
 * @brief I2C_NACK 函数。
 * @retval None
 */
void I2C_NACK(void) // 定义I2C_NACK 函数签名：I2C_NACK 函数
{ // 进入当前代码块
    SCL_L(); // 调用SCL_L 函数
    SDA_H(); // 调用SDA_H 函数
    I2C_Delay(); // 调用I2C_Delay 函数
    SCL_H(); // 调用SCL_H 函数
    I2C_Delay(); // 调用I2C_Delay 函数
    SCL_L(); // 调用SCL_L 函数
    I2C_Delay(); // 调用I2C_Delay 函数
} // 结束当前代码块

/**
 * @brief I2C_SendNACK 函数。
 * @retval None
 */
void I2C_SendNACK(void) // 定义I2C_SendNACK 函数签名：I2C_SendNACK 函数
{ // 进入当前代码块
    I2C_NACK(); // 调用I2C_NACK 函数
} // 结束当前代码块

/**
 * @brief I2C_SendBit 函数。
 * @param bit bit 变量。
 * @retval None
 */
static void I2C_SendBit(uint8_t bit) // 定义I2C_SendBit 函数签名：I2C_SendBit 函数
{ // 进入当前代码块
    SCL_L(); // 调用SCL_L 函数

    if (bit != 0U) // 判断 bit != 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        SDA_H(); // 调用SDA_H 函数
    } // 结束当前代码块
    else // 处理前面判断条件不成立时的备用逻辑
    { // 进入当前代码块
        SDA_L(); // 调用SDA_L 函数
    } // 结束当前代码块

    I2C_Delay(); // 调用I2C_Delay 函数
    SCL_H(); // 调用SCL_H 函数
    I2C_Delay(); // 调用I2C_Delay 函数
    SCL_L(); // 调用SCL_L 函数
    I2C_Delay(); // 调用I2C_Delay 函数
} // 结束当前代码块

/**
 * @brief I2C_ReadBit 函数。
 * @retval 函数执行结果或计算得到的返回值。
 */
static uint8_t I2C_ReadBit(void) // 定义I2C_ReadBit 函数签名：I2C_ReadBit 函数
{ // 进入当前代码块
    uint8_t bit; // 声明 bit 变量，供后续计算、状态保存或模块间传递使用

    SCL_L(); // 调用SCL_L 函数
    SDA_H(); // 调用SDA_H 函数
    I2C_Delay(); // 调用I2C_Delay 函数
    SCL_H(); // 调用SCL_H 函数
    I2C_Delay(); // 调用I2C_Delay 函数

    bit = (SDA_READ() != 0U) ? 1U : 0U; // 把 (SDA_READ() != 0U) ? 1U : 0U 写入 bit 变量

    SCL_L(); // 调用SCL_L 函数
    I2C_Delay(); // 调用I2C_Delay 函数

    return bit; // 将 bit 变量 返回给调用者
} // 结束当前代码块

/**
 * @brief I2C_SendByte 函数。
 * @param data data 变量。
 * @retval None
 */
void I2C_SendByte(uint8_t data) // 定义I2C_SendByte 函数签名：I2C_SendByte 函数
{ // 进入当前代码块
    uint8_t mask; // 声明 mask 变量，供后续计算、状态保存或模块间传递使用

    for (mask = 0x80U; mask != 0U; mask >>= 1U) // 按照 mask = 0x80U; mask != 0U; mask >>= 1U 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        I2C_SendBit((data & mask) ? 1U : 0U); // 调用I2C_SendBit 函数，参数为 (data & mask) ? 1U : 0U
    } // 结束当前代码块
} // 结束当前代码块

/**
 * @brief I2C_ReadByte 函数。
 * @param send_ack send_ack 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
uint8_t I2C_ReadByte(uint8_t send_ack) // 定义I2C_ReadByte 函数签名：I2C_ReadByte 函数
{ // 进入当前代码块
    uint8_t i; // 声明 循环索引，供后续计算、状态保存或模块间传递使用
    uint8_t data = 0U; // 定义 data 变量，初始值设置为 0

    SDA_H(); // 调用SDA_H 函数

    for (i = 0U; i < 8U; i++) // 按照 i = 0U; i < 8U; i++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        data <<= 1U; // 执行 data <<= 1U;，完成当前上下文中的具体处理
        data |= I2C_ReadBit(); // 执行 data |= I2C_ReadBit();，完成当前上下文中的具体处理
    } // 结束当前代码块

    if (send_ack == I2C_ACK) // 判断 send_ack == I2C_ACK 是否成立，以选择后续执行路径
    { // 进入当前代码块
        I2C_SendACK(); // 调用I2C_SendACK 函数
    } // 结束当前代码块
    else // 处理前面判断条件不成立时的备用逻辑
    { // 进入当前代码块
        I2C_NACK(); // 调用I2C_NACK 函数
    } // 结束当前代码块

    return data; // 将 data 变量 返回给调用者
} // 结束当前代码块

/**
 * @brief I2C_CheckDevice 函数。
 * @param dev7 dev7 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
int I2C_CheckDevice(uint8_t dev7) // 定义I2C_CheckDevice 函数签名：I2C_CheckDevice 函数
{ // 进入当前代码块
    int ret; // 声明 函数返回状态变量，供后续计算、状态保存或模块间传递使用

    I2C_Start(); // 调用I2C_Start 函数
    I2C_SendByte((uint8_t)((dev7 << 1U) | 0U)); // 调用I2C_SendByte 函数，参数为 (uint8_t)((dev7 << 1U) | 0U)
    ret = (I2C_WaitAck() == 0U) ? 0 : -1; // 把 (I2C_WaitAck() == 0U) ? 0 : -1 的计算结果 写入 函数返回状态变量
    I2C_Stop(); // 调用I2C_Stop 函数

    return ret; // 将 函数返回状态变量 返回给调用者
} // 结束当前代码块

/**
 * @brief I2C_WriteReg 函数。
 * @param dev7 dev7 变量。
 * @param reg reg 变量。
 * @param val 寄存器临时读写值。
 * @retval 函数执行结果或计算得到的返回值。
 */
int I2C_WriteReg(uint8_t dev7, uint8_t reg, uint8_t val) // 定义I2C_WriteReg 函数签名：I2C_WriteReg 函数
{ // 进入当前代码块
    return I2C_WriteRegs(dev7, reg, &val, 1U); // 将 I2C_WriteRegs(dev7, reg, &val, 1U) 返回给调用者
} // 结束当前代码块

/**
 * @brief I2C_ReadReg 函数。
 * @param dev7 dev7 变量。
 * @param reg reg 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
uint8_t I2C_ReadReg(uint8_t dev7, uint8_t reg) // 定义I2C_ReadReg 函数签名：I2C_ReadReg 函数
{ // 进入当前代码块
    uint8_t val = 0xFFU; // 定义 寄存器临时读写值，初始值设置为 0xFFU

    (void)I2C_ReadRegData(dev7, reg, &val); // 执行 (void)I2C_ReadRegData(dev7, reg, &val);，完成当前上下文中的具体处理

    return val; // 将 寄存器临时读写值 返回给调用者
} // 结束当前代码块

/**
 * @brief I2C_ReadRegData 函数。
 * @param dev7 dev7 变量。
 * @param reg reg 变量。
 * @param val 寄存器临时读写值。
 * @retval 函数执行结果或计算得到的返回值。
 */
int I2C_ReadRegData(uint8_t dev7, uint8_t reg, uint8_t *val) // 定义I2C_ReadRegData 函数签名：I2C_ReadRegData 函数
{ // 进入当前代码块
    if (val == 0) // 判断 val == 0 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return -4; // 将 -4 的计算结果 返回给调用者
    } // 结束当前代码块

    return I2C_ReadRegs(dev7, reg, val, 1U); // 将 I2C_ReadRegs(dev7, reg, val, 1U) 返回给调用者
} // 结束当前代码块

/**
 * @brief I2C_WriteRegs 函数。
 * @param dev7 dev7 变量。
 * @param reg reg 变量。
 * @param buf buf 变量。
 * @param len len 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
int I2C_WriteRegs(uint8_t dev7, uint8_t reg, const uint8_t *buf, uint16_t len) // 定义I2C_WriteRegs 函数签名：I2C_WriteRegs 函数
{ // 进入当前代码块
    uint16_t i; // 声明 循环索引，供后续计算、状态保存或模块间传递使用

    if ((buf == 0) || (len == 0U)) // 判断 (buf == 0) || (len == 0U) 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return -4; // 将 -4 的计算结果 返回给调用者
    } // 结束当前代码块

    I2C_Start(); // 调用I2C_Start 函数
    I2C_SendByte((uint8_t)((dev7 << 1U) | 0U)); // 调用I2C_SendByte 函数，参数为 (uint8_t)((dev7 << 1U) | 0U)
    if (I2C_WaitAck() != 0U) // 判断 I2C_WaitAck() != 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        I2C_Stop(); // 调用I2C_Stop 函数
        return -1; // 将 -1 的计算结果 返回给调用者
    } // 结束当前代码块

    I2C_SendByte(reg); // 调用I2C_SendByte 函数，参数为 reg
    if (I2C_WaitAck() != 0U) // 判断 I2C_WaitAck() != 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        I2C_Stop(); // 调用I2C_Stop 函数
        return -2; // 将 -2 的计算结果 返回给调用者
    } // 结束当前代码块

    for (i = 0U; i < len; i++) // 按照 i = 0U; i < len; i++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        I2C_SendByte(buf[i]); // 调用I2C_SendByte 函数，参数为 buf[i]
        if (I2C_WaitAck() != 0U) // 判断 I2C_WaitAck() != 0U 是否成立，以选择后续执行路径
        { // 进入当前代码块
            I2C_Stop(); // 调用I2C_Stop 函数
            return -3; // 将 -3 的计算结果 返回给调用者
        } // 结束当前代码块
    } // 结束当前代码块

    I2C_Stop(); // 调用I2C_Stop 函数

    return 0; // 将 0 返回给调用者
} // 结束当前代码块

/**
 * @brief I2C_ReadRegs 函数。
 * @param dev7 dev7 变量。
 * @param reg reg 变量。
 * @param buf buf 变量。
 * @param len len 变量。
 * @retval 函数执行结果或计算得到的返回值。
 */
int I2C_ReadRegs(uint8_t dev7, uint8_t reg, uint8_t *buf, uint16_t len) // 定义I2C_ReadRegs 函数签名：I2C_ReadRegs 函数
{ // 进入当前代码块
    uint16_t i; // 声明 循环索引，供后续计算、状态保存或模块间传递使用
    uint8_t ack; // 声明 ack 变量，供后续计算、状态保存或模块间传递使用

    if ((buf == 0) || (len == 0U)) // 判断 (buf == 0) || (len == 0U) 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return -4; // 将 -4 的计算结果 返回给调用者
    } // 结束当前代码块

    I2C_Start(); // 调用I2C_Start 函数
    I2C_SendByte((uint8_t)((dev7 << 1U) | 0U)); // 调用I2C_SendByte 函数，参数为 (uint8_t)((dev7 << 1U) | 0U)
    if (I2C_WaitAck() != 0U) // 判断 I2C_WaitAck() != 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        I2C_Stop(); // 调用I2C_Stop 函数
        return -1; // 将 -1 的计算结果 返回给调用者
    } // 结束当前代码块

    I2C_SendByte(reg); // 调用I2C_SendByte 函数，参数为 reg
    if (I2C_WaitAck() != 0U) // 判断 I2C_WaitAck() != 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        I2C_Stop(); // 调用I2C_Stop 函数
        return -2; // 将 -2 的计算结果 返回给调用者
    } // 结束当前代码块

    I2C_Start(); // 调用I2C_Start 函数
    I2C_SendByte((uint8_t)((dev7 << 1U) | 1U)); // 调用I2C_SendByte 函数，参数为 (uint8_t)((dev7 << 1U) | 1U)
    if (I2C_WaitAck() != 0U) // 判断 I2C_WaitAck() != 0U 是否成立，以选择后续执行路径
    { // 进入当前代码块
        I2C_Stop(); // 调用I2C_Stop 函数
        return -3; // 将 -3 的计算结果 返回给调用者
    } // 结束当前代码块

    for (i = 0U; i < len; i++) // 按照 i = 0U; i < len; i++ 的初始化、边界和步进条件重复执行循环体
    { // 进入当前代码块
        ack = ((i + 1U) < len) ? I2C_ACK : I2C_NACK_BIT; // 把 ((i + 1U) < len) ? I2C_ACK : I2C_NACK_BIT 的计算结果 写入 ack 变量
        buf[i] = I2C_ReadByte(ack); // 把 I2C_ReadByte(ack) 写入 buf[i]
    } // 结束当前代码块

    I2C_Stop(); // 调用I2C_Stop 函数

    return 0; // 将 0 返回给调用者
} // 结束当前代码块
