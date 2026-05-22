/**
 * @file    bsp_i2c_soft.c
 * @brief   软件模拟 I2C 驱动实现，当前使用 PB6/PB7 连接 MPU9250。
 */
#include "bsp_i2c_soft.h" // 提供软件 I2C 引脚宏和接口声明

#ifndef I2C_SOFT_DELAY_COUNT // 允许外部覆盖软件 I2C 延时长度
#define I2C_SOFT_DELAY_COUNT 100U // 默认软件 I2C 延时循环次数
#endif

/**
 * @brief 产生软件 I2C 的短延时。
 * @retval None
 */
static void I2C_Delay(void) // 软件 I2C 时序延时
{
    volatile uint32_t i; // 延时循环计数
 
    for (i = 0U; i < I2C_SOFT_DELAY_COUNT; i++) // 按配置次数空转
    {
        __NOP(); // 保持一个 CPU 空操作周期
    }
}

/**
 * @brief 初始化软件 I2C 引脚并尝试恢复总线。
 * @retval None
 */
void BSP_I2C_Soft_Init(void) // 初始化软件 I2C 总线
{
    GPIO_InitTypeDef GPIO_InitStructure; // GPIO 初始化结构体

    RCC_AHB1PeriphClockCmd(I2C_GPIO_CLK, ENABLE); // 使能 I2C GPIO 端口时钟

    SCL_H(); // 释放 SCL
    SDA_H(); // 释放 SDA

    GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN; // 配置 SCL 和 SDA 引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // 配置为输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // 配置为开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 配置 GPIO 翻转速度
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 配置内部上拉
    GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure); // 应用 GPIO 配置

    SCL_H(); // 初始化后释放 SCL
    SDA_H(); // 初始化后释放 SDA
    I2C_Delay(); // 等待总线电平稳定

    (void)BSP_I2C_Soft_RecoverBus(); // 尝试恢复可能被拉低的总线
}

/**
 * @brief 通过最多 9 个时钟脉冲释放被从机占用的 I2C 总线。
 * @retval 0 总线空闲；-1 总线仍异常。
 */
int BSP_I2C_Soft_RecoverBus(void) // 恢复软件 I2C 总线
{
    uint8_t i; // 时钟脉冲计数

    SDA_H(); // 释放 SDA
    SCL_H(); // 释放 SCL
    I2C_Delay(); // 等待总线电平稳定

    if (SDA_READ() != 0U) // SDA 已经为高说明总线空闲
    {
        return 0; // 无需恢复
    }

    for (i = 0U; i < 9U; i++) // 最多补 9 个 SCL 脉冲
    {
        SCL_L(); // 拉低 SCL
        I2C_Delay(); // 保持低电平时间
        SCL_H(); // 拉高 SCL
        I2C_Delay(); // 保持高电平时间

        if (SDA_READ() != 0U) // 从机释放 SDA 后可以停止补脉冲
        {
            break; // 总线已恢复
        }
    }

    I2C_Stop(); // 发送停止条件让从机回到空闲状态

    return ((SDA_READ() != 0U) && (SCL_READ() != 0U)) ? 0 : -1; // 检查 SCL/SDA 是否都回到高电平
}

/**
 * @brief 发送 I2C 起始条件。
 * @retval None
 */
void I2C_Start(void) // 产生 I2C START
{
    SDA_H(); // 确保 SDA 先处于高电平
    SCL_H(); // 确保 SCL 先处于高电平
    I2C_Delay(); // 保持总线空闲时间
    SDA_L(); // SCL 高电平期间拉低 SDA
    I2C_Delay(); // 保持起始条件
    SCL_L(); // 拉低 SCL 准备发送数据
    I2C_Delay(); // 保持低电平时间
}

/**
 * @brief 发送 I2C 停止条件。
 * @retval None
 */
void I2C_Stop(void) // 产生 I2C STOP
{
    SCL_L(); // 先保证 SCL 为低
    SDA_L(); // 先保证 SDA 为低
    I2C_Delay(); // 保持低电平时间
    SCL_H(); // 拉高 SCL
    I2C_Delay(); // 等待 SCL 稳定
    SDA_H(); // SCL 高电平期间释放 SDA
    I2C_Delay(); // 保持停止条件
}

/**
 * @brief 等待从机 ACK。
 * @retval 0 收到 ACK；1 收到 NACK。
 */
uint8_t I2C_WaitAck(void) // 读取从机应答位
{
    uint8_t nack; // 应答结果，1 表示未应答

    SDA_H(); // 释放 SDA 让从机驱动
    I2C_Delay(); // 等待 SDA 稳定
    SCL_H(); // 拉高 SCL 采样应答位
    I2C_Delay(); // 保持采样窗口

    nack = (SDA_READ() != 0U) ? 1U : 0U; // SDA 高为 NACK，低为 ACK

    SCL_L(); // 拉低 SCL 结束应答位
    I2C_Delay(); // 保持低电平时间

    return nack; // 返回应答状态
}

/**
 * @brief 主机发送 ACK。
 * @retval None
 */
void I2C_SendACK(void) // 发送 ACK 位
{
    SCL_L(); // 拉低 SCL 准备应答位
    SDA_L(); // SDA 拉低表示 ACK
    I2C_Delay(); // 保持 SDA 稳定
    SCL_H(); // 拉高 SCL 发送应答位
    I2C_Delay(); // 保持高电平时间
    SCL_L(); // 拉低 SCL 结束应答位
    I2C_Delay(); // 保持低电平时间
    SDA_H(); // 释放 SDA
}

/**
 * @brief 主机发送 NACK。
 * @retval None
 */
void I2C_NACK(void) // 发送 NACK 位
{
    SCL_L(); // 拉低 SCL 准备应答位
    SDA_H(); // SDA 释放为高表示 NACK
    I2C_Delay(); // 保持 SDA 稳定
    SCL_H(); // 拉高 SCL 发送应答位
    I2C_Delay(); // 保持高电平时间
    SCL_L(); // 拉低 SCL 结束应答位
    I2C_Delay(); // 保持低电平时间
}

/**
 * @brief 主机发送 NACK。
 * @retval None
 */
void I2C_SendNACK(void) // 发送 NACK 位的兼容接口
{
    I2C_NACK(); // 复用 NACK 时序
}

/**
 * @brief 发送 1 个 I2C 数据位。
 * @param bit 需要发送的位值，0 或非 0。
 * @retval None
 */
static void I2C_SendBit(uint8_t bit) // 发送单个数据位
{
    SCL_L(); // SCL 低电平期间准备 SDA

    if (bit != 0U) // 当前位为 1
    {
        SDA_H(); // 释放 SDA 发送 1
    }
    else // 当前位为 0
    {
        SDA_L(); // 拉低 SDA 发送 0
    }

    I2C_Delay(); // 等待 SDA 稳定
    SCL_H(); // 拉高 SCL 让从机采样
    I2C_Delay(); // 保持高电平时间
    SCL_L(); // 拉低 SCL 结束当前位
    I2C_Delay(); // 保持低电平时间
}

/**
 * @brief 读取 1 个 I2C 数据位。
 * @retval 读取到的位值，0 或 1。
 */
static uint8_t I2C_ReadBit(void) // 读取单个数据位
{
    uint8_t bit; // 采样到的位值

    SCL_L(); // SCL 低电平期间准备读位
    SDA_H(); // 释放 SDA 让从机驱动
    I2C_Delay(); // 等待 SDA 稳定
    SCL_H(); // 拉高 SCL 采样数据位
    I2C_Delay(); // 保持采样窗口

    bit = (SDA_READ() != 0U) ? 1U : 0U; // 读取 SDA 电平

    SCL_L(); // 拉低 SCL 结束当前位
    I2C_Delay(); // 保持低电平时间

    return bit; // 返回采样结果
}

/**
 * @brief 发送 1 个字节，高位先发。
 * @param data 需要发送的字节。
 * @retval None
 */
void I2C_SendByte(uint8_t data) // 发送一个 I2C 字节
{
    uint8_t mask; // 当前发送位掩码

    for (mask = 0x80U; mask != 0U; mask >>= 1U) // 从 bit7 到 bit0 发送
    {
        I2C_SendBit((data & mask) ? 1U : 0U); // 发送当前数据位
    }
}

/**
 * @brief 读取 1 个字节，高位先读。
 * @param send_ack 读完后是否发送 ACK。
 * @retval 读取到的字节。
 */
uint8_t I2C_ReadByte(uint8_t send_ack) // 读取一个 I2C 字节
{
    uint8_t i; // 位计数
    uint8_t data = 0U; // 接收数据缓存

    SDA_H(); // 释放 SDA 让从机输出

    for (i = 0U; i < 8U; i++) // 连续读取 8 个数据位
    {
        data <<= 1U; // 为下一位腾出最低位
        data |= I2C_ReadBit(); // 读入当前位
    }

    if (send_ack == I2C_ACK) // 调用方要求继续读取
    {
        I2C_SendACK(); // 发送 ACK
    }
    else // 调用方要求结束读取
    {
        I2C_NACK(); // 发送 NACK
    }

    return data; // 返回接收字节
}

/**
 * @brief 探测 7 位 I2C 设备地址是否有应答。
 * @param dev7 7 位 I2C 设备地址。
 * @retval 0 有应答；-1 无应答。
 */
int I2C_CheckDevice(uint8_t dev7) // 检查 I2C 设备是否在线
{
    int ret; // 探测结果

    I2C_Start(); // 发送起始条件
    I2C_SendByte((uint8_t)((dev7 << 1U) | 0U)); // 发送设备地址和写方向
    ret = (I2C_WaitAck() == 0U) ? 0 : -1; // 根据 ACK 判断设备是否存在
    I2C_Stop(); // 发送停止条件

    return ret; // 返回探测结果
}

/**
 * @brief 写 1 个设备寄存器。
 * @param dev7 7 位 I2C 设备地址。
 * @param reg 寄存器地址。
 * @param val 写入值。
 * @retval 0 写入成功；负数表示失败阶段。
 */
int I2C_WriteReg(uint8_t dev7, uint8_t reg, uint8_t val) // 写单个寄存器
{
    return I2C_WriteRegs(dev7, reg, &val, 1U); // 复用连续寄存器写接口
}

/**
 * @brief 读 1 个设备寄存器。
 * @param dev7 7 位 I2C 设备地址。
 * @param reg 寄存器地址。
 * @retval 读取值，失败时保持默认 0xFF。
 */
uint8_t I2C_ReadReg(uint8_t dev7, uint8_t reg) // 读单个寄存器
{
    uint8_t val = 0xFFU; // 读取失败时的默认值

    (void)I2C_ReadRegData(dev7, reg, &val); // 读取寄存器数据

    return val; // 返回寄存器值
}

/**
 * @brief 读 1 个设备寄存器并返回错误码。
 * @param dev7 7 位 I2C 设备地址。
 * @param reg 寄存器地址。
 * @param val 输出读取值的指针。
 * @retval 0 读取成功；负数表示失败阶段。
 */
int I2C_ReadRegData(uint8_t dev7, uint8_t reg, uint8_t *val) // 读单个寄存器并返回状态
{
    if (val == 0) // 检查输出指针
    {
        return -4; // 输出指针为空
    }

    return I2C_ReadRegs(dev7, reg, val, 1U); // 复用连续寄存器读接口
}

/**
 * @brief 从指定寄存器开始连续写入多个字节。
 * @param dev7 7 位 I2C 设备地址。
 * @param reg 起始寄存器地址。
 * @param buf 待写入数据缓冲区。
 * @param len 写入字节数。
 * @retval 0 写入成功；负数表示失败阶段。
 */
int I2C_WriteRegs(uint8_t dev7, uint8_t reg, const uint8_t *buf, uint16_t len) // 连续写寄存器
{
    uint16_t i; // 写入字节索引

    if ((buf == 0) || (len == 0U)) // 检查输入缓冲区和长度
    {
        return -4; // 参数无效
    }

    I2C_Start(); // 发送起始条件
    I2C_SendByte((uint8_t)((dev7 << 1U) | 0U)); // 发送设备地址和写方向
    if (I2C_WaitAck() != 0U) // 等待设备地址 ACK
    {
        I2C_Stop(); // 地址无应答时结束传输
        return -1; // 设备地址阶段失败
    }

    I2C_SendByte(reg); // 发送起始寄存器地址
    if (I2C_WaitAck() != 0U) // 等待寄存器地址 ACK
    {
        I2C_Stop(); // 寄存器地址无应答时结束传输
        return -2; // 寄存器地址阶段失败
    }

    for (i = 0U; i < len; i++) // 逐字节写入数据
    {
        I2C_SendByte(buf[i]); // 发送当前数据字节
        if (I2C_WaitAck() != 0U) // 等待数据字节 ACK
        {
            I2C_Stop(); // 数据无应答时结束传输
            return -3; // 数据阶段失败
        }
    }

    I2C_Stop(); // 发送停止条件

    return 0; // 写入成功
}

/**
 * @brief 从指定寄存器开始连续读取多个字节。
 * @param dev7 7 位 I2C 设备地址。
 * @param reg 起始寄存器地址。
 * @param buf 接收数据缓冲区。
 * @param len 读取字节数。
 * @retval 0 读取成功；负数表示失败阶段。
 */
int I2C_ReadRegs(uint8_t dev7, uint8_t reg, uint8_t *buf, uint16_t len) // 连续读寄存器
{
    uint16_t i; // 读取字节索引
    uint8_t ack; // 当前字节后的应答方式

    if ((buf == 0) || (len == 0U)) // 检查输出缓冲区和长度
    {
        return -4; // 参数无效
    }

    I2C_Start(); // 发送起始条件
    I2C_SendByte((uint8_t)((dev7 << 1U) | 0U)); // 发送设备地址和写方向
    if (I2C_WaitAck() != 0U) // 等待设备地址 ACK
    {
        I2C_Stop(); // 地址无应答时结束传输
        return -1; // 设备地址写阶段失败
    }

    I2C_SendByte(reg); // 发送待读取的寄存器地址
    if (I2C_WaitAck() != 0U) // 等待寄存器地址 ACK
    {
        I2C_Stop(); // 寄存器地址无应答时结束传输
        return -2; // 寄存器地址阶段失败
    }

    I2C_Start(); // 发送重复起始条件
    I2C_SendByte((uint8_t)((dev7 << 1U) | 1U)); // 发送设备地址和读方向
    if (I2C_WaitAck() != 0U) // 等待读方向地址 ACK
    {
        I2C_Stop(); // 地址无应答时结束传输
        return -3; // 设备地址读阶段失败
    }

    for (i = 0U; i < len; i++) // 逐字节读取数据
    {
        ack = ((i + 1U) < len) ? I2C_ACK : I2C_NACK_BIT; // 最后一个字节后发送 NACK
        buf[i] = I2C_ReadByte(ack); // 读取当前数据字节
    }

    I2C_Stop(); // 发送停止条件

    return 0; // 读取成功
}
