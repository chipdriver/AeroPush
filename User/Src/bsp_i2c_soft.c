/**
 * @file    bsp_i2c_soft.c
 * @brief   软件模拟 I2C 驱动实现，当前使用 PB6/PB7 连接 MPU9250。
 */
#include "bsp_i2c_soft.h"                  /* 引入软件 I2C 接口和引脚宏 */

#ifndef I2C_SOFT_DELAY_COUNT               /* 允许外部通过编译宏覆盖延时循环次数 */
#define I2C_SOFT_DELAY_COUNT  100U         /* 默认延时循环次数，决定软件 I2C 时序速度 */
#endif                                     /* 结束延时循环次数宏判断 */

/**
 * @brief  软件 I2C 短延时。
 * @param  None
 * @retval None
 * @note   用于保证 SCL/SDA 电平切换后有足够建立和保持时间。
 */
static void I2C_Delay(void)                /* 定义内部短延时函数 */
{                                          /* 函数体开始 */
    volatile uint32_t i;                   /* 使用 volatile 防止循环被优化掉 */

    for (i = 0U; i < I2C_SOFT_DELAY_COUNT; i++) /* 按固定次数执行空操作 */
    {                                      /* 循环体开始 */
        __NOP();                           /* 执行一个空指令拉开时序 */
    }                                      /* 循环体结束 */
}                                          /* 函数体结束 */

/**
 * @brief  初始化软件 I2C GPIO。
 * @param  None
 * @retval None
 * @note   PB6 作为 SCL，PB7 作为 SDA，均配置为开漏输出并打开内部上拉。
 */
void BSP_I2C_Soft_Init(void)               /* 定义软件 I2C 初始化函数 */
{                                          /* 函数体开始 */
    GPIO_InitTypeDef GPIO_InitStructure;   /* 定义 GPIO 初始化结构体 */

    RCC_AHB1PeriphClockCmd(I2C_GPIO_CLK, ENABLE); /* 使能 GPIOB 外设时钟 */

    SCL_H();                               /* 配置 GPIO 前先释放 SCL */
    SDA_H();                               /* 配置 GPIO 前先释放 SDA */

    GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN; /* 选择 PB6 和 PB7 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; /* 配置为普通输出模式 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; /* 配置为开漏输出，符合 I2C 总线特性 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /* 设置 GPIO 输出速度 */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; /* 打开内部上拉，外部上拉存在时也兼容 */
    GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure); /* 初始化 GPIOB 的 I2C 引脚 */

    SCL_H();                               /* 初始化后释放 SCL 为空闲高电平 */
    SDA_H();                               /* 初始化后释放 SDA 为空闲高电平 */
    I2C_Delay();                           /* 等待总线电平稳定 */

    (void)BSP_I2C_Soft_RecoverBus();       /* 尝试恢复可能被从机拉住的总线 */
}                                          /* 函数体结束 */

/**
 * @brief  尝试恢复被从机拉住的数据总线。
 * @param  None
 * @retval 0 表示恢复成功，负数表示恢复失败。
 * @note   当 SDA 被从机保持低电平时，最多输出 9 个 SCL 脉冲让从机释放总线。
 */
int BSP_I2C_Soft_RecoverBus(void)          /* 定义软件 I2C 总线恢复函数 */
{                                          /* 函数体开始 */
    uint8_t i;                             /* 定义 SCL 恢复脉冲计数变量 */

    SDA_H();                               /* 释放 SDA，让上拉电阻拉高数据线 */
    SCL_H();                               /* 释放 SCL，让上拉电阻拉高时钟线 */
    I2C_Delay();                           /* 等待总线电平稳定 */

    if (SDA_READ() != 0U)                  /* 如果 SDA 已经为高，说明总线没有被占住 */
    {                                      /* if 分支开始 */
        return 0;                          /* 直接返回恢复成功 */
    }                                      /* if 分支结束 */

    for (i = 0U; i < 9U; i++)              /* 最多输出 9 个时钟脉冲 */
    {                                      /* 循环体开始 */
        SCL_L();                           /* 拉低 SCL 产生时钟低电平 */
        I2C_Delay();                       /* 保持 SCL 低电平一小段时间 */
        SCL_H();                           /* 释放 SCL 产生时钟高电平 */
        I2C_Delay();                       /* 保持 SCL 高电平一小段时间 */

        if (SDA_READ() != 0U)              /* 检查从机是否已经释放 SDA */
        {                                  /* if 分支开始 */
            break;                         /* 已释放则提前结束恢复脉冲 */
        }                                  /* if 分支结束 */
    }                                      /* 循环体结束 */

    I2C_Stop();                            /* 发送 STOP，让总线回到空闲状态 */

    return ((SDA_READ() != 0U) && (SCL_READ() != 0U)) ? 0 : -1; /* 判断 SCL/SDA 是否都已回到高电平 */
}                                          /* 函数体结束 */

/**
 * @brief  产生 I2C 起始条件。
 * @param  None
 * @retval None
 * @note   当 SCL 为高电平时，SDA 从高到低跳变表示 START。
 */
void I2C_Start(void)                       /* 定义 I2C 起始条件函数 */
{                                          /* 函数体开始 */
    SDA_H();                               /* 先释放 SDA，确保数据线为高 */
    SCL_H();                               /* 再释放 SCL，确保时钟线为高 */
    I2C_Delay();                           /* 等待总线稳定在空闲状态 */
    SDA_L();                               /* 在 SCL 高电平时拉低 SDA，产生 START */
    I2C_Delay();                           /* 保持起始条件一小段时间 */
    SCL_L();                               /* 拉低 SCL，准备后续传输数据位 */
    I2C_Delay();                           /* 等待 SCL 低电平稳定 */
}                                          /* 函数体结束 */

/**
 * @brief  产生 I2C 停止条件。
 * @param  None
 * @retval None
 * @note   当 SCL 为高电平时，SDA 从低到高跳变表示 STOP。
 */
void I2C_Stop(void)                        /* 定义 I2C 停止条件函数 */
{                                          /* 函数体开始 */
    SCL_L();                               /* 先拉低 SCL，确保可以安全改变 SDA */
    SDA_L();                               /* 拉低 SDA，为 STOP 的上升沿做准备 */
    I2C_Delay();                           /* 等待 SDA/SCL 低电平稳定 */
    SCL_H();                               /* 释放 SCL 到高电平 */
    I2C_Delay();                           /* 保持 SCL 高电平一小段时间 */
    SDA_H();                               /* 在 SCL 高电平时释放 SDA，产生 STOP */
    I2C_Delay();                           /* 等待 STOP 条件被从机识别 */
}                                          /* 函数体结束 */

/**
 * @brief  等待从机 ACK。
 * @param  None
 * @retval 0 表示 ACK，1 表示 NACK。
 * @note   主机发送 8 位后释放 SDA，从机在第 9 个 SCL 高电平期间拉低 SDA 表示 ACK。
 */
uint8_t I2C_WaitAck(void)                  /* 定义等待 ACK 函数 */
{                                          /* 函数体开始 */
    uint8_t nack;                          /* 定义 NACK 状态变量 */

    SDA_H();                               /* 释放 SDA，让从机驱动 ACK 位 */
    I2C_Delay();                           /* 等待 SDA 释放后电平稳定 */
    SCL_H();                               /* 拉高 SCL，进入第 9 个时钟周期 */
    I2C_Delay();                           /* 等待从机 ACK 电平稳定 */

    nack = (SDA_READ() != 0U) ? 1U : 0U;   /* SDA 高表示 NACK，SDA 低表示 ACK */

    SCL_L();                               /* 拉低 SCL，结束 ACK 时钟周期 */
    I2C_Delay();                           /* 等待 SCL 低电平稳定 */

    return nack;                           /* 返回 ACK/NACK 判断结果 */
}                                          /* 函数体结束 */

/**
 * @brief  主机发送 ACK。
 * @param  None
 * @retval None
 * @note   主机读取一个字节后拉低 SDA，表示还要继续读取后续数据。
 */
void I2C_SendACK(void)                     /* 定义主机发送 ACK 函数 */
{                                          /* 函数体开始 */
    SCL_L();                               /* 拉低 SCL，准备驱动 SDA */
    SDA_L();                               /* 拉低 SDA，表示 ACK */
    I2C_Delay();                           /* 等待 ACK 电平稳定 */
    SCL_H();                               /* 拉高 SCL，让从机采样 ACK */
    I2C_Delay();                           /* 保持 ACK 时钟高电平 */
    SCL_L();                               /* 拉低 SCL，结束 ACK 位 */
    I2C_Delay();                           /* 等待 SCL 低电平稳定 */
    SDA_H();                               /* 释放 SDA，避免持续占用数据线 */
}                                          /* 函数体结束 */

/**
 * @brief  主机发送 NACK。
 * @param  None
 * @retval None
 * @note   主机读取最后一个字节后释放 SDA，表示本次读取结束。
 */
void I2C_NACK(void)                        /* 定义主机发送 NACK 函数 */
{                                          /* 函数体开始 */
    SCL_L();                               /* 拉低 SCL，准备释放 SDA */
    SDA_H();                               /* 释放 SDA，表示 NACK */
    I2C_Delay();                           /* 等待 NACK 电平稳定 */
    SCL_H();                               /* 拉高 SCL，让从机采样 NACK */
    I2C_Delay();                           /* 保持 NACK 时钟高电平 */
    SCL_L();                               /* 拉低 SCL，结束 NACK 位 */
    I2C_Delay();                           /* 等待 SCL 低电平稳定 */
}                                          /* 函数体结束 */

/**
 * @brief  主机发送 NACK。
 * @param  None
 * @retval None
 * @note   该函数是 I2C_NACK 的别名，用于提供更直观的命名。
 */
void I2C_SendNACK(void)                    /* 定义 NACK 别名函数 */
{                                          /* 函数体开始 */
    I2C_NACK();                            /* 调用实际的 NACK 发送函数 */
}                                          /* 函数体结束 */

/**
 * @brief  发送 1 个数据位。
 * @param  bit 要发送的数据位，0 表示低电平，非 0 表示高电平。
 * @retval None
 * @note   数据必须在 SCL 低电平期间改变，并在 SCL 高电平期间保持稳定。
 */
static void I2C_SendBit(uint8_t bit)       /* 定义内部发送位函数 */
{                                          /* 函数体开始 */
    SCL_L();                               /* 确保 SCL 为低，允许改变 SDA */

    if (bit != 0U)                         /* 判断当前要发送的位是否为 1 */
    {                                      /* if 分支开始 */
        SDA_H();                           /* 释放 SDA，发送逻辑 1 */
    }                                      /* if 分支结束 */
    else                                   /* 当前要发送的位为 0 */
    {                                      /* else 分支开始 */
        SDA_L();                           /* 拉低 SDA，发送逻辑 0 */
    }                                      /* else 分支结束 */

    I2C_Delay();                           /* 等待 SDA 电平稳定 */
    SCL_H();                               /* 拉高 SCL，让从机采样当前数据位 */
    I2C_Delay();                           /* 保持 SCL 高电平满足采样时间 */
    SCL_L();                               /* 拉低 SCL，结束当前数据位 */
    I2C_Delay();                           /* 等待 SCL 低电平稳定 */
}                                          /* 函数体结束 */

/**
 * @brief  读取 1 个数据位。
 * @param  None
 * @retval 读取到的数据位，0 或 1。
 * @note   主机释放 SDA 后，在 SCL 高电平期间采样从机输出的数据位。
 */
static uint8_t I2C_ReadBit(void)           /* 定义内部读取位函数 */
{                                          /* 函数体开始 */
    uint8_t bit;                           /* 定义读取到的数据位 */

    SCL_L();                               /* 确保 SCL 为低，准备读取下一位 */
    SDA_H();                               /* 释放 SDA，让从机驱动数据线 */
    I2C_Delay();                           /* 等待 SDA 释放后电平稳定 */
    SCL_H();                               /* 拉高 SCL，进入数据采样窗口 */
    I2C_Delay();                           /* 等待从机输出电平稳定 */

    bit = (SDA_READ() != 0U) ? 1U : 0U;    /* 读取 SDA 电平并转换成 0/1 */

    SCL_L();                               /* 拉低 SCL，结束当前数据位读取 */
    I2C_Delay();                           /* 等待 SCL 低电平稳定 */

    return bit;                            /* 返回读取到的数据位 */
}                                          /* 函数体结束 */

/**
 * @brief  发送 1 个字节。
 * @param  data 要发送的字节。
 * @retval None
 * @note   按 I2C 协议从最高位到最低位依次发送。
 */
void I2C_SendByte(uint8_t data)            /* 定义发送字节函数 */
{                                          /* 函数体开始 */
    uint8_t mask;                          /* 定义位掩码变量 */

    for (mask = 0x80U; mask != 0U; mask >>= 1U) /* 从 bit7 到 bit0 依次发送 */
    {                                      /* 循环体开始 */
        I2C_SendBit((data & mask) ? 1U : 0U); /* 发送当前掩码对应的数据位 */
    }                                      /* 循环体结束 */
}                                          /* 函数体结束 */

/**
 * @brief  读取 1 个字节。
 * @param  send_ack 读取后应答控制，I2C_ACK 表示发送 ACK，I2C_NACK_BIT 表示发送 NACK。
 * @retval 读取到的字节。
 * @note   按 I2C 协议从最高位到最低位依次读取，随后发送 ACK 或 NACK。
 */
uint8_t I2C_ReadByte(uint8_t send_ack)     /* 定义读取字节函数 */
{                                          /* 函数体开始 */
    uint8_t i;                             /* 定义循环计数变量 */
    uint8_t data = 0U;                     /* 定义并清零接收字节 */

    SDA_H();                               /* 释放 SDA，让从机输出数据 */

    for (i = 0U; i < 8U; i++)              /* 连续读取 8 个数据位 */
    {                                      /* 循环体开始 */
        data <<= 1U;                       /* 左移一位，为新读取的位腾位置 */
        data |= I2C_ReadBit();             /* 读取 1 位并合入接收字节 */
    }                                      /* 循环体结束 */

    if (send_ack == I2C_ACK)               /* 判断调用者是否要求发送 ACK */
    {                                      /* if 分支开始 */
        I2C_SendACK();                     /* 发送 ACK，表示还要继续读取 */
    }                                      /* if 分支结束 */
    else                                   /* 调用者要求发送 NACK */
    {                                      /* else 分支开始 */
        I2C_NACK();                        /* 发送 NACK，表示读取结束 */
    }                                      /* else 分支结束 */

    return data;                           /* 返回读取到的字节 */
}                                          /* 函数体结束 */

/**
 * @brief  探测指定 7 位地址的 I2C 设备是否应答。
 * @param  dev7 7 位 I2C 设备地址，不包含读写位。
 * @retval 0 表示收到 ACK，负数表示无应答。
 */
int I2C_CheckDevice(uint8_t dev7)          /* 定义 I2C 设备探测函数 */
{                                          /* 函数体开始 */
    int ret;                               /* 定义返回状态变量 */

    I2C_Start();                           /* 发送起始条件 */
    I2C_SendByte((uint8_t)((dev7 << 1U) | 0U)); /* 发送设备地址和写方向位 */
    ret = (I2C_WaitAck() == 0U) ? 0 : -1;  /* 根据 ACK 状态生成返回值 */
    I2C_Stop();                            /* 发送停止条件，释放总线 */

    return ret;                            /* 返回设备探测结果 */
}                                          /* 函数体结束 */

/**
 * @brief  向设备寄存器写入 1 个字节。
 * @param  dev7 7 位 I2C 设备地址。
 * @param  reg  寄存器地址。
 * @param  val  要写入的数据。
 * @retval 0 表示成功，负数表示失败。
 */
int I2C_WriteReg(uint8_t dev7, uint8_t reg, uint8_t val) /* 定义单字节寄存器写函数 */
{                                          /* 函数体开始 */
    return I2C_WriteRegs(dev7, reg, &val, 1U); /* 复用多字节写函数写入 1 个字节 */
}                                          /* 函数体结束 */

/**
 * @brief  从设备寄存器读取 1 个字节。
 * @param  dev7 7 位 I2C 设备地址。
 * @param  reg  寄存器地址。
 * @retval 读取到的数据；通信失败时返回 0xFF。
 * @note   为兼容旧接口保留，推荐新代码使用 I2C_ReadRegData 获取明确状态码。
 */
uint8_t I2C_ReadReg(uint8_t dev7, uint8_t reg) /* 定义兼容旧接口的单字节读取函数 */
{                                          /* 函数体开始 */
    uint8_t val = 0xFFU;                   /* 默认值设为 0xFF，便于失败时返回 */

    (void)I2C_ReadRegData(dev7, reg, &val); /* 调用带状态码的读取函数 */

    return val;                            /* 返回读取值或默认失败值 */
}                                          /* 函数体结束 */

/**
 * @brief  从设备寄存器读取 1 个字节并返回状态码。
 * @param  dev7 7 位 I2C 设备地址。
 * @param  reg  寄存器地址。
 * @param  val  保存读取结果的指针。
 * @retval 0 表示成功，负数表示失败。
 */
int I2C_ReadRegData(uint8_t dev7, uint8_t reg, uint8_t *val) /* 定义带状态码的单字节读取函数 */
{                                          /* 函数体开始 */
    if (val == 0)                          /* 检查输出指针是否为空 */
    {                                      /* if 分支开始 */
        return -4;                         /* 返回参数错误 */
    }                                      /* if 分支结束 */

    return I2C_ReadRegs(dev7, reg, val, 1U); /* 复用多字节读函数读取 1 个字节 */
}                                          /* 函数体结束 */

/**
 * @brief  向连续寄存器写入多个字节。
 * @param  dev7 7 位 I2C 设备地址。
 * @param  reg  起始寄存器地址。
 * @param  buf  待写入数据缓冲区。
 * @param  len  待写入字节数。
 * @retval 0 表示成功，负数表示失败。
 */
int I2C_WriteRegs(uint8_t dev7, uint8_t reg, const uint8_t *buf, uint16_t len) /* 定义多字节写函数 */
{                                          /* 函数体开始 */
    uint16_t i;                            /* 定义写入循环计数变量 */

    if ((buf == 0) || (len == 0U))         /* 检查输入缓冲区和长度是否合法 */
    {                                      /* if 分支开始 */
        return -4;                         /* 返回参数错误 */
    }                                      /* if 分支结束 */

    I2C_Start();                           /* 发送起始条件 */
    I2C_SendByte((uint8_t)((dev7 << 1U) | 0U)); /* 发送设备地址和写方向位 */
    if (I2C_WaitAck() != 0U)               /* 检查设备地址阶段是否收到 ACK */
    {                                      /* if 分支开始 */
        I2C_Stop();                        /* 失败时发送停止条件释放总线 */
        return -1;                         /* 返回设备地址无应答错误 */
    }                                      /* if 分支结束 */

    I2C_SendByte(reg);                     /* 发送起始寄存器地址 */
    if (I2C_WaitAck() != 0U)               /* 检查寄存器地址阶段是否收到 ACK */
    {                                      /* if 分支开始 */
        I2C_Stop();                        /* 失败时发送停止条件释放总线 */
        return -2;                         /* 返回寄存器地址无应答错误 */
    }                                      /* if 分支结束 */

    for (i = 0U; i < len; i++)             /* 逐字节写入数据缓冲区 */
    {                                      /* 循环体开始 */
        I2C_SendByte(buf[i]);              /* 发送当前数据字节 */
        if (I2C_WaitAck() != 0U)           /* 检查当前数据字节是否收到 ACK */
        {                                  /* if 分支开始 */
            I2C_Stop();                    /* 失败时发送停止条件释放总线 */
            return -3;                     /* 返回数据阶段无应答错误 */
        }                                  /* if 分支结束 */
    }                                      /* 循环体结束 */

    I2C_Stop();                            /* 所有字节写完后发送停止条件 */

    return 0;                              /* 返回写入成功 */
}                                          /* 函数体结束 */

/**
 * @brief  从连续寄存器读取多个字节。
 * @param  dev7 7 位 I2C 设备地址。
 * @param  reg  起始寄存器地址。
 * @param  buf  保存读取结果的缓冲区。
 * @param  len  待读取字节数。
 * @retval 0 表示成功，负数表示失败。
 * @note   读取最后一个字节后发送 NACK，其余字节后发送 ACK。
 */
int I2C_ReadRegs(uint8_t dev7, uint8_t reg, uint8_t *buf, uint16_t len) /* 定义多字节读函数 */
{                                          /* 函数体开始 */
    uint16_t i;                            /* 定义读取循环计数变量 */
    uint8_t ack;                           /* 定义每次读字节后的应答类型 */

    if ((buf == 0) || (len == 0U))         /* 检查输出缓冲区和长度是否合法 */
    {                                      /* if 分支开始 */
        return -4;                         /* 返回参数错误 */
    }                                      /* if 分支结束 */

    I2C_Start();                           /* 发送起始条件 */
    I2C_SendByte((uint8_t)((dev7 << 1U) | 0U)); /* 发送设备地址和写方向位 */
    if (I2C_WaitAck() != 0U)               /* 检查设备地址写阶段是否收到 ACK */
    {                                      /* if 分支开始 */
        I2C_Stop();                        /* 失败时发送停止条件释放总线 */
        return -1;                         /* 返回设备地址写阶段无应答错误 */
    }                                      /* if 分支结束 */

    I2C_SendByte(reg);                     /* 发送待读取的起始寄存器地址 */
    if (I2C_WaitAck() != 0U)               /* 检查寄存器地址阶段是否收到 ACK */
    {                                      /* if 分支开始 */
        I2C_Stop();                        /* 失败时发送停止条件释放总线 */
        return -2;                         /* 返回寄存器地址无应答错误 */
    }                                      /* if 分支结束 */

    I2C_Start();                           /* 发送重复起始条件，切换到读方向 */
    I2C_SendByte((uint8_t)((dev7 << 1U) | 1U)); /* 发送设备地址和读方向位 */
    if (I2C_WaitAck() != 0U)               /* 检查设备地址读阶段是否收到 ACK */
    {                                      /* if 分支开始 */
        I2C_Stop();                        /* 失败时发送停止条件释放总线 */
        return -3;                         /* 返回设备地址读阶段无应答错误 */
    }                                      /* if 分支结束 */

    for (i = 0U; i < len; i++)             /* 按长度连续读取数据 */
    {                                      /* 循环体开始 */
        ack = ((i + 1U) < len) ? I2C_ACK : I2C_NACK_BIT; /* 最后一个字节后发送 NACK */
        buf[i] = I2C_ReadByte(ack);        /* 读取当前字节并发送对应 ACK/NACK */
    }                                      /* 循环体结束 */

    I2C_Stop();                            /* 读取完成后发送停止条件 */

    return 0;                              /* 返回读取成功 */
}                                          /* 函数体结束 */
