#include "debug_log.h"                          // 引入调试日志模块头文件

#define DEBUG_LOG_BUF_SIZE 256                  // 定义格式化日志临时缓冲区大小

void Debug_Print(const char *str)               // 发送普通字符串日志
{                                               // Debug_Print 函数开始
    if(str == NULL)                             // 判断输入字符串指针是否为空
    {                                           // 空指针保护开始
        return;                                 // 输入无效时直接返回
    }                                           // 空指针保护结束

    if(mutexUart6log != NULL)                   // 判断串口日志递归互斥锁是否已创建
    {                                           // 互斥锁保护开始
        xSemaphoreTakeRecursive(mutexUart6log,  // 获取递归互斥锁
                                portMAX_DELAY); // 一直等待直到获取成功
    }                                           // 互斥锁保护结束

    BSP_DebugUart_SendString(str);              // 调用底层 USART6 发送字符串

    if(mutexUart6log != NULL)                   // 判断串口日志递归互斥锁是否已创建
    {                                           // 互斥锁释放保护开始
        xSemaphoreGiveRecursive(mutexUart6log); // 释放递归互斥锁
    }                                           // 互斥锁释放保护结束
}                                               // Debug_Print 函数结束

void Debug_Printf(const char *fmt, ...)         // 格式化并发送日志字符串
{                                               // Debug_Printf 函数开始
    if(fmt == NULL)                             // 判断格式化字符串指针是否为空
    {                                           // 空指针保护开始
        return;                                 // 格式字符串无效时直接返回
    }                                           // 空指针保护结束

    char log_buf[DEBUG_LOG_BUF_SIZE];           // 定义格式化日志缓存数组

    va_list args;                               // 定义可变参数列表

    va_start(args, fmt);                        // 初始化可变参数列表

    vsnprintf(log_buf,                          // 将格式化结果写入日志缓存
              sizeof(log_buf),                  // 限制最大写入长度
              fmt,                              // 使用调用者传入的格式字符串
              args);                            // 使用可变参数列表

    va_end(args);                               // 结束可变参数处理

    Debug_Print(log_buf);                       // 发送格式化后的日志字符串
}                                               // Debug_Printf 函数结束
