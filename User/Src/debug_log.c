#include "debug_log.h" // 提供调试日志输出接口

#define DEBUG_LOG_BUF_SIZE 256 // 格式化日志缓冲区长度

/**
 * @brief 通过调试串口输出字符串。
 * @param str 待输出字符串。
 * @retval None
 */
void Debug_Print(const char *str) // 输出调试字符串
{
    if (str == NULL) // 检查字符串指针
    {
        return; // 空指针不输出
    }

    if (mutexUart6log != NULL) // 串口日志互斥锁已创建
    {
        xSemaphoreTakeRecursive(mutexUart6log, // 获取递归互斥锁
                                portMAX_DELAY); // 一直等待到拿到锁
    }

    BSP_DebugUart_SendString(str); // 通过底层串口发送字符串

    if (mutexUart6log != NULL) // 串口日志互斥锁已创建
    {
        xSemaphoreGiveRecursive(mutexUart6log); // 释放递归互斥锁
    }
}

/**
 * @brief 格式化并通过调试串口输出调试信息。
 * @param fmt printf 风格格式字符串。
 * @retval None
 */
void Debug_Printf(const char *fmt, ...) // 输出格式化调试日志
{
    if (fmt == NULL) // 检查格式字符串指针
    {
        return; // 空指针不输出
    }

    char log_buf[DEBUG_LOG_BUF_SIZE]; // 格式化输出缓冲区
    va_list args; // 可变参数列表

    va_start(args, fmt); // 开始读取可变参数

    vsnprintf(log_buf, // 输出到本地缓冲区
              sizeof(log_buf), // 限制最大写入长度
              fmt, // 使用调用方格式字符串
              args); // 使用调用方参数列表

    va_end(args); // 结束读取可变参数

    Debug_Print(log_buf); // 复用串口字符串输出
}
