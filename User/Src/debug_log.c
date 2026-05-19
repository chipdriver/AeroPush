#include "debug_log.h" // 引入 debug_log.h 提供的接口、宏和类型定义

#define DEBUG_LOG_BUF_SIZE 256 // 定义 DEBUG_LOG_BUF_SIZE 变量为 256

/**
 * @brief 通过调试串口输出字符串。
 * @param str str 变量。
 * @retval None
 */
void Debug_Print(const char *str) // 定义Debug_Print 函数签名：通过调试串口输出字符串
{ // 进入当前代码块
    if (str == NULL) // 判断 str == NULL 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    if (mutexUart6log != NULL) // 判断 mutexUart6log != NULL 是否成立，以选择后续执行路径
    { // 进入当前代码块
        xSemaphoreTakeRecursive(mutexUart6log, // 调用xSemaphoreTakeRecursive 函数，参数为 mutexUart6log,
                                portMAX_DELAY); // 执行 portMAX_DELAY);，完成当前上下文中的具体处理
    } // 结束当前代码块

    BSP_DebugUart_SendString(str); // 调用通过调试串口发送字符串，参数为 str

    if (mutexUart6log != NULL) // 判断 mutexUart6log != NULL 是否成立，以选择后续执行路径
    { // 进入当前代码块
        xSemaphoreGiveRecursive(mutexUart6log); // 调用xSemaphoreGiveRecursive 函数，参数为 mutexUart6log
    } // 结束当前代码块
} // 结束当前代码块

/**
 * @brief 格式化并通过调试串口输出调试信息。
 * @param fmt fmt 变量。
 * @retval None
 */
void Debug_Printf(const char *fmt, ...) // 定义Debug_Printf 函数签名：格式化并通过调试串口输出调试信息
{ // 进入当前代码块
    if (fmt == NULL) // 判断 fmt == NULL 是否成立，以选择后续执行路径
    { // 进入当前代码块
        return; // 当前条件不满足继续处理，直接返回调用者
    } // 结束当前代码块

    char log_buf[DEBUG_LOG_BUF_SIZE]; // 声明 log_buf 变量，供后续计算、状态保存或模块间传递使用

    va_list args; // 执行 va_list args;，完成当前上下文中的具体处理

    va_start(args, fmt); // 调用va_start 函数，参数为 args, fmt

    vsnprintf(log_buf, // 调用vsnprintf 函数，参数为 log_buf,
              sizeof(log_buf), // 调用sizeof 函数，参数为 log_buf),
              fmt, // 继续传入 fmt 变量，作为当前多行调用或初始化列表的一项
              args); // 执行 args);，完成当前上下文中的具体处理

    va_end(args); // 调用va_end 函数，参数为 args

    Debug_Print(log_buf); // 调用通过调试串口输出字符串，参数为 log_buf
} // 结束当前代码块
