#include "modem_service.h" // 提供通信服务接口
#include "app_config.h" // 提供模拟 GNSS 配置
#include "bsp_a7670e_uart.h" // 提供 A7670E 串口收发接口
#include <stdio.h> // 提供 snprintf
#include <stdlib.h> // 提供 atof

#define A7670E_AT_RESP_BUF_SIZE 512U // A7670E AT 响应缓存长度

/**
 * @brief 从 A7670E 环形缓冲区读取一段 AT 响应字符串。
 * @param buf 响应字符串输出缓冲区。
 * @param buf_size 输出缓冲区长度。
 * @param timeout_ms 最大等待时间，单位 ms。
 * @param idle_ms 收到数据后的空闲结束时间，单位 ms。
 * @retval 实际读取到的字节数。
 */
static uint16_t ModemService_ReadResponse(char *buf,
                                          uint16_t buf_size,
                                          uint32_t timeout_ms,
                                          uint32_t idle_ms)
{
    TickType_t start_tick; // 本次读取开始 tick
    TickType_t last_rx_tick; // 最近一次收到字节的 tick
    uint8_t ch; // 从环形缓冲区取出的字节
    uint16_t count = 0U; // 已写入响应缓存的字节数

    if ((buf == NULL) || (buf_size == 0U)) // 检查输出缓冲区
    {
        return 0U; // 参数无效时不读取
    }

    memset(buf, 0, buf_size); // 清空响应缓存

    start_tick = xTaskGetTickCount(); // 记录读取起点
    last_rx_tick = start_tick; // 初始化空闲计时起点

    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms)) // 等待直到超时
    {
        if (BSP_A7670E_Uart_ReceiveByte(&ch) == 1U) // 环形缓冲区有新字节
        {
            last_rx_tick = xTaskGetTickCount(); // 刷新最近接收时间

            if (count < (uint16_t)(buf_size - 1U)) // 给字符串结束符保留空间
            {
                buf[count] = (char)ch; // 写入当前字节
                count++; // 推进写入计数
                buf[count] = '\0'; // 保持响应缓存始终为字符串
            }
        }
        else // 当前没有新字节
        {
            if ((count > 0U) &&
                ((xTaskGetTickCount() - last_rx_tick) >= pdMS_TO_TICKS(idle_ms))) // 已收到数据且空闲足够久
            {
                break; // 认为本次 AT 响应已结束
            }

            vTaskDelay(pdMS_TO_TICKS(1)); // 让出 CPU，等待后续串口中断填充缓冲区
        }
    }

    buf[count] = '\0'; // 确保输出以字符串结束

    return count; // 返回读取长度
}

/**
 * @brief 发送一条 AT 指令并读取原始响应。
 * @param cmd 不带换行符的 AT 指令字符串。
 * @param resp 响应字符串输出缓冲区。
 * @param resp_size 响应字符串输出缓冲区长度。
 * @param timeout_ms 最大等待响应时间，单位 ms。
 * @param idle_ms 收到数据后的空闲结束时间，单位 ms。
 * @retval 1U 表示读取到响应，0U 表示参数无效或未读到响应。
 */
static uint8_t ModemService_SendCmdReadResp(const char *cmd,
                                            char *resp,
                                            uint16_t resp_size,
                                            uint32_t timeout_ms,
                                            uint32_t idle_ms)
{
    uint16_t resp_len; // 本次 AT 响应实际读取长度

    if ((cmd == NULL) || (resp == NULL) || (resp_size == 0U)) // 检查 AT 指令和输出缓冲区参数
    {
        return 0U; // 参数无效时不访问串口
    }

    BSP_A7670E_Uart_RxClear(); // 清空 A7670E 接收环形缓冲区，避免旧响应干扰

    Debug_Printf("[A7670E] CMD=%s\r\n", cmd); // 输出当前发送的 AT 指令

    BSP_A7670E_Uart_SendString(cmd); // 发送 AT 指令正文

    BSP_A7670E_Uart_SendString("\r\n"); // 发送 AT 指令行结束符

    resp_len = ModemService_ReadResponse(resp, resp_size, timeout_ms, idle_ms); // 读取 A7670E 原始响应

    Debug_Printf("%s\r\n", resp); // 输出原始响应，便于串口日志直接排查模块状态

    if (resp_len > 0U) // 判断是否读到任意响应字节
    {
        return 1U; // 已经读取到响应
    }

    return 0U; // 超时或没有读取到响应
}

/**
 * @brief 发送 AT 指令并等待响应中出现期望关键字。
 * @param cmd 不带换行符的 AT 指令字符串。
 * @param expect 期望出现在响应中的关键字。
 * @param resp 响应字符串输出缓冲区。
 * @param resp_size 响应字符串输出缓冲区长度。
 * @param timeout_ms 最大等待响应时间，单位 ms。
 * @param idle_ms 收到数据后的空闲结束时间，单位 ms。
 * @retval 1U 表示响应包含期望关键字，0U 表示失败。
 */
static uint8_t ModemService_SendCmdAndWait(const char *cmd,
                                           const char *expect,
                                           char *resp,
                                           uint16_t resp_size,
                                           uint32_t timeout_ms,
                                           uint32_t idle_ms)
{
    if (expect == NULL) // 检查期望关键字指针
    {
        return 0U; // 期望关键字无效时不执行匹配
    }

    if (ModemService_SendCmdReadResp(cmd, resp, resp_size, timeout_ms, idle_ms) == 0U) // 发送指令并读取响应
    {
        return 0U; // 没有响应时认为本步失败
    }

    if (strstr(resp, expect) != NULL) // 查找期望关键字
    {
        return 1U; // 响应包含期望关键字
    }

    return 0U; // 响应内容不符合预期
}

/**
 * @brief 等待 LTE 数据域注册成功。
 * @param retry_count 最大查询次数。
 * @retval 1U 表示已注册到本地或漫游网络，0U 表示超过重试次数仍未注册。
 */
static uint8_t ModemService_WaitLteRegistered(uint32_t retry_count)
{
    char resp[A7670E_AT_RESP_BUF_SIZE]; // 保存 AT+CEREG? 原始响应
    uint32_t retry_index = 0U; // 当前注册状态查询次数

    for (retry_index = 0U; retry_index < retry_count; retry_index++) // 按最大次数轮询 LTE 注册状态
    {
        ModemService_SendCmdReadResp("AT+CEREG?", resp, sizeof(resp), 3000U, 300U); // 查询 EPS 数据域注册状态

        Debug_Printf("[NET] CEREG resp=%s\r\n", resp); // 输出 CEREG 原始响应，便于判断注册阶段

        if ((strstr(resp, "+CEREG: 0,1") != NULL) ||
            (strstr(resp, "+CEREG:0,1") != NULL) ||
            (strstr(resp, "+CEREG: 0,5") != NULL) ||
            (strstr(resp, "+CEREG:0,5") != NULL)) // 0,1 表示本地注册，0,5 表示漫游注册
        {
            return 1U; // LTE 数据域已经注册成功
        }

        vTaskDelay(pdMS_TO_TICKS(1000U)); // 等待约 1 秒后再次查询注册状态
    }

    return 0U; // 超过最大查询次数仍未注册
}

/**
 * @brief 打开 A7670E 数据网络。
 * @retval 1U 表示 NETOPEN 成功或网络已经打开，0U 表示失败。
 */
static uint8_t ModemService_NetOpen(void)
{
    char resp[A7670E_AT_RESP_BUF_SIZE]; // 保存 AT+NETOPEN 原始响应

    if (ModemService_SendCmdReadResp("AT+NETOPEN", resp, sizeof(resp), 15000U, 500U) == 0U) // 发送 NETOPEN 并等待模块响应
    {
        return 0U; // 没有响应时认为打开网络失败
    }

    if ((strstr(resp, "OK") != NULL) ||
        (strstr(resp, "+NETOPEN: 0") != NULL) ||
        (strstr(resp, "Network is already opened") != NULL)) // OK、NETOPEN 成功事件或已打开提示都认为成功
    {
        return 1U; // 数据网络已经可用
    }

    return 0U; // NETOPEN 响应不符合成功条件
}

/**
 * @brief 初始化 A7670E 4G 数据网络。
 * @retval 1U 表示 4G 网络初始化成功。
 * @retval 0U 表示 4G 网络初始化失败。
 */
uint8_t ModemService_NetInit(void)
{
    char resp[A7670E_AT_RESP_BUF_SIZE]; // 保存各阶段 AT 响应
    char apn_cmd[80]; // 保存按 APN 组合出来的 CGDCONT 指令
    int apn_cmd_len; // snprintf 返回的指令长度

    Debug_Print("[NET] init start\r\n"); // 输出 4G 网络初始化开始日志

    if (ModemService_SendCmdAndWait("AT", "OK", resp, sizeof(resp), 3000U, 200U) == 0U) // 检查 AT 基础通信是否正常
    {
        Debug_Print("[NET] AT failed\r\n"); // 输出 AT 基础通信失败日志
        return 0U; // 基础通信失败时停止网络初始化
    }

    if (ModemService_SendCmdAndWait("AT+CMEE=2", "OK", resp, sizeof(resp), 3000U, 200U) == 0U) // 打开详细错误提示
    {
        Debug_Print("[NET] CMEE failed\r\n"); // 输出详细错误模式配置失败日志
        return 0U; // CMEE 配置失败时停止网络初始化
    }

    if (ModemService_SendCmdAndWait("AT+CPIN?", "+CPIN: READY", resp, sizeof(resp), 3000U, 200U) == 0U) // 检查 SIM 卡是否就绪
    {
        Debug_Print("[NET] SIM not ready\r\n"); // 输出 SIM 卡未就绪日志
        return 0U; // SIM 卡未就绪时停止网络初始化
    }

    if (ModemService_SendCmdAndWait("AT+CSQ", "+CSQ:", resp, sizeof(resp), 3000U, 200U) == 0U) // 查询并确认信号质量响应存在
    {
        Debug_Print("[NET] CSQ failed\r\n"); // 输出信号质量查询失败日志
        return 0U; // 没有 CSQ 响应时停止网络初始化
    }

    if (ModemService_WaitLteRegistered(APP_NET_REGISTER_RETRY_COUNT) == 0U) // 等待 LTE 数据域注册成功
    {
        Debug_Print("[NET] CEREG failed\r\n"); // 输出 LTE 注册失败日志
        return 0U; // 注册失败时停止网络初始化
    }

    apn_cmd_len = snprintf(apn_cmd, sizeof(apn_cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", APP_A7670E_APN); // 使用配置的 APN 组合 PDP 上下文指令

    if ((apn_cmd_len <= 0) || ((uint32_t)apn_cmd_len >= sizeof(apn_cmd))) // 检查 APN 指令是否完整写入缓冲区
    {
        Debug_Print("[NET] APN command overflow\r\n"); // 输出 APN 指令组合失败日志
        return 0U; // APN 指令异常时停止网络初始化
    }

    if (ModemService_SendCmdAndWait(apn_cmd, "OK", resp, sizeof(resp), 3000U, 200U) == 0U) // 配置 PDP 上下文 APN
    {
        Debug_Print("[NET] CGDCONT failed\r\n"); // 输出 PDP 上下文配置失败日志
        return 0U; // APN 配置失败时停止网络初始化
    }

    if (ModemService_NetOpen() == 0U) // 打开数据网络
    {
        Debug_Print("[NET] NETOPEN failed\r\n"); // 输出数据网络打开失败日志
        return 0U; // 数据网络打开失败时停止网络初始化
    }

    if (ModemService_SendCmdAndWait("AT+IPADDR", "+IPADDR:", resp, sizeof(resp), 5000U, 300U) == 0U) // 查询并确认模块获得 IP 地址
    {
        Debug_Print("[NET] IPADDR failed\r\n"); // 输出 IP 地址查询失败日志
        return 0U; // 没有 IP 地址响应时停止网络初始化
    }

    Debug_Print("[NET] ready\r\n"); // 输出 4G 网络初始化成功日志

    return 1U; // 4G 网络初始化成功
}

/**
 * @brief 启动 A7670E MQTT 服务。
 * @retval 1U 表示 MQTT 服务启动成功，0U 表示启动失败。
 */
static uint8_t ModemService_MqttStart(void)
{
    char resp[A7670E_AT_RESP_BUF_SIZE]; // 保存 AT+CMQTTSTART 原始响应

    (void)ModemService_SendCmdReadResp("AT+CMQTTDISC=0,120", // 先尝试断开客户端 0 的 MQTT 连接，ERROR 也只作为状态清理日志
                                       resp, // 保存模块返回内容
                                       sizeof(resp), // 响应缓冲区大小
                                       5000U, // 最长等待 5000 ms
                                       300U); // 收到数据后 300 ms 无新数据则结束

    (void)ModemService_SendCmdReadResp("AT+CMQTTREL=0", // 再释放客户端 0，ERROR 也只作为状态清理日志
                                       resp, // 保存模块返回内容
                                       sizeof(resp), // 响应缓冲区大小
                                       5000U, // 最长等待 5000 ms
                                       300U); // 收到数据后 300 ms 无新数据则结束

    (void)ModemService_SendCmdReadResp("AT+CMQTTSTOP", // 最后停止 MQTT 服务，ERROR 也只作为状态清理日志
                                       resp, // 保存模块返回内容
                                       sizeof(resp), // 响应缓冲区大小
                                       5000U, // 最长等待 5000 ms
                                       300U); // 收到数据后 300 ms 无新数据则结束

    vTaskDelay(pdMS_TO_TICKS(1000U)); // 等待模块释放 MQTT 内部资源
    
    if (ModemService_SendCmdReadResp("AT+CMQTTSTART", resp, sizeof(resp), 10000U, 500U) == 0U) // 发送 MQTT 服务启动指令并读取响应
    {
        return 0U; // 没有响应时认为 MQTT 服务启动失败
    }

    if (strstr(resp, "+CMQTTSTART: 0") != NULL) // 模块返回 MQTT 服务启动成功事件
    {
        Debug_Print("[MQTT] start ok\r\n"); // 输出 MQTT 服务启动成功日志

        return 1U; // MQTT 服务启动成功
    }

    Debug_Printf("[MQTT] start resp=%s\r\n", resp); // 输出完整启动响应，重点排查 ERROR 原因

    return 0U; // 响应中没有成功标志
}

/**
 * @brief 申请 A7670E MQTT 客户端 0。
 * @retval 1U 表示客户端申请成功，0U 表示客户端申请失败。
 */
static uint8_t ModemService_MqttAcquireClient(void)
{
    char resp[A7670E_AT_RESP_BUF_SIZE]; // 保存 AT+CMQTTACCQ 原始响应
    char cmd[96]; // 保存 MQTT 客户端申请指令
    int cmd_len; // snprintf 返回的指令长度

    cmd_len = snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=0,\"%s\"", APP_MQTT_CLIENT_ID); // 使用配置的客户端 ID 组合 ACCQ 指令

    if ((cmd_len <= 0) || ((uint32_t)cmd_len >= sizeof(cmd))) // 检查客户端申请指令是否完整写入缓冲区
    {
        Debug_Print("[MQTT] ACCQ command overflow\r\n"); // 输出客户端申请指令组合失败日志

        return 0U; // 指令组合失败时不发送到模块
    }

    if (ModemService_SendCmdAndWait(cmd, "OK", resp, sizeof(resp), 5000U, 300U) == 0U) // 发送 MQTT 客户端申请指令并等待 OK
    {
        return 0U; // 响应不包含 OK 时认为客户端申请失败
    }

    Debug_Print("[MQTT] acquire ok\r\n"); // 输出 MQTT 客户端申请成功日志

    return 1U; // MQTT 客户端申请成功
}

/**
 * @brief 连接 A7670E MQTT 客户端到服务器。
 * @retval 1U 表示 MQTT 服务器连接成功，0U 表示连接失败。
 */
static uint8_t ModemService_MqttConnectServer(void)
{
    char resp[A7670E_AT_RESP_BUF_SIZE]; // 保存 AT+CMQTTCONNECT 原始响应
    char cmd[180]; // 保存 MQTT 服务器连接指令
    int cmd_len; // snprintf 返回的指令长度
    TickType_t start_tick; // 记录 MQTT 连接等待开始 tick
    uint8_t ch; // 保存从 A7670E 环形缓冲区读取的当前字节
    uint16_t resp_len = 0U; // 记录已经写入 resp 的响应长度
    char *connect_result; // 指向 +CMQTTCONNECT 异步连接结果
    char *connect_status; // 指向客户端 0 的 MQTT 连接状态前缀
    char *connect_line_end; // 指向 +CMQTTCONNECT 当前结果行的结束符

    cmd_len = snprintf(cmd, // 组合 MQTT 服务器连接指令
                       sizeof(cmd), // 限制写入连接指令缓冲区的长度
                       "AT+CMQTTCONNECT=0,\"%s\",%lu,%lu", // A7670E MQTT 连接指令格式
                       APP_MQTT_BROKER_ADDR, // 写入 MQTT 服务器地址和端口
                       (unsigned long)APP_MQTT_KEEPALIVE_SEC, // 写入 keepalive 秒数
                       (unsigned long)APP_MQTT_CLEAN_SESSION); // 写入 clean session 标志;

    if ((cmd_len <= 0) || ((uint32_t)cmd_len >= sizeof(cmd))) // 检查 MQTT 连接指令是否完整写入缓冲区
    {
        Debug_Print("[MQTT] CONNECT command overflow\r\n"); // 输出 MQTT 连接指令组合失败日志

        return 0U; // 指令组合失败时不发送到模块
    }

    memset(resp, 0, sizeof(resp)); // 清空 MQTT 连接响应缓冲区

    BSP_A7670E_Uart_RxClear(); // 清空 A7670E 接收环形缓冲区，避免旧响应干扰本次连接结果

    Debug_Printf("[A7670E] CMD=%s\r\n", cmd); // 输出当前发送的 MQTT 连接指令

    BSP_A7670E_Uart_SendString(cmd); // 发送 MQTT 连接 AT 指令正文

    BSP_A7670E_Uart_SendString("\r\n"); // 发送 MQTT 连接 AT 指令行结束符

    start_tick = xTaskGetTickCount(); // 记录等待 +CMQTTCONNECT 异步结果的起点

    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(60000U)) // 最长等待 60000 ms，直到收到 +CMQTTCONNECT 异步事件
    {
        if (BSP_A7670E_Uart_ReceiveByte(&ch) == 1U) // 持续从 A7670E 环形缓冲区读取响应字节
        {
            if (resp_len < (uint16_t)(sizeof(resp) - 1U)) // 预留字符串结束符空间
            {
                resp[resp_len] = (char)ch; // 将当前字节追加到响应缓冲区
                resp_len++; // 更新响应缓冲区有效长度
                resp[resp_len] = '\0'; // 保持响应缓冲区始终为字符串
            }
            else // MQTT 连接响应超过缓冲区容量
            {
                Debug_Printf("[MQTT] connect resp overflow=%s\r\n", resp); // 输出已收集到的响应，便于排查超长响应

                return 0U; // 响应缓冲区不足时认为本次连接失败
            }

            connect_result = strstr(resp, "+CMQTTCONNECT:"); // 每次追加后检查是否收到 MQTT 连接异步结果

            if (connect_result != NULL) // 已经收到 +CMQTTCONNECT 异步事件
            {
                if (strstr(resp, "+CMQTTCONNECT: 0,0") != NULL) // A7670E 返回 0,0 表示客户端 0 连接服务器成功
                {
                    Debug_Printf("%s\r\n", resp); // 输出包含 +CMQTTCONNECT 成功事件的完整响应

                    Debug_Print("[MQTT] connect ok\r\n"); // 输出 MQTT 服务器连接成功日志

                    return 1U; // MQTT 服务器连接成功
                }

                connect_status = strstr(resp, "+CMQTTCONNECT: 0,"); // 查找客户端 0 的 MQTT 连接结果前缀

                if ((connect_status != NULL) && (connect_status[sizeof("+CMQTTCONNECT: 0,") - 1U] != '\0')) // 确认结果码已经收到，避免只收到逗号时误判失败
                {
                    connect_line_end = strchr(connect_status, '\n'); // 查找异步结果行的换行结束符

                    if (connect_line_end == NULL) // 当前响应行可能还没有收到换行
                    {
                        connect_line_end = strchr(connect_status, '\r'); // 再查找异步结果行的回车结束符
                    }

                    if (connect_line_end != NULL) // 已经收到完整的 +CMQTTCONNECT 结果行
                    {
                        Debug_Printf("[MQTT] connect resp=%s\r\n", resp); // 输出完整连接响应，便于分析非 0 结果码

                        return 0U; // 收到客户端 0 的非 0,0 连接结果时认为连接失败
                    }
                }
            }
        }
        else // 当前没有新的 A7670E 响应字节
        {
            vTaskDelay(pdMS_TO_TICKS(1U)); // 让出 CPU，继续等待后续异步连接事件
        }
    }

    Debug_Printf("[MQTT] connect wait timeout resp=%s\r\n", resp); // 超时仍未收到 +CMQTTCONNECT 时输出已收集响应

    return 0U; // 超时未收到 MQTT 连接异步结果
}

/**
 * @brief 初始化并连接 A7670E MQTT 客户端。
 * @retval 1U 表示 MQTT 连接服务器成功。
 * @retval 0U 表示 MQTT 初始化或连接失败。
 */
uint8_t ModemService_MqttInit(void)
{
    Debug_Print("[MQTT] init start\r\n"); // 输出 MQTT 初始化开始日志

    if (ModemService_MqttStart() == 0U) // 启动 A7670E MQTT 服务
    {
        Debug_Print("[MQTT] start failed\r\n"); // 输出 MQTT 服务启动失败日志

        return 0U; // MQTT 服务启动失败时停止后续流程
    }

    if (ModemService_MqttAcquireClient() == 0U) // 申请 MQTT 客户端 0
    {
        Debug_Print("[MQTT] acquire failed\r\n"); // 输出 MQTT 客户端申请失败日志

        return 0U; // 客户端申请失败时停止后续流程
    }

    if (ModemService_MqttConnectServer() == 0U) // 连接 MQTT 服务器
    {
        Debug_Print("[MQTT] connect failed\r\n"); // 输出 MQTT 服务器连接失败日志

        return 0U; // MQTT 服务器连接失败时返回失败
    }

    Debug_Print("[MQTT] ready\r\n"); // 输出 MQTT 连接服务器成功日志

    return 1U; // MQTT 初始化并连接服务器成功
}

/**
 * @brief 将 NMEA 经纬度格式转换为十进制度。
 * @param value NMEA 数值字符串，纬度为 DDMM.MMMM，经度为 DDDMM.MMMM。
 * @param hemi 半球字符，N/E 为正，S/W 为负。
 * @retval 转换后的十进制度，参数无效时返回 0.0。
 */
static double ModemService_NmeaToDegree(const char *value, char hemi)
{
    double raw; // NMEA 原始数值
    int degree; // 整数度部分
    double minute; // 分钟部分
    double result; // 转换后的十进制度

    if ((value == NULL) || (value[0] == '\0')) // 检查输入字符串
    {
        return 0.0; // 空值不参与转换
    }

    raw = atof(value); // 转成浮点原始值

    degree = (int)(raw / 100.0); // 取出度数部分

    minute = raw - ((double)degree * 100.0); // 剩余部分为分钟

    result = (double)degree + minute / 60.0; // 分钟换算成度

    if ((hemi == 'S') || (hemi == 'W')) // 南纬或西经需要取负
    {
        result = -result; // 转成负坐标
    }

    return result; // 返回十进制度
}

/**
 * @brief 打开 A7670E GNSS 电源。
 * @retval 1U 表示 GNSS 上电命令返回成功，0U 表示失败。
 */
uint8_t ModemService_GnssInit(void)
{
    char resp[A7670E_AT_RESP_BUF_SIZE]; // 保存 GNSS 上电 AT 响应

    BSP_A7670E_Uart_RxClear(); // 清掉上一次 AT 响应残留

    BSP_A7670E_Uart_SendString("AT+CGNSSPWR=1\r\n"); // 打开 A7670E GNSS 电源

    ModemService_ReadResponse(resp, sizeof(resp), 3000U, 200U); // 等待模块返回

    Debug_Printf("[GNSS INIT] %s\r\n", resp); // 输出 GNSS 初始化原始响应

    if ((strstr(resp, "OK") != NULL) ||
        (strstr(resp, "READY") != NULL)) // OK 或 READY 都认为 GNSS 电源已打开
    {
        return 1U; // GNSS 上电成功
    }

    return 0U; // GNSS 上电失败
}

/**
 * @brief 查询并解析 A7670E 的 +CGPSINFO 定位结果。
 * @param gnss 输出 GNSS 定位数据。
 * @retval 1U 表示定位有效，0U 表示未定位或解析失败。
 */
uint8_t ModemService_ReadGnss(GnssData_t *gnss)
{
    char resp[A7670E_AT_RESP_BUF_SIZE]; // 保存 AT+CGPSINFO 完整响应
    char line[160]; // 保存 +CGPSINFO 单行内容
    char *start; // 当前解析起点
    char *end; // 当前解析终点
    char *field[10]; // +CGPSINFO 字段指针表
    char *token; // strtok 当前字段
    uint32_t line_len; // +CGPSINFO 单行长度
    uint8_t field_count = 0U; // 已解析字段数

    if (gnss == NULL) // 检查输出结构体
    {
        return 0U; // 空指针不读取
    }

    memset(gnss, 0, sizeof(GnssData_t)); // 默认清空输出定位数据

    BSP_A7670E_Uart_RxClear(); // 清除旧响应，避免影响本次解析

    BSP_A7670E_Uart_SendString("AT+CGPSINFO\r\n"); // 查询当前定位信息

    ModemService_ReadResponse(resp, sizeof(resp), 3000U, 300U); // 等待 +CGPSINFO 响应

    start = strstr(resp, "+CGPSINFO:"); // 定位到响应正文
    if (start == NULL) // 没有找到响应头
    {
        Debug_Printf("[GNSS] no +CGPSINFO resp=%s\r\n", resp); // 输出原始响应用于排查
        return 0U; // 本轮解析失败
    }

    end = strpbrk(start, "\r\n"); // 找到响应行结束位置
    if (end == NULL) // 如果没有换行
    {
        end = start + strlen(start); // 使用字符串末尾作为行结束
    }

    line_len = (uint32_t)(end - start); // 计算响应行长度
    if (line_len >= sizeof(line)) // 防止单行超出本地缓存
    {
        return 0U; // 响应异常过长
    }

    memset(line, 0, sizeof(line)); // 清空单行缓存
    memcpy(line, start, (size_t)line_len); // 拷贝 +CGPSINFO 单行
    line[line_len] = '\0'; // 补字符串结束符

    start = strstr(line, ":"); // 找到字段区起点
    if (start == NULL) // 响应行没有冒号
    {
        return 0U; // 格式不符合预期
    }

    start++; // 跳过冒号

    while ((*start == ' ') || (*start == '\t')) // 跳过字段前空白
    {
        start++; // 移动到第一个字段字符
    }

    if (*start == ',') // 第一个字段为空表示还没有定位
    {
        Debug_Print("[GNSS] no fix\r\n"); // 输出未定位提示
        return 0U; // 本轮无有效定位
    }

    token = strtok(start, ","); // 按逗号拆分字段

    while ((token != NULL) && (field_count < 10U)) // 收集最多 10 个字段
    {
        field[field_count] = token; // 保存当前字段起点
        field_count++; // 字段数加一
        token = strtok(NULL, ","); // 继续读取下一个字段
    }

    if (field_count < 4U) // 至少需要纬度、南北半球、经度、东西半球
    {
        return 0U; // 字段不足
    }

    if ((field[0] == NULL) || (field[1] == NULL) ||
        (field[2] == NULL) || (field[3] == NULL)) // 检查关键字段指针
    {
        return 0U; // 关键字段缺失
    }

    if ((field[0][0] == '\0') || (field[2][0] == '\0')) // 检查经纬度字符串
    {
        return 0U; // 经纬度为空
    }

    gnss->latitude = ModemService_NmeaToDegree(field[0], field[1][0]); // 转换纬度为十进制度
    gnss->longitude = ModemService_NmeaToDegree(field[2], field[3][0]); // 转换经度为十进制度

    if ((field_count > 6U) && (field[6] != NULL) && (field[6][0] != '\0')) // 响应里带高度字段
    {
        gnss->altitude_m = (float)atof(field[6]); // 写入高度
    }
    else // 没有高度字段
    {
        gnss->altitude_m = 0.0f; // 高度默认 0
    }

    gnss->speed_mps = 0.0f; // 本阶段不解析速度
    gnss->gps_num = 0U; // 本阶段不解析卫星数量
    gnss->fix_valid = 1U; // 经纬度解析成功即认为定位有效
    gnss->timestamp_ms = xTaskGetTickCount(); // 写入当前 tick 时间戳

    return 1U; // 返回定位有效
}

/**
 * @brief 构造模拟 GNSS 定位数据。
 * @param gnss 输出 GNSS 定位数据。
 * @retval None
 */
void ModemService_BuildSimGnss(GnssData_t *gnss) // 构造模拟 GNSS 数据
{
    static uint32_t gnss_count = 0; // 模拟轨迹步进计数

    if (gnss == NULL) // 检查输出指针
    {
        return; // 输出指针为空时不处理
    }

    gnss_count++; // 推进模拟定位点

    memset(gnss, 0, sizeof(GnssData_t)); // 清空 GNSS 数据结构体

    gnss->latitude = APP_SIM_GNSS_BASE_LAT + gnss_count * APP_SIM_GNSS_STEP; // 生成模拟纬度
    gnss->longitude = APP_SIM_GNSS_BASE_LON + gnss_count * APP_SIM_GNSS_STEP; // 生成模拟经度

    gnss->altitude_m = APP_SIM_GNSS_ALTITUDE_M; // 写入模拟高度
    gnss->speed_mps = APP_SIM_GNSS_SPEED_MPS; // 写入模拟速度

    gnss->gps_num = APP_SIM_GNSS_NUM; // 写入模拟卫星数量
    gnss->fix_valid = 1; // 标记模拟定位有效

    gnss->timestamp_ms = xTaskGetTickCount(); // 写入当前 tick 时间戳
}

/**
 * @brief 处理一条待发布 MQTT 消息。
 * @param msg 待发布 MQTT 消息。
 * @retval None
 */
void ModemService_Publish(const MqttPublishMsg_t *msg) // 处理 MQTT 发布请求
{
    if (msg == NULL) // 检查输入指针
    {
        return; // 输入指针为空时不处理
    }

    // Debug_Printf("[UART6 OUT] topic=%s payload=%s\r\n", // 本阶段只通过 USART6 输出遥测 JSON
    //              msg->topic, // 输出遥测主题
    //              msg->payload); // 输出遥测 JSON 负载
}
