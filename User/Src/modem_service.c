#include "modem_service.h" // 提供通信服务接口
#include "app_config.h" // 提供模拟 GNSS 配置
#include "bsp_a7670e_uart.h" // 提供 A7670E 串口收发接口
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

    Debug_Printf("[UART6 OUT] topic=%s payload=%s\r\n", // 本阶段只通过 USART6 输出遥测 JSON
                 msg->topic, // 输出遥测主题
                 msg->payload); // 输出遥测 JSON 负载
}
