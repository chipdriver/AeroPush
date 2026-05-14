#ifndef __MAIN_H                              // 防止 main.h 被重复包含
#define __MAIN_H                              // 定义主模块头文件保护宏

/* headler file inclution */                  // 头文件包含区域
#include "stm32f4xx.h"                        // 引入 STM32F4 基础定义
#include "app_main.h"                         // 引入应用层主入口接口
//void TimingDelay_Decrement(void);           // 保留的延时计数函数声明示例，当前未启用

#endif                                        // 结束主模块头文件保护宏
