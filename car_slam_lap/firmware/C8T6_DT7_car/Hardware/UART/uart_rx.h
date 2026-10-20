/**
 ******************************************************************************
 * @file    uart_rx.h
 * @brief   三路串口的接收中断入口（全工程唯一的 HAL_UART_RxCpltCallback）
 *
 * 为什么集中到一个文件：HAL 的回调是**全局符号**，两个 .c 各定义一份会链接报
 * multiply defined；如果分别被不同 build 配置编进去，还会出现"某个口静默收不到
 * 数据"这种极难查的问题。所以这里只做分发，各业务模块只暴露 OnByte()。
 *
 *   USART1  X5      115200 8N1   -> x5_link.c
 *   USART2  DT7     100000 8E1   -> DBUS.c
 *   USART3  slcan   115200 8N1   -> slcan 模块回包（暂不解析）
 ******************************************************************************
 */
#ifndef __UART_RX_H
#define __UART_RX_H

#include "main.h"

/** 给三路串口各挂一个 1 字节接收中断（上电调用一次） */
void UART_Rx_Start(void);

#endif /* __UART_RX_H */
