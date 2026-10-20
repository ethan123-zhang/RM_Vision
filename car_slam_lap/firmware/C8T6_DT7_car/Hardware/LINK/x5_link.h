/**
 ******************************************************************************
 * @file    x5_link.h
 * @brief   X5 <-> C8T6 链路（USART1，115200 8N1）
 *
 * 一条串口上跑两件事：
 *   收：X5 的 9 字节速度帧（50 Hz）  -> X5_Link_GetCmd()
 *   发：10 Hz 状态帧（7 字节）        -> X5_Link_SendStatus()
 *
 * 判断"X5 还在"的唯一依据是**最近 300 ms 内收到过一帧 CRC 正确的速度帧**。
 * 不能用"串口有没有数据"来判断：波特率不匹配时会收到一堆错误字节，
 * 看起来像在线，实际上是垃圾。
 ******************************************************************************
 */
#ifndef __X5_LINK_H
#define __X5_LINK_H

#include "main.h"

/** 超过这个时间没收到有效帧就认为上位机失联（毫秒） */
#define X5_LINK_TIMEOUT_MS 300u

/** 绑定串口并复位收帧状态机 */
void X5_Link_Init(UART_HandleTypeDef *huart);

/** 收到 1 字节（由 uart_rx.c 的接收中断转发） */
void X5_Link_OnByte(uint8_t byte);

/**
 * @brief  取当前速度指令
 * @retval 1 = 有新鲜指令；0 = 超时/还没收到过
 */
int X5_Link_GetCmd(int16_t *speed, int16_t *angular);

/** 链路是否存活（300 ms 内有有效帧） */
uint8_t X5_Link_Alive(void);

/** 上报状态帧（status 位见 protocol.h 的 PROTO_ST_*） */
void X5_Link_SendStatus(uint8_t status, uint8_t src);

/** 累计收到的有效帧数（调试/自检用） */
uint32_t X5_Link_GoodFrames(void);

#endif /* __X5_LINK_H */
