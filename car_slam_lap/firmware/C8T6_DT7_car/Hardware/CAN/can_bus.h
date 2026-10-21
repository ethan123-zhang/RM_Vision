/**
 ******************************************************************************
 * @file    can_bus.h
 * @brief   双驱无刷控制器指令层 —— 片内 CAN1 + SN65HVD230 收发器
 *
 * 实物链路（2026-10-21 按《CAN收发器原理图》确认）：
 *     C8T6 PA12 (CAN_TX) --> 收发器 H1-4 (D)
 *     C8T6 PA11 (CAN_RX) <-- 收发器 H1-3 (R)
 *     C8T6 3V3           --> 收发器 H1-2
 *     C8T6 GND           --> 收发器 H1-1
 *     收发器 H2-1 (CANH) --> 控制器 J3-1
 *     收发器 H2-2 (CANL) --> 控制器 J3-2
 *
 * 注意：SN65HVD230 是 3.3V **CAN 收发器**，不是「串口转CAN 模块」。
 * 它没有 UART，不能用 USART3 驱动；USART3 在本工程里只是备用/调试口。
 *
 * 协议依据：《双驱无刷电机控制器规格书》6.3/6.4
 *   - 扩展帧 ID = 0x1314，8 字节数据帧
 *   - 混控帧: [0xCD][0xAB][angular_L][angular_H][speed_L][speed_H][crc_L][crc_H]
 *   - 校验和 = 0xABCD ^ angular ^ speed（16bit 异或）
 *   - 控制器要求 5~20 ms 周期持续发送，超过 3~5 s 未发 → 判信号丢失
 *
 * 波特率：2026-10-21 实测这台控制器是 250K（can.c 的 Prescaler=8）；
 * PDF 写 500K，但 500K 下 can_ok=0。若换到 500K 批次，把 Prescaler 改回 4。
 ******************************************************************************
 */
#ifndef __CAN_BUS_H
#define __CAN_BUS_H

#include "main.h"
#include "can.h"

#define MOTOR_CAN_BITRATE    250000u   /* 实测可用：500K 不 ACK */
#define MOTOR_CAN_ID         0x1314u   /* 扩展帧 ID（文档规定） */

/**
 * @brief  绑定 CAN 句柄并启动：配置过滤器（收全部，便于收 ACK）→ Start → RX 中断
 * @param  hcan  CubeMX 初始化后的 CAN1 句柄（本工程是 &hcan）
 */
void CAN_Bus_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief  混控模式：发送 转弯角速度 + 直行速度（非阻塞，10 ms 调一次）
 * @param  angular  转弯角速度 [-1000, 1000]（正右转/负左转）
 * @param  speed    直行速度   [-1000, 1000]（正前进/负后退）
 */
void CAN_Send_MIX(int16_t angular, int16_t speed);

/** 最近一次 HAL_CAN_AddTxMessage 是否成功（0 = 邮箱满/初始化失败） */
uint8_t CAN_Bus_Okay(void);

/** 最近 500 ms 内是否收到控制器 ACK（校验通过才算） */
uint8_t CAN_Bus_AckOkay(void);

#endif /* __CAN_BUS_H */
