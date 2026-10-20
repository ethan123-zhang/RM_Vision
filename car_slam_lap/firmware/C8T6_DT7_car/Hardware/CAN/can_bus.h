/**
 ******************************************************************************
 * @file    can_bus.h
 * @brief   双驱无刷控制器指令层 —— 经「串口转CAN 模块」下发
 *
 * 硬件链路变了：C8T6 不再用片内 CAN 外设，而是
 *     USART3 (PB10/PB11) --slcan 文本帧--> 串口转CAN 模块 --CAN--> XHSMCUW50
 *
 * 所以这一层只负责"把 8 字节混控帧包装成 slcan 文本并发出去"，
 * 帧内容（0xABCD + angular + speed + 异或校验）与原来完全一致。
 *
 * 协议依据：《双驱动CAN控制接口使用说明》(CS-KZQ-SM-2024-3)
 *   - 扩展帧 ID = 0x1314，8 字节数据帧
 *   - 混控帧: [0xCD][0xAB][angular_L][angular_H][speed_L][speed_H][crc_L][crc_H]
 *   - 校验和 = 0xABCD ^ angular ^ speed（16bit 异或）
 *   - 控制器要求 5~20 ms 周期持续发送，超过 5 s 未发 → 判信号丢失
 *
 * 波特率注意：控制器侧实测是 250 kbit/s（CubeMX 里 CalculateBaudRate=250000），
 * 厂商文档写的 500K 与实测不符。这里按 250K 配给模块（slcan 命令 S5）。
 ******************************************************************************
 */
#ifndef __CAN_BUS_H
#define __CAN_BUS_H

#include "main.h"          /* 含 HAL 库及 stm32f1xx_hal.h */

#define MOTOR_CAN_ID         0x1314u    /* 扩展帧 ID（文档规定） */
#define MOTOR_CAN_BITRATE    250000u    /* 控制器实测 250 kbit/s */
#define MOTOR_FRAME_LEN      8u         /* 混控帧固定 8 字节 */

/**
 * @brief  绑定串口并打开模块通道（发 S5\r 设 250K，再发 O\r 开通道）
 * @param  huart 接串口转CAN 模块的那个串口（本工程是 huart3）
 */
void CAN_Bus_Init(UART_HandleTypeDef *huart);

/**
 * @brief  混控模式：发送 转弯角速度 + 直行速度
 * @param  angular  转弯角速度 [-1000, 1000]（正右转/负左转）
 * @param  speed    直行速度   [-1000, 1000]（正前进/负后退）
 */
void CAN_Send_MIX(int16_t angular, int16_t speed);

/** 最近一次向模块发送是否成功（0 = 串口发送失败/参数非法） */
uint8_t CAN_Bus_Okay(void);

#endif /* __CAN_BUS_H */
