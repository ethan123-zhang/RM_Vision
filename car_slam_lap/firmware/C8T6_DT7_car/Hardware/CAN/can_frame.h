/**
 ******************************************************************************
 * @file    can_frame.h
 * @brief   XHSMCUW50 双驱控制器「混控帧」编码 —— 纯 C，无 HAL，可在 PC 上测
 *
 * 依据《双驱无刷电机控制器规格书》6.3 / 6.4 节：
 *   - 扩展帧 ID 0x1314，8 字节数据帧，250 kbit/s（实测；PDF 写 500K）
 *   - 混控帧： [0xCD][0xAB][angular_L][angular_H][speed_L][speed_H][crc_L][crc_H]
 *   - 校验和 = 0xABCD ^ angular ^ speed（16 位异或，取低 16 位）
 *   - 控制器要求 5~20 ms 周期持续发送；超过 3~5 s 未收到则判信号丢失
 *
 * 为什么单独一层：帧布局和校验一旦写错，表现是「CAN 有帧、电机不动」，
 * 现场极难查。这里不 include 任何 HAL，PC 上用 gcc 逐字节断言。
 ******************************************************************************
 */
#ifndef __CAN_FRAME_H
#define __CAN_FRAME_H

#include <stdint.h>

#define CAN_FRAME_ID       0x1314u   /* 扩展帧 ID（文档规定） */
#define CAN_FRAME_DLC      8u        /* 混控帧固定 8 字节 */
#define CAN_FRAME_HEADER   0xABCDu   /* 帧头，小端发送为 CD AB */

/**
 * @brief  计算混控帧校验和：0xABCD ^ angular ^ speed
 * @param  angular  转弯角速度 [-1000, 1000]
 * @param  speed    直行速度   [-1000, 1000]
 * @retval 16 位校验和
 */
uint16_t CAN_Frame_Checksum(int16_t angular, int16_t speed);

/**
 * @brief  组混控帧（8 字节，小端）
 * @param  out      输出缓冲，至少 8 字节
 * @param  angular  转弯角速度 [-1000, 1000]
 * @param  speed    直行速度   [-1000, 1000]
 * @retval 写入字节数（固定 8）
 */
uint16_t CAN_Frame_BuildMix(uint8_t out[CAN_FRAME_DLC], int16_t angular, int16_t speed);

#endif /* __CAN_FRAME_H */
