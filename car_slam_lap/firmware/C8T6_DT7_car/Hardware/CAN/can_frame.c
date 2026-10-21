/**
 ******************************************************************************
 * @file    can_frame.c
 * @brief   XHSMCUW50 混控帧编码实现（无 HAL 依赖）
 ******************************************************************************
 */
#include "can_frame.h"

uint16_t CAN_Frame_Checksum(int16_t angular, int16_t speed)
{
    return (uint16_t)(CAN_FRAME_HEADER ^ (uint16_t)angular ^ (uint16_t)speed);
}

uint16_t CAN_Frame_BuildMix(uint8_t out[CAN_FRAME_DLC], int16_t angular, int16_t speed)
{
    uint16_t crc;

    if (out == 0)
    {
        return 0u;
    }

    crc = CAN_Frame_Checksum(angular, speed);

    /* 小端：帧头 0xABCD 先发 CD 再发 AB；两个 int16 同理。 */
    out[0] = (uint8_t)(CAN_FRAME_HEADER & 0xFFu);
    out[1] = (uint8_t)((CAN_FRAME_HEADER >> 8) & 0xFFu);
    out[2] = (uint8_t)((uint16_t)angular & 0xFFu);
    out[3] = (uint8_t)(((uint16_t)angular >> 8) & 0xFFu);
    out[4] = (uint8_t)((uint16_t)speed & 0xFFu);
    out[5] = (uint8_t)(((uint16_t)speed >> 8) & 0xFFu);
    out[6] = (uint8_t)(crc & 0xFFu);
    out[7] = (uint8_t)((crc >> 8) & 0xFFu);

    return CAN_FRAME_DLC;
}
