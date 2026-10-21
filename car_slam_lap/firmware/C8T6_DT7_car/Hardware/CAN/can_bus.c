/**
 ******************************************************************************
 * @file    can_bus.c
 * @brief   双驱控制器指令层 —— 片内 CAN1 -> SN65HVD230 -> 250K/500K CAN
 *
 * 发送：10 ms 一次，HAL_CAN_AddTxMessage 非阻塞；邮箱满就跳过这一拍，
 *       绝不 while 等发送完成（旧代码那样会把主循环拖死）。
 * 接收：过滤器收全部报文，HAL_CAN_RxFifo0MsgPendingCallback 里解析 ACK：
 *       [speedL][speedR][voltage][crc] 各 2 字节，小端；
 *       crc = speedL ^ speedR ^ voltage。
 *       ACK 的 CAN ID 文档未公开，所以不过滤；校验和不对的帧直接丢。
 ******************************************************************************
 */
#include "can_bus.h"

#include "can_frame.h"

static CAN_HandleTypeDef *s_hcan;
static volatile uint8_t s_ok = 1u;

static volatile uint32_t s_ack_ms;
static volatile uint8_t s_ack_valid;

void CAN_Bus_Init(CAN_HandleTypeDef *hcan)
{
    s_hcan = hcan;
    s_ok = 1u;
    s_ack_valid = 0u;
    s_ack_ms = HAL_GetTick();

    if (hcan == 0)
    {
        s_ok = 0u;
        return;
    }

    /* 过滤器 + HAL_CAN_Start + RX FIFO0 中断（实现在 can.c） */
    CAN_filter_Init();
}

void CAN_Send_MIX(int16_t angular, int16_t speed)
{
    uint8_t data[CAN_FRAME_DLC];
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox = 0u;

    if (s_hcan == 0)
    {
        s_ok = 0u;
        return;
    }

    (void)CAN_Frame_BuildMix(data, angular, speed);

    tx.ExtId = MOTOR_CAN_ID;
    tx.IDE = CAN_ID_EXT;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = CAN_FRAME_DLC;
    tx.TransmitGlobalTime = DISABLE;

    /* 非阻塞：邮箱满返回 HAL_BUSY，下一拍再发；不要在这里等。 */
    s_ok = (HAL_CAN_AddTxMessage(s_hcan, &tx, data, &mailbox) == HAL_OK) ? 1u : 0u;
}

uint8_t CAN_Bus_Okay(void)
{
    return s_ok;
}

uint8_t CAN_Bus_AckOkay(void)
{
    return (uint8_t)((s_ack_valid != 0u) &&
                     ((HAL_GetTick() - s_ack_ms) < 500u));
}

/**
 * @brief  HAL CAN RX FIFO0 回调（弱符号，全工程只允许这一份定义）
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx = {0};
    uint8_t d[CAN_FRAME_DLC] = {0};
    uint16_t speed_l;
    uint16_t speed_r;
    uint16_t voltage;
    uint16_t crc;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, d) != HAL_OK)
    {
        return;
    }
    if (rx.DLC != CAN_FRAME_DLC)
    {
        return;
    }

    /* 混控帧自己发出去的不会回环进来；若控制器回显，跳过。 */
    if ((d[0] == 0xCDu) && (d[1] == 0xABu))
    {
        return;
    }

    speed_l = (uint16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));
    speed_r = (uint16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8));
    voltage = (uint16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8));
    crc = (uint16_t)((uint16_t)d[6] | ((uint16_t)d[7] << 8));

    if ((uint16_t)(speed_l ^ speed_r ^ voltage) != crc)
    {
        return;
    }

    s_ack_ms = HAL_GetTick();
    s_ack_valid = 1u;
}
