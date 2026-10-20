/**
 ******************************************************************************
 * @file    can_bus.c
 * @brief   双驱控制器指令层 —— 8 字节混控帧 -> slcan 文本 -> 串口转CAN 模块
 *
 * 原来的实现直接操作 CAN 外设（PA11/PA12 + TJA1050）。改成外挂模块后，
 * CAN 外设可以完全不初始化，换板子也不用再焊收发器。
 *
 * 一帧实际发出去的东西（27 字节）：
 *   "T" + "00001314" + "8" + "CDAB" + angular(4 hex) + speed(4 hex) + crc(4 hex) + "\r"
 ******************************************************************************
 */
#include "can_bus.h"

#include "slcan.h"

static UART_HandleTypeDef *s_huart;
static volatile uint8_t s_ok = 1u;

static void slcan_send(const char *buf, uint16_t n)
{
    if ((s_huart == 0) || (n == 0u))
    {
        s_ok = 0u;
        return;
    }
    /* 27 字节 @115200 ≈ 2.3 ms，10 ms 周期下阻塞可接受 */
    s_ok = (HAL_UART_Transmit(s_huart, (uint8_t *)buf, n, 20u) == HAL_OK) ? 1u : 0u;
}

void CAN_Bus_Init(UART_HandleTypeDef *huart)
{
    char cmd[SLCAN_BUF_LEN];
    uint16_t n;
    uint8_t code;

    s_huart = huart;
    s_ok = 1u;

    /* 1) 设波特率：模块上电默认值不可靠，必须显式设置 */
    code = slcan_bitrate_code(MOTOR_CAN_BITRATE);
    n = slcan_encode_bitrate(code, cmd);
    slcan_send(cmd, n);
    HAL_Delay(20);

    /* 2) 打开通道：未 O 的模块不会把帧发上总线 */
    n = slcan_encode_open(cmd);
    slcan_send(cmd, n);
    HAL_Delay(20);
}

void CAN_Send_MIX(int16_t angular, int16_t speed)
{
    uint8_t mix[MOTOR_FRAME_LEN];
    char out[SLCAN_BUF_LEN];
    uint16_t start = 0xABCDu;
    uint16_t crc;
    uint16_t n;

    crc = (uint16_t)(start ^ (uint16_t)angular ^ (uint16_t)speed);

    /* 小端：先低字节。与厂商的混控帧逐字节一致。 */
    mix[0] = (uint8_t)(start & 0xFFu);
    mix[1] = (uint8_t)((start >> 8) & 0xFFu);
    mix[2] = (uint8_t)((uint16_t)angular & 0xFFu);
    mix[3] = (uint8_t)(((uint16_t)angular >> 8) & 0xFFu);
    mix[4] = (uint8_t)((uint16_t)speed & 0xFFu);
    mix[5] = (uint8_t)(((uint16_t)speed >> 8) & 0xFFu);
    mix[6] = (uint8_t)(crc & 0xFFu);
    mix[7] = (uint8_t)((crc >> 8) & 0xFFu);

    n = slcan_encode_ext(MOTOR_CAN_ID, mix, MOTOR_FRAME_LEN, out);
    slcan_send(out, n);
}

uint8_t CAN_Bus_Okay(void)
{
    return s_ok;
}
