/**
 ******************************************************************************
 * @file    x5_link.c
 * @brief   X5 <-> C8T6 链路实现
 ******************************************************************************
 */
#include "x5_link.h"

#include "protocol.h"
#include "usart.h"

static UART_HandleTypeDef *s_huart;

static proto_rx_t s_rx;
static proto_cmd_t s_cmd;

static volatile uint32_t s_last_frame_ms;
static volatile uint32_t s_good_frames;
static volatile uint8_t s_have_cmd;

void X5_Link_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    proto_rx_init(&s_rx, PROTO_DN_SOF0, PROTO_DN_SOF1);

    s_cmd.speed = 0;
    s_cmd.angular = 0;
    s_have_cmd = 0u;
    s_good_frames = 0u;
    /* 初值取当前时刻：上电瞬间不该被当成"已失联"而每小时打日志 */
    s_last_frame_ms = HAL_GetTick();
}

void X5_Link_OnByte(uint8_t byte)
{
    proto_frame_t frame;
    proto_cmd_t cmd;

    if (proto_rx_feed(&s_rx, byte, &frame) == 0)
    {
        return;
    }
    if (proto_parse_set_vel(&frame, &cmd) == 0)
    {
        return; /* 不是速度帧（比如对方发错方向），丢弃 */
    }

    s_cmd.speed = cmd.speed;
    s_cmd.angular = cmd.angular;
    s_last_frame_ms = HAL_GetTick();
    s_good_frames++;
    s_have_cmd = 1u;
}

int X5_Link_GetCmd(int16_t *speed, int16_t *angular)
{
    if ((s_have_cmd == 0u) || (X5_Link_Alive() == 0u))
    {
        return 0;
    }
    if (speed != 0)
    {
        *speed = s_cmd.speed;
    }
    if (angular != 0)
    {
        *angular = s_cmd.angular;
    }
    return 1;
}

uint8_t X5_Link_Alive(void)
{
    return (uint8_t)((HAL_GetTick() - s_last_frame_ms) < X5_LINK_TIMEOUT_MS);
}

void X5_Link_SendStatus(uint8_t status, uint8_t src)
{
    uint8_t buf[PROTO_FRAME_UP_LEN];
    uint16_t n;

    if (s_huart == 0)
    {
        return;
    }
    n = proto_build_status(buf, status, src);
    /* 7 字节 @115200 ≈ 0.6 ms，100 ms 一次，阻塞发送可以接受 */
    (void)HAL_UART_Transmit(s_huart, buf, n, 20u);
}

uint32_t X5_Link_GoodFrames(void)
{
    return s_good_frames;
}
