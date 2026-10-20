/**
 ******************************************************************************
 * @file    protocol.c
 * @brief   X5 <-> C8T6 串口链路协议实现（平台无关，可在 PC 上用 gcc 跑测试）
 ******************************************************************************
 */
#include "protocol.h"

#include <string.h>

/* ============================ 基础 ============================ */

uint8_t proto_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00u;
    uint16_t i;
    uint8_t b;

    for (i = 0u; i < len; i++)
    {
        crc ^= data[i];
        for (b = 0u; b < 8u; b++)
        {
            if (crc & 0x80u)
            {
                crc = (uint8_t)((uint8_t)(crc << 1) ^ 0x07u);
            }
            else
            {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

static int16_t proto_clamp_i16(int16_t v)
{
    if (v > PROTO_SPEED_MAX)
    {
        return PROTO_SPEED_MAX;
    }
    if (v < -PROTO_SPEED_MAX)
    {
        return -PROTO_SPEED_MAX;
    }
    return v;
}

static void proto_put_i16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v & 0xFFu);          /* 小端低字节 */
    p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFFu);
}

static int16_t proto_get_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* ============================ 组帧 ============================ */

uint16_t proto_build_set_vel(uint8_t *buf, int16_t speed, int16_t angular)
{
    speed = proto_clamp_i16(speed);
    angular = proto_clamp_i16(angular);

    buf[0] = PROTO_DN_SOF0;
    buf[1] = PROTO_DN_SOF1;
    buf[2] = 5u; /* LEN = CMD + 2 + 2 */
    buf[3] = PROTO_CMD_SET_VEL;
    proto_put_i16(&buf[4], speed);
    proto_put_i16(&buf[6], angular);
    buf[8] = proto_crc8(&buf[2], 6u); /* LEN + CMD + 4 字节数据 */

    return PROTO_FRAME_DN_LEN;
}

uint16_t proto_build_status(uint8_t *buf, uint8_t status, uint8_t src)
{
    buf[0] = PROTO_UP_SOF0;
    buf[1] = PROTO_UP_SOF1;
    buf[2] = 3u; /* LEN = CMD + status + src */
    buf[3] = PROTO_CMD_STATUS;
    buf[4] = status;
    buf[5] = src;
    buf[6] = proto_crc8(&buf[2], 4u);

    return PROTO_FRAME_UP_LEN;
}

/* ============================ 解帧 ============================ */

int proto_parse_set_vel(const proto_frame_t *frame, proto_cmd_t *out)
{
    if ((frame == NULL) || (out == NULL))
    {
        return 0;
    }
    if ((frame->cmd != PROTO_CMD_SET_VEL) || (frame->len != 4u))
    {
        return 0;
    }
    out->speed = proto_get_i16(&frame->payload[0]);
    out->angular = proto_get_i16(&frame->payload[2]);
    return 1;
}

int proto_parse_status(const proto_frame_t *frame, uint8_t *status, uint8_t *src)
{
    if ((frame == NULL) || (status == NULL) || (src == NULL))
    {
        return 0;
    }
    if ((frame->cmd != PROTO_CMD_STATUS) || (frame->len != 2u))
    {
        return 0;
    }
    *status = frame->payload[0];
    *src = frame->payload[1];
    return 1;
}

/* ============================ 收帧 ============================ */

void proto_rx_reset(proto_rx_t *rx)
{
    rx->idx = 0u;
    rx->len = 0u;
}

void proto_rx_init(proto_rx_t *rx, uint8_t sof0, uint8_t sof1)
{
    rx->sof0 = sof0;
    rx->sof1 = sof1;
    proto_rx_reset(rx);
}

int proto_rx_feed(proto_rx_t *rx, uint8_t byte, proto_frame_t *out)
{
    uint8_t need;

    switch (rx->idx)
    {
    case 0u: /* 等 SOF0 */
        if (byte == rx->sof0)
        {
            rx->buf[0] = byte;
            rx->idx = 1u;
        }
        return 0;

    case 1u: /* 等 SOF1 */
        if (byte == rx->sof1)
        {
            rx->buf[1] = byte;
            rx->idx = 2u;
        }
        else if (byte == rx->sof0) /* 0xA5 0xA5 -> 第二个仍可能是帧头 */
        {
            rx->buf[0] = byte;
        }
        else
        {
            proto_rx_reset(rx);
        }
        return 0;

    case 2u: /* LEN */
        if ((byte < PROTO_MIN_LEN) || (byte > PROTO_MAX_PAYLOAD))
        {
            proto_rx_reset(rx);
            return 0;
        }
        rx->buf[2] = byte;
        rx->len = byte;
        rx->idx = 3u;
        return 0;

    default:
        break;
    }

    need = (uint8_t)(3u + rx->len); /* SOF0 SOF1 LEN + LEN 个负载 */
    if (rx->idx < need)
    {
        rx->buf[rx->idx] = byte;
        rx->idx++;
        return 0;
    }

    /* 到这里 rx->idx == need，当前字节是 CRC */
    if (proto_crc8(&rx->buf[2], (uint16_t)(rx->len + 1u)) == byte)
    {
        uint8_t payload_len = (uint8_t)(rx->len - 1u); /* 去掉 CMD */

        out->cmd = rx->buf[3];
        out->len = payload_len;
        memcpy(out->payload, &rx->buf[4], payload_len);
        proto_rx_reset(rx);
        return 1;
    }

    proto_rx_reset(rx);
    return 0;
}
