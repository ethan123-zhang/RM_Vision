/**
 ******************************************************************************
 * @file    slcan.c
 * @brief   slcan 文本协议编码实现（无 HAL 依赖，PC 上可直接编译测试）
 ******************************************************************************
 */
#include "slcan.h"

static const char SLCAN_HEX[] = "0123456789ABCDEF";

/** 把一个字节写成 2 位大写十六进制 */
static void slcan_hex8(char *p, uint8_t v)
{
    p[0] = SLCAN_HEX[(v >> 4) & 0x0Fu];
    p[1] = SLCAN_HEX[v & 0x0Fu];
}

static uint16_t slcan_encode_data(const uint8_t *data, uint8_t len, char *out)
{
    uint8_t i;

    if (data == 0 || out == 0 || len > SLCAN_MAX_DATA)
    {
        return 0u;
    }
    out[0] = SLCAN_HEX[len & 0x0Fu];
    for (i = 0u; i < len; i++)
    {
        slcan_hex8(&out[1u + 2u * i], data[i]);
    }
    return (uint16_t)(1u + 2u * len);
}

uint16_t slcan_encode_ext(uint32_t id, const uint8_t *data, uint8_t len, char *out)
{
    uint8_t i;
    uint16_t n = 0u;

    if (out == 0 || id > SLCAN_ID_EXT_MAX)
    {
        return 0u;
    }
    out[n++] = 'T';
    for (i = 0u; i < 8u; i++)
    {
        out[n++] = SLCAN_HEX[(id >> (28u - 4u * i)) & 0x0Fu];
    }
    {
        uint16_t k = slcan_encode_data(data, len, &out[n]);
        if (k == 0u)
        {
            return 0u;
        }
        n = (uint16_t)(n + k);
    }
    out[n++] = '\r';
    return n;
}

uint16_t slcan_encode_std(uint32_t id, const uint8_t *data, uint8_t len, char *out)
{
    uint8_t i;
    uint16_t n = 0u;

    if (out == 0 || id > SLCAN_ID_STD_MAX)
    {
        return 0u;
    }
    out[n++] = 't';
    for (i = 0u; i < 3u; i++)
    {
        out[n++] = SLCAN_HEX[(id >> (8u - 4u * i)) & 0x0Fu];
    }
    {
        uint16_t k = slcan_encode_data(data, len, &out[n]);
        if (k == 0u)
        {
            return 0u;
        }
        n = (uint16_t)(n + k);
    }
    out[n++] = '\r';
    return n;
}

uint8_t slcan_bitrate_code(uint32_t bps)
{
    switch (bps)
    {
    case 10000u:
        return 0u;
    case 20000u:
        return 1u;
    case 50000u:
        return 2u;
    case 100000u:
        return 3u;
    case 125000u:
        return 4u;
    case 250000u:
        return 5u;
    case 500000u:
        return 6u;
    case 800000u:
        return 7u;
    case 1000000u:
        return 8u;
    default:
        return 0xFFu;
    }
}

uint16_t slcan_encode_bitrate(uint8_t code, char *out)
{
    if (out == 0 || code > 8u)
    {
        return 0u;
    }
    out[0] = 'S';
    out[1] = (char)('0' + code);
    out[2] = '\r';
    return 3u;
}

uint16_t slcan_encode_open(char *out)
{
    if (out == 0)
    {
        return 0u;
    }
    out[0] = 'O';
    out[1] = '\r';
    return 2u;
}

uint16_t slcan_encode_close(char *out)
{
    if (out == 0)
    {
        return 0u;
    }
    out[0] = 'C';
    out[1] = '\r';
    return 2u;
}
