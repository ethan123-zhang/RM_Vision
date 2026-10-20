/**
 ******************************************************************************
 * @file    slcan_test.c
 * @brief   slcan 编码测试（PC 上 gcc 编译，不需要硬件）
 *
 * 断言的是**逐字节**的字符串：串口转CAN 模块收到一个字符不对就会静默丢帧，
 * 现场表现是"电机不动但程序没报错"，所以必须在接线之前把编码锁死。
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>

#include "slcan.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (cond) {                                                     \
            g_pass++;                                                   \
        } else {                                                        \
            g_fail++;                                                   \
            printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, #cond);   \
        }                                                               \
    } while (0)

static void check_str(const char *tag, const char *got, uint16_t got_len,
                      const char *expect)
{
    if ((got_len == (uint16_t)strlen(expect)) &&
        (memcmp(got, expect, strlen(expect)) == 0))
    {
        g_pass++;
        printf("  ok    %-24s %s\n", tag, expect);
    }
    else
    {
        g_fail++;
        printf("  FAIL  %-24s 期望 \"%s\"  实际 \"%.*s\" (len=%u)\n",
               tag, expect, (int)got_len, got, got_len);
    }
}

/* 电机控制器的 8 字节混控帧：帧头 0xABCD + angular + speed + 16bit 异或（小端） */
static void build_mix_frame(int16_t angular, int16_t speed, uint8_t *buf)
{
    uint16_t start = 0xABCDu;
    uint16_t crc = (uint16_t)(start ^ (uint16_t)angular ^ (uint16_t)speed);

    buf[0] = (uint8_t)(start & 0xFFu);
    buf[1] = (uint8_t)((start >> 8) & 0xFFu);
    buf[2] = (uint8_t)((uint16_t)angular & 0xFFu);
    buf[3] = (uint8_t)(((uint16_t)angular >> 8) & 0xFFu);
    buf[4] = (uint8_t)((uint16_t)speed & 0xFFu);
    buf[5] = (uint8_t)(((uint16_t)speed >> 8) & 0xFFu);
    buf[6] = (uint8_t)(crc & 0xFFu);
    buf[7] = (uint8_t)((crc >> 8) & 0xFFu);
}

static void test_ext_frame(void)
{
    uint8_t mix[8];
    char out[SLCAN_BUF_LEN];
    uint16_t n;

    printf("[1] 扩展帧 (ID 0x1314, DLC 8)\n");

    /* CRC = 0xABCD ^ 0 ^ 0 = 0xABCD -> CD AB */
    build_mix_frame(0, 0, mix);
    CHECK(mix[0] == 0xCD && mix[1] == 0xAB && mix[6] == 0xCD && mix[7] == 0xAB);
    n = slcan_encode_ext(0x1314u, mix, 8u, out);
    check_str("mix(0,0)", out, n, "T000013148CDAB00000000CDAB\r");

    /* angular=400=0x0190 -> 90 01 ; speed=-200=0xFF38 -> 38 FF
       CRC = 0xABCD ^ 0x0190 ^ 0xFF38 = 0x5565 -> 65 55 */
    build_mix_frame(400, -200, mix);
    CHECK(mix[2] == 0x90 && mix[3] == 0x01);
    CHECK(mix[4] == 0x38 && mix[5] == 0xFF);
    CHECK(mix[6] == 0x65 && mix[7] == 0x55);
    n = slcan_encode_ext(0x1314u, mix, 8u, out);
    check_str("mix(400,-200)", out, n, "T000013148CDAB900138FF6555\r");

    /* 边界 ID：29 位全 1 */
    build_mix_frame(0, 0, mix);
    n = slcan_encode_ext(0x1FFFFFFFu, mix, 1u, out);
    check_str("id=29bit max, len=1", out, n, "T1FFFFFFF1CD\r");
}

static void test_std_frame(void)
{
    uint8_t data[2] = {0x11u, 0x22u};
    char out[SLCAN_BUF_LEN];
    uint16_t n;

    printf("[2] 标准帧 (ID 0x123, 2 字节)\n");
    n = slcan_encode_std(0x123u, data, 2u, out);
    check_str("std(0x123)", out, n, "t12321122\r");
}

static void test_commands(void)
{
    char out[SLCAN_BUF_LEN];
    uint16_t n;

    printf("[3] 通道命令 / 波特率\n");

    CHECK(slcan_bitrate_code(250000u) == 5u);   /* 电机控制器实测 250 kbit/s */
    CHECK(slcan_bitrate_code(500000u) == 6u);
    CHECK(slcan_bitrate_code(1000000u) == 8u);
    CHECK(slcan_bitrate_code(123456u) == 0xFFu);

    n = slcan_encode_bitrate(5u, out);
    check_str("bitrate 250k", out, n, "S5\r");

    n = slcan_encode_open(out);
    check_str("open", out, n, "O\r");

    n = slcan_encode_close(out);
    check_str("close", out, n, "C\r");
}

static void test_rejects(void)
{
    uint8_t data[9] = {0};
    char out[SLCAN_BUF_LEN];

    printf("[4] 非法参数必须拒绝（返回 0，不写半个帧）\n");

    CHECK(slcan_encode_ext(0x20000000u, data, 1u, out) == 0u);  /* ID 超 29 位 */
    CHECK(slcan_encode_std(0x800u, data, 1u, out) == 0u);       /* ID 超 11 位 */
    CHECK(slcan_encode_ext(0x1314u, data, 9u, out) == 0u);      /* DLC > 8 */
    CHECK(slcan_encode_ext(0x1314u, NULL, 1u, out) == 0u);      /* 数据为空 */
    CHECK(slcan_encode_ext(0x1314u, data, 1u, NULL) == 0u);     /* 输出为空 */
    CHECK(slcan_encode_bitrate(9u, out) == 0u);
}

int main(void)
{
    printf("slcan_test — 串口转CAN 文本协议编码\n\n");
    test_ext_frame();
    test_std_frame();
    test_commands();
    test_rejects();
    printf("\n通过 %d 项，失败 %d 项\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
