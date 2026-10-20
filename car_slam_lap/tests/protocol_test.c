/**
 ******************************************************************************
 * @file    protocol_test.c
 * @brief   协议层测试（PC 上 gcc 编译运行，不需要硬件）
 *
 * 覆盖：
 *   1. CRC-8/ATM 与外部标准校验值一致（crc8("123456789") == 0xF4）
 *   2. 组帧字节布局与协议文档逐字节一致
 *   3. 收帧：逐字节喂入、乱码重同步、CRC 错拒收、半帧恢复
 *   4. 上行 status 帧同一套逻辑
 *
 * 编译：见 tests/run_tests.ps1
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>

#include "protocol.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (cond) {                                                      \
            g_pass++;                                                    \
        } else {                                                         \
            g_fail++;                                                    \
            printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                \
    } while (0)

static void print_hex(const char *tag, const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    printf("  %-28s", tag);
    for (i = 0; i < len; i++)
    {
        printf("%02X ", buf[i]);
    }
    printf("\n");
}

/* ------------------------------------------------------------------ */
/* 1. CRC 与外部标准向量                                              */
/* ------------------------------------------------------------------ */
static void test_crc_external_vector(void)
{
    static const uint8_t ascii[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

    printf("[1] CRC-8/ATM\n");
    /* CRC-8/SMBUS 的公开 check 值：poly 0x07 / init 0x00 / 不反转 => 0xF4 */
    CHECK(proto_crc8(ascii, 9) == 0xF4u);

    /* 空数据 + 单字节自检，确认不是巧合 */
    CHECK(proto_crc8(ascii, 0) == 0x00u);
    CHECK(proto_crc8((const uint8_t *)"\x00", 1) == 0x00u);
    printf("  crc8(\"123456789\") = 0x%02X (期望 0xF4)\n", proto_crc8(ascii, 9));
}

/* ------------------------------------------------------------------ */
/* 2. 组帧布局                                                        */
/* ------------------------------------------------------------------ */
static void test_build_layout(void)
{
    uint8_t buf[PROTO_FRAME_DN_LEN];
    uint16_t n;

    printf("[2] build_set_vel 字节布局\n");

    n = proto_build_set_vel(buf, 0, 0);
    CHECK(n == PROTO_FRAME_DN_LEN);
    /* 帧头 / LEN / CMD 固定，数据全 0 */
    CHECK(buf[0] == 0xA5 && buf[1] == 0x5A && buf[2] == 0x05 && buf[3] == 0x01);
    CHECK(buf[4] == 0x00 && buf[5] == 0x00 && buf[6] == 0x00 && buf[7] == 0x00);
    CHECK(buf[8] == proto_crc8(&buf[2], 6));
    print_hex("(0,0)", buf, n);

    /* 400 = 0x0190 -> 90 01 ; -200 = 0xFF38 -> 38 FF（小端） */
    proto_build_set_vel(buf, 400, -200);
    CHECK(buf[4] == 0x90 && buf[5] == 0x01);
    CHECK(buf[6] == 0x38 && buf[7] == 0xFF);
    CHECK(buf[8] == proto_crc8(&buf[2], 6));
    print_hex("(400,-200)", buf, n);

    /* -1000 = 0xFC18 -> 18 FC ; 1000 = 0x03E8 -> E8 03 */
    proto_build_set_vel(buf, -1000, 1000);
    CHECK(buf[4] == 0x18 && buf[5] == 0xFC);
    CHECK(buf[6] == 0xE8 && buf[7] == 0x03);
    print_hex("(-1000,1000)", buf, n);

    /* 超量程必须被钳位，不能把 2000 原样发出去 */
    proto_build_set_vel(buf, 2000, -2000);
    CHECK(buf[4] == 0xE8 && buf[5] == 0x03);
    CHECK(buf[6] == 0x18 && buf[7] == 0xFC);
    print_hex("(2000,-2000) -> clamp", buf, n);

    proto_build_set_vel(buf, -1234, 567);
    CHECK(buf[4] == 0x18 && buf[5] == 0xFC);
    print_hex("(-1234,567) -> clamp", buf, n);
}

/* ------------------------------------------------------------------ */
/* 3. 收帧                                                           */
/* ------------------------------------------------------------------ */
static void feed_all(proto_rx_t *rx, const uint8_t *buf, uint16_t len,
                     proto_frame_t *out, int *got)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        if (proto_rx_feed(rx, buf[i], out))
        {
            (*got)++;
        }
    }
}

static void test_roundtrip(void)
{
    static const int16_t cases[][2] = {
        {0, 0}, {400, -200}, {-1000, 1000}, {1000, 1000}, {-1, 1}};
    uint8_t buf[PROTO_FRAME_DN_LEN];
    proto_rx_t rx;
    proto_frame_t frame;
    proto_cmd_t cmd;
    size_t i;

    printf("[3] 逐字节收帧 + 回环\n");

    proto_rx_init(&rx, PROTO_DN_SOF0, PROTO_DN_SOF1);

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        int got = 0;

        proto_build_set_vel(buf, cases[i][0], cases[i][1]);
        feed_all(&rx, buf, PROTO_FRAME_DN_LEN, &frame, &got);

        CHECK(got == 1);
        CHECK(frame.cmd == PROTO_CMD_SET_VEL);
        CHECK(frame.len == 4);
        CHECK(proto_parse_set_vel(&frame, &cmd) == 1);
        CHECK(cmd.speed == cases[i][0]);
        CHECK(cmd.angular == cases[i][1]);
    }
}

static void test_resync(void)
{
    uint8_t frame_buf[PROTO_FRAME_DN_LEN];
    /* 前面塞乱码：0x00 0x5A 0x5A 0xA5 0xFF（含半个帧头） */
    static const uint8_t junk[] = {0x00, 0x5A, 0x5A, 0xA5, 0xFF, 0x13};
    proto_rx_t rx;
    proto_frame_t frame;
    proto_cmd_t cmd;
    int got = 0;
    uint16_t i;

    printf("[4] 乱码后重同步\n");

    proto_rx_init(&rx, PROTO_DN_SOF0, PROTO_DN_SOF1);
    proto_build_set_vel(frame_buf, 123, -456);

    for (i = 0; i < sizeof(junk); i++)
    {
        proto_rx_feed(&rx, junk[i], &frame);
    }
    feed_all(&rx, frame_buf, PROTO_FRAME_DN_LEN, &frame, &got);

    CHECK(got == 1);
    CHECK(proto_parse_set_vel(&frame, &cmd) == 1);
    CHECK(cmd.speed == 123 && cmd.angular == -456);
}

static void test_bad_crc(void)
{
    uint8_t bad[PROTO_FRAME_DN_LEN];
    uint8_t good[PROTO_FRAME_DN_LEN];
    proto_rx_t rx;
    proto_frame_t frame;
    proto_cmd_t cmd;
    int got = 0;

    printf("[5] CRC 错必须拒收，且不影响下一帧\n");

    proto_rx_init(&rx, PROTO_DN_SOF0, PROTO_DN_SOF1);

    proto_build_set_vel(bad, 500, 500);
    bad[8] ^= 0xFF; /* 打坏 CRC */
    feed_all(&rx, bad, PROTO_FRAME_DN_LEN, &frame, &got);
    CHECK(got == 0);

    proto_build_set_vel(good, 321, -654);
    feed_all(&rx, good, PROTO_FRAME_DN_LEN, &frame, &got);
    CHECK(got == 1);
    CHECK(proto_parse_set_vel(&frame, &cmd) == 1);
    CHECK(cmd.speed == 321 && cmd.angular == -654);
}

static void test_half_frame(void)
{
    uint8_t a[PROTO_FRAME_DN_LEN];
    uint8_t b[PROTO_FRAME_DN_LEN];
    proto_rx_t rx;
    proto_frame_t frame;
    proto_cmd_t cmd;
    int got = 0;

    printf("[6] 半帧（掉字节）后能恢复\n");

    proto_rx_init(&rx, PROTO_DN_SOF0, PROTO_DN_SOF1);

    proto_build_set_vel(a, 111, 222);
    feed_all(&rx, a, 4, &frame, &got); /* 只喂前 4 字节就断 */
    CHECK(got == 0);

    proto_rx_reset(&rx); /* 工程上由超时/上电复位处理 */
    proto_build_set_vel(b, -777, 0);
    feed_all(&rx, b, PROTO_FRAME_DN_LEN, &frame, &got);
    CHECK(got == 1);
    CHECK(proto_parse_set_vel(&frame, &cmd) == 1);
    CHECK(cmd.speed == -777 && cmd.angular == 0);
}

static void test_status_frame(void)
{
    uint8_t buf[PROTO_FRAME_UP_LEN];
    proto_rx_t rx; /* X5 侧用上行帧头 */
    proto_frame_t frame;
    uint8_t status = 0;
    uint8_t src = 0xFF;
    uint16_t n;
    int got = 0;

    printf("[7] 上行 status 帧\n");

    n = proto_build_status(buf, PROTO_ST_CAN_OK | PROTO_ST_LINK_OK, PROTO_SRC_SERIAL);
    CHECK(n == PROTO_FRAME_UP_LEN);
    CHECK(buf[0] == 0x5A && buf[1] == 0xA5 && buf[2] == 0x03 && buf[3] == 0x81);
    CHECK(buf[6] == proto_crc8(&buf[2], 4));
    print_hex("status(can|link, serial)", buf, n);

    proto_rx_init(&rx, PROTO_UP_SOF0, PROTO_UP_SOF1);
    feed_all(&rx, buf, n, &frame, &got);
    CHECK(got == 1);
    CHECK(proto_parse_status(&frame, &status, &src) == 1);
    CHECK(status == (PROTO_ST_CAN_OK | PROTO_ST_LINK_OK));
    CHECK(src == PROTO_SRC_SERIAL);

    /* 用下行帧头初始化时，上行帧必须收不到 */
    proto_rx_init(&rx, PROTO_DN_SOF0, PROTO_DN_SOF1);
    got = 0;
    feed_all(&rx, buf, n, &frame, &got);
    CHECK(got == 0);
}

/* ------------------------------------------------------------------ */
/* 7.5 黄金向量：C 与 Python 两侧都断言这串字节                        */
/* ------------------------------------------------------------------ */
static void test_golden_vectors(void)
{
    /* 这些值来自协议文档定义的布局 + CRC-8/ATM。
       Python 侧 car_bridge/test/test_protocol.py 里是同一组常量，
       两边任何一侧改错（CRC 算法、字节序、LEN 含义）都会在这里炸掉。 */
    static const struct
    {
        int16_t speed;
        int16_t angular;
        const char *hex;
    } vel_cases[] = {
        {0, 0, "A55A050100000000EF"},
        {400, -200, "A55A0501900138FF70"},
        {-1000, 1000, "A55A050118FCE8034C"},
        {123, -456, "A55A05017B0038FEF2"},
    };
    static const struct
    {
        uint8_t status;
        uint8_t src;
        const char *hex;
    } st_cases[] = {
        {PROTO_ST_CAN_OK | PROTO_ST_LINK_OK, PROTO_SRC_SERIAL, "5AA50381030162"},
        {0x00, PROTO_SRC_NONE, "5AA5038100005A"},
    };
    char got[64];
    uint8_t buf[PROTO_FRAME_DN_LEN];
    uint16_t i;
    size_t k;

    printf("[7.5] 黄金向量\n");

    for (k = 0; k < sizeof(vel_cases) / sizeof(vel_cases[0]); k++)
    {
        proto_build_set_vel(buf, vel_cases[k].speed, vel_cases[k].angular);
        for (i = 0; i < PROTO_FRAME_DN_LEN; i++)
        {
            (void)snprintf(&got[i * 2], 3, "%02X", buf[i]);
        }
        if (strcmp(got, vel_cases[k].hex) != 0)
        {
            g_fail++;
            printf("  FAIL  set_vel(%d,%d)\n         期望 %s\n         实际 %s\n",
                   vel_cases[k].speed, vel_cases[k].angular, vel_cases[k].hex, got);
        }
        else
        {
            g_pass++;
        }
    }

    for (k = 0; k < sizeof(st_cases) / sizeof(st_cases[0]); k++)
    {
        proto_build_status(buf, st_cases[k].status, st_cases[k].src);
        for (i = 0; i < PROTO_FRAME_UP_LEN; i++)
        {
            (void)snprintf(&got[i * 2], 3, "%02X", buf[i]);
        }
        if (strcmp(got, st_cases[k].hex) != 0)
        {
            g_fail++;
            printf("  FAIL  status(0x%02X,%u)\n         期望 %s\n         实际 %s\n",
                   st_cases[k].status, st_cases[k].src, st_cases[k].hex, got);
        }
        else
        {
            g_pass++;
        }
    }
}

/* ------------------------------------------------------------------ */
/* 8. 打印固定帧，供 Python 侧比对                                     */
/* ------------------------------------------------------------------ */
static void print_vectors(void)
{
    uint8_t buf[PROTO_FRAME_DN_LEN];

    printf("\n[8] 交叉校验向量（Python test_protocol.py 里硬编码同样的值）\n");

    proto_build_set_vel(buf, 0, 0);
    print_hex("set_vel(0,0)", buf, PROTO_FRAME_DN_LEN);
    proto_build_set_vel(buf, 400, -200);
    print_hex("set_vel(400,-200)", buf, PROTO_FRAME_DN_LEN);
    proto_build_set_vel(buf, -1000, 1000);
    print_hex("set_vel(-1000,1000)", buf, PROTO_FRAME_DN_LEN);
    proto_build_set_vel(buf, 123, -456);
    print_hex("set_vel(123,-456)", buf, PROTO_FRAME_DN_LEN);

    proto_build_status(buf, PROTO_ST_CAN_OK | PROTO_ST_LINK_OK, PROTO_SRC_SERIAL);
    print_hex("status(0x03,1)", buf, PROTO_FRAME_UP_LEN);
    proto_build_status(buf, 0x00, PROTO_SRC_NONE);
    print_hex("status(0x00,0)", buf, PROTO_FRAME_UP_LEN);
}

int main(void)
{
    printf("protocol_test — X5<->C8T6 链路协议\n");
    printf("SOF dn=%02X %02X  up=%02X %02X  crc=poly 0x07\n\n",
           PROTO_DN_SOF0, PROTO_DN_SOF1, PROTO_UP_SOF0, PROTO_UP_SOF1);

    test_crc_external_vector();
    test_build_layout();
    test_roundtrip();
    test_resync();
    test_bad_crc();
    test_half_frame();
    test_status_frame();
    test_golden_vectors();
    print_vectors();

    printf("\n通过 %d 项，失败 %d 项\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
