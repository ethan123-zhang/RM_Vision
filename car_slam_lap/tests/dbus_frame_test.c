/**
 ******************************************************************************
 * @file    dbus_frame_test.c
 * @brief   DBUS 分帧 + 解码测试（PC 上 gcc 编译，不需要硬件）
 *
 * 2026-10-22 实测：本接收机输出 18 字节通道数据，**帧首没有 0xA9**，帧间
 * 空闲约 12 ms。固件原来要求帧首固定 0xA9，结果每个字节都被丢掉，status 里
 * 一直只有 uart2_rx。这个测试锁住三件事：
 *   1. 靠空闲间隔定界的 18 字节帧能解出来（含 s1/s2）；
 *   2. 带 0xA9 帧头的 19 字节变体也能解（剥头）；
 *   3. 半帧、噪声、s1/s2=0 的假帧不能污染 rc。
 *
 * 编译：见 tests/run_tests.ps1
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>

#include "DBUS.h"

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

/* HAL_GetTick 替身：测试手动控制"时间"，才能造出帧间空闲 */
static uint32_t g_tick;

uint32_t HAL_GetTick(void)
{
    return g_tick;
}

/* 实测抓包（_extract/ttl.bin）里的真实帧：摇杆居中、s1=1、s2=3(中档=半速) */
static const uint8_t k_center18[18] = {
    0x00, 0x04, 0x20, 0x00, 0x01, 0x78,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x04
};

/* ch0=+100, ch1=-50, ch2=0, ch3=+200, s1=1, s2=2(最下面=急停) */
static const uint8_t k_offcenter18[18] = {
    0x64, 0x74, 0x1E, 0x00, 0x91, 0x69,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x04
};

static void feed(const uint8_t *buf, int n)
{
    int i;
    for (i = 0; i < n; i++)
    {
        DBUS_OnByte(buf[i]);
    }
}

/* 隔 14 ms 再喂下一帧的第一个字节：上一帧在这里收尾并解码 */
static void next_frame_head(uint8_t first)
{
    g_tick += 14u;
    DBUS_OnByte(first);
}

static void expect_center(const char *tag)
{
    printf("  %-22s", tag);
    CHECK(rc.ch0 == 0);
    CHECK(rc.ch1 == 0);
    CHECK(rc.ch2 == 0);
    CHECK(rc.ch3 == 0);
    CHECK(rc.s1 == 1u);
    CHECK(rc.s2 == 3u);
    printf("ch=(%d,%d,%d,%d) s1=%u s2=%u\n",
           rc.ch0, rc.ch1, rc.ch2, rc.ch3, rc.s1, rc.s2);
}

int main(void)
{
    uint8_t buf[19];

    printf("dbus_frame_test — 18 字节无帧头帧的分帧/解码\n\n");

    /* ---------------------------------------------------------------- 1 */
    printf("[1] 实测帧（18 字节、无 0xA9）\n");
    g_tick = 1000u;
    DBUS_Init();
    feed(k_center18, 18);
    CHECK(DBUS_Frames() == 0u);       /* 帧末才收满，等下一个空闲才收尾 */
    CHECK(DBUS_AnyByte() == 1u);
    next_frame_head(k_center18[0]);
    CHECK(DBUS_Frames() == 1u);
    expect_center("空闲定界");
    CHECK(DBUS_Alive() == 1u);
    CHECK(DBUS_Estop() == 0u);        /* s2=3 是中档（半速），不是急停 */

    /* ---------------------------------------------------------------- 2 */
    printf("\n[2] 带 0xA9 帧头的 19 字节变体\n");
    g_tick = 2000u;
    DBUS_Init();
    buf[0] = 0xA9u;
    memcpy(&buf[1], k_center18, 18);
    feed(buf, 19);
    next_frame_head(0x00);
    CHECK(DBUS_Frames() == 1u);
    expect_center("剥掉帧头");
    CHECK(DBUS_Estop() == 0u);

    /* ---------------------------------------------------------------- 3 */
    printf("\n[3] 半帧后重同步\n");
    g_tick = 3000u;
    DBUS_Init();
    feed(k_center18, 10);             /* 只来了 10 字节就断了 */
    next_frame_head(k_center18[0]);   /* 空闲：半帧丢弃，这个字节开新帧 */
    CHECK(DBUS_Frames() == 0u);
    feed(&k_center18[1], 17);         /* 补满 18 字节 */
    next_frame_head(0x00);
    CHECK(DBUS_Frames() == 1u);
    expect_center("半帧丢弃后正常");

    /* ---------------------------------------------------------------- 4 */
    printf("\n[4] 噪声/非法帧不污染 rc\n");
    g_tick = 4000u;
    DBUS_Init();
    memcpy(buf, k_center18, 18);
    buf[7] = 0xFFu;                   /* 保留字节被噪声打乱 */
    feed(buf, 18);
    next_frame_head(0x00);
    CHECK(DBUS_Frames() == 0u);

    g_tick = 4500u;
    DBUS_Init();                      /* 上一段噪声留在缓冲里，先复位 */
    memcpy(buf, k_center18, 18);
    buf[5] = 0x00u;                   /* s1/s2 = 0：这台遥控器的 OFF 档就是 0 */
    feed(buf, 18);
    next_frame_head(0x00);
    CHECK(DBUS_Frames() == 1u);       /* 合法帧：0 不能当噪声丢掉 */
    CHECK(rc.s1 == 0u);
    CHECK(rc.s2 == 0u);
    CHECK(DBUS_Estop() == 0u);
    printf("  %-22s 0 是合法档位，未被丢\n", "s1/s2=0");

    /* ---------------------------------------------------------------- 5 */
    printf("\n[5] 摇杆偏移帧\n");
    g_tick = 5000u;
    DBUS_Init();
    feed(k_offcenter18, 18);
    next_frame_head(k_offcenter18[0]);
    CHECK(DBUS_Frames() == 1u);
    CHECK(rc.ch0 == 100);
    CHECK(rc.ch1 == -50);
    CHECK(rc.ch2 == 0);
    CHECK(rc.ch3 == 200);
    CHECK(rc.s1 == 1u);
    CHECK(rc.s2 == 2u);
    CHECK(DBUS_Estop() == 1u);        /* s2=2（最下面）= 急停 */
    printf("  %-22s ch=(%d,%d,%d,%d) s1=%u s2=%u\n", "解出的摇杆/拨码",
           rc.ch0, rc.ch1, rc.ch2, rc.ch3, rc.s1, rc.s2);

    /* ---------------------------------------------------------------- 6 */
    printf("\n[6] 超时判活 / 字节活动位\n");
    CHECK(DBUS_AnyByte() == 1u);      /* 刚喂过字节：500 ms 内应为 1 */
    g_tick += 150u;
    CHECK(DBUS_Alive() == 0u);
    CHECK(DBUS_Estop() == 0u);        /* 数据不新鲜时不能锁死急停 */
    CHECK(DBUS_AnyByte() == 1u);      /* 150 ms 还在窗口内 */
    g_tick += 400u;
    CHECK(DBUS_AnyByte() == 0u);      /* 超过 500 ms 没有新字节 */
    CHECK(DBUS_Frames() > 0u);        /* 整帧计数是锁存的，不受影响 */

    printf("\n通过 %d 项，失败 %d 项\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
