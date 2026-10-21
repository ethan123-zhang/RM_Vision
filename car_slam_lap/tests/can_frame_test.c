/**
 ******************************************************************************
 * @file    can_frame_test.c
 * @brief   XHSMCUW50 混控帧编码测试（PC 上 gcc 编译，不需要硬件）
 *
 * 锁定的是「接线上车之前」最容易错、又最难现场排查的东西：
 *   - 帧头是不是 CD AB（小端）
 *   - angular / speed 的字节序
 *   - 校验和是不是 0xABCD ^ angular ^ speed
 *
 * 黄金向量与《双驱无刷电机控制器规格书》6.4 节一致。
 * 编译：见 tests/run_tests.ps1
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>

#include "can_frame.h"

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

static void check_frame(const char *tag, int16_t angular, int16_t speed,
                        const uint8_t *want)
{
    uint8_t out[CAN_FRAME_DLC];
    uint16_t n;
    int i;

    memset(out, 0, sizeof(out));
    n = CAN_Frame_BuildMix(out, angular, speed);
    CHECK(n == CAN_FRAME_DLC);
    CHECK(memcmp(out, want, CAN_FRAME_DLC) == 0);

    printf("  %-16s", tag);
    for (i = 0; i < (int)CAN_FRAME_DLC; i++)
    {
        printf("%02X ", out[i]);
    }
    printf("\n");
}

int main(void)
{
    static const uint8_t zero[8] = {0xCD, 0xAB, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xAB};
    static const uint8_t fwd_right[8] = {0xCD, 0xAB, 0x90, 0x01, 0x38, 0xFF, 0x65, 0x55};
    static const uint8_t back_left[8] = {0xCD, 0xAB, 0x18, 0xFC, 0xE8, 0x03, 0x3D, 0x54};

    printf("can_frame_test — XHSMCUW50 混控帧编码\n\n");

    printf("[1] 校验和\n");
    CHECK(CAN_Frame_Checksum(0, 0) == 0xABCDu);
    CHECK(CAN_Frame_Checksum(400, -200) == 0x5565u);
    CHECK(CAN_Frame_Checksum(-1000, 1000) == 0x543Du);
    /* 异或对两个字段对称：交换 angular/speed 校验和不变 */
    CHECK(CAN_Frame_Checksum(-200, 400) == CAN_Frame_Checksum(400, -200));
    printf("  crc(400,-200) = 0x%04X (期望 0x5565)\n", CAN_Frame_Checksum(400, -200));

    printf("\n[2] 黄金向量（逐字节）\n");
    check_frame("set_vel(0,0)", 0, 0, zero);
    check_frame("set_vel(400,-200)", 400, -200, fwd_right);
    check_frame("set_vel(-1000,1000)", -1000, 1000, back_left);

    printf("\n[3] 边界\n");
    {
        uint8_t tmp[CAN_FRAME_DLC];
        CHECK(CAN_Frame_BuildMix(NULL, 0, 0) == 0u);
        CHECK(CAN_Frame_BuildMix(tmp, 32767, -32768) == CAN_FRAME_DLC);
    }

    printf("\n通过 %d 项，失败 %d 项\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
