/**
 ******************************************************************************
 * @file    DBUS.c
 * @brief   DT7 接收机解码 —— USART2，100000 8E1
 *
 * 分帧方式见 DBUS.h 的文件头说明：靠帧间空闲（>2 ms）定界，不再要求 0xA9。
 ******************************************************************************
 */
#include "DBUS.h"

RC_TypeDef rc;

static uint8_t s_buf[DBUS_BUF_LEN];
static uint8_t s_cnt;
static uint32_t s_last_byte_ms;
static volatile uint32_t s_last_ms;
static volatile uint32_t s_frames;
static volatile uint8_t s_any_byte;

/**
 * @brief  把缓冲里攒的这一段当成一帧处理；不合规就丢掉
 *
 * 兼容两种长度：
 *   18 字节 —— 本接收机实测格式，直接解；
 *   19 字节且首字节 0xA9 —— 标准 DR16 变体，剥掉帧头再解。
 */
static void dbus_finish_frame(void)
{
    uint8_t n = s_cnt;
    uint8_t i;
    uint8_t *p = s_buf;

    s_cnt = 0u;

    if ((n == (uint8_t)(DBUS_FRAME_LEN + 1u)) && (p[0] == DBUS_HEADER))
    {
        p++;
        n = DBUS_FRAME_LEN;
    }
    if (n != DBUS_FRAME_LEN)
    {
        return; /* 半帧/多字节：等下一个空闲重新定界 */
    }

    /* 实测帧的字节 6..16 恒为 0。没有帧头可校验时，这是区分"真帧"和
       "PA3 噪声凑出来的 18 字节"最便宜的办法（随机字节同时为 0 的概率≈0）。
       注意：s1/s2 可能是 0（这台遥控器的 OFF 档就编码成 0），不能拿它当合法性
       判据——2026-10-22 就是这么把 OFF 档的整帧误杀掉的。 */
    for (i = 6u; i <= 16u; i++)
    {
        if (p[i] != 0u)
        {
            return;
        }
    }

    DBUS_Decode(p);
    s_last_ms = HAL_GetTick();
    s_frames++;
}

void DBUS_Init(void)
{
    s_cnt = 0u;
    s_frames = 0u;
    s_any_byte = 0u;
    s_last_ms = HAL_GetTick();
    s_last_byte_ms = s_last_ms;
}

void DBUS_Decode(uint8_t *buf)
{
    rc.ch0 = ((buf[0] | (buf[1] << 8)) & 0x07FF) - 1024;
    rc.ch1 = (((buf[1] >> 3) | (buf[2] << 5)) & 0x07FF) - 1024;
    rc.ch2 = (((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10)) & 0x07FF) - 1024;
    rc.ch3 = (((buf[4] >> 1) | (buf[5] << 7)) & 0x07FF) - 1024;

    /* S1/S2 在字节5的高4位（bit偏移44/46，非字节17！见数据手册） */
    rc.s2 = (uint8_t)((buf[5] >> 4) & 0x03);
    rc.s1 = (uint8_t)((buf[5] >> 6) & 0x03);
}

void DBUS_OnByte(uint8_t byte)
{
    uint32_t now = HAL_GetTick();

    s_any_byte = 1u;

    /* 帧内字节间隔 0.1 ms，帧间空闲 ≈12 ms：>2 ms 一定是新帧的第一字节，
       先把上一段收尾，再从这个字节开始攒下一帧。 */
    if ((s_cnt > 0u) && ((uint32_t)(now - s_last_byte_ms) > DBUS_GAP_MS))
    {
        dbus_finish_frame();
    }
    s_last_byte_ms = now;

    if (s_cnt < DBUS_BUF_LEN)
    {
        s_buf[s_cnt] = byte;
        s_cnt++;
    }
    else
    {
        /* 异常长的突发：不是本协议，丢掉，等下一个空闲间隔重新定界 */
        s_cnt = 0u;
    }
}

uint8_t DBUS_Alive(void)
{
    return (uint8_t)((HAL_GetTick() - s_last_ms) < DBUS_TIMEOUT_MS);
}

uint8_t DBUS_Estop(void)
{
    /* 必须在数据新鲜时才算急停：接收机拔掉后 rc 会停留在最后一帧，
       如果那一帧恰好是 s2=3，就会永久锁死，看起来像"串口指令没生效"。 */
    if ((DBUS_Alive() != 0u) && (rc.s2 == DBUS_ESTOP_S2))
    {
        return 1u;
    }
    return 0u;
}

uint32_t DBUS_Frames(void)
{
    return s_frames;
}

uint8_t DBUS_AnyByte(void)
{
    /* 实时位：500 ms 内有字节才算 1。
       以前是"上电以来收到过就永远为 1"，接收机中途停了也看不出来。 */
    if (s_any_byte == 0u)
    {
        return 0u;
    }
    return (uint8_t)((HAL_GetTick() - s_last_byte_ms) < DBUS_ANY_TIMEOUT_MS);
}
