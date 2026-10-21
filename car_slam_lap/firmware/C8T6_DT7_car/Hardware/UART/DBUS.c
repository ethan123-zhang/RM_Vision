/**
 ******************************************************************************
 * @file    DBUS.c
 * @brief   DT7 接收机解码 —— USART2，100000 8E1
 ******************************************************************************
 */
#include "DBUS.h"

RC_TypeDef rc;

static uint8_t s_buf[DBUS_FRAME_LEN];
static uint8_t s_cnt;
static volatile uint32_t s_last_ms;
static volatile uint32_t s_frames;

void DBUS_Init(void)
{
    s_cnt = 0u;
    s_frames = 0u;
    s_last_ms = HAL_GetTick();
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
    /* 帧头同步：接收机没接/PA3 悬空时会有随机字节，不校验帧头就会
       把 18 个噪声字节当成一帧，DBUS_Alive() 误判为在线，进而用随机
       摇杆值驱动电机。 */
    if ((s_cnt == 0u) && (byte != DBUS_HEADER))
    {
        return;
    }

    s_buf[s_cnt] = byte;
    s_cnt++;
    if (s_cnt < DBUS_FRAME_LEN)
    {
        return;
    }

    s_cnt = 0u;
    DBUS_Decode(s_buf);
    s_last_ms = HAL_GetTick();
    s_frames++;
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
