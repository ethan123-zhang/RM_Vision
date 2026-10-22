/**
 ******************************************************************************
 * @file    DBUS.h
 * @brief   DT7 接收机（DBUS，100000 8E1）—— USART2
 *
 * 2026-10-22 实测修正：本接收机一帧是 **18 字节通道数据 + 约 12 ms 空闲**，
 * 帧首没有 0xA9。原来的实现要求帧首必须是 0xA9，于是每个字节都在 s_cnt==0
 * 处被丢掉：status 一直只有 uart2_rx、没有 dbus_frame / rc_ok。
 *
 * 现在用「空闲间隔」定界：帧内字节间隔只有 0.1 ms，帧间空闲 >2 ms，二者差
 * 两个数量级，比找帧头可靠。若换成带 0xA9 帧头的标准 DR16（19 字节一帧），
 * 同一套代码也能认：多出来的首字节会被剥掉。
 *
 * 接收中断统一在 uart_rx.c，本文件只保留"喂字节 + 解码 + 判活"。
 ******************************************************************************
 */
#ifndef __DBUS_H
#define __DBUS_H

#include "stm32f1xx_hal.h"

#define DBUS_FRAME_LEN   18u   /* 一帧的通道数据字节数（不含可选的 0xA9） */
#define DBUS_HEADER      0xA9u /* 标准 DR16 帧首；本接收机不输出，兼容用 */
#define DBUS_BUF_LEN     20u   /* 18（无帧头）或 19（带 0xA9）都装得下 */
#define DBUS_GAP_MS      2u    /* 帧间空闲阈值；帧内字节间隔 ≈0.1 ms */
#define DBUS_ANY_TIMEOUT_MS 500u /* uart2_rx 调试位的时间窗：500 ms 内有字节 */
#define DBUS_TIMEOUT_MS  100u  /* 超过 100 ms 没解出整帧 = 遥控器失联 */
#define DBUS_ESTOP_S2    2u    /* s2=2（最下面 ATTI）= 硬急停；中档 3 = 半速 */

typedef struct
{
    int16_t ch0;    /* 右摇杆左右 */
    int16_t ch1;    /* 右摇杆上下 */
    int16_t ch2;    /* 左摇杆左右 */
    int16_t ch3;    /* 左摇杆上下 */
    uint8_t s1;     /* 左三段拨码 */
    uint8_t s2;     /* 右三段拨码 */
} RC_TypeDef;

extern RC_TypeDef rc;

/** 复位收帧状态（上电调用一次，接收中断由 UART_Rx_Start() 挂上） */
void DBUS_Init(void);

/** 收到 1 字节（由 uart_rx.c 转发） */
void DBUS_OnByte(uint8_t byte);

/** 把 18 字节数据帧解成 rc（可单独测试） */
void DBUS_Decode(uint8_t *buf);

/** 100 ms 内有整帧 = 在线 */
uint8_t DBUS_Alive(void);

/** s2 == 3 且数据新鲜 = 硬急停 */
uint8_t DBUS_Estop(void);

/** 累计收到的整帧数 */
uint32_t DBUS_Frames(void);

/** 最近 500 ms 内收到过任意字节（实时：区分"线没通"和"有字节但帧不对"） */
uint8_t DBUS_AnyByte(void);

#endif /* __DBUS_H */
