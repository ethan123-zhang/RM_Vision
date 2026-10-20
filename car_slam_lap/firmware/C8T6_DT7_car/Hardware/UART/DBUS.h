/**
 ******************************************************************************
 * @file    DBUS.h
 * @brief   DT7 接收机（DBUS，100000 8E1，18 字节一帧）—— USART2
 *
 * 改动说明：接收中断统一挪到 uart_rx.c，本文件只保留"喂字节 + 解码 + 判活"，
 * 解码逻辑一行没动。遥控器现在挂在 USART2（PA2/PA3）上。
 ******************************************************************************
 */
#ifndef __DBUS_H
#define __DBUS_H

#include "stm32f1xx_hal.h"

#define DBUS_FRAME_LEN   18u   /* DBUS 固定 18 字节一帧 */
#define DBUS_TIMEOUT_MS  100u  /* 超过 100 ms 没整帧 = 遥控器失联 */
#define DBUS_ESTOP_S2    3u    /* s2 拨到底 = 硬急停（压过串口） */

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

/** 把 18 字节原始帧解成 rc（可单独测试） */
void DBUS_Decode(uint8_t *buf);

/** 100 ms 内有整帧 = 在线 */
uint8_t DBUS_Alive(void);

/** s2 == 3 且数据新鲜 = 硬急停 */
uint8_t DBUS_Estop(void);

/** 累计收到的整帧数 */
uint32_t DBUS_Frames(void);

#endif /* __DBUS_H */
