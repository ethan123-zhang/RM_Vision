#ifndef __DBUS_H
#define __DBUS_H
#include "stm32f1xx_hal.h"
#include "usart.h"

typedef struct
{
    int16_t ch0;    //右摇杆左右
    int16_t ch1;    //右摇杆上下
    int16_t ch2;    //左摇杆左右
    int16_t ch3;    //左摇杆上下
    uint8_t  s1;     //左三段拨码
    uint8_t  s2;     //右三段拨码
}RC_TypeDef;

void DBUS_Init(UART_HandleTypeDef *huart);
void DBUS_Decode(uint8_t *buf);

#endif
