#include "dbus.h"

RC_TypeDef rc;
uint8_t dbus_buf[18];
uint8_t dbus_cnt = 0;
uint8_t  rx_byte;

void DBUS_Init(UART_HandleTypeDef *huart)
{
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

void DBUS_Decode(uint8_t *buf)
{
    rc.ch0 = ((buf[0] | (buf[1] << 8)) & 0x07FF) - 1024;
    rc.ch1 = (((buf[1] >> 3) | (buf[2] << 5)) & 0x07FF) - 1024;
    rc.ch2 = (((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10)) & 0x07FF) - 1024;
    rc.ch3 = (((buf[4] >> 1) | (buf[5] << 7)) & 0x07FF) - 1024;

    /* S1/S2 在字节5的高4位（bit偏移44/46，非字节17！见数据手册） */
    rc.s2 = (buf[5] >> 4) & 0x03;   /* S1 = 字节5 bit4-5，值 1上 2下 3中 */
    rc.s1 = (buf[5] >> 6) & 0x03;   /* S2 = 字节5 bit6-7 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        dbus_buf[dbus_cnt++] = rx_byte;
        if (dbus_cnt >= 18)
        {
            DBUS_Decode(dbus_buf);
            dbus_cnt = 0;
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);  /* 关键：必须重新启动下一次接收 */
    }
}
