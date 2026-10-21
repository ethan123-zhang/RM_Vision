/**
 ******************************************************************************
 * @file    uart_rx.c
 * @brief   三路串口接收中断分发
 ******************************************************************************
 */
#include "uart_rx.h"

#include "DBUS.h"
#include "usart.h"
#include "x5_link.h"

static uint8_t s_byte1;
static uint8_t s_byte2;
static uint8_t s_byte3;

void UART_Rx_Start(void)
{
    (void)HAL_UART_Receive_IT(&huart1, &s_byte1, 1u);
    (void)HAL_UART_Receive_IT(&huart2, &s_byte2, 1u);
    (void)HAL_UART_Receive_IT(&huart3, &s_byte3, 1u);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        X5_Link_OnByte(s_byte1);
        (void)HAL_UART_Receive_IT(&huart1, &s_byte1, 1u);
    }
    else if (huart == &huart2)
    {
        DBUS_OnByte(s_byte2);
        (void)HAL_UART_Receive_IT(&huart2, &s_byte2, 1u);
    }
    else if (huart == &huart3)
    {
        /* USART3 是备用/调试口，不参与电机控制；收到就丢，
           但必须重新挂中断，否则只会收到第一个字节。 */
        (void)HAL_UART_Receive_IT(&huart3, &s_byte3, 1u);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* 溢出/帧错误会让 Receive_IT 停摆。不重新挂上，该口从此静默。 */
    if (huart == &huart1)
    {
        (void)HAL_UART_Receive_IT(&huart1, &s_byte1, 1u);
    }
    else if (huart == &huart2)
    {
        (void)HAL_UART_Receive_IT(&huart2, &s_byte2, 1u);
    }
    else if (huart == &huart3)
    {
        (void)HAL_UART_Receive_IT(&huart3, &s_byte3, 1u);
    }
}
