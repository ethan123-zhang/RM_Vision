/**
 * @file    stm32f1xx_hal.h  （PC 侧极简替身，不是真 HAL）
 * @brief   只为了让 gcc -fsyntax-only 能检查固件模块的语法与类型。
 *
 * 用途：本机没有 Keil / arm-none-eabi-gcc，无法编译固件。但这个替身足以把
 * protocol.c、can_frame.c、x5_link.c、can_bus.c、DBUS.c、uart_rx.c 过一遍
 * 语法和类型检查（函数签名、参数个数、类型不匹配都会报）。
 *
 * 不覆盖：main.c（依赖 RCC/NVIC/GPIO 等大量结构体）、以及任何运行时行为。
 * 真正的编译仍然必须在 Keil 里点一次。
 */
#ifndef __HAL_STUB_H
#define __HAL_STUB_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    HAL_OK = 0x00,
    HAL_ERROR = 0x01,
    HAL_BUSY = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

typedef struct
{
    uint32_t instance;
} UART_HandleTypeDef;

typedef struct
{
    uint32_t instance;
} CAN_HandleTypeDef;

typedef struct
{
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
    uint32_t TransmitGlobalTime;
} CAN_TxHeaderTypeDef;

typedef struct
{
    uint32_t DLC;
} CAN_RxHeaderTypeDef;

#define CAN_ID_EXT      0x00000004u
#define CAN_RTR_DATA    0x00000000u
#define CAN_RX_FIFO0    0x00000000u
#define DISABLE         0u
#define ENABLE          1u

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan,
                                       CAN_TxHeaderTypeDef *pHeader,
                                       uint8_t aData[], uint32_t *pTxMailbox);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo,
                                       CAN_RxHeaderTypeDef *pHeader,
                                       uint8_t aData[]);

uint32_t HAL_GetTick(void);
void HAL_IncTick(void);
void HAL_Delay(uint32_t ms);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData,
                                      uint16_t Size);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

/* ---------------------------------------------------------------------------
 * 下面这些只为让 main.c（含 SystemClock_Config 的 CubeMX 样板）也能过语法检查。
 * 只保留 main.c 真正用到的成员名，不追求与真 HAL 一致。
 * ------------------------------------------------------------------------- */
void HAL_Init(void);
void __disable_irq(void);

typedef struct
{
    uint32_t PLLState;
    uint32_t PLLSource;
    uint32_t PLLMUL;
} RCC_PLLInitTypeDef;

typedef struct
{
    uint32_t OscillatorType;
    uint32_t HSEState;
    uint32_t HSEPredivValue;
    uint32_t HSIState;
    RCC_PLLInitTypeDef PLL;
} RCC_OscInitTypeDef;

typedef struct
{
    uint32_t ClockType;
    uint32_t SYSCLKSource;
    uint32_t AHBCLKDivider;
    uint32_t APB1CLKDivider;
    uint32_t APB2CLKDivider;
} RCC_ClkInitTypeDef;

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *s);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *s, uint32_t latency);

#define RCC_OSCILLATORTYPE_HSE 0x00000001u
#define RCC_HSE_ON 0x00010000u
#define RCC_HSE_PREDIV_DIV1 0x00000000u
#define RCC_HSI_ON 0x00000001u
#define RCC_PLL_ON 0x00000002u
#define RCC_PLLSOURCE_HSE 0x00010000u
#define RCC_PLL_MUL9 0x00280000u
#define RCC_CLOCKTYPE_HCLK 0x00000002u
#define RCC_CLOCKTYPE_SYSCLK 0x00000001u
#define RCC_CLOCKTYPE_PCLK1 0x00000004u
#define RCC_CLOCKTYPE_PCLK2 0x00000008u
#define RCC_SYSCLKSOURCE_PLLCLK 0x00000002u
#define RCC_SYSCLK_DIV1 0x00000000u
#define RCC_HCLK_DIV1 0x00000000u
#define RCC_HCLK_DIV2 0x00000800u
#define FLASH_LATENCY_2 0x00000002u

#endif /* __HAL_STUB_H */
