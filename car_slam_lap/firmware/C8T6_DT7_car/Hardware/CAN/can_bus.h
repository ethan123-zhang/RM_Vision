/**
 ******************************************************************************
 * @file    can_bus.h
 * @brief   XHSMCUW50 双驱无刷控制器 CAN 驱动 —— 头文件
 * @note    协议依据：《双驱动CAN控制接口使用说明》(CS-KZQ-SM-2024-3)
 *          +《双驱无刷电机控制器规格书》6.3/6.4 节
 *
 *          波特率 500K，扩展帧 ID 0x1314，8 字节数据帧
 ******************************************************************************
 */
#ifndef __CAN_BUS_H
#define __CAN_BUS_H

#include "main.h"          /* 含 HAL 库及 stm32f1xx_hal.h */


extern CAN_HandleTypeDef hcan;


void CAN_Send_MIX(int16_t angular,int16_t speed);
void CAN_Send(uint8_t *buf, uint16_t len);
void CAN_SetMsg_HAL(CAN_TxHeaderTypeDef *TxHeader, uint8_t *buf, uint16_t len);


// /* ========================== CAN 通信参数 ========================== */
// #define CAN_BUS_ID            0x1314U   /* 扩展帧 ID（文档规定）          */
// #define CAN_BUS_PERIOD_MS     10U       /* 指令发送周期：文档要求 5~20ms  */
// #define CAN_BUS_FRAME_LEN     8U        /* 指令帧固定 8 字节              */

// /* ========================== ACK 反馈帧结构 ========================== */
// /* 控制器回传：左轮速度 / 右轮速度 / 电池电压 / 校验和（各 2 字节）       */
// typedef struct
// {
//     int16_t  speedL;        /* 左轮实际速度反馈                          */
//     int16_t  speedR;        /* 右轮实际速度反馈                          */
//     uint16_t batteryVol;    /* 电池电压，单位 100mV（380 表示 38V）       */
//     uint16_t crc;           /* 收到的校验和                              */
//     uint8_t  valid;         /* 1 = 最近一帧校验通过                      */
// } CAN_AckFrame_t;

// /* ========================== 对外接口 ========================== */

// /**
//  * @brief  CAN 初始化：配置过滤器 + 启动 + 使能接收中断
//  * @param  hcan  CAN 句柄指针（CubeMX 已初始化外设）
//  */
// void CAN_Bus_Init(CAN_HandleTypeDef *hcan);

// /**
//  * @brief  混控模式：发送 转弯角速度 + 直行速度（默认推荐）
//  * @param  angular  转弯角速度 [-1000, 1000]（正右转/负左转，方向以实测为准）
//  * @param  speed    直行速度   [-1000, 1000]（正前进/负后退）
//  * @retval HAL_OK / HAL_ERROR
//  */
// HAL_StatusTypeDef CAN_Bus_SendMix(int16_t angular, int16_t speed);

// /**
//  * @brief  独立模式：发送 左/右轮速度（需先与厂商确认模式切换方式）
//  * @param  speedL  左轮速度 [-1000, 1000]
//  * @param  speedR  右轮速度 [-1000, 1000]
//  * @retval HAL_OK / HAL_ERROR
//  */
// HAL_StatusTypeDef CAN_Bus_SendIndependent(int16_t speedL, int16_t speedR);

// /**
//  * @brief  底层发送（供上面两个函数调用）
//  * @param  data  8 字节数据
//  * @param  len   数据长度
//  */
// HAL_StatusTypeDef CAN_Bus_Transmit(uint8_t *data, uint16_t len);

// /**
//  * @brief  收到一帧数据时调用（在 HAL_CAN_RxFifo0MsgPendingCallback 中调用）
//  * @param  pHeader  接收帧头
//  * @param  pData    8 字节数据
//  */
// void CAN_Bus_OnRxMessage(CAN_RxHeaderTypeDef *pHeader, uint8_t *pData);

// /* 最近一次有效的 ACK 反馈（中断中更新，主循环只读） */
// extern volatile CAN_AckFrame_t g_ack;

#endif /* __CAN_BUS_H */
