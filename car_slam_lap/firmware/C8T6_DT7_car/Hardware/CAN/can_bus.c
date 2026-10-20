/**
 ******************************************************************************
 * @file    can_bus.c
 * @brief   XHSMCUW50 双驱无刷控制器 CAN 驱动
 *
 * 协议速查（详见 README.md）：
 *   - 扩展帧 ID = 0x1314，波特率 500K，8 字节数据帧
 *   - 混控帧:  [0xCD][0xAB][angular_L][angular_H][speed_L][speed_H][crc_L][crc_H]
 *   - 独立帧:  [0xCD][0xAB][speedL_L][speedL_H][speedR_L][speedR_H][crc_L][crc_H]
 *   - ACK 帧:  [speedL_L][speedL_H][speedR_L][speedR_H][vol_L][vol_H][crc_L][crc_H]
 *   - 校验和 = 0xABCD ^ 字段2 ^ 字段3（16bit 异或，取低 16 位）
 *   - 必须 5~20ms 周期持续发送，超过 5s 未发 → 控制器判信号丢失
 ******************************************************************************
 */
 #include "can_bus.h"
#include <string.h>
#include <stdio.h>
CAN_TxHeaderTypeDef TxHeader;
// static uint8_t lvdai_can_send_data[8];
//发送函数

// void CAN_cmd_motor(uint16_t speedL, uint16_t speedR)
// {
//     CAN_TxHeaderTypeDef txHeader;
//     uint32_t mailbox;
    
//     txHeader.IDE   = CAN_ID_EXT;         /* 扩展帧 */
//     txHeader.ExtId = 0x1314U;         /* 扩展 ID */
//     txHeader.RTR   = CAN_RTR_DATA;       /* 数据帧 */
//     txHeader.DLC   = 0x08;                /* 数据长度 8 */
//     lvdai_can_send_data[0]=0xCD;
//     lvdai_can_send_data[1]=0xAB;
//     lvdai_can_send_data[2]=(speedL >> 8) & 0xFF;
//     lvdai_can_send_data[3]=speedL & 0xFF;
//     lvdai_can_send_data[4]=(speedR >> 8) & 0xFF;
//     lvdai_can_send_data[5]=speedR & 0xFF;

//     uint16_t crc = 0;
//     for(uint8_t i = 0; i < 6; i++)
//     {
//         crc ^= lvdai_can_send_data[i];
//     }

//     lvdai_can_send_data[6]=(crc >> 8) & 0xFF;
//     lvdai_can_send_data[7]=crc & 0xFF;
//     HAL_CAN_AddTxMessage(&hcan, &txHeader, lvdai_can_send_data, &mailbox);
// }

/*
 * 函数名：CAN_SetMsg
 * 描述  ：CAN通信报文内容设置,设置一个数据内容为0-7的数据包
 * 输入  ：发送报文结构体
 * 输出  : 无
 * 调用  ：外部调用
 */	 
void CAN_SetMsg_HAL(CAN_TxHeaderTypeDef *TxHeader, uint8_t *buf, uint16_t len)
{
  

    TxHeader->ExtId = 0x1314;             // 扩展ID，和原来一致
    TxHeader->IDE = CAN_ID_EXT;           // 扩展帧
    TxHeader->RTR = CAN_RTR_DATA;         // 数据帧
    TxHeader->DLC = len;                  // 数据长度
    TxHeader->TransmitGlobalTime = DISABLE;// HAL库必须加这个！！很多人漏了这里导致发不出报文

}

void CAN_Send(uint8_t *buf, uint16_t len)
{
	uint32_t TxMailbox = 0;
	/*设置要发送的报文*/
	CAN_SetMsg_HAL(&TxHeader, buf, len);
  if (HAL_CAN_AddTxMessage(&hcan,&TxHeader,buf,&TxMailbox) != HAL_OK)
  {
    Error_Handler();
  }
	
	/* 等待发送完成 */
	while(HAL_CAN_IsTxMessagePending(&hcan, TxMailbox));
}

typedef struct __attribute__((packed))

//定义发送函数中数据的结构体
{
	uint16_t start;  //帧头
	int16_t angular; //角速度
	int16_t speed;  //直行速度
	uint16_t checksum; //crc异或校验
	
}SerialCommand;
SerialCommand shuju;

//混控模式 MIX   angular角速度      speed直行速度
void CAN_Send_MIX(int16_t angular,int16_t speed)
{
    uint8_t buf[8]={0};
    shuju.start=(uint16_t)0xABCD;
    shuju.angular=angular;
    shuju.speed=speed;
    shuju.checksum=(uint16_t)(shuju.start^shuju.angular^shuju.speed);

    memcpy(buf, (uint8_t*)&shuju, sizeof(shuju));

    CAN_Send(buf,8);
}


// /* CAN 句柄指针（初始化时保存，供发送/过滤使用） */
// static CAN_HandleTypeDef *s_hcan = NULL;

// /* 最近一次有效的 ACK 反馈（中断更新） */
// volatile CAN_AckFrame_t g_ack;

// /* ========================== 内部函数 ========================== */

// /**
//  * @brief  计算校验和：帧头与两个数据字段 16bit 异或，取低 16 位
//  * @note   规格书 6.4 节：除 CRC 外其它数据异或，取低 16 位。
//  *         注意 CAN 说明文档示例代码里的"乘法"是笔误，以规格书为准。
//  */
// static uint16_t CAN_Bus_Checksum(uint16_t start, uint16_t f2, uint16_t f3)
// {
//     return (uint16_t)(start ^ f2 ^ f3);
// }

// /**
//  * @brief  填充指令帧：帧头 0xCD 0xAB + 两个 int16 字段 + CRC（小端序）
//  * @param  buf     输出缓冲区（8 字节）
//  * @param  field2  字段2（混控=angular，独立=speedL）
//  * @param  field3  字段3（混控=speed，  独立=speedR）
//  */
// static void CAN_Bus_BuildFrame(uint8_t *buf, int16_t field2, int16_t field3)
// {
//     const uint16_t start = 0xABCDU;              /* 帧头 */
//     const uint16_t crc   = CAN_Bus_Checksum(start,
//                                             (uint16_t)field2,
//                                             (uint16_t)field3);

//     buf[0] = (uint8_t)(start & 0xFF);            /* 0xCD（先发） */
//     buf[1] = (uint8_t)((start >> 8) & 0xFF);     /* 0xAB */
//     buf[2] = (uint8_t)(field2 & 0xFF);           /* 字段2 低字节 */
//     buf[3] = (uint8_t)((field2 >> 8) & 0xFF);    /* 字段2 高字节 */
//     buf[4] = (uint8_t)(field3 & 0xFF);           /* 字段3 低字节 */
//     buf[5] = (uint8_t)((field3 >> 8) & 0xFF);    /* 字段3 高字节 */
//     buf[6] = (uint8_t)(crc & 0xFF);              /* CRC 低字节 */
//     buf[7] = (uint8_t)((crc >> 8) & 0xFF);       /* CRC 高字节 */
// }

// /* ========================== 对外接口 ========================== */

// /**
//  * @brief  CAN 初始化：过滤器 + 启动 + 使能 FIFO0 接收中断
//  * @note   需在 CubeMX 的 MX_CAN1_Init()（外设时钟/GPIO/NVIC）之后调用
//  */
// void CAN_Bus_Init(CAN_HandleTypeDef *hcan)
// {
//     s_hcan = hcan;
//     memset((void *)&g_ack, 0, sizeof(g_ack));    /* 清空 ACK */

//     /* ---- 接收过滤器：扩展 ID 0x1314 ---- */
//     CAN_FilterTypeDef filter = {0};
//     filter.FilterActivation     = ENABLE;
//     filter.FilterMode           = CAN_FILTERMODE_IDMASK;   /* 掩码模式 */
//     filter.FilterScale          = CAN_FILTERSCALE_32BIT;   /* 32 位 */
//     filter.FilterBank           = 0;                       /* F103 单 CAN 用组 0 */
//     filter.FilterFIFOAssignment = CAN_RX_FIFO0;            /* 关联 FIFO0 */
//     /* 扩展 ID 需要左移 3 位后填入寄存器高低位（与文档示例一致） */
//     filter.FilterIdHigh = (uint16_t)((((uint32_t)CAN_BUS_ID << 3)
//                                       | CAN_ID_EXT | CAN_RTR_DATA) >> 16);
//     filter.FilterIdLow  = (uint16_t)(((uint32_t)CAN_BUS_ID << 3)
//                                      | CAN_ID_EXT | CAN_RTR_DATA);
//     /*
//      * 掩码全 0 = 不过滤、接收全部报文。
//      * 原因：控制器回传 ACK 帧的 ID 文档未公开，先全部接收便于调试；
//      * 确认 ACK 的 ID 后，可把掩码改为 0xFFFF 只收 0x1314。
//      */
//     filter.FilterMaskIdHigh = 0x0000;
//     filter.FilterMaskIdLow  = 0x0000;
//     if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
//     {
//         Error_Handler();
//     }

//     /* 启动 CAN，并使能 FIFO0 接收中断（回调见 main.c） */
//     if (HAL_CAN_Start(hcan) != HAL_OK)
//     {
//         Error_Handler();
//     }
//     if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING)
//         != HAL_OK)
//     {
//         Error_Handler();
//     }
// }

// /**
//  * @brief  混控模式：发送 转弯角速度 + 直行速度
//  */
// HAL_StatusTypeDef CAN_Bus_SendMix(int16_t angular, int16_t speed)
// {
//     uint8_t buf[CAN_BUS_FRAME_LEN] = {0};
//     CAN_Bus_BuildFrame(buf, angular, speed);     /* 帧头+angular+speed+CRC */

// // ========== 新增串口打印，放在BuildFrame之后、发送之前 ==========
//     printf("CAN_TX | angular:%d, speed:%d | DATA:", angular, speed);
//     for(uint8_t i = 0; i < CAN_BUS_FRAME_LEN; i++)
//     {
//         printf("%02X ", buf[i]);
//     }
//     printf("\r\n");
//     HAL_StatusTypeDef send_ret = CAN_Bus_Transmit(buf, CAN_BUS_FRAME_LEN);
//     printf("CAN send return = %d\r\n", send_ret);
//     // ===============================================================

//     return CAN_Bus_Transmit(buf, CAN_BUS_FRAME_LEN);
// }

// /**
//  * @brief  独立模式：发送 左/右轮速度（需先与厂商确认模式切换方式）
//  */
// HAL_StatusTypeDef CAN_Bus_SendIndependent(int16_t speedL, int16_t speedR)
// {
//     uint8_t buf[CAN_BUS_FRAME_LEN] = {0};
//     CAN_Bus_BuildFrame(buf, speedL, speedR);     /* 帧头+speedL+speedR+CRC */
//     return CAN_Bus_Transmit(buf, CAN_BUS_FRAME_LEN);
// }

// /**
//  * @brief  底层发送：扩展帧 0x1314，8 字节数据
//  * @note   HAL_CAN_AddTxMessage 为邮箱写入，非阻塞；发送成功与否
//  *         由硬件自动重传（AutoRetransmission=ENABLE）保证
//  */
// HAL_StatusTypeDef CAN_Bus_Transmit(uint8_t *data, uint16_t len)
// {
//     CAN_TxHeaderTypeDef txHeader = {0};
//     uint32_t mailbox = 0;

//     txHeader.ExtId = CAN_BUS_ID;         /* 扩展 ID */
//     txHeader.IDE   = CAN_ID_EXT;         /* 扩展帧 */
//     txHeader.RTR   = CAN_RTR_DATA;       /* 数据帧 */
//     txHeader.DLC   = len;                /* 数据长度 8 */

//     return HAL_CAN_AddTxMessage(s_hcan, &txHeader, data, &mailbox);
// }

// /**
//  * @brief  解析控制器回传的 ACK 帧（在接收中断回调中调用）
//  * @note   ACK 帧格式：speedL(int16) + speedR(int16) + vol(uint16) + crc(uint16)
//  *         校验通过才更新 g_ack，防止坏帧污染数据
//  */
// void CAN_Bus_OnRxMessage(CAN_RxHeaderTypeDef *pHeader, uint8_t *pData)
// {
//     uint16_t speedL_u, speedR_u, vol;

//     if (pHeader->DLC < CAN_BUS_FRAME_LEN)
//     {
//         return;                            /* 长度不足，丢弃 */
//     }

//     /* 小端组合各 16bit 字段 */
//     speedL_u = (uint16_t)(pData[0] | (pData[1] << 8));
//     speedR_u = (uint16_t)(pData[2] | (pData[3] << 8));
//     vol      = (uint16_t)(pData[4] | (pData[5] << 8));

//     /* 校验：除 CRC 外三字段异或（与发送侧算法一致） */
//     if ((uint16_t)(speedL_u ^ speedR_u ^ vol) != (uint16_t)(pData[6] | (pData[7] << 8)))
//     {
//         return;                            /* 校验失败，丢弃 */
//     }

//     /* 写入全局 ACK */
//     g_ack.speedL      = (int16_t)speedL_u;
//     g_ack.speedR      = (int16_t)speedR_u;
//     g_ack.batteryVol  = vol;
//     g_ack.crc         = (uint16_t)(pData[6] | (pData[7] << 8));
//     g_ack.valid       = 1;
// }
