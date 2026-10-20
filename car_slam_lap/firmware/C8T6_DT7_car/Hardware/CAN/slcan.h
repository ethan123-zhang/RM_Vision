/**
 ******************************************************************************
 * @file    slcan.h
 * @brief   slcan 文本协议编码（串口 -> CAN 模块）—— 平台无关
 *
 * 为什么单独一层：C8T6 不再直接驱动 CAN 外设，而是把 CAN 帧用**文本**发给
 * "串口转CAN 模块"，由模块上总线。文本协议写错了不会崩，只会静默丢帧，
 * 所以在 PC 上用 gcc 把编码结果逐字节断言（见 tests/slcan_test.c）。
 *
 * slcan 帧格式（CANable / candleLight 那一套，\r 结尾）：
 *   扩展帧  T IIIIIIII L DD..DD \r      I = 8 位十六进制 ID，L = 1 位十六进制 DLC
 *   标准帧  t III L DD..DD \r           I = 3 位十六进制
 *   设波特率 S5\r  (S0=10k S1=20k S2=50k S3=100k S4=125k S5=250k S6=500k S7=800k S8=1M)
 *   开通道  O\r        关通道  C\r        状态  F\r
 ******************************************************************************
 */
#ifndef __SLCAN_H
#define __SLCAN_H

#include <stdint.h>

/** 最长一帧："T" + 8 ID + 1 DLC + 16 data + '\r' + 结束符 = 27 字节，留够 32 */
#define SLCAN_BUF_LEN    32u
#define SLCAN_MAX_DATA   8u
#define SLCAN_ID_EXT_MAX 0x1FFFFFFFu
#define SLCAN_ID_STD_MAX 0x7FFu

/**
 * @brief  编码一帧扩展帧
 * @param  id    29 位扩展 ID
 * @param  data  数据（最多 8 字节）
 * @param  len   DLC
 * @param  out   输出缓冲，至少 SLCAN_BUF_LEN
 * @retval 写入字节数（含 '\r'）；参数非法返回 0
 */
uint16_t slcan_encode_ext(uint32_t id, const uint8_t *data, uint8_t len, char *out);

/** 编码一帧标准帧（11 位 ID），参数非法返回 0 */
uint16_t slcan_encode_std(uint32_t id, const uint8_t *data, uint8_t len, char *out);

/**
 * @brief  波特率 -> slcan 的 S 编号
 * @retval 0..8；不支持的波特率返回 0xFF
 */
uint8_t slcan_bitrate_code(uint32_t bps);

/** 编码设波特率命令 "S<code>\r"；非法 code 返回 0 */
uint16_t slcan_encode_bitrate(uint8_t code, char *out);

/** 编码开通道 "O\r" */
uint16_t slcan_encode_open(char *out);

/** 编码关通道 "C\r" */
uint16_t slcan_encode_close(char *out);

#endif /* __SLCAN_H */
