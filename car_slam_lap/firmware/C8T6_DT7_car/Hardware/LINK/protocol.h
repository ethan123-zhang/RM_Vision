/**
 ******************************************************************************
 * @file    protocol.h
 * @brief   X5 <-> C8T6 串口链路协议（平台无关）
 *
 * 设计约束：
 *   1. 不 include 任何 HAL 头 —— PC 上 gcc 能直接编译并跑测试向量，
 *      固件与 PC 用的是同一份代码，避免"两边各写一遍 CRC 然后对不上"。
 *   2. 收帧是逐字节状态机：串口每来 1 字节调一次 proto_rx_feed()，
 *      可以在中断里调，不需要缓冲区/超时。
 *   3. 帧内所有多字节字段为小端（C8T6 是 Cortex-M3 LE，X5 侧用 '<h'）。
 *
 * 帧格式（LEN 含 CMD 字节本身）：
 *
 *   下行 X5 -> C8T6（50 Hz，9 字节）
 *   +------+------+-----+-----+----------+----------+------+
 *   | 0xA5 | 0x5A | LEN | CMD | speed i16| ang  i16 | CRC8 |
 *   |      |      | 0x05| 0x01|  LE      |  LE      |      |
 *   +------+------+-----+-----+----------+----------+------+
 *
 *   上行 C8T6 -> X5（10 Hz，7 字节）
 *   +------+------+-----+-----+--------+-----+------+
 *   | 0x5A | 0xA5 | LEN | CMD | status | src | CRC8 |
 *   |      |      | 0x03| 0x81|  bit   | 0-2 |      |
 *   +------+------+-----+-----+--------+-----+------+
 *
 *   CRC8 = poly 0x07 / init 0x00 / 不反转 / xorout 0x00（即 CRC-8/ATM），
 *          覆盖范围：LEN 字节 + 其后的 LEN 个字节（含 CMD）。
 *          外部校验向量：crc8("123456789") == 0xF4。
 *
 *   speed   > 0 前进   [-1000, 1000]
 *   angular > 0 右转   [-1000, 1000]
 ******************************************************************************
 */
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

/* ---- 帧标识 ---- */
#define PROTO_DN_SOF0        0xA5u   /* 下行帧首字节 */
#define PROTO_DN_SOF1        0x5Au
#define PROTO_UP_SOF0        0x5Au   /* 上行帧首字节 */
#define PROTO_UP_SOF1        0xA5u

#define PROTO_MAX_PAYLOAD    8u      /* LEN 上限（含 CMD） */
#define PROTO_MIN_LEN        2u      /* CMD + 至少 1 字节负载 */

#define PROTO_CMD_SET_VEL    0x01u
#define PROTO_CMD_STATUS     0x81u

#define PROTO_SPEED_MAX      1000
#define PROTO_FRAME_DN_LEN   9u      /* build_set_vel 输出长度 */
#define PROTO_FRAME_UP_LEN   7u      /* build_status 输出长度 */

/* ---- status 位 ---- */
/* 注意 CAN_OK 的含义很窄：只表示"往串口转CAN 模块的 UART 写成功了"。
   它**不能**证明 CAN 总线上真的有帧——模块没上电、没开通道（O）、波特率不对，
   这一位照样是 1。真正确认电机链路要靠轮子有没有转（验收 A2）。 */
#define PROTO_ST_CAN_OK      0x01u
#define PROTO_ST_LINK_OK     0x02u
#define PROTO_ST_RC_OK       0x04u
#define PROTO_ST_ESTOP       0x08u

/* ---- 控制源（status.src） ---- */
#define PROTO_SRC_NONE       0u
#define PROTO_SRC_SERIAL     1u
#define PROTO_SRC_RC         2u

/** 解码后的速度指令 */
typedef struct
{
    int16_t speed;      /**< [-1000, 1000]，>0 前进 */
    int16_t angular;    /**< [-1000, 1000]，>0 右转 */
} proto_cmd_t;

/** 一帧完整报文 */
typedef struct
{
    uint8_t cmd;
    uint8_t len;                      /**< 负载长度（不含 CMD） */
    uint8_t payload[PROTO_MAX_PAYLOAD];
} proto_frame_t;

/** 收帧状态机 */
typedef struct
{
    uint8_t sof0;
    uint8_t sof1;
    uint8_t buf[3u + PROTO_MAX_PAYLOAD + 1u];
    uint8_t idx;
    uint8_t len;
} proto_rx_t;

/* ---------------- 基础 ---------------- */

/** CRC-8/ATM；len 单位字节 */
uint8_t proto_crc8(const uint8_t *data, uint16_t len);

/** 组下行帧；speed/angular 自动钳位到 ±1000。返回帧长 */
uint16_t proto_build_set_vel(uint8_t *buf, int16_t speed, int16_t angular);

/** 组上行状态帧。返回帧长 */
uint16_t proto_build_status(uint8_t *buf, uint8_t status, uint8_t src);

/** 从下行帧解出速度指令；成功返回 1 */
int proto_parse_set_vel(const proto_frame_t *frame, proto_cmd_t *out);

/** 从上行帧解出 status/src；成功返回 1 */
int proto_parse_status(const proto_frame_t *frame, uint8_t *status, uint8_t *src);

/* ---------------- 收帧 ---------------- */

/** 复位并指定本端期望的帧首两字节（C8T6 用 DN，X5 用 UP） */
void proto_rx_init(proto_rx_t *rx, uint8_t sof0, uint8_t sof1);

/** 清状态机（丢掉半帧），保留 sof 配置 */
void proto_rx_reset(proto_rx_t *rx);

/**
 * @brief  喂一个字节
 * @param  rx   状态机
 * @param  byte 收到的字节
 * @param  out  解出完整且 CRC 正确的帧时写入
 * @retval 1 收到一帧有效报文；0 否则
 */
int proto_rx_feed(proto_rx_t *rx, uint8_t byte, proto_frame_t *out);

#endif /* __PROTOCOL_H */
