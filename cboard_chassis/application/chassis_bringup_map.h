/**
  ******************************************************************************
  * @file       chassis_bringup_map.h
  * @brief      底盘调车用的纯逻辑部分：轮位映射表 + 单电机扫描时序。
  *             这个文件不依赖 HAL / FreeRTOS，主机上可以直接跑测试
  *             （cboard_chassis/tests），上板时由 chassis_bringup.c 调用。
  *
  *             背景：官方 17.chassis_task 的解算把四个轮子写死成 0..3 号轮位，
  *             并且默认右前、右后两个电机是"反装"的。实车换了车、换了轮子、
  *             换了接线以后，这两件事都得重新标定，但官方代码里没有地方改。
  *             这里把"物理轮位 -> CAN ID"和"正电流方向"抽成两张表，
  *             解算公式一行不动，改表就能对上实车。
  ******************************************************************************
  */

#ifndef CHASSIS_BRINGUP_MAP_H
#define CHASSIS_BRINGUP_MAP_H

#include <stdint.h>

/* 四轮底盘，轮位数固定 4 */
#define CHASSIS_WHEEL_NUM 4

/* ==========================================================================
 *  标定表 1：轮位 -> C620 CAN ID
 *
 *  轮位索引沿用官方解算顺序，物理含义（标准装法，见 README §3）：
 *      0 = 右前 FR   1 = 左前 FL   2 = 左后 RL   3 = 右后 RR
 *
 *  自检（CHASSIS_BRINGUP_MODE = 1）会让 ID 1..4 依次正转/反转，
 *  看清楚"哪个 ID 转的是哪个角"，把 ID 填到对应的轮位上。
 *  四个值必须是 1..8 且互不相同，否则 chassis_map_valid() 返回 0。
 * ========================================================================== */
#ifndef CHASSIS_WHEEL_CAN_ID_0
#define CHASSIS_WHEEL_CAN_ID_0 1 /* 右前 */
#endif
#ifndef CHASSIS_WHEEL_CAN_ID_1
#define CHASSIS_WHEEL_CAN_ID_1 2 /* 左前 */
#endif
#ifndef CHASSIS_WHEEL_CAN_ID_2
#define CHASSIS_WHEEL_CAN_ID_2 3 /* 左后 */
#endif
#ifndef CHASSIS_WHEEL_CAN_ID_3
#define CHASSIS_WHEEL_CAN_ID_3 4 /* 右后 */
#endif

/* ==========================================================================
 *  标定表 2：轮位 -> 正电流方向
 *
 *  +1：正电流让轮子顶部往车头方向走（正电流 = 前进）
 *  -1：反装，正电流让轮子倒着走，需要取反
 *
 *  自检时看 +3000 那一段：轮子顶部往车头走填 +1，往车尾走填 -1。
 *  官方标准车上右前/右后是反装的，所以实车大概率不是全 +1。
 * ========================================================================== */
#ifndef CHASSIS_WHEEL_DIR_0
#define CHASSIS_WHEEL_DIR_0 (+1)
#endif
#ifndef CHASSIS_WHEEL_DIR_1
#define CHASSIS_WHEEL_DIR_1 (+1)
#endif
#ifndef CHASSIS_WHEEL_DIR_2
#define CHASSIS_WHEEL_DIR_2 (+1)
#endif
#ifndef CHASSIS_WHEEL_DIR_3
#define CHASSIS_WHEEL_DIR_3 (+1)
#endif

/* ==========================================================================
 *  单电机扫描时序（单位 ms / 电流单位同 C620：-16384..16384）
 *
 *  每个 ID 一轮： 静默 PRE -> +I ON -> 停 MID -> -I ON -> 停 TAIL
 *  四个 ID 扫完再静默 GAP，然后重来，方便反复观察。
 * ========================================================================== */
#define CHASSIS_SWEEP_CURRENT 3000 /* 空载约 3.7 A，够看清转向 */

#define CHASSIS_SWEEP_PRE_MS 400
#define CHASSIS_SWEEP_ON_MS 1200
#define CHASSIS_SWEEP_MID_MS 600
#define CHASSIS_SWEEP_TAIL_MS 1200
#define CHASSIS_SWEEP_GAP_MS 2000

/* 单个 ID 占用时间：400 + 1200 + 600 + 1200 + 1200 = 4600 ms */
#define CHASSIS_SWEEP_ONE_MS                                                  \
  (CHASSIS_SWEEP_PRE_MS + CHASSIS_SWEEP_ON_MS + CHASSIS_SWEEP_MID_MS +        \
   CHASSIS_SWEEP_ON_MS + CHASSIS_SWEEP_TAIL_MS)

/* 整轮周期：4600 * 4 + 2000 = 20400 ms */
#define CHASSIS_SWEEP_CYCLE_MS                                                \
  (CHASSIS_SWEEP_ONE_MS * CHASSIS_WHEEL_NUM + CHASSIS_SWEEP_GAP_MS)

/* 扫描状态机在某一时刻的输出 */
typedef struct {
  uint8_t can_id; /* 0 = 四路全零；1..4 = 这一刻在驱动这一路 */
  int16_t current;
} chassis_sweep_step_t;

/**
 * @brief  检查标定表是否合法（ID 在 1..8 且互不重复）
 * @retval 1 合法，0 非法
 */
int chassis_map_valid(void);

/**
 * @brief  轮位索引 -> 该轮位对应的 C620 CAN ID
 * @param  wheel_index 0..3，越界返回 0
 */
uint8_t chassis_map_wheel_can_id(uint8_t wheel_index);

/**
 * @brief  把"按轮位排布的电流"映射成"按 CAN ID 排布的电流"，并套用方向表
 * @param  wheel 长度 4，下标 = 轮位索引
 * @param  can   长度 4，下标 = CAN ID - 1（直接喂给 CAN_cmd_chassis）
 * @retval 0 成功；-1 轮位表有线序错误/ID 越界；-2 表里有重复 ID
 * @note   失败时 can[] 全部写 0，绝不让错表驱动电机
 */
int chassis_map_wheel_to_can(const int16_t wheel[CHASSIS_WHEEL_NUM],
                             int16_t can[CHASSIS_WHEEL_NUM]);

/**
 * @brief  把"按 CAN ID 的在线位图"换成"按轮位的在线位图"
 * @param  can_id_mask bit(k) = CAN ID (k+1) 在线，bit0 对应 ID 1
 * @retval bit(i) = 轮位 i 在线
 */
uint8_t chassis_map_online_mask(uint8_t can_id_mask);

/**
 * @brief  单电机扫描状态机：给定扫描开始后经过的毫秒数，算出这一刻该发什么
 * @param  elapsed_ms 从扫描开始累计的毫秒数（超过一个周期自动取模）
 * @param  step       输出
 */
void chassis_sweep_eval(uint32_t elapsed_ms, chassis_sweep_step_t *step);

#endif
