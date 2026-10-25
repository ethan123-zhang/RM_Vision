/**
  ******************************************************************************
  * @file       chassis_bringup.h
  * @brief      底盘调试层：按键验 ID、单电机扫描、调试 CAN 帧、带超时的启动等待。
  *
  *             这一层是"加在官方框架上面"的，不改官方解算：
  *               - chassis_bringup_map.h  标定表 + 扫描时序 + 按键状态机（纯逻辑）
  *               - chassis_bringup.c      上板侧：发 CAN、收反馈、打调试帧
  *
  *             调车顺序见 cboard_chassis/README.md：
  *               阶段 0  CHASSIS_BRINGUP_MODE = 2  按 KEY 验电调编号（当前阶段）
  *               阶段 1  CHASSIS_BRINGUP_MODE = 1  逐个 ID 自动扫描，填两张表
  *               阶段 2  遥控通道确认（看 0x1F1 帧）
  *               阶段 3  CHASSIS_BRINGUP_MODE = 0  整车闭环
  ******************************************************************************
  */

#ifndef CHASSIS_BRINGUP_H
#define CHASSIS_BRINGUP_H

#include "struct_typedef.h"

#include "chassis_task.h"
#include "chassis_bringup_map.h"

/* ==========================================================================
 *  运行模式
 *    0 = 正常跑车（官方逻辑，速度和电流都过标定表）
 *    1 = 单电机扫描（不依赖遥控器/陀螺仪，按裸 ID 1..4 依次正反转）
 *    2 = 按键验 ID（按 KEY N 次，停 3 s 后驱动 N 号电调）
 * ========================================================================== */
#define CHASSIS_BRINGUP_MODE_NORMAL 0
#define CHASSIS_BRINGUP_MODE_SWEEP 1
#define CHASSIS_BRINGUP_MODE_KEY_ID 2

#ifndef CHASSIS_BRINGUP_MODE
#define CHASSIS_BRINGUP_MODE CHASSIS_BRINGUP_MODE_KEY_ID
#endif

/* C 板 KEY 按键：PA0，按下为低（官方 6.key_exit 例程同款） */
#ifndef CHASSIS_KEY_PIN
#define CHASSIS_KEY_PIN GPIO_PIN_0
#endif
#ifndef CHASSIS_KEY_PORT
#define CHASSIS_KEY_PORT GPIOA
#endif
#ifndef CHASSIS_KEY_DEBOUNCE_MS
#define CHASSIS_KEY_DEBOUNCE_MS 20
#endif

/* 按键反馈：按一次响 50 ms，按超界响 300 ms（蜂鸣器需先静音 test_task） */
#ifndef CHASSIS_KEY_BEEP_MS
#define CHASSIS_KEY_BEEP_MS 50
#endif
#ifndef CHASSIS_KEY_BEEP_ERROR_MS
#define CHASSIS_KEY_BEEP_ERROR_MS 300
#endif

/* 调车模式（非 0）下静音 test_task：本工程没有云台电机，yaw/pitch/trigger
   永远掉线，它会一直报警，把按键反馈盖掉。正常模式仍保留官方报警行为。 */
#ifndef CHASSIS_BRINGUP_MUTE_TEST_TASK
#define CHASSIS_BRINGUP_MUTE_TEST_TASK                                        \
  (CHASSIS_BRINGUP_MODE != CHASSIS_BRINGUP_MODE_NORMAL)
#endif

/* 启动等待超时：官方是死等，这里等不到就走，没遥控时主循环只会发 0 电流 */
#ifndef CHASSIS_BRINGUP_WAIT_MS
#define CHASSIS_BRINGUP_WAIT_MS 3000
#endif

/* ==========================================================================
 *  调试 CAN 帧（挂到 USB-CAN 分析仪上看）
 *    0x1F0  25 ms 一次：4 个轮位的实际转速 rpm（int16 little endian）
 *    0x1F1  100 ms 一次：[0] 在线掩码 [1] S1|S2<<2 [2..4] 摇杆 [5] 模式
 *                        [6] 遥控在线 [7] 底盘状态机；**按键模式下 [7] = 当前按键计数**
 *  不需要时把 CHASSIS_DEBUG_CAN 置 0，总线上就只剩电机的帧。
 * ========================================================================== */
#ifndef CHASSIS_DEBUG_CAN
#define CHASSIS_DEBUG_CAN 1
#endif

#define CHASSIS_DEBUG_ID_RPM 0x1F0
#define CHASSIS_DEBUG_ID_STATUS 0x1F1
#define CHASSIS_DEBUG_RPM_PERIOD_MS 25
#define CHASSIS_DEBUG_STATUS_PERIOD_MS 100

/**
 * @brief  轮位索引 i 对应的电机反馈指针（按标定表查 CAN ID）
 * @note   标定表非法时返回编号 0 的反馈，只为不空指针；此时输出会被映射层清零
 */
const motor_measure_t *chassis_bringup_measure_point(uint8_t wheel_index);

/**
 * @brief  等 4 个电机 + 遥控器上线，最多等 CHASSIS_BRINGUP_WAIT_MS 毫秒
 * @note   标定表非法时响三声并返回（主循环随后只发 0 电流）
 */
void chassis_bringup_wait_ready(void);

/**
 * @brief  单电机扫描主循环，不会返回
 * @param  chassis 只用来打调试帧
 */
void chassis_bringup_sweep_loop(chassis_move_t *chassis);

/**
 * @brief  按键验 ID 主循环，不会返回
 *         按 KEY N 次，停手 3 s 后驱动 N 号电调 1.5 s
 * @param  chassis 只用来打调试帧
 */
void chassis_bringup_key_id_loop(chassis_move_t *chassis);

/**
 * @brief  轮位电流 -> CAN 电流（内部就是 chassis_map_wheel_to_can，失败时全 0）
 */
int chassis_bringup_wheel_to_can(const int16_t wheel[CHASSIS_WHEEL_NUM],
                                 int16_t can[CHASSIS_WHEEL_NUM]);

/**
 * @brief  每周期调一次：自己按时间节流，发 0x1F0 / 0x1F1 调试帧
 */
void chassis_bringup_status_update(const chassis_move_t *chassis);

#endif
