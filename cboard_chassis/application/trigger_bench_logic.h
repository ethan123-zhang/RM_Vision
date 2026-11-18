/**
  ******************************************************************************
  * @file       trigger_bench_logic.h
  * @brief      拨弹盘台架模式的纯逻辑：档位、送弹速率、软启动、电流环、故障判据。
  *
  *             这个文件不依赖 HAL / FreeRTOS，主机 gcc 可以直接测
  *             （cboard_chassis/tests/test_trigger_bench.c），上板时由
  *             trigger_bench.c 调用。
  *
  *             用途（2026-11-18 定）：不再做"最高空载转速"扫描，只服务
  *             "比赛能不能用"的四件事——连续送弹不卡、松手停得住、断电即停、
  *             送弹速率够不够点射节奏。
  *
  *             单位约定（容易错的地方，写在这里）：
  *               - C610 反馈 0x207 的 speed_rpm 是**电机转子 rpm**
  *               - 拨弹盘减速比 36:1，盘轴 rpm = 转子 rpm / 36
  *               - C610 电流指令范围 ±10000，台架限幅 ±3000
  *               - 17 mm 拨弹盘一圈 18 个弹位：发/秒 = 盘轴 rpm / 60 × 18
  ******************************************************************************
  */

#ifndef TRIGGER_BENCH_LOGIC_H
#define TRIGGER_BENCH_LOGIC_H

#include <stdint.h>

/* 减速比：M2006 + P36，2026-11-17 用户确认 */
#define TRIGGER_BENCH_GEAR_RATIO 36

/* 一圈弹位（官方 20.standard_robot shoot.h 的 FULL_COUNT） */
#define TRIGGER_BENCH_BALLS_PER_REV 18

/* ==========================================================================
 *  档位：短按 KEY 循环切换 1->2->3->4->1
 *  单位是**盘轴 rpm**，不是转子 rpm；转子目标由 ×36 得到。
 *
 *  为什么这么定：3V3 冷却优先每发 +10 热量、冷却 24/s，持续射速上限
 *  2.4 发/秒 ≈ 盘轴 8 rpm；1 档 10 rpm（3 发/秒）已覆盖持续射速，
 *  4 档 50 rpm（15 发/秒）留给短点射。再高只会多送弹、多烧热量。
 * ========================================================================== */
#define TRIGGER_BENCH_LEVELS 4
#define TRIGGER_BENCH_DISC_RPM_1 10 /* ≈ 3 发/秒 */
#define TRIGGER_BENCH_DISC_RPM_2 20 /* ≈ 6 发/秒 */
#define TRIGGER_BENCH_DISC_RPM_3 30 /* ≈ 9 发/秒 */
#define TRIGGER_BENCH_DISC_RPM_4 50 /* ≈ 15 发/秒，短点射 */

/* ==========================================================================
 *  按键与运行窗口
 * ========================================================================== */
/* 长按判定：按住超过这个时长 = 开始转；不足 = 短按切档 */
#define TRIGGER_BENCH_HOLD_KEY_MS 300u
/* 软启动：从 0 线性升到目标，避免按下瞬间电流打满 */
#define TRIGGER_BENCH_SOFT_START_MS 300u
/* 单次运行上限：防止按键卡住一直转 */
#define TRIGGER_BENCH_MAX_RUN_MS 20000u

/* ==========================================================================
 *  电流环（每 2 ms 一次）：P + 带限幅的 I，单位为 C610 电流指令
 * ========================================================================== */
#define TRIGGER_BENCH_CURRENT_LIMIT 3000
#define TRIGGER_BENCH_KP 0.5f
#define TRIGGER_BENCH_KI 0.002f
#define TRIGGER_BENCH_I_LIMIT 300.0f

/* ==========================================================================
 *  故障判据
 * ========================================================================== */
/* 超速：转子 rpm 超过 8000（≈222 盘轴 rpm）立即断电流 */
#define TRIGGER_BENCH_OVERSPEED_RPM 8000
/* 失速（卡滞）：运行中实测低于目标 1/4 且持续 500 ms —— 卡弹的软件特征 */
#define TRIGGER_BENCH_STALL_DIV 4
#define TRIGGER_BENCH_STALL_MS 500u

/* 电流环状态 */
typedef struct {
  float integral;
} trigger_bench_pid_t;

/**
 * @brief  档位 -> 盘轴目标 rpm；非法档位返回 0
 */
int16_t trigger_bench_disc_rpm(uint8_t level);

/**
 * @brief  档位 -> 转子目标 rpm（= 盘轴 × 36）；非法档位返回 0
 */
int16_t trigger_bench_rotor_target(uint8_t level);

/**
 * @brief  转子 rpm -> 盘轴 rpm（四舍五入），只用于报告和调试帧
 */
int16_t trigger_bench_disc_from_rotor(int16_t rotor_rpm);

/**
 * @brief  档位对应的送弹速率（发/秒），用于报告和 README 对照
 */
float trigger_bench_feed_rate(uint8_t level);

/**
 * @brief  软启动后的目标转子 rpm
 * @param  elapsed_ms   本次按住已经过的毫秒数
 * @param  rotor_target 档位对应的稳态转子目标
 * @retval 前 SOFT_START_MS 线性上升，之后等于目标
 */
int16_t trigger_bench_hold_target(uint32_t elapsed_ms, int16_t rotor_target);

void trigger_bench_pid_reset(trigger_bench_pid_t *p);

/**
 * @brief  电流环一步：给出本周期应下发的 C610 电流指令
 * @param  p            电流环状态（积分项会更新）
 * @param  rotor_target 目标转子 rpm
 * @param  rotor_measured 反馈转子 rpm
 * @retval [-TRIGGER_BENCH_CURRENT_LIMIT, +TRIGGER_BENCH_CURRENT_LIMIT]
 */
int16_t trigger_bench_current(trigger_bench_pid_t *p, int16_t rotor_target,
                              int16_t rotor_measured);

/**
 * @brief  松开按键后的刹车电流：纯 P、反向于当前转向，让盘尽快停住
 * @note   过冲（松手后还多送几发）直接由它决定；积分不参与，避免反冲
 */
int16_t trigger_bench_brake(int16_t rotor_measured);

/**
 * @brief  超速判据：|转子 rpm| 超过 8000 返回 1
 */
uint8_t trigger_bench_overspeed(int16_t rotor_rpm);

/**
 * @brief  失控判据：|转子 rpm| 超过本档目标的 1.5 倍（下限 1000）返回 1
 * @note   防的是"电流符号反了导致正反馈跑飞"：档位 1 的目标只有 360 转子 rpm，
 *         等跑到 8000 才断的话盘已经多转两圈多；按档位相对值判只多转小半圈
 */
uint8_t trigger_bench_runaway(int16_t rotor_rpm, int16_t rotor_target);

/**
 * @brief  失速判据：|转子 rpm| 低于 |目标|/4（下限 50 rpm）返回 1
 */
uint8_t trigger_bench_stalled(int16_t rotor_rpm, int16_t rotor_target);

/* ==========================================================================
 *  按键状态机（纯逻辑，主机可测）
 *
 *  这是上一版翻车的地方：HAL 按下时读回 GPIO_PIN_RESET(0)，如果把"0"
 *  直接当"按下了"用，整个按住/松开逻辑就会反过来——上电自己转、按键反而停。
 *  所以极性在这里被**显式固定成 1 = 按下**，调用方必须先把 HAL 的
 *  RESET/高电平翻译成 1/0，再由主机测试把时序钉死。
 * ========================================================================== */
#define TRIGGER_BENCH_KEY_DEBOUNCE_MS 20u

typedef struct {
  uint8_t raw_last;    /* 上次原始电平，1 = 按下 */
  uint8_t stable;      /* 去抖后的状态，1 = 按下 */
  uint8_t hold_fired;  /* 本次按住是否已经报过 hold_event */
  uint32_t change_ms;  /* 原始电平最后一次变化的时刻 */
  uint32_t press_ms;   /* 去抖后确认按下的时刻 */
  uint8_t hold_event;  /* 本次 update 产生的边沿：按住达到阈值 */
  uint8_t short_event; /* 本次 update 产生的边沿：短按后松开 */
} trigger_bench_key_t;

void trigger_bench_key_reset(trigger_bench_key_t *k);

/**
 * @brief  喂一次电平，产出边沿事件
 * @param  raw_pressed 1 = 按键按下（极性由调用方翻译好，见上面的警告）
 * @param  now_ms      当前毫秒
 * @note   hold_event / short_event 只在本次调用有效，下次调用自动清零
 */
void trigger_bench_key_update(trigger_bench_key_t *k, uint8_t raw_pressed,
                              uint32_t now_ms);

/**
 * @brief  当前是否按住（去抖后）
 */
uint8_t trigger_bench_key_is_pressed(const trigger_bench_key_t *k);

#endif
