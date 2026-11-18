/**
  ******************************************************************************
  * @file       trigger_bench.h
  * @brief      拨弹盘台架模式：按住转 / 松开停（比赛行为的最小复现）
  *
  *             硬件前提：C610 电调 CAN ID = 7，接在 C 板 CAN1（PD0/PD1）。
  *             反馈 0x207 已被 CAN_receive.c 解进 motor_chassis[6]；
  *             控制帧 0x1FF 由本模块自己发到 CAN1（官方 CAN_cmd_gimbal 发的是
  *             CAN2，不能直接复用）。
  *
  *             操作：
  *               短按 KEY（< 300 ms） = 切档 1->2->3->4->1，蜂鸣 N 声表示第 N 档
  *               按住 KEY（>= 300 ms） = 以当前档位转，松开立即刹车停
  *               掉线 / 超速 / 失速 / 按住超过 20 s = 断电流并响 3 声
  *
  *             与底盘调车模式互斥：CHASSIS_BRINGUP_MODE = 5 时，
  *             chassis_task 只跑本模式，不跑底盘解算。
  ******************************************************************************
  */

#ifndef TRIGGER_BENCH_H
#define TRIGGER_BENCH_H

#include "struct_typedef.h"

#include "chassis_task.h"
#include "trigger_bench_logic.h"

/* 调试帧（挂在 USB-CAN 分析仪上，CAN1，25 ms 一次）
 *   0x1F3  [0] 档位 1..4
 *          [1] 状态 0 空闲 / 1 运行中 / 2 运行超时（按键卡住）
 *                    / 3 反馈掉线 / 4 超速或失控 / 5 失速（卡弹）
 *          [2:3] 目标转子 rpm（int16 大端）
 *          [4:5] 实测转子 rpm（int16 大端）
 *          [6:7] 当前下发电流（int16 大端）
 *   盘轴 rpm = 转子 rpm / 36；送弹速率 = 盘轴 rpm / 60 × 18 */
#define TRIGGER_BENCH_DEBUG_ID 0x1F3
#define TRIGGER_BENCH_DEBUG_PERIOD_MS 25

typedef enum {
  TRIGGER_BENCH_IDLE = 0,
  TRIGGER_BENCH_RUN = 1,
  TRIGGER_BENCH_FAULT_TIMEOUT = 2,
  TRIGGER_BENCH_FAULT_OFFLINE = 3,
  TRIGGER_BENCH_FAULT_OVERSPEED = 4,
  TRIGGER_BENCH_FAULT_STALL = 5
} trigger_bench_state_e;

/**
 * @brief  拨弹盘台架主循环，不会返回
 *         短按切档（10/20/30/50 盘轴 rpm），按住转、松开刹车停
 * @param  chassis 未使用，仅为与其它调车模式入口保持同一签名
 */
void trigger_bench_loop(chassis_move_t *chassis);

#endif
