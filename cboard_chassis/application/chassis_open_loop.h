/**
  ******************************************************************************
  * @file       chassis_open_loop.h
  * @brief      固定电流开环单电机测试（机械诊断用，纯逻辑部分）。
  *
  *             用途：给指定电机（固定 CAN ID 4）施加**恒定电流指令**，不经过
  *             速度环，观察轮子是否仍有"每转一次顿一下"的机械/换向现象。
  *             C620 内部仍闭合其电流环（指令单位同官方 CAN 电流），固件侧
  *             只做"发固定值 + 超时/超速/掉线保护"，不做任何速度反馈调节。
  *
  *             4号轮静摩擦很大：250~700 恒流顶不动（实测角度几乎不变）。
  *             因此开环分两段：先用 1500 起转脉冲把转子转起来（无速度环，
  *             只是恒流），一旦 |rpm| 达到 100 就切到用户选定的恒定保持电流。
  *
  *             按 KEY N 次（1..4）选择保持电流档位，停手 3 s 后开始；
  *             运行中再按一次 KEY 立即取消。
  * ==========================================================================*/
#ifndef CHASSIS_OPEN_LOOP_H
#define CHASSIS_OPEN_LOOP_H

#include <stdint.h>

/* 固定测试的电机：CAN ID 4（右后轮位，见标定表） */
#define CHASSIS_OPEN_LOOP_MOTOR 4u

/* 电流档位：KEY 按 1/2/3/4 次对应这四档（保持段电流，单位同 C620）。
   450 略高于 400rpm 时的动摩擦（约 430~440），用于观察近稳态。 */
#define CHASSIS_OPEN_LOOP_I1 450
#define CHASSIS_OPEN_LOOP_I2 550
#define CHASSIS_OPEN_LOOP_I3 650
#define CHASSIS_OPEN_LOOP_I4 750
#define CHASSIS_OPEN_LOOP_LEVELS 4u

/* 起转脉冲：恒流 1500，最多持续 2 s，转速达到 100rpm 即切保持段 */
#define CHASSIS_OPEN_LOOP_KICK_CURRENT 1500
#define CHASSIS_OPEN_LOOP_KICK_MAX_MS 2000u
#define CHASSIS_OPEN_LOOP_BREAKAWAY_RPM 100

/* 保持段：切过去以后恒定输出 8 s，然后归零释放 */
#define CHASSIS_OPEN_LOOP_HOLD_MS 8000u

/* 本模式硬上限：任何阶段都不超过 1500（与速度模式限流一致） */
#define CHASSIS_OPEN_LOOP_CURRENT_LIMIT 1500

/* 超速保护：绝对转子 rpm 超过它就取消（轮子架空，但仍保留兜底） */
#define CHASSIS_OPEN_LOOP_RPM_LIMIT 900

/**
 * @brief  KEY 次数 -> 保持段电流档位
 * @param  level 1..4
 * @retval 对应电流；越界返回 0
 */
int16_t chassis_open_loop_hold_current(uint8_t level);

/**
 * @brief  起转段是否已经成功脱开静摩擦
 * @param  rpm 实测转子 rpm
 * @retval 1 = |rpm| 达到 CHASSIS_OPEN_LOOP_BREAKAWAY_RPM
 */
uint8_t chassis_open_loop_breakaway_ok(int16_t rpm);

/**
 * @brief  起转脉冲是否还在允许的时长内
 * @param  elapsed_ms 从开始输出算起的毫秒数
 * @retval 1 = 继续起转段；0 = 起转脉冲超时
 */
uint8_t chassis_open_loop_kick_running(uint32_t elapsed_ms);

/**
 * @brief  保持段是否还在时长窗口内
 * @param  elapsed_ms 从切入保持段算起的毫秒数
 * @retval 1 = 继续保持段；0 = 到时，应归零
 */
uint8_t chassis_open_loop_hold_running(uint32_t elapsed_ms);

/**
 * @brief  转速保护判断
 * @param  rpm 实测转子 rpm
 * @retval 1 = 超过 CHASSIS_OPEN_LOOP_RPM_LIMIT，需要取消
 */
uint8_t chassis_open_loop_overspeed(int16_t rpm);

#endif
