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
  *             按 KEY N 次（1..4）选择电流档位，停手 3 s 后开始输出；
  *             运行中再按一次 KEY 立即取消。
  * ==========================================================================*/
#ifndef CHASSIS_OPEN_LOOP_H
#define CHASSIS_OPEN_LOOP_H

#include <stdint.h>

/* 固定测试的电机：CAN ID 4（右后轮位，见标定表） */
#define CHASSIS_OPEN_LOOP_MOTOR 4u

/* 电流档位：KEY 按 1/2/3/4 次对应这四档。单位同 C620 电流指令。 */
#define CHASSIS_OPEN_LOOP_I1 250
#define CHASSIS_OPEN_LOOP_I2 400
#define CHASSIS_OPEN_LOOP_I3 550
#define CHASSIS_OPEN_LOOP_I4 700
#define CHASSIS_OPEN_LOOP_LEVELS 4u

/* 本模式自己的硬上限，任何档位都不会超过它 */
#define CHASSIS_OPEN_LOOP_CURRENT_LIMIT 800

/* 单次输出时长：到时自动归零并释放 */
#define CHASSIS_OPEN_LOOP_DURATION_MS 12000u

/* 超速保护：绝对转子 rpm 超过它就取消（轮子架空，但仍保留兜底） */
#define CHASSIS_OPEN_LOOP_RPM_LIMIT 900

/**
 * @brief  KEY 次数 -> 电流档位
 * @param  level 1..4
 * @retval 对应电流；越界返回 0
 */
int16_t chassis_open_loop_current(uint8_t level);

/**
 * @brief  转速保护判断
 * @param  rpm 实测转子 rpm
 * @retval 1 = 超过 CHASSIS_OPEN_LOOP_RPM_LIMIT，需要取消
 */
uint8_t chassis_open_loop_overspeed(int16_t rpm);

/**
 * @brief  本次输出是否还在时长窗口内
 * @param  elapsed_ms 从开始输出算起的毫秒数
 * @retval 1 = 继续输出；0 = 到时，应归零
 */
uint8_t chassis_open_loop_running(uint32_t elapsed_ms);

#endif
