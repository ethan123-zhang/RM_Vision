/**
  ******************************************************************************
  * @file       trigger_bench.c
  * @brief      拨弹盘台架模式上板实现（C610 ID7 / CAN1 / 0x207 反馈 / 0x1FF 控制）
  *
  *             行为对齐比赛用法：按住转、松开停。台架上要验证的四件事：
  *               1) 选定档位下连续送弹不卡（软件侧：失速判据 + 0x1F3 电流/转速）
  *               2) 松手停得住（松开后走纯 P 刹车，看还多送几发）
  *               3) 断电即停（直接关电池开关，硬件行为）
  *               4) 送弹速率（盘轴 rpm / 60 × 18 发/秒）
  *
  *             不碰云台、不碰底盘解算、不改官方 task 的内部结构。
  ******************************************************************************
  */

#include "trigger_bench.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "main.h"
#include "can.h"

#include "CAN_receive.h"
#include "detect_task.h"
#include "chassis_bringup.h"
#include "bsp_buzzer.h"

/* 蜂鸣器参数与 chassis_bringup.c 保持一致 */
#define TRIGGER_BENCH_BEEP_PSC 1
#define TRIGGER_BENCH_BEEP_PWM 30000
#define TRIGGER_BENCH_BEEP_ON_MS 50u
#define TRIGGER_BENCH_BEEP_OFF_MS 120u

/* 刹车/归零阈值：|转子 rpm| 低于它就不再给刹车电流，避免来回抖 */
#define TRIGGER_BENCH_BRAKE_DEAD_RPM 30

/* ==========================================================================
 *  时间、按键、蜂鸣器的小工具（本文件私有，不依赖 chassis_bringup.c 的静态函数）
 * ========================================================================== */

static uint32_t trigger_bench_ms_since(TickType_t start) {
  TickType_t ticks = xTaskGetTickCount() - start;
  return (uint32_t)(((uint64_t)ticks * 1000u) / (uint32_t)configTICK_RATE_HZ);
}

static void trigger_bench_key_gpio_init(void) {
  GPIO_InitTypeDef gpio;

  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Pin = CHASSIS_KEY_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CHASSIS_KEY_PORT, &gpio);
}

/* 读原始电平，并把极性统一成"1 = 按下"。
 * 内部上拉 + 按键接地：读回 GPIO_PIN_RESET(0) 才是按下。
 * 上一版就是在这里把 0 当成"按下"的返回值，导致上电自己转、按键反而停。 */
static uint8_t trigger_bench_key_raw_pressed(void) {
  return (HAL_GPIO_ReadPin(CHASSIS_KEY_PORT, CHASSIS_KEY_PIN) ==
          GPIO_PIN_RESET)
             ? 1u
             : 0u;
}

/* 非阻塞蜂鸣：不占用控制周期，避免 CAN 断流让 C610 进保护 */
static uint8_t s_beep_toggles_left;
static uint8_t s_beep_is_on;
static uint32_t s_beep_next_ms;

static void trigger_bench_beep_request(uint8_t times, uint32_t now_ms) {
  if (times == 0u) {
    return;
  }
  s_beep_toggles_left = (uint8_t)(times * 2u);
  s_beep_is_on = 1;
  buzzer_on(TRIGGER_BENCH_BEEP_PSC, TRIGGER_BENCH_BEEP_PWM);
  s_beep_next_ms = now_ms + TRIGGER_BENCH_BEEP_ON_MS;
}

static void trigger_bench_beep_tick(uint32_t now_ms) {
  if (s_beep_toggles_left == 0u) {
    return;
  }
  if ((int32_t)(now_ms - s_beep_next_ms) < 0) {
    return;
  }

  s_beep_toggles_left--;
  if (s_beep_toggles_left == 0u) {
    buzzer_off();
    s_beep_is_on = 0;
    return;
  }

  s_beep_is_on = (uint8_t)(!s_beep_is_on);
  if (s_beep_is_on) {
    buzzer_on(TRIGGER_BENCH_BEEP_PSC, TRIGGER_BENCH_BEEP_PWM);
    s_beep_next_ms = now_ms + TRIGGER_BENCH_BEEP_ON_MS;
  } else {
    buzzer_off();
    s_beep_next_ms = now_ms + TRIGGER_BENCH_BEEP_OFF_MS;
  }
}

/* ==========================================================================
 *  CAN：控制帧走 CAN1 的 0x1FF，第 4–5 字节 = ID7 的电流
 *  注意：C620 轮子只监听 0x200，收到 0x1FF 不会动；但若总线上一节
 *        还有 ID5/6/8 的 C610，它们会跟着响，本台架默认没有。
 * ========================================================================== */

static void trigger_bench_send_current(int16_t current) {
  CAN_TxHeaderTypeDef tx;
  uint32_t mailbox = 0;
  uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  tx.StdId = 0x1FFu;
  tx.ExtId = 0;
  tx.IDE = CAN_ID_STD;
  tx.RTR = CAN_RTR_DATA;
  tx.DLC = 8;

  data[4] = (uint8_t)((uint16_t)current >> 8);
  data[5] = (uint8_t)((uint16_t)current & 0xFFu);

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &tx, data, &mailbox);
}

#if CHASSIS_DEBUG_CAN
static void trigger_bench_send_debug(uint8_t level, uint8_t state,
                                     int16_t target_rpm, int16_t measured_rpm,
                                     int16_t current) {
  CAN_TxHeaderTypeDef tx;
  uint32_t mailbox = 0;
  uint8_t data[8];

  tx.StdId = TRIGGER_BENCH_DEBUG_ID;
  tx.ExtId = 0;
  tx.IDE = CAN_ID_STD;
  tx.RTR = CAN_RTR_DATA;
  tx.DLC = 8;

  data[0] = level;
  data[1] = state;
  data[2] = (uint8_t)((uint16_t)target_rpm >> 8);
  data[3] = (uint8_t)((uint16_t)target_rpm & 0xFFu);
  data[4] = (uint8_t)((uint16_t)measured_rpm >> 8);
  data[5] = (uint8_t)((uint16_t)measured_rpm & 0xFFu);
  data[6] = (uint8_t)((uint16_t)current >> 8);
  data[7] = (uint8_t)((uint16_t)current & 0xFFu);

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &tx, data, &mailbox);
}
#endif /* CHASSIS_DEBUG_CAN */

/* ==========================================================================
 *  主循环：按住转 / 松开刹车
 * ========================================================================== */

void trigger_bench_loop(chassis_move_t *chassis) {
  trigger_bench_key_t key;
  trigger_bench_pid_t pid;
  TickType_t start = xTaskGetTickCount();

  uint8_t level = 1;
  uint8_t state = TRIGGER_BENCH_IDLE;
  uint8_t last_fault = 0;
  uint32_t run_start = 0;
  uint32_t last_debug = 0;
  uint32_t stall_ms = 0;
  int16_t measured_rpm = 0;
  int16_t target_rpm = 0;
  int16_t command = 0;

  (void)chassis;

  trigger_bench_key_gpio_init();
  trigger_bench_key_reset(&key);
  trigger_bench_pid_reset(&pid);
  trigger_bench_send_current(0);

  for (;;) {
    uint32_t now = trigger_bench_ms_since(start);
    uint8_t fault = 0;

    measured_rpm = get_trigger_motor_measure_point()->speed_rpm;

    /* 电平 -> 事件：短按切档、按住够久开始转（纯逻辑，主机测试覆盖） */
    trigger_bench_key_update(&key, trigger_bench_key_raw_pressed(), now);

    if (key.short_event) {
      level = (uint8_t)((level % TRIGGER_BENCH_LEVELS) + 1u);
      trigger_bench_beep_request(level, now);
    }

    if (key.hold_event && state == TRIGGER_BENCH_IDLE) {
      state = TRIGGER_BENCH_RUN;
      run_start = now;
      stall_ms = 0;
      last_fault = 0;
      target_rpm = 0;
      trigger_bench_pid_reset(&pid);
      trigger_bench_beep_request(1, now);
    }

    if (state == TRIGGER_BENCH_RUN) {
      uint32_t elapsed = now - run_start;

      if (!trigger_bench_key_is_pressed(&key)) {
        /* 松开：立即停，本周期就给一帧刹车电流 */
        state = TRIGGER_BENCH_IDLE;
        target_rpm = 0;
        trigger_bench_pid_reset(&pid);
        command = trigger_bench_brake(measured_rpm);
        trigger_bench_beep_request(1, now);
      } else if (toe_is_error(TRIGGER_MOTOR_TOE)) {
        fault = TRIGGER_BENCH_FAULT_OFFLINE;
      } else if (trigger_bench_overspeed(measured_rpm) ||
                 trigger_bench_runaway(measured_rpm,
                                       trigger_bench_rotor_target(level))) {
        fault = TRIGGER_BENCH_FAULT_OVERSPEED;
      } else if (elapsed >= TRIGGER_BENCH_MAX_RUN_MS) {
        fault = TRIGGER_BENCH_FAULT_TIMEOUT;
      } else {
        int16_t level_target = trigger_bench_rotor_target(level);

        target_rpm = trigger_bench_hold_target(elapsed, level_target);

        /* 软启动完成后再判失速（卡弹的软件特征） */
        if (elapsed >= (uint32_t)(TRIGGER_BENCH_SOFT_START_MS + 200u)) {
          if (trigger_bench_stalled(measured_rpm, level_target)) {
            stall_ms += CHASSIS_CONTROL_TIME_MS;
            if (stall_ms >= TRIGGER_BENCH_STALL_MS) {
              fault = TRIGGER_BENCH_FAULT_STALL;
            }
          } else {
            stall_ms = 0;
          }
        } else {
          stall_ms = 0;
        }

        if (fault == 0u) {
          command = trigger_bench_current(&pid, target_rpm, measured_rpm);
        }
      }
    } else {
      /* 空闲：还有残速就纯 P 刹车，停下就发 0 */
      target_rpm = 0;
      if ((measured_rpm > TRIGGER_BENCH_BRAKE_DEAD_RPM) ||
          (measured_rpm < -TRIGGER_BENCH_BRAKE_DEAD_RPM)) {
        command = trigger_bench_brake(measured_rpm);
      } else {
        command = 0;
      }
    }

    if (fault != 0u) {
      last_fault = fault;
      state = TRIGGER_BENCH_IDLE;
      command = 0;
      target_rpm = 0;
      stall_ms = 0;
      trigger_bench_pid_reset(&pid);
      trigger_bench_beep_request(3, now);
    }

    trigger_bench_send_current(command);

#if CHASSIS_DEBUG_CAN
    if ((uint32_t)(now - last_debug) >= TRIGGER_BENCH_DEBUG_PERIOD_MS) {
      uint8_t report =
          (uint8_t)((state == TRIGGER_BENCH_RUN) ? TRIGGER_BENCH_RUN
                                                : last_fault);
      last_debug = now;
      trigger_bench_send_debug(level, report, target_rpm, measured_rpm,
                               command);
    }
#else
    (void)last_debug;
#endif

    trigger_bench_beep_tick(now);

    vTaskDelay(pdMS_TO_TICKS(CHASSIS_CONTROL_TIME_MS));
  }
}
