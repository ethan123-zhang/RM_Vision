/**
  ******************************************************************************
  * @file       chassis_bringup.c
  * @brief      底盘调试层上板实现
  * @note       只调用官方已有的接口（CAN_cmd_chassis / detect_hook / 遥控器指针），
  *             不改官方 task 的内部结构。
  ******************************************************************************
  */

#include "chassis_bringup.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "can.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "remote_control.h"

#include "bsp_buzzer.h"
#include "chassis_speed_test.h"

/* 标定表非法时提醒用，参数抄官方 test_task.c */
#define CHASSIS_BEEP_PSC 1
#define CHASSIS_BEEP_PWM 30000

static uint32_t chassis_bringup_ms_since(TickType_t start) {
  TickType_t ticks = xTaskGetTickCount() - start;
  /* 用 64 位乘，避免跑满 70 分钟后 ticks*1000 溢出把扫描时序打乱 */
  return (uint32_t)(((uint64_t)ticks * 1000u) / (uint32_t)configTICK_RATE_HZ);
}

#if CHASSIS_DEBUG_CAN
static void chassis_bringup_can_send(uint32_t std_id, uint8_t data[8]) {
  CAN_TxHeaderTypeDef tx;
  uint32_t mailbox = 0;

  tx.StdId = std_id;
  tx.ExtId = 0;
  tx.IDE = CAN_ID_STD;
  tx.RTR = CAN_RTR_DATA;
  tx.DLC = 8;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &tx, data, &mailbox);
}

/* 摇杆 ±660 塞进 int8：除以 8 后范围 ±82，README 里写清楚恢复方法 */
static int8_t chassis_bringup_i8(int16_t value) {
  int16_t v = (int16_t)(value / 8);
  if (v > 127) {
    v = 127;
  } else if (v < -128) {
    v = -128;
  }
  return (int8_t)v;
}
#endif /* CHASSIS_DEBUG_CAN */

static uint8_t chassis_bringup_motor_online_mask(void) {
  uint8_t can_mask = 0;
  uint8_t i;

  for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
    if (!toe_is_error((uint8_t)(CHASSIS_MOTOR1_TOE + i))) {
      can_mask |= (uint8_t)(1u << i);
    }
  }
  return can_mask;
}

static uint8_t chassis_bringup_ready(void) {
  uint8_t can_mask = chassis_bringup_motor_online_mask();
  uint8_t need = (uint8_t)((1u << CHASSIS_WHEEL_NUM) - 1u);

  if ((can_mask & need) != need) {
    return 0;
  }
  if (toe_is_error(DBUS_TOE)) {
    return 0;
  }
  return 1;
}

static void chassis_bringup_beep(uint8_t times) {
  uint8_t i;
  for (i = 0; i < times; i++) {
    buzzer_on(CHASSIS_BEEP_PSC, CHASSIS_BEEP_PWM);
    vTaskDelay(pdMS_TO_TICKS(80));
    buzzer_off();
    vTaskDelay(pdMS_TO_TICKS(120));
  }
}

const motor_measure_t *chassis_bringup_measure_point(uint8_t wheel_index) {
  uint8_t can_id = chassis_map_wheel_can_id(wheel_index);

  if (can_id < 1 || can_id > 8) {
    /* 表写错了：先回 0 号电机的数据把指针撑住，输出侧一定会被清零 */
    return get_chassis_motor_measure_point(0);
  }
  return get_chassis_motor_measure_point((uint8_t)(can_id - 1));
}

int chassis_bringup_wheel_to_can(const int16_t wheel[CHASSIS_WHEEL_NUM],
                                 int16_t can[CHASSIS_WHEEL_NUM]) {
  return chassis_map_wheel_to_can(wheel, can);
}

void chassis_bringup_wait_ready(void) {
  TickType_t start = xTaskGetTickCount();
  TickType_t timeout = pdMS_TO_TICKS(CHASSIS_BRINGUP_WAIT_MS);

  if (!chassis_map_valid()) {
    /* 表非法 = 四个轮子的电流全是 0，车一定不动。用蜂鸣器喊出来，别让它静默失败 */
    chassis_bringup_beep(3);
    return;
  }

  while (!chassis_bringup_ready() &&
         (TickType_t)(xTaskGetTickCount() - start) < timeout) {
    vTaskDelay(pdMS_TO_TICKS(CHASSIS_CONTROL_TIME_MS));
  }
}

/* 按键模式下的"已按次数"，供 0x1F1 调试帧上报。
   只在按键模式编进来，否则非按键模式下"只写不读"会吃一个编译告警。 */
#if CHASSIS_BRINGUP_MODE == CHASSIS_BRINGUP_MODE_KEY_ID
static uint8_t s_key_count;
#endif

/* ==========================================================================
 *  按键验 ID（阶段 0）
 * ========================================================================== */

static void chassis_bringup_key_gpio_init(void) {
  GPIO_InitTypeDef gpio;

  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Pin = CHASSIS_KEY_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CHASSIS_KEY_PORT, &gpio);
}

typedef struct {
  uint8_t raw_last;
  uint8_t stable;
  uint32_t change_ms;
} chassis_key_gpio_t;

/* 返回 1 = 去抖之后检测到一次"按下"（下降沿） */
static uint8_t chassis_bringup_key_edge(chassis_key_gpio_t *g,
                                        uint32_t now_ms) {
  /* 0 = 按下（内部上拉，按键接地） */
  uint8_t raw = (HAL_GPIO_ReadPin(CHASSIS_KEY_PORT, CHASSIS_KEY_PIN) ==
                 GPIO_PIN_RESET)
                    ? 0u
                    : 1u;

  if (raw != g->raw_last) {
    /* 电平还在跳，重新计时 */
    g->raw_last = raw;
    g->change_ms = now_ms;
    return 0;
  }

  if (g->stable != raw &&
      (uint32_t)(now_ms - g->change_ms) >= (uint32_t)CHASSIS_KEY_DEBOUNCE_MS) {
    g->stable = raw;
    if (g->stable == 0u) {
      return 1;
    }
  }

  return 0;
}

void chassis_bringup_key_id_loop(chassis_move_t *chassis) {
  chassis_key_state_t st;
  chassis_key_gpio_t gpio;
  TickType_t start;
  uint32_t beep_until_ms = 0;
  uint8_t id;
  int16_t cur;
  int16_t can_current[CHASSIS_WHEEL_NUM];
  uint8_t i;

  chassis_bringup_key_gpio_init();

  gpio.raw_last = 1;
  gpio.stable = 1;
  gpio.change_ms = 0;

  chassis_key_reset(&st);
#if CHASSIS_BRINGUP_MODE == CHASSIS_BRINGUP_MODE_KEY_ID
  s_key_count = 0;
#endif

  CAN_cmd_chassis(0, 0, 0, 0);

  start = xTaskGetTickCount();

  for (;;) {
    uint32_t now = chassis_bringup_ms_since(start);
    uint8_t press_rc;

    if (chassis_bringup_key_edge(&gpio, now)) {
      press_rc = chassis_key_press(&st, now);
      if (press_rc != 0u) {
        /* 响一下表示"这一下设备收到了"，不用盯着分析仪也能数 */
        buzzer_on(CHASSIS_BEEP_PSC, CHASSIS_BEEP_PWM);
        beep_until_ms =
            now + ((press_rc == 2u) ? CHASSIS_KEY_BEEP_ERROR_MS
                                    : CHASSIS_KEY_BEEP_MS);
        if (beep_until_ms == 0u) {
          beep_until_ms = 1u;
        }
      }
    }

    if (beep_until_ms != 0u && now >= beep_until_ms) {
      buzzer_off();
      beep_until_ms = 0;
    }

    chassis_key_tick(&st, now, &id, &cur);
#if CHASSIS_BRINGUP_MODE == CHASSIS_BRINGUP_MODE_KEY_ID
    s_key_count = st.count;
#endif

    for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
      can_current[i] = 0;
    }
    if (id >= 1 && id <= CHASSIS_WHEEL_NUM) {
      can_current[id - 1] = cur;
    }

    CAN_cmd_chassis(can_current[0], can_current[1], can_current[2],
                    can_current[3]);
    chassis_bringup_status_update(chassis);

    vTaskDelay(pdMS_TO_TICKS(CHASSIS_CONTROL_TIME_MS));
  }
}

/* 架空单轮速度测试：裸 CAN ID，不依赖尚未标定的整车方向表。
 * 任意按键取消；掉线取消且恢复后不会自行重启。 */
void chassis_bringup_speed_loop(chassis_move_t *chassis) {
  chassis_key_state_t selection;
  chassis_key_gpio_t gpio = {1, 1, 0};
  chassis_speed_test_t speed;
  TickType_t start = xTaskGetTickCount();
  uint32_t run_start = 0, last_debug = 0;
  uint8_t active = 0, fault = 0;
  chassis_bringup_key_gpio_init();
  chassis_key_reset(&selection);
  chassis_speed_reset(&speed);
  CAN_cmd_chassis(0, 0, 0, 0);
  for (;;) {
    uint32_t now = chassis_bringup_ms_since(start);
    int16_t currents[4] = {0, 0, 0, 0};
    int16_t target = 0, measured = 0, current = 0, unused_current;
    uint8_t requested = 0;
    if (chassis_bringup_key_edge(&gpio, now)) {
      if (active) {
        active = 0;
        chassis_key_reset(&selection);
        chassis_speed_reset(&speed);
      } else {
        fault = 0;
        (void)chassis_key_press(&selection, now);
      }
    }
    if (!active) {
      chassis_key_tick(&selection, now, &requested, &unused_current);
      if (requested >= 1 && requested <= 4) {
        /* 即使未上线，也消费这次请求，避免随后上电时突然启动。 */
        chassis_key_reset(&selection);
        if (!toe_is_error((uint8_t)(CHASSIS_MOTOR1_TOE + requested - 1))) {
          active = requested;
          run_start = now;
          chassis_speed_reset(&speed);
        } else {
          fault = 2;
        }
      }
    }
    if (active) {
      uint32_t elapsed = now - run_start;
      if (elapsed >= CHASSIS_SPEED_DURATION_MS ||
          toe_is_error((uint8_t)(CHASSIS_MOTOR1_TOE + active - 1))) {
        if (elapsed < CHASSIS_SPEED_DURATION_MS) fault = 2;
        active = 0;
        chassis_speed_reset(&speed);
      } else {
        measured = get_chassis_motor_measure_point(active - 1)->speed_rpm;
        target = chassis_speed_target(elapsed);
        if (measured > 1500 || measured < -1500) {
          fault = 3;
          active = 0;
          target = 0;
          chassis_speed_reset(&speed);
        } else {
          current = chassis_speed_step(&speed, target, measured, 1);
          currents[active - 1] = current;
        }
      }
    }
    CAN_cmd_chassis(currents[0], currents[1], currents[2], currents[3]);
#if CHASSIS_DEBUG_CAN
    if ((uint32_t)(now - last_debug) >= 25u) {
      /* 0x1F2: ID, 状态(0空闲/1运行), 目标rpm, 实际rpm, 输出电流；均大端。 */
      uint8_t data[8];
      last_debug = now;
      data[0] = active;
      data[1] = active ? 1u : fault;
      data[2] = (uint8_t)((uint16_t)target >> 8);
      data[3] = (uint8_t)target;
      data[4] = (uint8_t)((uint16_t)measured >> 8);
      data[5] = (uint8_t)measured;
      data[6] = (uint8_t)((uint16_t)current >> 8);
      data[7] = (uint8_t)current;
      chassis_bringup_can_send(0x1F2, data);
    }
#else
    (void)last_debug;
#endif
    chassis_bringup_status_update(chassis);
    vTaskDelay(pdMS_TO_TICKS(CHASSIS_CONTROL_TIME_MS));
  }
}

void chassis_bringup_sweep_loop(chassis_move_t *chassis) {
  TickType_t start;
  chassis_sweep_step_t step;
  int16_t can_current[CHASSIS_WHEEL_NUM];
  uint8_t i;

  /* 先发一帧全零，清掉上电前可能残留的指令 */
  can_current[0] = 0;
  can_current[1] = 0;
  can_current[2] = 0;
  can_current[3] = 0;
  CAN_cmd_chassis(0, 0, 0, 0);

  start = xTaskGetTickCount();

  for (;;) {
    chassis_sweep_eval(chassis_bringup_ms_since(start), &step);

    for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
      can_current[i] = 0;
    }
    if (step.can_id >= 1 && step.can_id <= CHASSIS_WHEEL_NUM) {
      can_current[step.can_id - 1] = step.current;
    }

    CAN_cmd_chassis(can_current[0], can_current[1], can_current[2],
                    can_current[3]);
    chassis_bringup_status_update(chassis);

    vTaskDelay(pdMS_TO_TICKS(CHASSIS_CONTROL_TIME_MS));
  }
}

void chassis_bringup_status_update(const chassis_move_t *chassis) {
#if CHASSIS_DEBUG_CAN
  static TickType_t last_rpm_tick = 0;
  static TickType_t last_status_tick = 0;

  TickType_t now = xTaskGetTickCount();
  uint8_t data[8];
  uint8_t i;

  if (chassis == 0) {
    return;
  }

  if ((TickType_t)(now - last_rpm_tick) >=
      pdMS_TO_TICKS(CHASSIS_DEBUG_RPM_PERIOD_MS)) {
    last_rpm_tick = now;
    for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
      int16_t rpm = 0;
      if (chassis->motor_chassis[i].chassis_motor_measure != 0) {
        rpm = chassis->motor_chassis[i].chassis_motor_measure->speed_rpm;
      }
      data[i * 2] = (uint8_t)((uint16_t)rpm >> 8);
      data[i * 2 + 1] = (uint8_t)((uint16_t)rpm & 0xFFu);
    }
    chassis_bringup_can_send(CHASSIS_DEBUG_ID_RPM, data);
  }

  if ((TickType_t)(now - last_status_tick) >=
      pdMS_TO_TICKS(CHASSIS_DEBUG_STATUS_PERIOD_MS)) {
    const RC_ctrl_t *rc = get_remote_control_point();

    last_status_tick = now;

    data[0] = chassis_map_online_mask(chassis_bringup_motor_online_mask());
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = chassis_map_valid() ? (uint8_t)CHASSIS_BRINGUP_MODE : 0xFFu;
    data[6] = toe_is_error(DBUS_TOE) ? 0 : 1;
#if CHASSIS_BRINGUP_MODE == CHASSIS_BRINGUP_MODE_KEY_ID
    /* 按键模式：这一字节用来在分析仪上看"设备数到的按键次数"，
       比听蜂鸣器可靠；0 表示计数已兑现或还没按 */
    data[7] = s_key_count;
#else
    data[7] = (uint8_t)chassis->chassis_mode;
#endif

    if (rc != 0) {
      data[1] = (uint8_t)((rc->rc.s[0] & 0x03u) | ((rc->rc.s[1] & 0x03u) << 2));
      data[2] = (uint8_t)chassis_bringup_i8(rc->rc.ch[0]);
      data[3] = (uint8_t)chassis_bringup_i8(rc->rc.ch[1]);
      data[4] = (uint8_t)chassis_bringup_i8(rc->rc.ch[2]);
    }

    chassis_bringup_can_send(CHASSIS_DEBUG_ID_STATUS, data);
  }
#else
  (void)chassis;
#endif
}
