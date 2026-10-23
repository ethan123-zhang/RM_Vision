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
    data[7] = (uint8_t)chassis->chassis_mode;

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
