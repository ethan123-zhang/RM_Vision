/**
  ******************************************************************************
  * @file       chassis_bringup_map.c
  * @brief      轮位映射表与单电机扫描时序的实现（纯逻辑，无 HAL 依赖）
  * @note       改标定只改 chassis_bringup_map.h 里的 #define，这里不用动。
  ******************************************************************************
  */

#include "chassis_bringup_map.h"

/* 把宏表落成数组；下标 = 官方解算的轮位索引 0..3 */
static const uint8_t chassis_wheel_can_id[CHASSIS_WHEEL_NUM] = {
    CHASSIS_WHEEL_CAN_ID_0, CHASSIS_WHEEL_CAN_ID_1, CHASSIS_WHEEL_CAN_ID_2,
    CHASSIS_WHEEL_CAN_ID_3};

static const int8_t chassis_wheel_dir[CHASSIS_WHEEL_NUM] = {
    CHASSIS_WHEEL_DIR_0, CHASSIS_WHEEL_DIR_1, CHASSIS_WHEEL_DIR_2,
    CHASSIS_WHEEL_DIR_3};

int chassis_map_valid(void) {
  uint8_t used[8];
  uint8_t i;
  uint8_t id;

  for (i = 0; i < 8; i++) {
    used[i] = 0;
  }

  for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
    id = chassis_wheel_can_id[i];
    if (id < 1 || id > 8) {
      return 0;
    }
    if (used[id - 1]) {
      return 0; /* 两个轮位抢同一个 ID，必然有一个轮子不受控 */
    }
    used[id - 1] = 1;
  }

  for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
    if (chassis_wheel_dir[i] != 1 && chassis_wheel_dir[i] != -1) {
      return 0;
    }
  }

  return 1;
}

uint8_t chassis_map_wheel_can_id(uint8_t wheel_index) {
  if (wheel_index >= CHASSIS_WHEEL_NUM) {
    return 0;
  }
  return chassis_wheel_can_id[wheel_index];
}

int chassis_map_wheel_to_can(const int16_t wheel[CHASSIS_WHEEL_NUM],
                             int16_t can[CHASSIS_WHEEL_NUM]) {
  uint8_t i;
  uint8_t slot;

  if (wheel == 0 || can == 0) {
    return -1;
  }

  /* 先清零：万一表是错的，输出也必须是"四路 0 电流"，不能是上一拍的残留值 */
  for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
    can[i] = 0;
  }

  if (!chassis_map_valid()) {
    /* 区分两种非法：越界 -1、重复 -2，方便上板时看返回值定位 */
    uint8_t seen[8];
    for (i = 0; i < 8; i++) {
      seen[i] = 0;
    }
    for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
      slot = chassis_wheel_can_id[i];
      if (slot < 1 || slot > 8) {
        return -1;
      }
      if (seen[slot - 1]) {
        return -2;
      }
      seen[slot - 1] = 1;
    }
    return -1;
  }

  for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
    slot = (uint8_t)(chassis_wheel_can_id[i] - 1);
    can[slot] = (int16_t)(chassis_wheel_dir[i] * wheel[i]);
  }

  return 0;
}

uint8_t chassis_map_online_mask(uint8_t can_id_mask) {
  uint8_t out = 0;
  uint8_t i;

  for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
    if (can_id_mask & (uint8_t)(1u << (chassis_wheel_can_id[i] - 1))) {
      out |= (uint8_t)(1u << i);
    }
  }

  return out;
}

void chassis_sweep_eval(uint32_t elapsed_ms, chassis_sweep_step_t *step) {
  uint32_t t;
  uint32_t local;
  uint8_t index;

  if (step == 0) {
    return;
  }

  step->can_id = 0;
  step->current = 0;

  t = elapsed_ms % (uint32_t)CHASSIS_SWEEP_CYCLE_MS;

  /* 四路都扫完了，进间歇段：全零，让观察者有喘息和记录的时间 */
  if (t >= (uint32_t)CHASSIS_SWEEP_ONE_MS * CHASSIS_WHEEL_NUM) {
    return;
  }

  index = (uint8_t)(t / (uint32_t)CHASSIS_SWEEP_ONE_MS);
  local = t % (uint32_t)CHASSIS_SWEEP_ONE_MS;

  /* 扫描阶段直接按裸 CAN ID 1..4 走，不受标定表影响：
     这一步的目的就是"先弄明白哪个 ID 是哪个轮子"。 */
  step->can_id = (uint8_t)(index + 1);

  if (local < (uint32_t)CHASSIS_SWEEP_PRE_MS) {
    return;
  }
  local -= (uint32_t)CHASSIS_SWEEP_PRE_MS;

  if (local < (uint32_t)CHASSIS_SWEEP_ON_MS) {
    step->current = CHASSIS_SWEEP_CURRENT;
    return;
  }
  local -= (uint32_t)CHASSIS_SWEEP_ON_MS;

  if (local < (uint32_t)CHASSIS_SWEEP_MID_MS) {
    return;
  }
  local -= (uint32_t)CHASSIS_SWEEP_MID_MS;

  if (local < (uint32_t)CHASSIS_SWEEP_ON_MS) {
    step->current = (int16_t)(-CHASSIS_SWEEP_CURRENT);
    return;
  }

  /* 尾停段：保持 0 */
}
