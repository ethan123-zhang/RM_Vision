/**
  ******************************************************************************
  * @file       trigger_bench_logic.c
  * @brief      拨弹盘台架模式的纯逻辑实现（无 HAL / 无 FreeRTOS，可在主机上测）
  ******************************************************************************
  */

#include "trigger_bench_logic.h"

int16_t trigger_bench_disc_rpm(uint8_t level) {
  switch (level) {
  case 1:
    return (int16_t)TRIGGER_BENCH_DISC_RPM_1;
  case 2:
    return (int16_t)TRIGGER_BENCH_DISC_RPM_2;
  case 3:
    return (int16_t)TRIGGER_BENCH_DISC_RPM_3;
  case 4:
    return (int16_t)TRIGGER_BENCH_DISC_RPM_4;
  default:
    return 0;
  }
}

int16_t trigger_bench_rotor_target(uint8_t level) {
  int16_t disc = trigger_bench_disc_rpm(level);
  return (int16_t)(disc * TRIGGER_BENCH_GEAR_RATIO);
}

int16_t trigger_bench_disc_from_rotor(int16_t rotor_rpm) {
  /* 先放大 10 倍做四舍五入，避免直接截断在 36 附近抖一位 */
  int32_t scaled = (int32_t)rotor_rpm * 10;
  int32_t disc10 = scaled / (int32_t)TRIGGER_BENCH_GEAR_RATIO;

  if (disc10 >= 0) {
    disc10 = (disc10 + 5) / 10;
  } else {
    disc10 = (disc10 - 5) / 10;
  }
  return (int16_t)disc10;
}

float trigger_bench_feed_rate(uint8_t level) {
  int16_t disc = trigger_bench_disc_rpm(level);
  return (float)disc * (float)TRIGGER_BENCH_BALLS_PER_REV / 60.0f;
}

int16_t trigger_bench_hold_target(uint32_t elapsed_ms, int16_t rotor_target) {
  int32_t value;

  if (rotor_target == 0) {
    return 0;
  }
  if (elapsed_ms >= TRIGGER_BENCH_SOFT_START_MS) {
    return rotor_target;
  }

  value = (int32_t)rotor_target * (int32_t)elapsed_ms /
          (int32_t)TRIGGER_BENCH_SOFT_START_MS;
  return (int16_t)value;
}

void trigger_bench_pid_reset(trigger_bench_pid_t *p) {
  if (p != 0) {
    p->integral = 0.0f;
  }
}

int16_t trigger_bench_current(trigger_bench_pid_t *p, int16_t rotor_target,
                              int16_t rotor_measured) {
  float error;
  float integral;
  float output;

  if (p == 0) {
    return 0;
  }

  error = (float)rotor_target - (float)rotor_measured;

  integral = p->integral + TRIGGER_BENCH_KI * error;
  if (integral > TRIGGER_BENCH_I_LIMIT) {
    integral = TRIGGER_BENCH_I_LIMIT;
  } else if (integral < -TRIGGER_BENCH_I_LIMIT) {
    integral = -TRIGGER_BENCH_I_LIMIT;
  }
  p->integral = integral;

  output = TRIGGER_BENCH_KP * error + integral;
  if (output > (float)TRIGGER_BENCH_CURRENT_LIMIT) {
    output = (float)TRIGGER_BENCH_CURRENT_LIMIT;
  } else if (output < -(float)TRIGGER_BENCH_CURRENT_LIMIT) {
    output = -(float)TRIGGER_BENCH_CURRENT_LIMIT;
  }

  return (int16_t)output;
}

int16_t trigger_bench_brake(int16_t rotor_measured) {
  float output = -(float)TRIGGER_BENCH_KP * (float)rotor_measured;

  if (output > (float)TRIGGER_BENCH_CURRENT_LIMIT) {
    output = (float)TRIGGER_BENCH_CURRENT_LIMIT;
  } else if (output < -(float)TRIGGER_BENCH_CURRENT_LIMIT) {
    output = -(float)TRIGGER_BENCH_CURRENT_LIMIT;
  }

  return (int16_t)output;
}

uint8_t trigger_bench_overspeed(int16_t rotor_rpm) {
  if (rotor_rpm > TRIGGER_BENCH_OVERSPEED_RPM) {
    return 1;
  }
  if (rotor_rpm < -TRIGGER_BENCH_OVERSPEED_RPM) {
    return 1;
  }
  return 0;
}

uint8_t trigger_bench_runaway(int16_t rotor_rpm, int16_t rotor_target) {
  int32_t limit;
  int32_t magnitude = rotor_rpm;

  if (rotor_target == 0) {
    return 0;
  }

  limit = ((rotor_target > 0 ? (int32_t)rotor_target : -(int32_t)rotor_target) *
           3) /
          2;
  if (limit < 1000) {
    limit = 1000;
  }

  if (magnitude < 0) {
    magnitude = -magnitude;
  }

  return (magnitude > limit) ? 1u : 0u;
}

uint8_t trigger_bench_stalled(int16_t rotor_rpm, int16_t rotor_target) {
  int32_t limit;
  int32_t magnitude = rotor_rpm;

  if (rotor_target == 0) {
    return 0;
  }

  if (magnitude < 0) {
    magnitude = -magnitude;
  }

  limit = ((rotor_target > 0 ? (int32_t)rotor_target : -(int32_t)rotor_target) /
           TRIGGER_BENCH_STALL_DIV);
  if (limit < 50) {
    limit = 50;
  }

  return (magnitude < limit) ? 1u : 0u;
}

/* ==========================================================================
 *  按键状态机
 *
 *  极性约定：入参 raw_pressed = 1 表示按下。HAL 的 GPIO_PIN_RESET(0)
 *  必须在调用方翻译成 1，否则整段逻辑会反过来（上一版就是栽在这里）。
 * ========================================================================== */

void trigger_bench_key_reset(trigger_bench_key_t *k) {
  if (k == 0) {
    return;
  }
  k->raw_last = 0; /* 0 = 未按下 */
  k->stable = 0;
  k->hold_fired = 0;
  k->change_ms = 0;
  k->press_ms = 0;
  k->hold_event = 0;
  k->short_event = 0;
}

void trigger_bench_key_update(trigger_bench_key_t *k, uint8_t raw_pressed,
                              uint32_t now_ms) {
  uint8_t raw;

  if (k == 0) {
    return;
  }

  raw = (raw_pressed != 0u) ? 1u : 0u;

  k->hold_event = 0;
  k->short_event = 0;

  if (raw != k->raw_last) {
    /* 电平还在跳：重新计时，不认 */
    k->raw_last = raw;
    k->change_ms = now_ms;
  } else if (k->stable != raw &&
             (uint32_t)(now_ms - k->change_ms) >=
                 TRIGGER_BENCH_KEY_DEBOUNCE_MS) {
    /* 稳定了：认这次电平变化 */
    k->stable = raw;
    if (k->stable) {
      k->press_ms = now_ms;
      k->hold_fired = 0;
    } else if ((uint32_t)(now_ms - k->press_ms) < TRIGGER_BENCH_HOLD_KEY_MS) {
      k->short_event = 1;
    }
  }

  /* 按住达到阈值：整次按住只报一次 */
  if (k->stable && !k->hold_fired &&
      (uint32_t)(now_ms - k->press_ms) >= TRIGGER_BENCH_HOLD_KEY_MS) {
    k->hold_event = 1;
    k->hold_fired = 1;
  }
}

uint8_t trigger_bench_key_is_pressed(const trigger_bench_key_t *k) {
  return (k != 0 && k->stable != 0u) ? 1u : 0u;
}
