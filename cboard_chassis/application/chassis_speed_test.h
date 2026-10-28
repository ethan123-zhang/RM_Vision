#ifndef CHASSIS_SPEED_TEST_H
#define CHASSIS_SPEED_TEST_H
#include <stdint.h>

/* 电机转子 rpm，不是减速箱输出轮 rpm；控制周期固定 2 ms。
 * 独立于整车 PID，避免把 rpm 增益误用于官方 m/s 环。
 * 初始值只供架空低速验证，未经过实车整定。 */
#define CHASSIS_SPEED_RPM 600
#define CHASSIS_SPEED_KP 2.0f
#define CHASSIS_SPEED_KI 0.002f
#define CHASSIS_SPEED_CURRENT_LIMIT 1500
#define CHASSIS_SPEED_INTEGRAL_LIMIT 400.0f
#define CHASSIS_SPEED_DURATION_MS 6000u

typedef struct { float integral; } chassis_speed_test_t;
void chassis_speed_reset(chassis_speed_test_t *s);
int16_t chassis_speed_step(chassis_speed_test_t *s, int16_t target,
                          int16_t measured, uint8_t enabled);
int16_t chassis_speed_target(uint32_t elapsed_ms);
#endif
