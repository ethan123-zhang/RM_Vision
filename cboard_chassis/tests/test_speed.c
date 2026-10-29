#include <assert.h>
#include <stdio.h>
#include "chassis_speed_test.h"

int main(void) {
  chassis_speed_test_t s;
  int i;
  chassis_speed_reset(&s);
  assert(chassis_speed_step(&s, 600, 0, 0) == 0);
  assert(s.integral == 0);
  assert(chassis_speed_step(&s, 600, 0, 1) > 0);
  assert(chassis_speed_step(&s, -600, 0, 1) < 0);
  chassis_speed_reset(&s);
  for (i = 0; i < 10000; ++i)
    assert(chassis_speed_step(&s, 600, 0, 1) <= CHASSIS_SPEED_CURRENT_LIMIT);
  assert(s.integral <= CHASSIS_SPEED_INTEGRAL_LIMIT);
  chassis_speed_reset(&s);
  for (i = 0; i < 10000; ++i)
    assert(chassis_speed_step(&s, 600, -2000, 1) == CHASSIS_SPEED_CURRENT_LIMIT);
  assert(s.integral == 0); /* 持续饱和不得积分累积 */
  for (i = 0; i < 10000; ++i)
    assert(chassis_speed_step(&s, -600, 2000, 1) == -CHASSIS_SPEED_CURRENT_LIMIT);
  assert(s.integral == 0);
  assert(chassis_speed_step(&s, 600, 0, 0) == 0);
  assert(s.integral == 0);
  assert(chassis_speed_step(&s, 0, 200, 1) < 0);
  assert(chassis_speed_target(0) == 600);
  assert(chassis_speed_target(2000) == 600);
  assert(chassis_speed_target(7999) == 600);
  assert(chassis_speed_target(8000) == 0);
  assert(chassis_speed_target(8999) == 0);
  assert(chassis_speed_target(9000) == -600);
  assert(chassis_speed_target(16999) == -600);
  assert(chassis_speed_target(17000) == 0);
  assert(chassis_speed_target(17999) == 0);
  assert(CHASSIS_SPEED_DURATION_MS == 18000u);
  assert(chassis_speed_target(18000) == 0);
  puts("speed: PASS (limits, anti-windup, feedback loss, braking, timing)");
  return 0;
}
