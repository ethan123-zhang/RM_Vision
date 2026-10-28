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
  assert(chassis_speed_target(1999) == 600);
  assert(chassis_speed_target(2000) == 0);
  assert(chassis_speed_target(3000) == -600);
  assert(chassis_speed_target(5000) == 0);
  assert(chassis_speed_target(6000) == 0);
  puts("speed: PASS (limits, anti-windup, feedback loss, braking, timing)");
  return 0;
}
