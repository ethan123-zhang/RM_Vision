/* 固定电流开环测试的纯逻辑断言：档位电流、超速保护、时长窗口。
   运行：pwsh -File cboard_chassis/tests/run_open_loop_tests.ps1 */
#include <assert.h>
#include <stdio.h>
#include "chassis_open_loop.h"

int main(void) {
  /* 档位映射：KEY 1..4 次 -> 250/400/550/700，越界 0 */
  assert(chassis_open_loop_current(1) == CHASSIS_OPEN_LOOP_I1);
  assert(chassis_open_loop_current(2) == CHASSIS_OPEN_LOOP_I2);
  assert(chassis_open_loop_current(3) == CHASSIS_OPEN_LOOP_I3);
  assert(chassis_open_loop_current(4) == CHASSIS_OPEN_LOOP_I4);
  assert(chassis_open_loop_current(0) == 0);
  assert(chassis_open_loop_current(5) == 0);
  assert(chassis_open_loop_current(255) == 0);

  /* 任何档位都不能超过本模式硬上限 */
  for (uint8_t level = 1; level <= CHASSIS_OPEN_LOOP_LEVELS; ++level) {
    int16_t current = chassis_open_loop_current(level);
    assert(current > 0);
    assert(current <= CHASSIS_OPEN_LOOP_CURRENT_LIMIT);
  }
  /* 档位单调递增，便于"先低档再高档"地找转速 */
  assert(CHASSIS_OPEN_LOOP_I1 < CHASSIS_OPEN_LOOP_I2);
  assert(CHASSIS_OPEN_LOOP_I2 < CHASSIS_OPEN_LOOP_I3);
  assert(CHASSIS_OPEN_LOOP_I3 < CHASSIS_OPEN_LOOP_I4);

  /* 超速保护：|rpm| 超过 900 才取消，恰好 900 不取消 */
  assert(chassis_open_loop_overspeed(0) == 0);
  assert(chassis_open_loop_overspeed(899) == 0);
  assert(chassis_open_loop_overspeed(900) == 0);
  assert(chassis_open_loop_overspeed(901) == 1);
  assert(chassis_open_loop_overspeed(-900) == 0);
  assert(chassis_open_loop_overspeed(-901) == 1);

  /* 时长窗口：12 s，最后 1 ms 仍在输出，到点即停 */
  assert(CHASSIS_OPEN_LOOP_DURATION_MS == 12000u);
  assert(chassis_open_loop_running(0) == 1);
  assert(chassis_open_loop_running(1) == 1);
  assert(chassis_open_loop_running(11999) == 1);
  assert(chassis_open_loop_running(12000) == 0);
  assert(chassis_open_loop_running(12001) == 0);
  assert(chassis_open_loop_running(0xFFFFFFFFu) == 0);

  /* 固定对象是 CAN ID 4，防止后续误改成别的轮 */
  assert(CHASSIS_OPEN_LOOP_MOTOR == 4u);

  printf("open_loop tests passed\n");
  return 0;
}
