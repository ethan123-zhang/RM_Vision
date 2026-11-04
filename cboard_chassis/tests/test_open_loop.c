/* 固定电流开环测试的纯逻辑断言：保持电流档、起转脉冲、超速保护、时长窗口。
   运行：pwsh -File cboard_chassis/tests/run_open_loop_tests.ps1 */
#include <assert.h>
#include <stdio.h>
#include "chassis_open_loop.h"

int main(void) {
  /* 保持段档位：KEY 1..4 次 -> 450/550/650/750，越界 0 */
  assert(chassis_open_loop_hold_current(1) == CHASSIS_OPEN_LOOP_I1);
  assert(chassis_open_loop_hold_current(2) == CHASSIS_OPEN_LOOP_I2);
  assert(chassis_open_loop_hold_current(3) == CHASSIS_OPEN_LOOP_I3);
  assert(chassis_open_loop_hold_current(4) == CHASSIS_OPEN_LOOP_I4);
  assert(chassis_open_loop_hold_current(0) == 0);
  assert(chassis_open_loop_hold_current(5) == 0);
  assert(chassis_open_loop_hold_current(255) == 0);

  /* 档位单调递增，便于"先低档再高档"地找稳态转速 */
  assert(CHASSIS_OPEN_LOOP_I1 < CHASSIS_OPEN_LOOP_I2);
  assert(CHASSIS_OPEN_LOOP_I2 < CHASSIS_OPEN_LOOP_I3);
  assert(CHASSIS_OPEN_LOOP_I3 < CHASSIS_OPEN_LOOP_I4);

  /* 任何档位与起转脉冲都不能超过本模式硬上限 */
  for (uint8_t level = 1; level <= CHASSIS_OPEN_LOOP_LEVELS; ++level) {
    int16_t current = chassis_open_loop_hold_current(level);
    assert(current > 0);
    assert(current <= CHASSIS_OPEN_LOOP_CURRENT_LIMIT);
  }
  assert(CHASSIS_OPEN_LOOP_KICK_CURRENT > CHASSIS_OPEN_LOOP_I4);
  assert(CHASSIS_OPEN_LOOP_KICK_CURRENT <= CHASSIS_OPEN_LOOP_CURRENT_LIMIT);

  /* 起转判定：|rpm| 达到 100 才算脱开静摩擦 */
  assert(CHASSIS_OPEN_LOOP_BREAKAWAY_RPM == 100);
  assert(chassis_open_loop_breakaway_ok(0) == 0);
  assert(chassis_open_loop_breakaway_ok(99) == 0);
  assert(chassis_open_loop_breakaway_ok(100) == 1);
  assert(chassis_open_loop_breakaway_ok(-99) == 0);
  assert(chassis_open_loop_breakaway_ok(-100) == 1);

  /* 起转脉冲窗口：2 s */
  assert(CHASSIS_OPEN_LOOP_KICK_MAX_MS == 2000u);
  assert(chassis_open_loop_kick_running(0) == 1);
  assert(chassis_open_loop_kick_running(1999) == 1);
  assert(chassis_open_loop_kick_running(2000) == 0);
  assert(chassis_open_loop_kick_running(5000) == 0);

  /* 保持段窗口：8 s */
  assert(CHASSIS_OPEN_LOOP_HOLD_MS == 8000u);
  assert(chassis_open_loop_hold_running(0) == 1);
  assert(chassis_open_loop_hold_running(7999) == 1);
  assert(chassis_open_loop_hold_running(8000) == 0);
  assert(chassis_open_loop_hold_running(0xFFFFFFFFu) == 0);

  /* 超速保护：|rpm| 超过 900 才取消，恰好 900 不取消 */
  assert(chassis_open_loop_overspeed(0) == 0);
  assert(chassis_open_loop_overspeed(900) == 0);
  assert(chassis_open_loop_overspeed(901) == 1);
  assert(chassis_open_loop_overspeed(-900) == 0);
  assert(chassis_open_loop_overspeed(-901) == 1);

  /* 固定对象是 CAN ID 4，防止后续误改成别的轮 */
  assert(CHASSIS_OPEN_LOOP_MOTOR == 4u);

  printf("open_loop tests passed\n");
  return 0;
}
