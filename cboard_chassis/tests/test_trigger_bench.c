/* 拨弹盘台架模式的纯逻辑断言：档位、送弹速率、软启动、电流环限幅、刹车、故障判据。
   运行：pwsh -File cboard_chassis/tests/run_trigger_tests.ps1
   上板实测在 application/trigger_bench.c，这里只保证"单位、时序、限幅"不出错。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "trigger_bench_logic.h"

int main(void) {
  /* 档位 -> 盘轴 rpm：1..4 = 10/20/30/50（比赛可用的低转速），越界 0 */
  assert(trigger_bench_disc_rpm(1) == TRIGGER_BENCH_DISC_RPM_1);
  assert(trigger_bench_disc_rpm(2) == TRIGGER_BENCH_DISC_RPM_2);
  assert(trigger_bench_disc_rpm(3) == TRIGGER_BENCH_DISC_RPM_3);
  assert(trigger_bench_disc_rpm(4) == TRIGGER_BENCH_DISC_RPM_4);
  assert(trigger_bench_disc_rpm(0) == 0);
  assert(trigger_bench_disc_rpm(5) == 0);
  assert(trigger_bench_disc_rpm(255) == 0);

  /* 档位单调递增 */
  assert(TRIGGER_BENCH_DISC_RPM_1 < TRIGGER_BENCH_DISC_RPM_2);
  assert(TRIGGER_BENCH_DISC_RPM_2 < TRIGGER_BENCH_DISC_RPM_3);
  assert(TRIGGER_BENCH_DISC_RPM_3 < TRIGGER_BENCH_DISC_RPM_4);

  /* 减速比 36:1：转子目标 = 盘轴 * 36（最容易写错的一处） */
  assert(TRIGGER_BENCH_GEAR_RATIO == 36);
  assert(trigger_bench_rotor_target(1) == 360);
  assert(trigger_bench_rotor_target(2) == 720);
  assert(trigger_bench_rotor_target(3) == 1080);
  assert(trigger_bench_rotor_target(4) == 1800);
  assert(trigger_bench_rotor_target(0) == 0);

  /* 送弹速率：一圈 18 弹位，发/秒 = 盘轴 rpm / 60 * 18 = 盘轴 * 0.3 */
  assert(TRIGGER_BENCH_BALLS_PER_REV == 18);
  assert(fabsf(trigger_bench_feed_rate(1) - 3.0f) < 1e-4f);
  assert(fabsf(trigger_bench_feed_rate(2) - 6.0f) < 1e-4f);
  assert(fabsf(trigger_bench_feed_rate(3) - 9.0f) < 1e-4f);
  assert(fabsf(trigger_bench_feed_rate(4) - 15.0f) < 1e-4f);
  assert(fabsf(trigger_bench_feed_rate(0) - 0.0f) < 1e-4f);

  /* 转子 -> 盘轴四舍五入，正负对称 */
  assert(trigger_bench_disc_from_rotor(0) == 0);
  assert(trigger_bench_disc_from_rotor(36) == 1);
  assert(trigger_bench_disc_from_rotor(18) == 1);
  assert(trigger_bench_disc_from_rotor(-18) == -1);
  assert(trigger_bench_disc_from_rotor(360) == 10);
  assert(trigger_bench_disc_from_rotor(1800) == 50);
  assert(trigger_bench_disc_from_rotor(-1800) == -50);

  /* 软启动：300 ms 内线性升到目标，之后保持 */
  assert(TRIGGER_BENCH_SOFT_START_MS == 300u);
  assert(trigger_bench_hold_target(0u, 1800) == 0);
  assert(trigger_bench_hold_target(150u, 1800) == 900);
  assert(trigger_bench_hold_target(300u, 1800) == 1800);
  assert(trigger_bench_hold_target(5000u, 1800) == 1800);
  assert(trigger_bench_hold_target(1000u, 0) == 0);

  /* 按键与运行窗口 */
  assert(TRIGGER_BENCH_HOLD_KEY_MS == 300u);
  assert(TRIGGER_BENCH_MAX_RUN_MS == 20000u);

  /* 电流环：永不超过台架限幅，误差方向正确 */
  assert(TRIGGER_BENCH_CURRENT_LIMIT == 3000);
  {
    trigger_bench_pid_t pid;
    int16_t cmd;

    trigger_bench_pid_reset(&pid);
    cmd = trigger_bench_current(&pid, 1800, 0);
    assert(cmd > 0);
    assert(cmd <= TRIGGER_BENCH_CURRENT_LIMIT);

    /* 大误差必须打到限幅，而不是继续线性放大 */
    trigger_bench_pid_reset(&pid);
    cmd = trigger_bench_current(&pid, 8000, 0);
    assert(cmd == TRIGGER_BENCH_CURRENT_LIMIT);

    /* 反向误差给负电流 */
    trigger_bench_pid_reset(&pid);
    cmd = trigger_bench_current(&pid, 0, 8000);
    assert(cmd == -TRIGGER_BENCH_CURRENT_LIMIT);

    /* 接近目标时电流很小（不会一直顶着） */
    trigger_bench_pid_reset(&pid);
    cmd = trigger_bench_current(&pid, 1800, 1790);
    assert(cmd >= 0);
    assert(cmd < 50);

    /* 积分限幅：连续同向误差后，输出仍被电流限幅夹住 */
    trigger_bench_pid_reset(&pid);
    for (int i = 0; i < 4000; ++i) {
      cmd = trigger_bench_current(&pid, 8000, 0);
      assert(cmd <= TRIGGER_BENCH_CURRENT_LIMIT);
      assert(cmd >= -TRIGGER_BENCH_CURRENT_LIMIT);
    }
    assert(pid.integral <= TRIGGER_BENCH_I_LIMIT);
  }

  /* 刹车：纯 P、与转向相反，且不超过限幅 */
  assert(trigger_bench_brake(0) == 0);
  assert(trigger_bench_brake(1000) < 0);
  assert(trigger_bench_brake(-1000) > 0);
  assert(trigger_bench_brake(1000) == -500);
  assert(trigger_bench_brake(-1000) == 500);
  assert(trigger_bench_brake(8000) == -TRIGGER_BENCH_CURRENT_LIMIT);
  assert(trigger_bench_brake(-8000) == TRIGGER_BENCH_CURRENT_LIMIT);

  /* 超速：|转子 rpm| > 8000 才断；档位 4 的 1800 不得误触发 */
  assert(TRIGGER_BENCH_OVERSPEED_RPM == 8000);
  assert(trigger_bench_rotor_target(TRIGGER_BENCH_LEVELS) <
         TRIGGER_BENCH_OVERSPEED_RPM);
  assert(trigger_bench_overspeed(0) == 0);
  assert(trigger_bench_overspeed(8000) == 0);
  assert(trigger_bench_overspeed(8001) == 1);
  assert(trigger_bench_overspeed(-8000) == 0);
  assert(trigger_bench_overspeed(-8001) == 1);

  /* 失控：超过本档目标 1.5 倍就断（方向反了会跑飞，靠它兜住） */
  assert(trigger_bench_runaway(0, 1800) == 0);
  assert(trigger_bench_runaway(2700, 1800) == 0);
  assert(trigger_bench_runaway(2701, 1800) == 1);
  assert(trigger_bench_runaway(-2701, 1800) == 1);
  /* 低目标（1 档 360）：下限 1000 兜住，避免小目标被噪声误触发 */
  assert(trigger_bench_runaway(999, 360) == 0);
  assert(trigger_bench_runaway(1001, 360) == 1);

  /* 失速（卡弹）：|rpm| < 目标/4 判失速，恰好 1/4 不算；目标 0 不判 */
  assert(TRIGGER_BENCH_STALL_DIV == 4);
  assert(trigger_bench_stalled(0, 1800) == 1);
  assert(trigger_bench_stalled(449, 1800) == 1);
  assert(trigger_bench_stalled(450, 1800) == 0);
  assert(trigger_bench_stalled(1799, 1800) == 0);
  assert(trigger_bench_stalled(-1200, 1800) == 0);
  assert(trigger_bench_stalled(0, 0) == 0);
  assert(trigger_bench_stalled(0, -1800) == 1);
  /* 低目标：90 rpm 门限（360/4），不会因为下限 50 而失效 */
  assert(trigger_bench_stalled(89, 360) == 1);
  assert(trigger_bench_stalled(90, 360) == 0);
  assert(trigger_bench_stalled(0, 100) == 1);

  /* ===== 按键状态机（上一版翻车的回归测试）=====
     极性约定：raw_pressed = 1 表示按下。谁把 HAL 的 RESET(0) 直接当按下传进来，
     下面第一条 steady 断言就会挂——这正是"上电自己转、按键反而停"的根因。 */
  {
    trigger_bench_key_t key;

    trigger_bench_key_reset(&key);
    /* 松手时：不是按下、没有事件 */
    trigger_bench_key_update(&key, 0, 0);
    assert(trigger_bench_key_is_pressed(&key) == 0);
    assert(key.hold_event == 0);
    assert(key.short_event == 0);

    /* 按住：20 ms 去抖后才认（raw=1 必须被认成"按下"） */
    trigger_bench_key_update(&key, 1, 100); /* 电平变化，重新计时 */
    trigger_bench_key_update(&key, 1, 110);
    assert(trigger_bench_key_is_pressed(&key) == 0);
    trigger_bench_key_update(&key, 1, 121); /* 变化后 21 ms -> 认按下 */
    assert(trigger_bench_key_is_pressed(&key) == 1);
    assert(key.press_ms == 121);

    /* 按住不足 300 ms：没有 hold_event */
    trigger_bench_key_update(&key, 1, 400);
    assert(key.hold_event == 0);
    assert(key.short_event == 0);

    /* 到 300 ms：hold_event 只报一次 */
    trigger_bench_key_update(&key, 1, 421);
    assert(key.hold_event == 1);
    trigger_bench_key_update(&key, 1, 430);
    assert(key.hold_event == 0);

    /* 松开（按住 > 300 ms）：不算短按，不产生 short_event */
    trigger_bench_key_update(&key, 0, 600);
    trigger_bench_key_update(&key, 0, 621);
    assert(trigger_bench_key_is_pressed(&key) == 0);
    assert(key.short_event == 0);
    assert(key.hold_event == 0);

    /* 短按：按下 -> 松开 < 300 ms -> short_event */
    trigger_bench_key_update(&key, 1, 700);
    trigger_bench_key_update(&key, 1, 721);
    assert(trigger_bench_key_is_pressed(&key) == 1);
    trigger_bench_key_update(&key, 0, 780);
    trigger_bench_key_update(&key, 0, 801);
    assert(trigger_bench_key_is_pressed(&key) == 0);
    assert(key.short_event == 1);
    assert(key.hold_event == 0);

    /* 抖动：20 ms 内的跳变不认 */
    trigger_bench_key_reset(&key);
    trigger_bench_key_update(&key, 1, 1000);
    trigger_bench_key_update(&key, 0, 1005);
    trigger_bench_key_update(&key, 1, 1010);
    trigger_bench_key_update(&key, 1, 1015);
    assert(trigger_bench_key_is_pressed(&key) == 0);
    assert(key.hold_event == 0);
    assert(key.short_event == 0);
  }

  printf("trigger_bench tests passed\n");
  return 0;
}
