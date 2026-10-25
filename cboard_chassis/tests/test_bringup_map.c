/**
  ******************************************************************************
  * @file       test_bringup_map.c
  * @brief      chassis_bringup_map 的主机侧测试（不需要板子、不需要 Keil）。
  *
  *             编译时用 -DCONFIG_xxx 选一套标定表，同一份测试跑四种配置：
  *               CONFIG_DEFAULT  默认表 1/2/3/4 + 全 +1
  *               CONFIG_PERMUTED 故意打乱：3/4/1/2 + {+,-,+,-}
  *               CONFIG_DUP      重复 ID 1/1/3/4（应判非法）
  *               CONFIG_BADID    ID 越界 1/2/9/4（应判非法）
  *
  *             期望值在这里是**写死**的，不从被测代码里推，避免自证。
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>

#include "chassis_bringup_map.h"

static int g_checks;
static int g_fails;

#define CHECK(cond, msg)                                                      \
  do {                                                                        \
    g_checks++;                                                               \
    if (!(cond)) {                                                            \
      g_fails++;                                                              \
      printf("  FAIL  line %d: %s\n", __LINE__, msg);                         \
    }                                                                         \
  } while (0)

#if defined(CONFIG_PERMUTED)
/* 轮位 i -> CAN ID / 方向，故意不规则，专门抓"位置放错、符号漏乘" */
static const int exp_id[CHASSIS_WHEEL_NUM] = {3, 4, 1, 2};
static const int exp_dir[CHASSIS_WHEEL_NUM] = {+1, -1, +1, -1};
#elif defined(CONFIG_DUP)
static const int exp_id[CHASSIS_WHEEL_NUM] = {1, 1, 3, 4};
static const int exp_dir[CHASSIS_WHEEL_NUM] = {+1, +1, +1, +1};
#elif defined(CONFIG_BADID)
static const int exp_id[CHASSIS_WHEEL_NUM] = {1, 2, 9, 4};
static const int exp_dir[CHASSIS_WHEEL_NUM] = {+1, +1, +1, +1};
#else
static const int exp_id[CHASSIS_WHEEL_NUM] = {1, 2, 3, 4};
static const int exp_dir[CHASSIS_WHEEL_NUM] = {+1, +1, +1, +1};
#endif

static void test_timing_constant(void) {
  /* 400 + 1200 + 600 + 1200 + 1200 = 4600；*4 + 2000 = 20400 */
  CHECK(CHASSIS_SWEEP_ONE_MS == 4600, "单个 ID 占用时间应为 4600 ms");
  CHECK(CHASSIS_SWEEP_CYCLE_MS == 20400, "整轮扫描周期应为 20400 ms");
}

static void expect_sweep(uint32_t t, uint8_t id, int16_t cur, const char *why) {
  chassis_sweep_step_t s;
  s.can_id = 0xEE; /* 故意先弄脏，确认函数自己会写 */
  s.current = 0x7EEE;

  chassis_sweep_eval(t, &s);
  g_checks++;
  if (s.can_id != id || s.current != cur) {
    g_fails++;
    printf("  FAIL  t=%5u (%s): 期望 id=%u cur=%d，实际 id=%u cur=%d\n", t, why,
           (unsigned)id, (int)cur, (unsigned)s.can_id, (int)s.current);
  }
}

static void test_sweep_timeline(void) {
  /* 边界逐个卡：每一段的两端都要对 */
  expect_sweep(0, 1, 0, "ID1 静默");
  expect_sweep(399, 1, 0, "ID1 静默末");
  expect_sweep(400, 1, CHASSIS_SWEEP_CURRENT, "ID1 正转起");
  expect_sweep(1599, 1, CHASSIS_SWEEP_CURRENT, "ID1 正转末");
  expect_sweep(1600, 1, 0, "ID1 中停起");
  expect_sweep(2199, 1, 0, "ID1 中停末");
  expect_sweep(2200, 1, -CHASSIS_SWEEP_CURRENT, "ID1 反转起");
  expect_sweep(3399, 1, -CHASSIS_SWEEP_CURRENT, "ID1 反转末");
  expect_sweep(3400, 1, 0, "ID1 尾停起");
  expect_sweep(4599, 1, 0, "ID1 尾停末");

  expect_sweep(4600, 2, 0, "ID2 静默");
  expect_sweep(9200, 3, 0, "ID3 静默");
  expect_sweep(13800, 4, 0, "ID4 静默");
  expect_sweep(14200, 4, CHASSIS_SWEEP_CURRENT, "ID4 正转起");
  expect_sweep(18000, 4, 0, "ID4 尾停");

  expect_sweep(18400, 0, 0, "扫描间歇：四路全零");
  expect_sweep(20399, 0, 0, "间歇末尾仍全零");

  expect_sweep(20400, 1, 0, "回绕到 ID1");
  expect_sweep(20800, 1, CHASSIS_SWEEP_CURRENT, "回绕后 ID1 正转");
  expect_sweep(20400u * 3 + 20800u, 1, CHASSIS_SWEEP_CURRENT, "多圈回绕仍正确");
}

static void test_id_table(void) {
  int i;
  for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
    g_checks++;
    if (chassis_map_wheel_can_id((uint8_t)i) != (uint8_t)exp_id[i]) {
      g_fails++;
      printf("  FAIL  轮位 %d 的 ID 表项应为 %d，实际 %u\n", i, exp_id[i],
             (unsigned)chassis_map_wheel_can_id((uint8_t)i));
    }
  }
  CHECK(chassis_map_wheel_can_id(CHASSIS_WHEEL_NUM) == 0, "轮位越界返回 0");
  CHECK(chassis_map_wheel_can_id(200) == 0, "轮位大越界返回 0");
}

static void test_valid_flag(void) {
#if defined(CONFIG_DUP) || defined(CONFIG_BADID)
  CHECK(chassis_map_valid() == 0, "非法表应判 0");
#else
  CHECK(chassis_map_valid() == 1, "合法表应判 1");
#endif
}

static void test_wheel_to_can(void) {
  int16_t wheel[CHASSIS_WHEEL_NUM];
  int16_t can[CHASSIS_WHEEL_NUM];
  int i;
  int rc;

  for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
    wheel[i] = (int16_t)((i + 1) * 100); /* 100 200 300 400，全是正数便于看符号 */
  }
  memset(can, 0x5A, sizeof(can));
  rc = chassis_map_wheel_to_can(wheel, can);

#if defined(CONFIG_DUP)
  CHECK(rc == -2, "重复 ID 应返回 -2");
  for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
    CHECK(can[i] == 0, "失败时输出必须全 0");
  }
  return;
#elif defined(CONFIG_BADID)
  CHECK(rc == -1, "ID 越界应返回 -1");
  for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
    CHECK(can[i] == 0, "失败时输出必须全 0");
  }
  return;
#else
  CHECK(rc == 0, "合法表应返回 0");
#endif

  /* 通用校验：第 exp_id[k]-1 个 CAN 槽位 = wheel[k] * exp_dir[k] */
  for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
    int slot = exp_id[i] - 1;
    int expect = exp_dir[i] * (int)wheel[i];
    g_checks++;
    if ((int)can[slot] != expect) {
      g_fails++;
      printf("  FAIL  CAN 槽 %d 应为 %d，实际 %d\n", slot, expect, (int)can[slot]);
    }
  }

#if defined(CONFIG_PERMUTED)
  /* 显式向量，防止上面的"通用校验"和被测算错成同一个样子 */
  CHECK(can[0] == 300, "permuted: can[0] 应为 ID1 = 轮位2 = +300");
  CHECK(can[1] == -400, "permuted: can[1] 应为 ID2 = 轮位3 取反 = -400");
  CHECK(can[2] == 100, "permuted: can[2] 应为 ID3 = 轮位0 = +100");
  CHECK(can[3] == -200, "permuted: can[3] 应为 ID4 = 轮位1 取反 = -200");
#elif !defined(CONFIG_DUP) && !defined(CONFIG_BADID)
  CHECK(can[0] == 100 && can[1] == 200 && can[2] == 300 && can[3] == 400,
        "默认表应为恒等映射");
#endif
}

static void test_online_mask(void) {
  int i;
  uint8_t in, out, expect;
  int cases[] = {0x1, 0x2, 0x4, 0x8, 0xA, 0xF, 0x0};
  int k;

  for (k = 0; k < (int)(sizeof(cases) / sizeof(cases[0])); k++) {
    in = (uint8_t)cases[k];
    expect = 0;
    for (i = 0; i < CHASSIS_WHEEL_NUM; i++) {
      if (in & (1u << (exp_id[i] - 1))) {
        expect |= (uint8_t)(1u << i);
      }
    }
    out = chassis_map_online_mask(in);
    g_checks++;
    if (out != expect) {
      g_fails++;
      printf("  FAIL  在线位图 0x%02X 应为 0x%02X，实际 0x%02X\n", in, expect, out);
    }
  }

#if defined(CONFIG_PERMUTED)
  CHECK(chassis_map_online_mask(0x01) == 0x04, "permuted: 只有 ID1 在线 -> 轮位2");
  CHECK(chassis_map_online_mask(0x08) == 0x02, "permuted: 只有 ID4 在线 -> 轮位1");
#endif
}

/* ---------- 按键验 ID 状态机 ---------- */

static void expect_key(chassis_key_state_t *st, uint32_t t, uint8_t id,
                       int16_t cur, const char *why) {
  uint8_t got_id = 0xEE;
  int16_t got_cur = 0x7EEE;

  chassis_key_tick(st, t, &got_id, &got_cur);
  g_checks++;
  if (got_id != id || got_cur != cur) {
    g_fails++;
    printf("  FAIL  t=%5u (%s): 期望 id=%u cur=%d，实际 id=%u cur=%d\n", t, why,
           (unsigned)id, (int)cur, (unsigned)got_id, (int)got_cur);
  }
}

static void test_key_quiet_rule(void) {
  chassis_key_state_t st;

  chassis_key_reset(&st);
  CHECK(st.count == 0 && st.spin_id == 0, "reset 后应清零");

  /* 按一次，2.999 s 还不该动，3.000 s 才动 */
  CHECK(chassis_key_press(&st, 1000) == 1, "第 1 次按键应被计数");
  CHECK(st.count == 1, "计数应为 1");
  expect_key(&st, 1000, 0, 0, "刚按完");
  expect_key(&st, 3999, 0, 0, "静默 2999 ms，还不到点");
  expect_key(&st, 4000, 1, CHASSIS_KEY_SPIN_CURRENT, "静默满 3000 ms，转 1 号");

  /* 转动 1.5 s：开始时刻 4000，5499 还在转，5500 停 */
  expect_key(&st, 5499, 1, CHASSIS_KEY_SPIN_CURRENT, "转动中");
  expect_key(&st, 5500, 0, 0, "转满 1500 ms，停");
  expect_key(&st, 9000, 0, 0, "停完不应自己再转");
}

static void test_key_press_resets_timer(void) {
  chassis_key_state_t st;

  chassis_key_reset(&st);
  chassis_key_press(&st, 0);
  chassis_key_press(&st, 2900); /* 差 100 ms 就满 3 s，应该把计时重置 */
  expect_key(&st, 5899, 0, 0, "从最后一次按键算 2999 ms，不动");
  expect_key(&st, 5900, 2, CHASSIS_KEY_SPIN_CURRENT, "从最后一次按键算满 3 s，转 2 号");
}

static void test_key_triple_and_overflow(void) {
  chassis_key_state_t st;
  uint8_t rc;
  int i;

  chassis_key_reset(&st);
  chassis_key_press(&st, 0);
  chassis_key_press(&st, 100);
  chassis_key_press(&st, 200);
  CHECK(st.count == 3, "连按 3 次，计数 3");
  expect_key(&st, 3200, 3, CHASSIS_KEY_SPIN_CURRENT, "转 3 号");

  /* 满 4 次以后再按一次 = 超界，计数清零，不驱动 */
  chassis_key_reset(&st);
  for (i = 0; i < CHASSIS_KEY_MAX_ID; i++) {
    chassis_key_press(&st, (uint32_t)i * 100);
  }
  CHECK(st.count == CHASSIS_KEY_MAX_ID, "按满 4 次，计数 4");
  rc = chassis_key_press(&st, 500);
  CHECK(rc == 2, "第 5 次按键应报超界");
  CHECK(st.count == 0, "超界后计数清零");
  expect_key(&st, 9999, 0, 0, "超界后静默再久也不驱动");
}

static void test_key_press_during_spin(void) {
  chassis_key_state_t st;

  chassis_key_reset(&st);
  chassis_key_press(&st, 0);
  expect_key(&st, 3000, 1, CHASSIS_KEY_SPIN_CURRENT, "转 1 号");

  /* 转动中途按键：先记下，等这次转完再按 3 s 规则出下一次 */
  CHECK(chassis_key_press(&st, 3500) == 1, "转动中按键也要计数");
  expect_key(&st, 4400, 1, CHASSIS_KEY_SPIN_CURRENT, "原转动不受影响");
  expect_key(&st, 4500, 0, 0, "原转动到点结束");
  expect_key(&st, 6499, 0, 0, "距中途那次按键 2999 ms");
  expect_key(&st, 6500, 1, CHASSIS_KEY_SPIN_CURRENT, "距中途那次按键满 3 s，转 1 号");
}

int main(void) {
  printf("=== test_bringup_map ===\n");
  test_timing_constant();
  test_sweep_timeline();
  test_id_table();
  test_valid_flag();
  test_wheel_to_can();
  test_online_mask();
  test_key_quiet_rule();
  test_key_press_resets_timer();
  test_key_triple_and_overflow();
  test_key_press_during_spin();

  printf("  %d checks, %d failed\n", g_checks, g_fails);
  if (g_fails != 0) {
    printf("RESULT: FAIL\n");
    return 1;
  }
  printf("RESULT: PASS\n");
  return 0;
}
