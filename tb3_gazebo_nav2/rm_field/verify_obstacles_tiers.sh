#!/usr/bin/env bash
# 动态箱三档验收：依次跑 slow(0.3) / fast(0.6) / faster(1.0 m/s)，每档独立起栈→验收→停栈。
#   bash verify_obstacles_tiers.sh                  # 三档
#   RM_OBSTACLE_TIERS="fast" bash verify_obstacles_tiers.sh
# 前置：无障基线 results/baseline_route.json 得存在（没有就先 capture_route.py）。
# 产出：results/report_nav2_obstacles.json（slow，老文件名）+ _fast.json + _faster.json
# 退出码：0 = 三档全过；1 = 有档位失败/起栈失败。
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
source /opt/ros/jazzy/setup.bash 2>/dev/null
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RM_DYN_TIER="${RM_DYN_TIER:-}"

TIERS=(${RM_OBSTACLE_TIERS:-slow fast faster})
[ -f "$ROOT/results/baseline_route.json" ] || { echo "缺无障基线 results/baseline_route.json"; exit 2; }

declare -a SUMMARY=()
FAIL=0
for t in "${TIERS[@]}"; do
  echo
  echo "################ 动态箱档位 $t ################"
  if ! bash "$ROOT/nav2_obstacles.sh" --tier "$t"; then
    SUMMARY+=("$t: 起栈失败")
    FAIL=$((FAIL+1))
    bash "$ROOT/stop_nav2_field.sh" >/dev/null 2>&1
    continue
  fi
  python3 "$ROOT/verify_nav2_obstacles.py" --tier "$t"
  rc=$?
  bash "$ROOT/stop_nav2_field.sh" >/dev/null 2>&1
  if [ "$rc" = 0 ]; then
    SUMMARY+=("$t: PASS")
  else
    SUMMARY+=("$t: FAIL (verify rc=$rc)")
    FAIL=$((FAIL+1))
  fi
done

echo
echo "################ 汇总 ################"
for s in "${SUMMARY[@]}"; do echo "  $s"; done
echo
if [ "$FAIL" = 0 ]; then
  echo "OK: 动态箱三档验收全过"
  exit 0
fi
echo "FAIL: $FAIL 档没过（看 results/report_nav2_obstacles*.json 的 checks）"
exit 1