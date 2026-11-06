#!/usr/bin/env bash
# 一键跑围挡碰撞验收：起带围挡世界 → verify_fence.py → 停服务器
# 用法: bash run_fence_test.sh
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
echo "=== 1. 起带围挡服务器 ==="
bash start_server.sh "$ROOT/worlds/rmul2027_3v3_field_fenced.sdf" fenced
echo
echo "=== 2. 围挡碰撞验收 ==="
FIELD_ROOT="$ROOT" GZ_WORLD=rmul2027_3v3 python3 verify_fence.py
RC=$?
echo
echo "=== 3. 停服务器 ==="
bash stop_field.sh
exit $RC