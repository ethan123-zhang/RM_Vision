#!/usr/bin/env bash
# 对比 box 碰撞 vs 网格碰撞：启动到就绪时间、稳态 RTF、CPU 占用
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
HZ=$(getconf CLK_TCK)

cpu_pct() {  # 3 秒内平均 CPU%
  local pid=$1
  local a b
  a=$(awk '{print $14+$15}' /proc/$pid/stat)
  sleep 3
  b=$(awk '{print $14+$15}' /proc/$pid/stat)
  echo "scale=1; ($b-$a)*100/(3*$HZ)" | bc
}

run_one() {
  local tag=$1 world=$2
  pkill -f "gz sim" 2>/dev/null; sleep 1.5
  local t0
  t0=$(date +%s%N)
  nohup gz sim -s -r "$world" > "$ROOT/logs/perf_$tag.log" 2>&1 &
  local i
  for i in $(seq 1 240); do
    gz service -l 2>/dev/null | grep -q '/world/rmul2027_3v3/create' && break
    sleep 0.5
  done
  local t1 tready
  t1=$(date +%s%N)
  tready=$(( (t1 - t0) / 1000000 ))
  local pid
  pid=$(pgrep -f 'gz sim -s' | head -1)
  sleep 8
  echo "== $tag =="
  echo "world_ready_ms=$tready  pid=$pid"
  timeout 8 gz topic -e -t /world/rmul2027_3v3/stats -n 1 2>/dev/null \
    | grep -E 'iterations|real_time_factor|sim_time' | head -6
  echo "cpu_pct(3s avg)=$(cpu_pct "$pid")"
  echo "rss_mb=$(awk '/VmRSS/{print int($2/1024)}' /proc/$pid/status)"
  echo "server_log_errors=$(grep -cE '\[Err\]' "$ROOT/logs/perf_$tag.log")"
}

run_one box "$ROOT/worlds/rmul2027_3v3_field.sdf"
run_one mesh "$ROOT/worlds/rmul2027_3v3_field_meshcollision.sdf"
pkill -f "gz sim" 2>/dev/null
echo done