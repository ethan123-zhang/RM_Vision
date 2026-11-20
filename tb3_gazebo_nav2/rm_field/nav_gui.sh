#!/usr/bin/env bash
# 地图上点坐标 → 机器人走过去（GUI 模式：Gazebo GUI + RViz + 地图点选窗口 + /goal_pose 桥）。
#   bash nav_gui.sh
# 用法（推荐用"地图点选窗口"，不依赖 RViz 的工具选择）：
#   1) 等三个窗口起来（WSLg；本机 llvmpipe 软渲染，慢一点正常）：
#      地图点选 / Gazebo Sim / RViz
#   2) 【地图点选】窗口里直接 **左键点地图** = 去那里；右键 = 重定位；中键 = 取消
#   3) 想用 RViz：先点工具栏第一个图标 "Nav2 Goal"，再在地图上点+拖；"2D Goal Pose" 走 /goal_pose 桥
#   4) 定位偏了：地图点选窗口右键、或 RViz 的 "2D Pose Estimate" 重新给位姿
#   5) 命令行等价：bash goto_xy.sh 2.5 0.5 0
#   6) 停止：bash stop_nav2_field.sh
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
source /opt/ros/jazzy/setup.bash 2>/dev/null
export DISPLAY="${DISPLAY:-:0}"

echo "=== 0. 生成 RViz 配置（Nav2 Goal 排第一；2D Goal Pose 也在）==="
python3 make_rviz_config.py || { echo "致命: RViz 配置生成失败"; exit 1; }

echo "=== 1. 起栈（Gazebo GUI + RViz）==="
RM_RVIZ_CONFIG="$ROOT/config/rm_field_click.rviz" bash "$ROOT/nav2_field.sh" --gui || exit 1

echo "=== 2. 起 /goal_pose 桥（RViz 2D Goal Pose 用）==="
bash "$ROOT/start_goal_bridge.sh" || exit 1

echo "=== 3. 起地图点选窗口（最不容易点错的那个）==="
bash "$ROOT/start_map_click.sh" || echo "警告: 地图点选窗口没起来（RViz 仍可用）"

cat <<'EOF'

=== 现在可以点地图了 ===
【地图点选】窗口（推荐）：
  - 左键点地图 = 机器人去那里（再点别处 = 换目标）
  - 右键点地图 = 把 AMCL 初始位姿设到那里（定位偏了用它）
  - 中键 = 取消当前目标
  - 绿三角=机器人、蓝叉=目标、灰线=规划路径
RViz（可选）：
  - 先用工具栏第一个图标 "Nav2 Goal"（启动默认就选它），再在地图上点一下、拖出朝向松手
  - "2D Goal Pose" 走 /goal_pose 桥；"2D Pose Estimate" 重定位
命令行：bash goto_xy.sh X Y [YAW]        停止：bash stop_nav2_field.sh
EOF