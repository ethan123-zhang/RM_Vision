# 交接 2026-11-04 晚：演示链路跑通 + 终点判据力臂修复

> 状态：可交付演示。练习车「建图 → 重定位 → 示教 → 回放」全流程在新地图上端到端跑通两次。
> 机器状态：X5 在线、重定位驱动在跑、`bridge_node` 已停（串口释放给遥控）、无残留 `follow_path`。

## 1. 本次做了什么

1. 用新地图 `~/car_ws/maps/venue2-20261104`（`map_*.bin` 143 MB、`path.yaml` 321 点/13.54 m）排练完整演示链路。
2. 排练连续暴露三个终点缺陷，全部修掉并上机复验（详见 §3）。
3. 补齐 `drive_lap.sh` 的演示必需参数（`W_MAX` 等）与数值类型规范。
4. 交付演示当天操作手册：`car_slam_lap/docs/demo-runbook.md`。
5. 沉淀技能：更新 `wiring-mcu-chassis-into-ros2`（路线复现收口律、示教规范、参数透传、Odin 队列溢出），新建 `explaining-with-causal-chains`。

## 2. 关键结论：车"原地转就偏"是力臂几何，不是调参

- Odin1（`base_frame=imu`）离车体转心 **0.267 m**（标定 `(+0.2626, −0.0471)`）。
- 控制器收到 `v=0, w≠0` 时，差速两轮反向 → 车绕**转心**转；传感器点则绕转心画圆：
  `传感器位姿 = 转心 + R(yaw)·offset`。
- 原地转 Δψ 时传感器点位移 `2·|o|·sin(Δψ/2)`：Δψ=155° → ≈0.52 m；实测 180° 时 0.531 m
  vs 理论 0.532 m，二值一致。
- 因此**凡是"到某位姿"的判据，必须比较转心，不能比较传感器点**：
  `转心 = 传感器位姿 − R(yaw)·offset`（`follow_path._rotation_center()`）。

## 3. 排练暴露的三个缺陷与修法（提交 `867a400`）

| 现象（现场日志） | 根因 | 修法 |
| :--- | :--- | :--- |
| `最终对齐时已漂离终点 0.22 m > goal_tol=0.20 m` | `align` 位置判据用传感器点，原地转 155° 扫 0.52 m | 终点判据改用**转心**（`_goal_cx/_goal_cy`） |
| `大转角原地对齐超时（15.0 s）` | 示教末段停车录出 3 个重合尾点（间距 0.2 mm），纯追踪以脚下点为目标 → 方位角抖；且运动段 `d` 卡 0.33 m、`w` 恒定 0.57 rad/s（纯追踪对点目标稳定绕圈） | 末段改**瞄准终点转心 + 先对准再低速直行**（`GOAL_APPROACH_ALIGN_RAD=0.35`、`GOAL_APPROACH_VMAX=0.12`）；`near_end` 必须同时满足"真的走过半程"（闭合路线起点≈末点） |
| `最终朝向对齐超时（失败，未走完）` | 固定 `ALIGN_TIMEOUT_S=5 s`；`w_max=0.35` 时 176° 需约 9 s | 超时按初始角度自适应 `max(5, 1.5·|Δyaw|/(0.8·w_max))` |

## 4. 其他现场问题

- **转弯太猛**：`drive_lap.sh` 只透传 `v_max`，`w_max` 用节点内建默认 **1.2 rad/s（≈69°/s）**。
  现在透传 `W_MAX`（默认 0.35）、`RETURN_W_MAX`（0.30）。
- **参数类型崩**：`MAX_TIME_S=280` 被 ROS 2 推断为 INTEGER，`max_time_s` 是 DOUBLE →
  `InvalidParameterTypeException`。脚本把所有数值统一补小数点并对非数字输入报错。
- **示教中止「原始里程计停更 0.52 s」**：Odin1 主机侧回调队列溢出
  （`Queue TF_ODOM/WIWC is full, dropping oldest data`），非雷达掉电；板端 CPU 调频改
  `performance` 后正常。不要为此放宽时效互锁。
- **示教规范**：回放只前进。末段"倒着蹭"进终点 → 回放抵达时朝向差 176°、必须原地掉头。
  正确做法：**末段一路向前开过起点 0.3–0.5 m 再停**。

## 5. 验收数据（新地图，同一台车）

| 项 | 21:08 圈（倒车示教 315 点/14.18 m） | **21:16 圈（重示教 303 点/14.99 m）** |
| :--- | :--- | :--- |
| 结论 | 成功（末尾掉头 176°） | **成功**，终点朝向误差 0.01 rad |
| 总用时 | 106.9 s | **110.2 s**（回起点 26.4 + 跟踪 83.8） |
| 行驶 | 15.36 m | 14.65 m |
| 横向误差 | max 0.386 / rms 0.133 m | max 0.269 / **rms 0.111 m** |
| 急弯停车原地对齐 | 2 次 | **0 次** |
| 里程计 | 接受 1150 / 拒绝 0 | 接受 1184 / **拒绝 0** |

## 6. 演示命令（照抄）

```bash
bash ~/car_slam_lap/x5/scripts/start_mapping.sh demo_YYYYmmdd   # 遥控开 5–10 min
bash ~/car_slam_lap/x5/scripts/save_map.sh
bash ~/car_slam_lap/x5/scripts/stop_mapping.sh
bash ~/car_slam_lap/x5/scripts/start_relocal.sh ~/car_ws/maps/demo_YYYYmmdd
pgrep -f "lib/car_bridge/bridge_node$" | xargs -r kill          # 示教前停桥接
bash ~/car_slam_lap/x5/scripts/teach_map_route.sh ~/car_ws/maps/demo_YYYYmmdd
ros2 run car_bridge bridge_node &
VMAX=0.15 W_MAX=0.35 RETURN_W_MAX=0.30 \
BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471 \
START_MAX_OFFSET_M=2.0 RETURN_TIMEOUT_S=150 MAX_TIME_S=320 \
bash ~/car_slam_lap/x5/scripts/drive_lap.sh ~/car_ws/maps/demo_YYYYmmdd
```

## 7. 遗留

- 7–8 m 处急弯横向误差 0.20–0.21 m（其余全程 ≤0.06 m）：可试 `v_max=0.12` 或放大 `w_max` 再验。
- 建图质量/重定位成功率只有两次样本，演示前建议再完整彩排一次（含从头建图）。
- 路线与地图 `sha256` 绑定：**建图 → 示教 → 回放必须同一场次**，换图后旧路线自动拒跑。