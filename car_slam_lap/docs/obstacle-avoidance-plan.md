# 实时障碍物绕行实施计划

> 执行方式：本次聊天直接实施，使用 executing-plans / TDD / verification-before-completion；沿用 E:\RM 当前检出，分支 codex/car-local-obstacle-avoidance。用户已明确要求开始实施，不重复请求方案确认。

**目标：** 保留回起点和示教路线回放，为后来摆放的障碍增加检测、局部绕行与停车保护。
**设计：** [obstacle-avoidance-design.md](obstacle-avoidance-design.md)
**技术：** ROS 2 Humble、Python 3、NumPy、tf2、Odin1 PointCloud2。

## 任务 1：观测网格与规划（纯计算）

- [x] 新增 `test/test_obstacle_grid.py`，测试射线清除、障碍持久化、车身尺寸、可绕/堵死/未知、折线路径碰撞。
- [x] 执行网格行为回归并确认通过。
- [x] 新增 `car_bridge/obstacle_grid.py`，接口 `LocalGrid.observe(points, origin, robot_center, now, clear_occupied=True, min_points=1)`；通过 `grid.view(center, now)` 获得 `plan(start, goal)`、`segment_free(a,b)`、`motion_free(pose,v,w)`。已检测占据保守保留，未知空间不能用于绕行。
- [x] 重跑同一测试并验证单障碍仿真到达目标。

## 任务 2：节点接入与状态保持

- [x] 新增发布速度行为测试：原始点云未就绪/过期必停；回起点与旋转也受保护；绕行实际接回才推进路线进度。
- [x] 运行新增行为回归，确认实际发布速度与任务状态符合安全判据。
- [x] 新增 `car_bridge/obstacle_avoidance.py`；消息时间 TF 变换、地面/车身过滤、有限点数与窗口、规划时间限额、失败关闭。
- [x] 在 `follow_path.py` 添加可选控制器：默认关闭保持旧行为；避障开启时所有非零发布必须经过保护，临时路径在原安全门后执行。
- [x] 补充原地左右观察与超时、碰撞圆占据拒转、扫描中点云失效停车的真实发布回归。
- [x] 保存实际轨迹附属 `.yaml.obstacles.npz` 完整诊断快照，验证无 pickle 读取、无网格跳过和导出失败保留 YAML。
- [x] 运行原有全部回归与新增集成仿真；当前 215 项单元测试全部通过。

## 任务 3：启动、部署、板端证据

- [x] 新增独立 `config/obstacle_avoidance.yaml`、`start_relocal_obstacles.sh`、`drive_lap_obstacles.sh`；旧脚本仅添加可选参数文件入口。
- [x] 补充 NumPy/点云工具依赖，提供不发布运动指令的点云诊断入口。
- [x] 在 X5 备份源代码和驱动配置；上传精确文件并构建 car_bridge，构建通过。
- [x] 真实点云静止检查：帧名、频率、有效点、高度带、可见扇区、处理耗时、里程计连续性；保存证据。
- [x] 更新操作说明，明确实车绕行验收状态、尺寸与盲区限制、关闭避障的原流程。
- [x] 执行软件回归与部署构建检查；仅在验证支持的范围内报告完成。
- [ ] 完成真实整圈绕行验收：箱体前产生实际局部路径、横向绕行、实际接回并满足最终成功判据。

小角度标定和低速实车试验均在用户授权后执行。当前 run2、run3 回起点与起始对齐成功，
发现箱体后 `local_plans=0`、`rejoins=0` 并安全失败停车，尚未通过绕行验收。
run4 仅增加完整快照、沿用同算法重现，正在采样，结果未定。
