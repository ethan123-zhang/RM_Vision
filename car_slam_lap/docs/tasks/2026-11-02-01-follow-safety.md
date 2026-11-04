# 任务 01：路线跟踪安全前置与终点判定

由 Codex 指挥、DeepSeek Harness 实现。全程简体中文。用户已授权开始实施；本任务仅本地编码，不连接硬件、不部署、不烧写、不提交或推送。

## 已核实事实

- 实体车为 C8T6 双驱、X5、Odin1；不要修改 cboard_chassis。
- 板端 ROS2 Humble。原始里程计话题 `/odin1/odometry`，frame=odom，child=imu；QoS 为传感器数据配置。
- 厂商驱动 include/host_sdk_sample.h:1950-1985 会缓存并重发 odom->imu，更新 TF 时间戳。因此 TF 新鲜不代表原始位姿新鲜。
- 现有 follow_path 的 `_pose()` 只检查 TF 查询成功；终点主要根据剩余弧长判定；对齐超时也写成走完。
- demo/venue2/path.yaml 为历史 odom 路线，不能改名为 map；保持原始示教数据。
- Codex 本轮先完成版本/坐标审计；跨重启空间一致性、回起点及硬件运动仍待验收，不能宣称这些能力完成。

## 允许范围

仅允许修改或新增：
- x5/car_bridge/car_bridge/follow_path.py
- x5/car_bridge/car_bridge/localization_guard.py（需要时新增，不依赖 ROS）
- x5/car_bridge/test/test_follow_safety.py（新增）
- x5/car_bridge/test/test_localization_guard.py（需要时新增）

其他文件不得修改。先读现有代码，保留已有启动宽限期、SIGINT、实际轨迹保存修复。不要无关重构。新测试用 unittest，支持直接运行和 unittest discover。先写能复现缺陷的测试再修复。使用本机 Python：E:/AI_Agent/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe；若缺 PyYAML，不要自行安装或连外网，在交付中注明，Codex 会安排验证环境。

## 【输入输出定义】

1. 新增 follow_path 参数：odom_topic=/odin1/odometry、odom_timeout_s=0.5、allow_legacy_odom=False。
2. 默认仅接受 frame_id=map 的路线。历史 odom 路线只有显式 allow_legacy_odom=True 才可运行，并警告仅供已人工对齐的历史诊断，不代表跨会话安全。不接受空 frame、任意未知 frame。
3. 原始 Odometry 订阅必须用 qos_profile_sensor_data，兼容 best effort 发布者。
4. 记录原始 header.stamp 的严格递增及接收 monotonic 时间：首次非零 stamp 可建立基线；重复、倒退、零 stamp 不得刷新有效更新时间；超时立即零指令并故障退出。数据时间倒退可直接故障，不自动恢复运动。收到数据必须有限，四元数范数非零，frame/child 与约定相符。
5. 启动宽限期继续仅发零。宽限结束前没有合格原始里程计时，禁止发非零命令并报告原因。
6. 保留 TF 查询用于地图坐标转换；原始里程计检查与 TF 可用检查都必须通过。
7. 完成必须同时满足路线进度接近末段、当前位置到最后点的欧氏距离 <= goal_tol，以及启用朝向对齐时角度误差 <= 0.10 rad。闭合路线起点不能直接认定完成。
8. 最终对齐超时标记失败，不能写“走完一圈”。成功/故障通过 summary 明确区分；main 正常完成 exit 0，运行故障 exit 非0，用户 SIGINT 可为130。异常退出应尽力发零并保存已有实际轨迹；不得掩盖原异常。

## 【核心算法逻辑伪代码】

```text
加载并检查路线参考系，未授权的 odom 拒绝
订阅原始里程计，校验帧名/数值/时间推进
tick:
    启动宽限 -> 零指令
    底盘异常、原始里程计未就绪或停更 -> 零指令/失败
    地图 TF 不可用 -> 零指令，按现有 TF 超时失败
    原有纯追踪
    进度到末段且实际终点距离达标 -> 最终朝向对齐
    对齐达标 -> 成功；超时 -> 失败
finally: 尽力停车并保存实际轨迹
```

## 【异常与边界条件】

- TF 重发且新鲜、但原始 odometry 停更：必须停车。
- 原始消息继续收到但 stamp 固定：不能续命。
- 车辆静止但 stamp 正常推进：有效，不得把位置不变误判成停更。
- 全部时间差使用 monotonic；不将原始设备时钟与主机墙钟直接相减。
- NaN/Inf、非法姿态、错 frame、错 child：拒绝，禁止生成非零指令。
- 不添加自动绕障、地图格式转换、回起点或未经验证的定位置信度。

## 【验收测试用例】

必须覆盖：默认拒绝历史 odom；显式 legacy 可加载并警告；地图路线可加载；未知 frame 拒绝；启动宽限仅零；没有 odometry 零输出；TF 正常但 odometry 停更零输出；重复 stamp 不续命；正常静止更新通过；倒退、零 stamp、NaN/Inf 和错 frame/child 拒绝；终点索引到了但实际很远不能成功；闭合路线起点不能成功；终点位置和朝向都满足才成功；对齐超时明确失败；异常/SIGINT 收尾保留停车与保存行为。

无需安装 ROS，可用最小测试替身加载 follow_path，但必须执行实际 tick/回调逻辑断言输出，不接受只搜索源码字符串的测试。不要过度模拟整个 ROS。

交付列出修改文件、设计取舍、测试命令及真实输出、未验证项。不要把本任务说成整车导航完成。
