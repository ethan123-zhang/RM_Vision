# car_slam_lap 实施交接：安全基础与回起点已完成软件验收

日期：2026-11-02。用户要求立即停止，2026-11-03继续。本文件优先于同日早先的“仅讨论未实施”交接。

## 目标与分工

实体练习车：C8T6双驱 + RDK X5 + Odin1 + DT7。遥控示教后，下次通过旧地图定位，先回示教首点并对齐，再完整执行原路线。第一版限起点附近空旷通道，不做任意地点绕障返回。Codex负责契约/指挥/审查/独立验收；DeepSeek Harness负责编码。

X5 SSH：sunrise@10.77.156.239。认证信息不写入交接。用户已确认同一venue2场地、设备固定在车上且静止。**未确认架空、摇杆回中、急停保持**；用户在该确认问题处要求停止，因此不得直接续跑串口自检或运动。

## 当前准确状态

| 项目 | 状态与证据 |
| --- | --- |
| SSH/环境/版本/旧地图盘点 | 已完成。Ubuntu22.04.5/aarch64/ROS2 Humble。venue1/venue2仍在 |
| 静止传感器检查 | 已完成。Odin1能连接、加载旧图并报告重定位；无车轮运动 |
| 任务01：安全门/终点判定 | 已实现并返修；Codex独立47项unittest+轨迹10/10+协议11/11通过 |
| 任务01：实际ROS验证 | X5 /tmp隔离代码、ROS_DOMAIN_ID=93回放通过；56条原始odom、124条cmd、64条非零；停更后0.50653秒转零，末三条零，全部有限；串口桥接未启动 |
| 任务02：小范围返回首点与起始对齐 | 已实现，独立62项unittest通过；简化运动模型从(0.2,0.1)、yaw90度开始，依次return/align_start/run/align，27.75秒完成 |
| 任务03：地图路线绑定/严格map示教 | **已取消，未完成、未验收**。只新增test_route_bundle.py骨架，route_bundle.py尚不存在；其余任务03功能尚未落地 |
| 正式部署/烧写/串口自检/实车运动 | **本轮都没有执行**。板端正式安装包保持原版，仅/tmp下有隔离代码 |

重要：当前工作树有任务03未完成测试，它导入尚不存在的route_bundle，因此**直接全量unittest discover预期失败**。不能把任务02阶段62项通过误报为当前整树已通过。明天优先完成任务03，或先只运行已验收的两个测试文件。

## 本轮修复与新增

- follow_path.py：默认拒绝旧odom路线；严格frame和有限数；直接订阅原始里程计、sensor_data QoS；不让TF缓存替代新鲜度；错误数据锁存故障；真实终点距离/朝向联合判定；对齐漂离或超时失败；异常停车、保存、退出码。
- localization_guard.py：无ROS依赖的路线参考系与原始里程计检查。
- path_io.py：兼容性保留的require_frame/require_finite可选开关。
- 回起点：默认return_to_start=True，先return再align_start，均成功才run；首帧偏差超过0.5米拒绝；默认位置/朝向容差0.1米/0.1rad，返回限速0.08m/s、0.4rad/s，总超时30秒；回起点距离不计入路线进度。
- 新测试：test_follow_safety.py、test_localization_guard.py；test_path_io.py新增严格读取测试；test_route_bundle.py为未完成任务骨架。
- 上述返回参数是**软件初值，不是实车已验收范围**。

独立审查曾复现并退回两项缺陷：align时漂离2米仍成功，以及有效里程计后的NaN仍短时输出0.35m/s；均已修复并补回归。不要仅依据Harness“完成”报告。

## 板端关键事实

1. 实际导入：~/car_ws/install/car_bridge/lib/python3.10/site-packages/car_bridge/follow_path.py。它与car_ws/src对应文件字节一致；电脑代码与其逐行一致。~/car_slam_lap中另有旧副本，不要拿它覆盖较新版本。
2. venue2/path.yaml为odom、330点、15.66米。旧路线不能改标签为map，也不能直接当作跨启动路线。
3. 驱动发布odom->imu、odom->map。include/host_sdk_sample.h:1952-1985缓存重发odom->imu并更新stamp，因此TF“新鲜”可能是旧位姿。
4. 原始设备stamp约千秒，与主机Unix墙钟不同；新鲜度用主机monotonic记录有效接收时间，设备stamp仅判断推进。
5. 静止bag10.4765秒：odom107条，最大接收间隔0.1603176秒，无倒退/非法位姿；odom->map65次。64个配对样本中odom stamp减最近map stamp为0～0.097562秒。
6. 静止map<-imu约(-0.045,0.089,0.019)m、yaw179.13度。只有设备输出，未用地面标记证明精度。
7. 现有start_mapping会启动持续发零的bridge，可能抢DT7控制权；新定位示教入口必须只录制，不自动启动桥接。

## 保存位置

- 实施计划：docs/superpowers/plans/2026-11-02-car-route-execution.md。
- 所有任务卡：car_slam_lap/docs/tasks/2026-11-02-01-follow-safety.md、01-review-fixes.md、02-return-start.md、03-map-route-binding.md。
- 实验与可读证据：docs/experiments/2026-11-02-car-route-readiness/README.md及evidence/。
- 原始bag、板端源码/配置/日志：_probe/car-route-20261102/board/。
- 任务01冻结包：_probe/car-route-20261102/task01-staging.zip；X5 /tmp/car-route-task01/是其隔离副本。
- 完整收尾快照与文件SHA256：_probe/car-route-20261102/closeout/。
- ROS隔离验证脚本：_probe/car-route-20261102/verify_ros_isolation.py。它使用独立ROS域，不启动串口。

## 明天从这里继续

1. 先读本交接、实验记录及任务03。检查git diff，保留其他任务和用户原有改动，不整库暂存。
2. 恢复DeepSeek可见会话，完成任务03。当前test_route_bundle.py就是未完成测试，不要误当既有回归故障。
3. Codex检查实际diff、补独立失败场景，跑全量单测/原有轨迹和协议回归。bash脚本在X5先bash -n；新代码先/tmp隔离验证，避免覆盖正式包。
4. 任务03完成后再准备定位模式的新示教路线：稳定地图、严格map记录、不回退odom、地图/路线内容哈希绑定。不能自动从静止样本生成真实路线；需要用户现场DT7示教。
5. 正式运动前，重新确认架空/急停/控制权、零指令上下行与watchdog、控制器ACK、速度和转向标定，以及Odin1与车心外参。冻结验收阈值再测。
6. 跟踪器180度掉头切角、复杂交叉、防跳段和累计里程限制的实用性仍需改进与实测，不能承诺已解决。历史终点误差0.48米不算新方案达标。

## Harness接续注意

会话ID：session-7b2475c2-66fd-4098-b303-9d47724acad6。前三轮分别为任务01、返修、任务02；第四轮任务03已按用户要求取消。会话持久化位置：C:/Users/hp/.deep-seek-harness-mcp/services/b9ddd702ef1b6aa7438d32bd/sessions/--E-RM-car_slam_lap--/。

本轮MCP插件0.3.1默认旧运行时无法直接适配本机新版。已完成的恢复办法保存在_probe/car-route-20261102/harness-client.mjs、npx-preload.mjs、harness-web-client.mjs：

- 使用本机已装@deepseek-ai/dsh 0.1.5-rc.1，并保持workspace-write、--no-open。
- doctor已通过；API凭据仅进程内读取既有凭据，不打印、不写任务卡。
- 新Web URL含token，先GET根URL换cookie；原生POST /api/workspace/create、session/create、session/prompt等，payload={args:{request:...}}。旧点号方法/旧裸payload不兼容。
- 原生API已成功提交并跟踪同一个可见会话；用session/page读取有限快照，避免把完整推理流输出到对话。
- 明天启动后URL/端口/token会变化，不要依赖旧链接。复用持久化会话前先确认没有运行中的任务。
- 曾因清理测试目录请求danger-full-access，Codex取消，未授予该权限；43个本轮生成的空目录已由Codex在校验路径后非递归清理。另有probe_pc_e3vvh诊断目录留待核实，不要扩大权限或误删用户数据。

## 【输入输出定义】

目标输入：同图的map示教路线、持续原始位姿、有效地图变换、底盘状态。输出：受限速度、任务结果、实际轨迹和独立验收证据。当前地图路线配套机制尚未完成。

## 【核心算法逻辑伪代码】

严格校验地图路线 -> 原始里程计与地图定位就绪 -> 首点范围检查 -> 返回首点 -> 起始朝向对齐 -> 从首段跟踪 -> 进度/距离/朝向联合完成；异常停车。任务03正在补第一段。

## 【异常与边界条件】

用户已要求停止，不再执行串口或运动。静止输出可用不等于定位精度通过；软件/模型测试通过不等于实车通过；未完成测试骨架不等于模块已实现。未提交推送、未部署正式包、未烧写。

## 【验收测试用例】

已验收：安全47项、加入回起点后62项、轨迹10/10、协议11/11；任务01真实ROS隔离停更0.50653秒转零。明天补：bundle篡改/错图/错frame、严格map示教、陈旧地图TF、完整ROS集成、地面标记精度和分级实车测试。

明天可直接说：继续 E:\RM\car_slam_lap，先读 docs/handoffs/2026-11-02-car-slam-lap-execution.md，从被取消的任务03地图路线绑定继续，仍由DeepSeek编码、Codex独立验收，实车动作先确认现场条件。
