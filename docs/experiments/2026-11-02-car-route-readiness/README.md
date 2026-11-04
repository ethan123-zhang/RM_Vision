# 小车路线复现：板端盘点与静止定位检查

日期：2026-11-02。用户已授权开始实施，并确认设备固定在小车上、同一 venue2 场地、车身静止。本轮没有启动车轮。

## 已核实结果

- X5 已通过 SSH 登录；Ubuntu 22.04.5，aarch64，内核6.1.83，ROS2 Humble。
- 存在 ~/car_slam_lap、~/car_ws、~/odin_ws。初次盘点时没有 Odin1、bridge_node 或 follow_path 进程。
- venue1 和 venue2 地图仍在。venue2 专有地图153671196字节，路线330点、15.66米，frame_id=odom。
- 实际 Python 模块从 ~/car_ws/install/car_bridge/lib/python3.10/site-packages/car_bridge/follow_path.py 导入；它与 ~/car_ws/src 中版本字节哈希一致。
- 下载比较发现：电脑跟踪器与板端 car_ws 源文件逐行相同（字节哈希差异来自换行）；板端 ~/car_slam_lap 中另有旧版本，保留不覆盖。
- 板端原有测试：test_path_io.py 8/8、test_protocol.py 11/11。电脑协议11/11、安装到隔离目录的 PyYAML 后轨迹8/8。

## 驱动证据与判断

板端 include/host_sdk_sample.h：

- 1286-1287：原始 odometry 的 frame=odom、child=imu。
- 1378-1394：标准里程计更新 odom->imu 并缓存位姿和原始时间。
- 1554-1568：重定位变换发布 odom->map。
- 1952-1985：缓存重发线程按 tf_extra_publish_rate（当前50Hz）更新 TF stamp，即使没有新的原始位姿，也可能继续生成新时间戳。
- 因此不能用“TF查得到且stamp新”作为动态定位停更的唯一判断。必须单独检查原始里程计的到达及其时间推进；这仍不等于已经有可信的定位质量评分。

host_sdk_sample.cpp:1234-1245：重定位变换回调首次触发时打印 relocalization success，该日志证明收到设备相应输出，不证明空间精度达标。

## 静止探测

通过有界 timeout 启动 Odin1 驱动，未启动桥接或跟踪器。沿用板端现有重定位配置，没有改控制代码和配置文件。

1. 首次45秒探测：设备USB3.2连接，加载旧地图，日志报告重定位成功，结束正常停止流。
2. 第二次50秒探测：原始 odometry 为 odom/imu；时间约1415秒，明显不是主机Unix墙钟。因此保护逻辑不能直接用设备stamp减主机墙钟。
3. 第二次 map<-imu 查询得到约(-0.045,0.089,0.019)m，yaw约179.13度；相邻样本有更新。用户确认同场地、固定、静止。缺少独立地面标记测量，不能据此宣称跨会话精度通过。
4. 探测结束确认 host_sdk_sample、bridge_node、follow_path 均未运行。
5. rosbag采集10.4765秒、4209条消息：odom107条、TF4102条。原始odom最大接收间隔0.1603176秒，无时间戳倒退/非法位姿；odom->map65次。64个配对样本的设备stamp差0～0.097562秒。见evidence/sensor_stats.json、map_stamp_stats.json。

原始驱动源码快照、配置和日志保存在 E:/RM/_probe/car-route-20261102/board；该目录按仓库规则不入库。可提交的精简日志和文件哈希放本实验目录。

## 实施状态

实施计划：docs/superpowers/plans/2026-11-02-car-route-execution.md。

第一张任务卡及返修已由Codex独立验收：47项unittest、轨迹10/10、协议11/11。任务02回起点与起始对齐完成，累计62项unittest独立通过；另有简化运动模型27.75秒完成的验证。

任务01冻结包在X5 /tmp下以ROS_DOMAIN_ID=93回放真实bag：56条odom、124条cmd_vel、64条非零，停更后0.50653秒转零，末三条零、全部有限，follower按预期故障退出1。串口桥接未启动；验证程序exit0。见evidence/task01-ros-isolation.json与task01-ros-follow.log。

任务03地图路线绑定已被用户要求停止而取消：当前仅test_route_bundle.py骨架，模块尚不存在；当前整树全量测试预期会因此失败，不能沿用任务02阶段通过结论。未部署正式包、未烧写、未串口自检或实车运动。完整接续见docs/handoffs/2026-11-02-car-slam-lap-execution.md。

## 【输入输出定义】

输入是现有地图/轨迹、板端实际源码、原始里程计和 TF。输出是版本清单、定位数据基线、可验证的修复任务。当前无电机运动输出。

## 【核心算法逻辑伪代码】

核对实际导入路径 -> 比较版本 -> 检查驱动变换与时间源 -> 有界静止采样 -> 保存原始证据 -> 委派最小修复 -> 独立验收。

## 【异常与边界条件】

主机时间与设备时间分开；TF缓存重发不能续原始定位的命；旧odom路线不能仅修改标签；地图加载成功不代表定位精度达标；不根据源码推断硬件已通过故障停车验收。

## 【验收测试用例】

原有协议/轨迹回归；原始位姿停更但TF继续更新；重复时间戳；静止但时间推进；时间倒退；非法数值；路线参考系限制；进度、终点距离和朝向联合完成；异常退出收尾。
