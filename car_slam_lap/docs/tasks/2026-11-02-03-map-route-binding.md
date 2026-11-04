# 任务03：定位模式重新示教与地图路线绑定

本任务在任务02通过Codex独立验收后执行。只本地编码和离线测试，不SSH、不部署、不操作硬件、不提交推送、不扩大权限。保留历史地图和示教数据。

## 允许修改

- 新增 x5/car_bridge/car_bridge/route_bundle.py、test/test_route_bundle.py、test/test_record_map_route.py。
- 修改 x5/car_bridge/car_bridge/record_path.py、follow_path.py、test/test_follow_safety.py、setup.py。
- 新增 x5/scripts/teach_map_route.sh；修改 x5/scripts/drive_lap.sh。
- README.md 仅追加新流程说明，保留用户已有入口和历史记录。

## 已核实事实

同场地固定静止采集10.476秒：原始odometry107条，最大接收间隔0.1603秒；odom->map65条。64个可配对原始样本的 device stamp 减最近map变换stamp为0～0.097562秒；两者使用同一设备时间域。必须与主机monotonic分开。

## 【输入输出定义】

1. route_bundle.py为不依赖ROS的模块：
   - file_sha256(path)：流式计算文件SHA256。
   - write_bundle(path_file,map_file,base_frame='imu')：验证显式map轨迹、有限数、至少3点且路线有非零长度，写 path_file+'.manifest.json'，返回manifest路径。
   - validate_bundle(path_file,map_file,base_frame='imu')：校验manifest及当前地图/路线，返回metadata，失败ValueError且原因清楚。
   - metadata包括schema_version=1、map_sha256、path_sha256、frame_id='map'、base_frame、point_count、start={x,y,yaw}。以文件内容识别地图，文件换目录不必失败；修改任何地图或路线字节必须失败。JSON原子写。metadata只证明配套，不声称空间精度已验收。
   - 可提供 python -m car_bridge.route_bundle validate --path ... --map ... 的只读检查入口，不提供把旧odom改成map的转换。
2. follow_path增加 map_file=''、require_map_bundle=True。默认map模式必须有地图文件及有效bundle，base_frame必须匹配。require_map_bundle=False只用于隔离诊断并明确警告；历史odom诊断仍遵循allow_legacy_odom显式许可和任务02禁止自动返回的限制。测试旧helper可明确require_map_bundle=False，但必须新增默认严格行为测试。
3. record_path增加map_file=''。原来的无map_file建图录制保留兼容。提供map_file时进入严格定位示教模式：world_frame必须map、fallback_frame必须空、use_odom_pose必须False，不能退回odom。
4. 严格模式使用任务01OdomGuard和sensor_data QoS；校验原始数据。查询 map<-base_frame 与 odom<-map，检查后者stamp与原始msg stamp的绝对差<=0.5秒，避免陈旧地图变换被拼接成看似有效定位。地图TF未就绪时只等待，不录制，不提供bundle；收到过有效定位后，定位/原始数据故障时中止录制、返回非0，保留诊断轨迹但不得留下可供正式运行的bundle。未就绪等待最多30秒。
5. 只有正常结束、没有定位故障且轨迹足够时写bundle；autosave可以保存诊断轨迹，但不要在运行中提前认证最终bundle。计算地图hash不要每个callback重复执行。
6. teach_map_route.sh只启动严格录制器，绝不启动bridge/follow/mapping。用户先手动启动旧地图重定位，再用DT7示教；如果桥接正在发指令，拒绝录制并说明控制权问题，不自动杀其他进程。默认输出map目录下route_map.yaml；若输出或manifest已存在，拒绝覆盖。默认base_frame=imu，与当前设备配置一致；不要编造静态TF。
7. 脚本选图沿用map_path.txt或唯一.bin（多个候选时失败，不随意head -1）。核对厂商实际配置文件custom_map_mode=2、relocalization_map_abs_path指向的地图内容与选中地图相同；本检查不等于证明定位精度。
8. drive_lap.sh默认使用route_map.yaml，显式传入其他路径也必须校验bundle；将map_file传给follow。不得默认放行旧path.yaml。保留手工起bridge的操作方式，不自动启动底盘。启动提示写清“先回起点再跟踪；限起点附近空旷通道，无自动绕障”。

## 【核心算法逻辑伪代码】

```text
地图已稳定并启动重定位
检查选中地图与驱动配置，确认无自动发送者抢DT7控制
严格原始里程计/地图TF都有效后才开始map坐标录制
正常结束 -> 保存路线 -> 配套内容哈希与首点metadata
自动执行前验证内容、frame和base -> 任务02回起点状态机
```

## 【异常与边界条件】

缺map、旧odom、NaN/Inf、空路线、少于3点、零长度、改图改路线、未知schema、base_frame不一致均拒绝。不得自动重写历史路线或移动坐标原点。位置静止但时间更新仍是正常数据；原地采集不能产出可执行的零长度路线。用户Ctrl-C正常结束示教，故障中止不能留下“有效bundle”。脚本不得把查得到TF当成唯一定位成功依据。

## 【验收测试用例】

bundle正常往返/跨目录；改地图、改路线、错base、错schema、缺manifest、旧odom、缺frame、非法值、零长度拒绝；record严格模式不回退；mapTF未就绪不录点；原始数据和mapTF陈旧时退出且不出有效bundle；正常map示教的点与首点一致；生产默认拒绝无bundle；显式隔离诊断有警告；旧47项、任务02全部回归、path_io与protocol继续通过。脚本至少bash -n（本机没有bash则报告，Codex在板端验证）。

先失败测试后实现。交付范围内改动、实际验证、限制与后续实车步骤。不要将软件测试通过说成物理定位精度或整车导航通过。
