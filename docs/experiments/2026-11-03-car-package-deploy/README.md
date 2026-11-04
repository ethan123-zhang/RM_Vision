# 任务03新包正式部署与关机接续

日期：2026-11-03。对象为C8T6双驱练习车。用户同意继续部署准备；随后要求X5关机以改用移动电源，示教尚未开始。

## 当前状态

任务03新包已部署到 `~/car_ws/src/car_bridge`、`~/car_ws/install/car_bridge` 和 `~/car_slam_lap/x5/` 对应包/脚本目录。正式安装入口与冻结代码逐模块哈希一致。没有修改地图、历史轨迹、现有配置、固件或另一台C板底盘，没有启动真实串口桥接或电驱运动。

本轮结束已执行sync及sudo shutdown，SSH主动断开；本机复查两次Ping超时、SSH TCP22连接超时。X5系统已离线，等待用户换移动电源后重新开机；这不代表动力电池或MCU物理断电。

## 发布物和备份

- 发布包：`_probe/car-deploy-20261103/release.zip`，由任务03冻结ZIP的代码/测试/脚本与必要的package.xml、setup.cfg、resource文件组成。冻结清单中所有条目在本机和板端逐一核验。
- 发布包SHA256：`426c0872e58c8815ea402971429c876b75c8b9beb446f3e95a956eebb2a2c0cb`。
- 正式部署目录：`/home/sunrise/car-deployments/route03-20261103-P93p63/`；指针 `~/car-deployments/current-route03-deployment.txt`。
- 旧版本 `backup.tar` 包含原car_ws源码、build/install、config及car_slam_lap/x5；板端sha256sum校验通过。备份SHA256：`fccdac662cbb6db3cbb0efe4f6374566ff9f362fac6552e9bab115af93b4447f`。
- 回退入口：`bash /home/sunrise/car-deployments/route03-20261103-P93p63/rollback.sh`，执行前停止项目进程；保留被替换目录，再将旧文件恢复到原绝对路径，以保留symlink-install关系。回退后应在新shell重新source环境。
- 本地部署证据：`_probe/car-deploy-20261103/route03-20261103-P93p63/`，包含旧版本备份、回退脚本、构建/测试/拒绝检查/dry-run日志。
- 本地归档：`_probe/car-deploy-20261103/car-deploy-20261103-evidence.tar.gz`，本机SHA256 `82c624dae675572c3a58106556ef126ac9aa5cc92a66cacad4d7144c26540563`。

首次尝试的部署检查脚本错误地把预期退出码1触发为ERR回滚，已实际恢复旧目录；首次目录为 `route03-20261103-Dj0dKY`。已将预期失败命令放入if/else捕获状态，第二次正式部署及全部检查成功。该错误属于部署检查脚本，不是录制器故障，不删除或覆盖首次证据。

## 【输入输出定义】

输入：已独立验收的任务03冻结包、ROS打包元数据、当前旧版本。输出：正式安装包、可回退备份、安装后实际导入哈希与无串口验证证据。桥接参数核对为 `/dev/ttyS1`、115200、50Hz、cmd_timeout=0.30秒、status_timeout=0.50秒、mps_at_max=0.8、radps_at_max=2.0、invert_speed=false、invert_angular=true；速度比例为待实测确认的默认配置，不能宣称已标定。

## 【核心算法逻辑伪代码】

确认无控制进程与串口占用 → 校验冻结条目 → 暂存测试 → 备份原源码/构建/安装/脚本/配置 → 替换包与脚本 → colcon build → 重新source正式环境 → 逐个导入并核验哈希 → 隔离ROS域无定位拒绝检查 → dry-run不打开串口 → 保存证据；非预期失败才回退。

## 【异常与边界条件】

安装脚本未整体运行，避免依赖升级、udev重置和默认配置覆盖；本次只部署冻结软件和必要元数据。dry-run无上行状态是预期现象，不能解释成真实串口链路失败。尚未做新安装包的ROS真实串口零指令、速度/转向实测标定、真实map路线示教或自动回放。

## 【验收测试用例】

本轮通过：108项unittest、轨迹10/10、协议11/11、全部shell脚本bash语法、colcon构建、正式安装模块哈希、8个ROS入口；ROS_DOMAIN_ID=97且无传感器/桥接时，严格录制器30秒缺定位退出1、未生成manifest；桥接dry_run=true运行后结束且串口始终未打开。

## 开机后接续

用户先改用移动电源并开机，保持急停；重新确认IP、进程和串口占用以及正式部署指针。动力/USB供电切换后重新确认Odin1识别和旧图重定位。先准备新的独立输出路径与严格map录制，待明确告知“录制已就绪”后用户才用DT7示教，不能提前开车。录制不启动串口桥接，以免抢遥控。自动回放前仍需ROS真实串口检查与速度/转向标定；当前没有真实新map路线。
