# PROGRESS

## 现在进行时（2026-11-06 16:10）

Odin1 小车「建图→示教→自主巡线」演示的**第一轮全链路已跑通**：`venue3` 地图上自主跑圈连续
成功 2 次，横向误差 rms **0.138 m**、回起点+起始对齐 **9.0 s**、补位往返 **0 次**（关键修法是
补上 Odin1 相对转心的力臂 `BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471`，并把转速放进
0.5–0.7 rad/s 的可用窗口）。建图录像板上出片 1280×1036 / 3309 帧 / 330.9 s / 86.7 MB。

当前阻塞：**板端 RDK X5 离线**（11:15 起 SSH 超时，疑似断电）；磁盘只剩 **2.7 GB / 29 GB**；
`demo-1106` 的示教路线已按要求删除（旧文件备份在 `demo/demo-1106/route_map_旧示教_20261106.yaml`），
**重定位之后的示教与自主尚未做**；`x5/scripts/reset_run.sh` 未在板端验证。

离开前交付：三份现场文档（[指令单](docs/demo-command-sheet.md)、[明天全流程](docs/demo-day-fresh.md)、
[断了怎么重来](docs/reset-and-restart.md)）、[操作知识速查](docs/knowledge/odin1-demo-operations.md)
与完整进度总结（[handoff](docs/handoffs/2026-11-06-odin1-demo-closeout.md)）。

**下一步（按序）**

1. 板子上电后部署并校验 `reset_run.sh`（`bash -n`），再跑清盘：`maps/*` + `odin_ws/.../map/*`
   + 缓存，目标可用空间 ≥ 9 GB。
2. 用分钟级新目录重新建图（`VIDEO_STRIDE=2`，**存图前先停录像**），验收 `.bin` ≥ 200 MB。
3. 重定位 → 示教（停稳按一次 Ctrl-C，验收 `route_map.yaml` + `manifest.json`）→ 自主
   （参数见文档，先慢速走通再提速）。

---

## 历史节点

| 日期 | 事件 |
| :--- | :--- |
| 2026-11-04 | 力臂标定 `calibrate_lever_arm`：转 169.9°，传感器位移 0.531 m（理论 0.532 m），偏移 (+0.2626, −0.0471) |
| 2026-11-05 | 避障分支收尾（216 项本地测试全绿）；Odin1 建图/录像/示教/自主链路首次打通；参数定版 |
| 2026-11-06 | `venue3` 自主跑圈 2 次成功（rms 0.138 m）；发现"录像与地图回传抢 I/O → 存图卡死"；`demo-1106` 重图完成；板端离线 |