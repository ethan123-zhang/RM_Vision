# RMUL2027 3V3 场地接入 Gazebo：比例/碰撞/台面高度/观感（2026-11-01）

会话工作目录 `E:\RM\tb3_gazebo_nav2\`，详细记录在
**[`tb3_gazebo_nav2/rm_field/README.md`](../../../tb3_gazebo_nav2/rm_field/README.md)**（§4 验收、§9 台面高度、§10 观感改造）。
本文只做“做了什么 / 数字是什么 / 怎么复跑 / 剩什么”的归档。

## 起点

用户把 RMUL2027 3V3 场地 CAD（`3D_CAD_Model/RMUL2027_3V3_V1.0.0.stp`，12.600 × 8.600 m、
台面 z=0、台下 0.503 m）导进 Gazebo 后**看不到“凸起的平台”**，对话框面像是平的。

第一轮结论：模型没漏做，是**渲染/视角**问题。第二轮补做观感改造后证实了这一点，
并顺带把用户指出的“没看到围栏”做成可选变体。

## 轮次 1：把 0.503 m 做成可测的数字

新增三台取证相机（`worlds/visual_capture.sdf`）+ `verify_height.py`（几何预测 ↔ 渲染实测）
+ `annotate_height.py`（标注图）。

| 相机 | 关键实测 | 判据 |
|---|---|---|
| 正立面全景 1920×1080 | 亮带 1734 × 69 px；墙高 **501.3 mm**（焦距换算）/ **494.1 mm**（12.6 m 宽度做标尺）；宽高比 25.50 vs 图纸 25.05 | 对 503 mm 误差 0.3% / 1.8% |
| 墙面近景 1280×720 | 墙高 **451 px = 画面 62.6%**；上下边缘误差 0.9 / 3.1 px | 半米肉眼可见 |
| 低机位斜视 1600×900 | 亮面 bbox 与“零厚度平板”预测差 ≤3.5 px；台面下缘再往下 **74 px 侧壁带**（预测 75.0 px） | 是 0.503 m 实体，不是贴图 |

11 条断言全部通过，报告 `rm_field/results/report_height.json`，图 `rm_field/shots/height_annotated_*.png`。

## 轮次 2：用户反馈“还是平面 + 小车”，做观感诊断

不猜、也不依赖 GUI 截图（WSLg 抓图不可靠，见
[issue 记录](../issues/2026-11-01-wslg-gui-visibility.md)），而是把 GUI 的相机位姿搬进离屏渲染，
逐行读亮度剖面。旧配色：台面 185.6 / 侧壁 124.7 / 地面 48.1 / 背景 39.6 —— 侧壁在画，
但**地面 ≈ 背景、投影 ≈ 天空**，台子读成“浮在黑洞上的卡片”。

四级原因（按贡献）：① 俯角 12° 使 0.503 m 只有 20~40 px；② 地面 ≈ 背景；③ **投影亮度 ≈ 天空**（主因）；
④ 单一材质、无竖直参照物。

改造（**只动颜色/光照/相机 + 新增围栏，场地几何未动**）：环境光 0.55→0.35、主光偏顶光、
加补光、地面 0.03→0.09 灰且平面 60→400 m、天空压暗、GUI 相机换到 +y 侧、
删掉世界里的 `<gui><camera><track>`（它会覆盖 `--gui-config` 的 camera_pose）、
新增 0.30 m 围挡（`models/rmul2027_3v3_fence`，visual + collision + 红顶沿）。

改造后亮度阶梯（同一条中心列，离屏 / GUI 实拍）：
天空 42/56、投影地面 88、亮地面 95/90、侧壁 117~123/117、围栏 137/137、台面 214/214。

几何线预测 ↔ 实测（`annotate_gui_view.py`）：围栏顶 −4.9 px、平台边 −1.0 px、地面 −3.7 px。

## 复跑

```bash
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field
bash capture_height.sh      # 三张取证图 + 台面高度验收（结论：全部通过）
python3 annotate_height.py  # 带 0.503 m 标注的证据图
bash capture_probe.sh gui_fenced_view.png worlds/gui_capture_fenced.sdf /gui_view/image
python3 probe_report.py shots/gui_fenced_view.png   # 六级亮度阶梯
python3 annotate_gui_view.py                        # GUI 视角标注 + 预测/实测
bash gui_final.sh                                   # GUI：带围挡世界（默认）
WORLD=$PWD/worlds/rmul2027_3v3_field.sdf bash gui_final.sh   # 1:1 无围挡世界
```

比例/碰撞验收不受本轮影响（只改颜色/光照）：`results/report_field_test.json`、`results/report_geometry.json` 仍有效；
`verify_height.py` 重跑数字不变。

## 遗留

1. 围栏碰撞未实测：需补“机器人 0.7 m/s 撞边不越界、贴边被挡住”的测试项，并把 `verify_field.py`
   边缘探针判据从“掉到地面”改成“被围栏挡住”（1:1 世界仍保留原判据）。
2. 围栏 0.30 m 是“TB3 爬不上去”的工程估计，赛规若有数值需按赛规改。
3. Nav2 地图（顶视渲染 → `map.pgm/yaml`）未开始。