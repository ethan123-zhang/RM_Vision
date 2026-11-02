# ZSKJ-Camera 帧率实测（2026-11-02）

> 一句话结论：这台 ZSKJ-Camera 在 MJPG 下**只有 640×480 / 640×360 能跑 30 fps**，
> 800×600 为 20 fps，1024×768 与 1280×720 为 10 fps，1280×1024 为 5 fps；
> **1920×1080 及以上不支持**（驱动回退到更低模式）。YUYV 模式除 640×480 外都掉到
> 6.7～8.3 fps。工具：`tools/cam_fps/cam_fps.py`。

## 环境指纹

| 项 | 值 |
| :--- | :--- |
| 相机 | ZSKJ-Camera，`USB\VID_1BCA&PID_28C5&MI_00`，UVC（usbvideo.inf） |
| 主机 | Windows 笔记本（HP OMEN，另有内置 HP True Vision FHD Camera） |
| Python | 3.10.1 |
| OpenCV | 4.13.0 |
| 后端 | Media Foundation（`--backend msmf`）与 DirectShow（对照） |
| 测量 | 每模式 4 s、丢弃前 10 帧、按帧到达时间戳计算 |

## 方法

1. `--mode list`：用 ctypes 调 DirectShow `ICreateDevEnum` 枚举设备名，确认
   **索引 0 = ZSKJ-Camera**（索引 1 = HP True Vision，2/3 为虚拟相机）。
2. `--mode scan --exact`：对每个分辨率请求 MJPG（再单独跑 YUYV），回读驱动协商的
   分辨率/帧率，并用时间戳实测真实帧率；`exact=NO` 表示驱动没有接受该分辨率。
3. 交叉验证：同一模式用 DSHOW 复测，并对 1080p 请求下 DSHOW 报出的 113 fps 做
   画面相关性检查（与 MSMF 同模式画面相关性 −0.25，确认是伪流而非真实画面）。

## 结果：MJPG（MSMF，实测）

| 分辨率 | 驱动是否支持 | 协商帧率 | 实测帧率 | 单帧 read |
| :--- | :---: | ---: | ---: | ---: |
| 1280×1024 | ✅ | 5 | **4.9** | 204 ms |
| 1280×720 | ✅ | 10 | **9.8** | 102 ms |
| 1024×768 | ✅ | 10 | **9.8** | 102 ms |
| 800×600 | ✅ | 20 | **19.6** | 51 ms |
| 640×480 | ✅ | 30 | **30.0** | 33 ms |
| 640×360 | ✅ | 30 | **30.0** | 33 ms |
| 1920×1080 | ❌ 回退 1280×1024@5 | — | — | — |
| 320×240 | ❌ 回退 640×360@30 | — | — | — |
| 1280×960 / 1280×800 / 1600×1200 / 2048×1536 / 2560×1440 / 2592×1944 | ❌ 回退 | — | — | — |

## 结果：YUYV（MSMF，实测）

| 分辨率 | 实测帧率 | 注 |
| :--- | ---: | :--- |
| 1280×1024 | 6.7 | 高于 MJPG 的 5，但带宽受限 |
| 1280×720 | 8.3 | |
| 1024×768 | 8.3 | |
| 800×600 | 8.3 | |
| 640×480 | 30.3 | 与 MJPG 相同 |
| 640×360 | 30.2 | |

## 关键发现

1. **DSHOW 后端不可信**：`cap.set(FOURCC, MJPG)` 后回读是 `YUY2`，高分辨率被
   USB 带宽压到 6.6～8.3 fps；同分辨率 MSMF 走 MJPG 才是真实档位（800×600：
   DSHOW 8.3 → MSMF 20）。测帧率默认用 MSMF。
2. **驱动拒绝的分辨率会给出伪帧率**：1080p 请求在 DSHOW 下报 113 fps、320×240
   请求报 187 fps，但协商值显示已回退，且画面与同模式 MSMF 画面不相关 → 假数据。
   扫描必须用 `--exact` 过滤，否则会把回退模式的帧率错误标到请求分辨率上。
3. **相机是固定像素率档位**：约 9～10 M 像素/秒（640×480@30、800×600@20、
   1024×768@10、1280×720@10、1280×1024@5），不是驱动限流。
4. 1080p 不可用；若下游需要 1080p 或 60 fps，这颗相机不满足，需换设备。

## 复现命令

```powershell
python tools\cam_fps\cam_fps.py --mode list
python tools\cam_fps\cam_fps.py --device ZSKJ --mode scan --exact `
    --fourcc MJPG --fps 60 --seconds 4
python tools\cam_fps\cam_fps.py --device ZSKJ --mode scan --exact `
    --backend msmf --fourcc YUYV --fps 30 --seconds 4
python tools\cam_fps\cam_fps.py --device ZSKJ --mode single `
    --width 640 --height 480 --fps 30 --fourcc MJPG --seconds 10 --csv intervals.csv
```

## 可复用结论

Windows 上用 OpenCV 测 UVC 帧率：优先 MSMF；必须回读协商分辨率并只统计
`exact` 组合；DSHOW 的 FOURCC 可能被换成 YUY2，且 SetFormat 失败后会输出伪流
帧率。这是跨项目可复用的通用坑，后续可沉淀到 aiVault（嵌入式/边缘推理）。