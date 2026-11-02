# Windows USB 相机帧率测试（`cam_fps.py`）

在 Windows 上实测 UVC 相机的**真实**帧率、协商格式与丢帧/抖动，而不是相信
驱动报出来的 `CAP_PROP_FPS`。

## 依赖

Python 3.8+ 与 `opencv-python`，不需要其他包。设备名枚举用 `ctypes` 直接调用
DirectShow 的 `ICreateDevEnum`，与 OpenCV DSHOW 后端同源，所以索引一致。

## 用法

```powershell
# 1) 列出设备（索引 / 名字 / 能否打开 / 默认格式）
python tools\cam_fps\cam_fps.py --mode list

# 2) 扫描：分辨率 × 帧率请求 × 像素格式，标出驱动是否真的接受了请求
python tools\cam_fps\cam_fps.py --device ZSKJ --mode scan --exact `
    --fourcc MJPG --fps 60 --seconds 4

# 3) 单配置长测：min/median/p95/max 帧间隔，可导出每帧间隔 CSV
python tools\cam_fps\cam_fps.py --device ZSKJ --mode single `
    --width 640 --height 480 --fps 30 --fourcc MJPG --seconds 10 `
    --csv intervals.csv

# 4) 带预览窗口（板子接显示器或在桌面运行；按 q 退出）
python tools\cam_fps\cam_fps.py --camera 0 --mode single `
    --width 1280 --height 720 --fps 30 --show
```

`--device` 用设备名片段选相机（如 `ZSKJ`），`--camera` 用索引；两者都不给时用索引 0。

## 测量口径

- **不看 `CAP_PROP_FPS` 下结论**：驱动常回 0 或标称值。真实帧率用每帧到达的
  `perf_counter` 时间戳算：`fps = (n-1)/(t_last - t_first)`，先丢弃 `--warmup` 帧。
- 同时报 `read ms`（单次 `read()` 耗时）和帧间隔分位数：读帧时间 ≈ 帧间隔说明相机
  真的只有那么快；读帧时间远小于帧间隔说明在等相机，反之说明解码/拷贝是瓶颈。
- `dups`：连续两帧稀疏采样签名相同（疑似驱动重复发同一缓冲）。签名取全分辨率
  稀疏像素，避免 8×8 缩略图把传感器噪声平均掉、把静态场景误判成重复帧。
- `exact`：驱动回读的分辨率是否等于请求。**NO = 该分辨率不被支持**，此时测到的
  帧率属于别的模式，不能当这个分辨率的结果。
- `--backend auto`（默认）先试 MSMF 再试 DSHOW。两者行为可能完全不同，见下。

## Windows 实测踩到的坑

1. **DSHOW 会把 MJPG 请求换成 YUY2**（回读 `neg4=YUY2`），高分辨率因此被 USB 带宽
   压到 6～8 fps；同一模式用 MSMF 才是 MJPG 的真实档位（例如 800×600：8.3 → 20 fps）。
   测帧率优先 `--backend msmf` / `auto`。
2. **驱动拒绝的分辨率会返回"伪流"**：DSHOW 在 SetFormat 失败后仍能快速读出
   113～187 fps 的帧，但内容与真实场景不一致（1440p/1080p 请求实测出 113 fps、
   30 fps 请求实测出 187 fps），必须用 `--exact` 过滤掉。
3. 打开失败常见于相机被其他程序占用；脚本会重试 3 次再报 `open failed`。
4. 快速反复开关同一台 UVC 相机偶尔会让驱动卡住一瞬，脚本重试可覆盖；持续失败就
   拔插一次 USB。

## 这台机器上的 ZSKJ-Camera 实测结果

直接取自 `--exact` 扫描（MJPG，MSMF）：相机只支持 640×480 及以下 30 fps，
更高分辨率按固定像素率降帧。完整数据与命令见
`docs/experiments/2026-11-02-zskj-camera-fps/README.md`。