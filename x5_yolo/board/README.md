# RDK X5 单类 person YOLOv8n 快速包（v8：576 定稿）

自包含，只依赖镜像自带的 `hobot_dnn` (pyeasy_dnn) / OpenCV / numpy。

## 文件

| 文件 | 说明 |
|---|---|
| `yolov8_person1.py` | 板端推理 + 分段测速（复用 canvas、单线程 OpenCV、2/3 段流水线、`--crop`、`--cpost`） |
| `postprocess.cpp` / `build_post.sh` | C++ DFL+sigmoid+NMS，ctypes 调用，无需 Python 头文件 |
| `yolov8n_person1_bayese_640x640_nv12.bin` | 640 输入，3.7 MB |
| `yolov8n_person1_512_bayese_512x512_nv12.bin` | 512 输入，3.4 MB，BPU 约 4.5 ms |
| `yolov8n_person1_576_bayese_576x576_nv12.bin` | 576 输入，3.5 MB，精度接近 640 |
| `person.names` | 单类标签 |
| `bus.jpg` | 测试图 |

## 640 基准

```bash
cd ~/rdk_model_zoo/yolov8_person1

python3 yolov8_person1.py --model yolov8n_person1_bayese_640x640_nv12.bin \
  --img bus.jpg --bench 200 --cv-threads 4

python3 yolov8_person1.py --model yolov8n_person1_bayese_640x640_nv12.bin \
  --img bus.jpg --bench 200 --pipeline --cv-threads 4

python3 yolov8_person1.py --model yolov8n_person1_bayese_640x640_nv12.bin \
  --img bus.jpg --bench 200 --crop --cv-threads 4
```

## 512 基准（冲 100+）

```bash
python3 yolov8_person1.py \
  --model yolov8n_person1_512_bayese_512x512_nv12.bin --size 512 \
  --img bus.jpg --bench 200 --cv-threads 4

python3 yolov8_person1.py \
  --model yolov8n_person1_512_bayese_512x512_nv12.bin --size 512 \
  --img bus.jpg --bench 200 --pipeline --cv-threads 4

python3 yolov8_person1.py \
  --model yolov8n_person1_512_bayese_512x512_nv12.bin --size 512 \
  --img bus.jpg --bench 200 --pipeline3 --cv-threads 1
```

## 576 基准（精度/速度折中）

```bash
python3 yolov8_person1.py \
  --model yolov8n_person1_576_bayese_576x576_nv12.bin --size 576 \
  --img bus.jpg --bench 200 --pipeline --cv-threads 4 --cpost

python3 yolov8_person1.py \
  --model yolov8n_person1_576_bayese_576x576_nv12.bin --size 576 \
  --img bus.jpg --bench 200 --pipeline3 --cv-threads 1 --cpost
```

## 摄像头（USB UVC）

```bash
# 720p60 MJPG（推荐先测）
python3 yolov8_person1.py \
  --model yolov8n_person1_576_bayese_576x576_nv12.bin --size 576 \
  --camera 0 --cam-width 1280 --cam-height 720 --cam-fps 60 --fourcc MJPG \
  --bench 300 --cpost --cv-threads 4

# 480p120（要 100+ 只能用这档；但 480p 上采样到 576 会损失细节）
python3 yolov8_person1.py \
  --model yolov8n_person1_576_bayese_576x576_nv12.bin --size 576 \
  --camera 0 --cam-width 640 --cam-height 480 --cam-fps 120 --fourcc MJPG \
  --bench 300 --cpost --cv-threads 4

# 1080p30
python3 yolov8_person1.py \
  --model yolov8n_person1_576_bayese_576x576_nv12.bin --size 576 \
  --camera 0 --cam-width 1920 --cam-height 1080 --cam-fps 30 --fourcc MJPG \
  --bench 300 --cpost --cv-threads 4
```

脚本会打印实际协商到的分辨率/帧率、`end-to-end FPS` 和 `capture FPS`，并保存
一张带框的 `person_result.jpg`。摄像头是 148° 鱼眼、-17% 畸变，当前未做去畸变；
自瞄前需要按实际内参做一次 undistort。

### 实时窗口 / 浏览器预览

```bash
# 本机桌面窗口（板子接 HDMI，或在桌面终端运行）
python3 yolov8_person1.py \
  --model yolov8n_person1_576_bayese_576x576_nv12.bin --size 576 \
  --camera 0 --cam-width 1280 --cam-height 720 --cam-fps 60 \
  --bench 0 --cpost --show

# SSH 无 X11：开 MJPEG 流，PC 浏览器看
python3 yolov8_person1.py \
  --model yolov8n_person1_576_bayese_576x576_nv12.bin --size 576 \
  --camera 0 --cam-width 1280 --cam-height 720 --cam-fps 60 \
  --bench 0 --cpost --http 8080
# 浏览器打开 http://<板子IP>:8080/
```

`--bench 0` 表示一直跑到按 `q`（窗口）或 `Ctrl-C`。SSH 下 `--show` 需要
`ssh -X` 或 `export DISPLAY=:0`；最省事的是 `--http`。

## 摄像头实测与瓶颈（USB2.0 OV2710）

实测 `end-to-end FPS == capture FPS`，说明模型不是瓶颈，`cap.read()` 才是：

| 档位 | 协商 | 实际读帧 | 读一帧 |
|---|---|---|---|
| 1080p30 MJPG | 1920x1080@30 | 11.2 FPS | ~80 ms |
| 720p60 MJPG | 1280x720@60 | 24.3 FPS | ~36 ms |
| 480p120 MJPG | 640x480@120 | 47.9 FPS | ~20 ms |

读帧时间随像素数线性增长 → **OpenCV 在用 CPU 软解 MJPG**，USB2.0 带宽不是唯一限制。
先用 `cam_probe.py` 只测取帧确认：

```bash
python3 cam_probe.py --camera 0
```

要 100+ FPS，有三条路（按推荐顺序）：

1. **MIPI CSI 相机**：直接 NV12，不经过 USB/MJPG 解码，X5 有双 CSI；
2. **USB3 相机 + YUYV/NV12**：无解码，带宽足够；
3. **USB2 相机 + 硬件 JPEG 解码**（RDK VPU / `sp` codec / GStreamer 硬解）：
   能把 720p 的解码从 36 ms 降到几 ms，但 720p60 仍被 60 fps 采集限制。

另外这颗 148° 鱼眼、-17% 畸变，对远距离小目标不友好；自瞄建议 60~90° FOV。

## 实测（bus.jpg 1080p，200 次）

| 配置 | pre | bpu | post | total | FPS |
|---|---|---|---|---|---|
| 640 serial, cv4 | 3.38 | 6.67 | 3.49 | 13.54 ms | 73.9 |
| 640 pipeline | — | — | — | 12.15 ms | 82.3 |
| 640 pipeline3 | — | — | — | 16.35 ms | 61.2 |
| **512 serial, cv4** | **2.28** | **4.41** | **3.06** | **9.76 ms** | **102.5** |
| **512 pipeline** | — | — | — | **8.68 ms** | **115.2** |
| 512 pipeline3 | — | — | — | 8.96 ms | 111.6 |
| 640 pipeline + cpost | — | — | 3.80 | 13.50 ms | 74.0 |
| 640 pipeline3 + cpost | — | — | 3.82 | 11.46 ms | **87.2** |
| **512 pipeline + cpost** | — | — | **2.41** | **7.39 ms** | **135.3** |
| 512 pipeline3 + cpost | — | — | 2.43 | 7.70 ms | 130.0 |
| 576 pipeline + cpost | — | — | 2.85 | 10.97 ms | 91.1 |
| **576 pipeline3 + cpost** | — | — | 3.27 | **9.60 ms** | **104.1** |

**最终选择：576 + 三段流水线 + C++ 后处理 = 104.1 FPS，person AP 0.7178。**
640 的 C++ 后处理受 2 MB 输出拷贝拖累，最好 87.2 FPS；512 虽 135 FPS 但 AP 只有 0.557。

## C++ 后处理（推荐）

```bash
bash build_post.sh                     # g++ -O3 -shared -fPIC -> libpost.so

# 640
python3 yolov8_person1.py --model yolov8n_person1_bayese_640x640_nv12.bin \
  --img bus.jpg --bench 200 --pipeline --cv-threads 4 --cpost

# 512
python3 yolov8_person1.py \
  --model yolov8n_person1_512_bayese_512x512_nv12.bin --size 512 \
  --img bus.jpg --bench 200 --pipeline --cv-threads 4 --cpost
```

`postprocess.cpp` 只做 DFL + sigmoid + NMS，不依赖 OpenCV；Python 通过 ctypes
传 6 个 float 指针，不需要 `python3-dev`。

## 精度对照（COCO val 200 张，person AP@0.5）

| 输入 | AP@0.5 |
|---|---|
| 640 | **0.7244** |
| 576 | **0.7178**（只差 0.0066） |
| 512 | 0.5572 |

576 几乎不掉精度，是推荐折中；512 掉得太多，只适合目标较大的场景。

## v7 改了什么

1. 增加 `postprocess.cpp` / `build_post.sh`，`--cpost` 走 C++ DFL+sigmoid+NMS；
2. 640/512 共用同一套后处理，anchor 按 `(size, stride)` 缓存；
3. 保留 canvas 复用、`cv2.setNumThreads`、两段流水线。

## 如果 `hobot_dnn` 导入失败

说明镜像版本不匹配（需要 RDK OS 3.x 自带的 `hobot_dnn`）。此时改用
`hbm_runtime` 的新版 runtime（GitHub `samples/vision/ultralytics_yolo`）。
