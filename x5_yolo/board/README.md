# RDK X5 单类 person YOLOv8n 快速包（v7：640 + 512 + C++ 后处理）

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

512 + C++ 后处理 + 两段流水线是当前最佳：**135.3 FPS**。
640 的 C++ 后处理没有超过 Python 两段（74 vs 82），因为 640 的 2 MB
输出取数拷贝占了大头；640 最好的是 pipeline3 + cpost（87.2）。

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
