# RDK X5 单类 person YOLOv8n 快速包（v6：640 + 512）

自包含，只依赖镜像自带的 `hobot_dnn` (pyeasy_dnn) / OpenCV / numpy。

## 文件

| 文件 | 说明 |
|---|---|
| `yolov8_person1.py` | 板端推理 + 分段测速（复用 canvas、单线程 OpenCV、2/3 段流水线、`--crop`） |
| `yolov8n_person1_bayese_640x640_nv12.bin` | 640 输入，3.7 MB |
| `yolov8n_person1_512_bayese_512x512_nv12.bin` | 512 输入，3.4 MB，BPU 约 4.5 ms |
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

## 实测（bus.jpg 1080p，200 次）

| 配置 | pre | bpu | post | total | FPS |
|---|---|---|---|---|---|
| 640 serial, cv4 | 3.38 | 6.67 | 3.49 | 13.54 ms | 73.9 |
| 640 pipeline | — | — | — | 12.15 ms | 82.3 |
| 640 pipeline3 | — | — | — | 16.35 ms | 61.2 |
| **512 serial, cv4** | **2.28** | **4.41** | **3.06** | **9.76 ms** | **102.5** |
| **512 pipeline** | — | — | — | **8.68 ms** | **115.2** |
| 512 pipeline3 | — | — | — | 8.96 ms | 111.6 |

512 + 两段流水线是当前最佳：**115 FPS**。3 段流水线受 GIL 影响，不再提升。

## 精度（COCO val 200 张，person AP@0.5）

| 输入 | AP@0.5 |
|---|---|
| 640 | **0.7244** |
| 512 | 0.5572 |

512 快，但 person AP 掉 **0.167（相对 -23%）**。自瞄远距离小目标建议 640；
512 只适合“速度优先、目标较大”的场景。

## v6 改了什么

1. 增加 512 输入模型与 `--size`；
2. 640/512 共用同一套后处理，anchor 按 `(size, stride)` 缓存；
3. 保留 v5 的 canvas 复用、`cv2.setNumThreads`、三段流水线。

## 如果 `hobot_dnn` 导入失败

说明镜像版本不匹配（需要 RDK OS 3.x 自带的 `hobot_dnn`）。此时改用
`hbm_runtime` 的新版 runtime（GitHub `samples/vision/ultralytics_yolo`）。
