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

## 已有实测（640，bus.jpg 1080p）

| 配置 | total | FPS |
|---|---|---|
| serial, cv_threads=1 | 16.51 ms | 60.6 |
| serial, cv_threads=4 | 13.54 ms | 73.9 |
| pipeline（2 段） | 12.15 ms | 82.3 |
| pipeline3（3 段） | 16.35 ms | 61.2 |
| serial --crop | 12.92 ms | 77.4 |

3 段流水线反而更慢：多一次 2 MB 输出拷贝 + Python GIL 争用。
**Python 线程方案的上限约 82 FPS**；100+ 走 512 或 C++ 后处理。

## v6 改了什么

1. 增加 512 输入模型与 `--size`；
2. 640/512 共用同一套后处理，anchor 按 `(size, stride)` 缓存；
3. 保留 v5 的 canvas 复用、`cv2.setNumThreads`、三段流水线。

## 如果 `hobot_dnn` 导入失败

说明镜像版本不匹配（需要 RDK OS 3.x 自带的 `hobot_dnn`）。此时改用
`hbm_runtime` 的新版 runtime（GitHub `samples/vision/ultralytics_yolo`）。
