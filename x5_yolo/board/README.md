# RDK X5 单类 person YOLOv8n 快速包（v5）

自包含，只依赖镜像自带的 `hobot_dnn` (pyeasy_dnn) / OpenCV / numpy。

## 文件

| 文件 | 说明 |
|---|---|
| `yolov8_person1.py` | 板端推理 + 分段测速（v2：NEAREST resize、预分配 NV12、可选流水线） |
| `yolov8n_person1_bayese_640x640_nv12.bin` | X5 bayes-e 量化模型，3.7 MB |
| `person.names` | 单类标签 |
| `bus.jpg` | 测试图 |

## 跑

```bash
cd ~/rdk_model_zoo/yolov8_person1

# 1) 看预处理三段耗时
python3 yolov8_person1.py --model yolov8n_person1_bayese_640x640_nv12.bin \
  --img bus.jpg --profile

# 2) 串行基准
python3 yolov8_person1.py --model yolov8n_person1_bayese_640x640_nv12.bin \
  --img bus.jpg --bench 200

# 3) 两段流水线（pre 与 bpu 重叠）
python3 yolov8_person1.py --model yolov8n_person1_bayese_640x640_nv12.bin \
  --img bus.jpg --bench 200 --pipeline

# 4) 三段流水线（pre / bpu / post 各一线程，最快）
python3 yolov8_person1.py --model yolov8n_person1_bayese_640x640_nv12.bin \
  --img bus.jpg --bench 200 --pipeline3 --cv-threads 4
```

输出示例：

```
serial x200: pre 6.9 | bpu 6.3 | post 3.0 | total 16.2 ms | 61.7 FPS
pipeline x200: 9.4 ms/frame | 106.4 FPS
```

模型输出是 raw logits（不含 Sigmoid/DFL），DFL/sigmoid/NMS 都在 CPU 做，
不会出现 INT8 检测头下溢。

## v5 改了什么

1. **三段流水线** `--pipeline3`：pre / BPU(+输出拷贝) / post 各一个线程；
2. anchor 网格预计算，后处理不再每帧 `tile/repeat`；
3. 后处理输入改为 `(array, shape, scale)`，避免跨线程持有 SDK buffer。

## 下一步

如果 `pre` 仍 > 10 ms，说明瓶颈在 `BGR→YUV_I420`：
下一步把模型重编为 `input_type_rt: rgb`，直接喂 RGB HWC uint8，省掉整段 YUV 转换。

## 如果 `hobot_dnn` 导入失败

说明镜像版本不匹配（需要 RDK OS 3.x 自带的 `hobot_dnn`）。此时改用
`hbm_runtime` 的新版 runtime（GitHub `samples/vision/ultralytics_yolo`）。
