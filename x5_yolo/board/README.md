# RDK X5 单类 person YOLOv8n 快速包

自包含，不依赖 `rdk_model_zoo` 其余代码；只用镜像自带的 `hobot_dnn` (pyeasy_dnn)。

## 文件

| 文件 | 说明 |
|---|---|
| `yolov8_person1.py` | 板端推理 + 分段测速（letterbox / NV12 / DFL / NMS） |
| `yolov8n_person1_bayese_640x640_nv12.bin` | X5 bayes-e 量化模型，3.7 MB |
| `person.names` | 单类标签 |
| `bus.jpg` | 测试图 |

## 板上运行

```bash
# 在板子上（默认用户 sunrise）
mkdir -p ~/rdk_model_zoo/yolov8_person1
cd ~/rdk_model_zoo/yolov8_person1
unzip ~/rdk_model_zoo_yolov8_person.zip -d .

python3 yolov8_person1.py \
  --model yolov8n_person1_bayese_640x640_nv12.bin \
  --img bus.jpg --save person_result.jpg --bench 200
```

输出示例：

```
outputs:
  [0] cls_80 ... (1, 80, 80, 1)
  [1] box_80 ... (1, 80, 80, 64)
...
detections: 4, postprocess 1.8 ms
bench x200: pre 1.8 ms | bpu 7.9 ms | post 1.5 ms | total 11.2 ms | 89.3 FPS
```

模型输出是 raw logits（不含 Sigmoid/DFL），DFL/sigmoid/NMS 都在 CPU 做，
所以不会出现 INT8 检测头下溢。

## 如果 `hobot_dnn` 导入失败

说明镜像版本不匹配（需要 RDK OS 3.x 自带的 `hobot_dnn`）。此时改用
`hbm_runtime` 的新版 runtime（GitHub `samples/vision/ultralytics_yolo`）。
