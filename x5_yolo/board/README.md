# RDK X5 单类 person YOLOv8n 快速包（v3）

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

# 3) 预处理与 BPU 重叠（推荐）
python3 yolov8_person1.py --model yolov8n_person1_bayese_640x640_nv12.bin \
  --img bus.jpg --bench 200 --pipeline
```

输出示例：

```
serial x200: pre 6.9 | bpu 6.3 | post 3.0 | total 16.2 ms | 61.7 FPS
pipeline x200: 9.4 ms/frame | 106.4 FPS
```

模型输出是 raw logits（不含 Sigmoid/DFL），DFL/sigmoid/NMS 都在 CPU 做，
不会出现 INT8 检测头下溢。

## v3 改了什么

1. `cv2.setNumThreads(1)`，避免 OpenCV 线程池和 BPU 驱动抢核；
2. NV12 打包改用 `(N,2)` 连续写，替代转置 + `reshape`；
3. 新增 `--crop`：中心裁 640×640，不 resize（自瞄 ROI 场景的快速路径）；
4. `--cv-threads` 可调。

## 下一步

如果 `pre` 仍 > 10 ms，说明瓶颈在 `BGR→YUV_I420`：
下一步把模型重编为 `input_type_rt: rgb`，直接喂 RGB HWC uint8，省掉整段 YUV 转换。

## 如果 `hobot_dnn` 导入失败

说明镜像版本不匹配（需要 RDK OS 3.x 自带的 `hobot_dnn`）。此时改用
`hbm_runtime` 的新版 runtime（GitHub `samples/vision/ultralytics_yolo`）。
