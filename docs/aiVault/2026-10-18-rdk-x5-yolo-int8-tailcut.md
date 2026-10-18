---
title: RDK X5 YOLOv8 单类切尾巴：从 INT8 下溢到 104 FPS
tags: [RDK-X5, BPU, YOLOv8, INT8, PTQ, 边缘部署]
updated: 2026-10-18
sources:
  - RM_Vision/x5_yolo/README.md
  - RM_Vision/docs/decisions/2026-10-17-yolo-person-head-x5.md
  - RM_Vision/docs/decisions/2026-10-18-yolo-input-576.md
---

# 结论

RDK X5（Bayes-e，10 TOPS）跑 YOLOv8n、只检测 person：

| 输入 | 最佳配置 | FPS | person AP@0.5 |
|---|---|---|---|
| 640 | pipeline3 + C++ post | 87.2 | 0.7244 |
| **576** | **pipeline3 + C++ post** | **104.1** | **0.7178** |
| 512 | pipeline + C++ post | 135.3 | 0.5572 |

（COCO val 200 张，person 类；模型为 COCO 权重做 head surgery，不重训。）

最终选 576：精度只比 640 低 0.9%，帧率进入 100+。

# 现象

1. 全 INT8 量化后，检测头类别分数全 0，mAP=0；框回归正常。
2. 检测头放 CPU 后，帧率仍然很低；预处理/后处理各占十几毫秒。
3. 多线程 pipeline 有时反而更慢。
4. USB2.0 摄像头标称 720p60/480p120，实测只有 24/48 FPS。

# 根因与解法

## 1. 下溢不是类别数，是 Sigmoid + 主干量化误差

YOLOv8 类别分支是逐类 sigmoid（BCE），不是 softmax；80 类不共享概率预算。
实测类别 logits 集中在 -20 附近，Sigmoid 后 1e-9；主干 INT8 误差把 logits 推得更负，
Sigmoid 输出被压成 0。只保检测头无效，保主干才恢复。

解法：**切尾巴**。BPU 只输出 raw logits（每尺度 cls + box，NHWC float32），
DFL/Sigmoid/NMS 全部放 CPU。这样 INT8 图里没有会饱和的 Sigmoid。

## 2. 检测头卷积不能放 CPU

YOLOv8n head 约 1.0~1.2 GMAC（~2.3 GFLOP）；X5 是 8×A55@1.5GHz，
实测有效算力 15~25 GFLOPS，head 卷积要 90~150 ms。CPU 只适合做
DFL + sigmoid + NMS（C++ 可到 1 ms 级）。

## 3. 单类头手术

COCO 80 类模型里 person 就是 `cv3.*` 的第 0 个输出通道；
把三个尺度的 `cv3` 末层 1×1 卷积权重切片成 1 通道，不重训，
person 检测结果与原模型逐 anchor 一致（score 1.3e-7、box 1.5e-4 px）。
494 张 COCO person AP@0.5 0.7800 → 0.7800。

## 4. 输入尺寸是最大杠杆，但有精度悬崖

640 / 576 / 512 对比（同一批 200 张）：

| 输入 | AP | 说明 |
|---|---|---|
| 640 | 0.7244 | 基准 |
| 576 | 0.7178 | 只掉 0.9% |
| 512 | 0.5572 | 掉 23%，小目标被杀 |

512 不是线性掉点：640 下 20~30 px 的人，到 512 只剩 8~16 px。
**自瞄不要轻易用 512**；576 是甜点。

## 5. Python 侧的隐藏成本

- 每帧 `np.full((640,640,3), 114)` + 整块 blit：约 12 ms；
  复用 canvas 后降到 ~0.5 ms。
- `np.concatenate` + 转置拼 NV12：4.5 ms；改成预分配 + `(N,2)` 连续写后 ~1 ms。
- `cv2.setNumThreads(4)` 对 resize 有利（1080p→640 从 17 ms 到 3 ms），
  但会和 BPU 线程抢核；pipeline 里用 1 或 4 要实测。

## 6. GIL 是 Python 流水线的天花板

2 段 pipeline（pre 线程 + bpu/post 主线程）在 640 上 82 FPS；
3 段（pre/bpu/post）因为多一次 2 MB 输出拷贝 + GIL，反而降到 61 FPS。
把后处理换成 C++（ctypes 调用 `.so`，调用期间释放 GIL）后，
576 的三段 pipeline 反而最快：104 FPS。

结论：**Python 多线程不是越多越好；要么减少拷贝，要么把计算移出 GIL（C++/ctypes）**。

## 7. C++ 后处理不是万能

640 的 C++ post 反而比 Python 慢（74 vs 82 FPS），因为 640 的 6 个输出
约 2 MB，`t.buffer` 取数/拷贝占大头，C++ 省下的解码计算抵不过。
512/576 输出小，C++ 收益明显；640 想再快需要零拷贝输出，而不是换语言。

## 8. 摄像头端到端瓶颈：MJPG 软解

| 档位 | 读一帧 | 实测 FPS |
|---|---|---|
| 1080p30 MJPG | ~80 ms | 11.2 |
| 720p60 MJPG | ~36 ms | 24.3 |
| 480p120 MJPG | ~20 ms | 47.9 |

`end-to-end FPS == capture FPS`，说明模型在等摄像头。读帧时间随像素线性增长，
是 OpenCV 在 A55 上软解 JPEG。要 100+：
MIPI CSI（NV12 直出）> USB3 + YUYV/NV12 > USB2 + 硬件 JPEG 解码。
148° 鱼眼/-17% 畸变也不适合远距离小目标。

# 复现要点

```bash
# head surgery + opset 11
python tools/make_person_head.py --src yolov8n.onnx --dst person1_nhwc.onnx --layout nhwc
python tools/to_opset11.py --src person1_nhwc.onnx --dst person1_op11.onnx

# 校准 + 编译（hb_mapper 1.24.3，X5 bayes-e）
python tools/make_calibration.py --images ... --labels ... --out calib --num 32
hb_mapper makertbin --config config_x5.yaml --model-type onnx

# 板端
python3 yolov8_person1.py --model person1_576.bin --size 576 \
  --camera 0 --cam-width 1280 --cam-height 720 --cam-fps 60 \
  --bench 300 --pipeline3 --cv-threads 1 --cpost
```

# 可复用结论

1. 检测头下溢先查 **Sigmoid/DFL 是否在 INT8 图里**，不要先怪类别数。
2. 切尾巴的标准形态：BPU 输出 raw logits，CPU 做 DFL/Sigmoid/NMS。
3. 输入尺寸是精度/速度的最大旋钮；512 对小目标有悬崖。
4. 预处理 canvas 复用、NV12 连续打包，能省十几毫秒。
5. Python 多线程受 GIL 限制；C++/ctypes 或减少拷贝才是出路。
6. 端到端 FPS 等于采集 FPS 时，先测 `cap.read()`，别怪模型。
7. 任何优化都要分段计时：pre / BPU / post / capture，否则容易优化错对象。
