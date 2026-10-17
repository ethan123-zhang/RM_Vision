# YOLOv8n → RDK X5 单类（person）检测头

目标：在 RDK X5 上跑 **YOLOv8n 640、仅 person 一类**，BPU 跑主干+neck+head 卷积，
CPU 只做 DFL/Sigmoid/解码/NMS，端到端冲 100~150 FPS。

## 为什么这样切

1. YOLOv8 的类别分支是**逐类 sigmoid（BCE）**，不是 softmax；80 类不会把 person 概率摊薄。
   下溢来自 **主干 INT8 量化误差 + Sigmoid 饱和**，不是类别数。
2. 因此把 **Sigmoid/DFL 移出 INT8 图**，BPU 只输出 raw logits；这也是 X5 官方
   model zoo 的做法（6 个输出头：每尺度 cls + box，NHWC、float32）。
3. 检测头卷积（cv2/cv3）不能放 CPU：约 1.0–1.2 GMAC，A55 上要 90–150 ms。
   CPU 只做 DFL + sigmoid + NMS（<1 ms，C++）。

## 已完成并在本机验证

| 项 | 结果 |
|---|---|
| head surgery | `cv3.*` 的 80 通道切片为 person 通道（权重切片，不重训） |
| 数值 parity（10 张） | max score diff **1.3e-7**，max box diff **1.5e-4 px** |
| person AP@0.5（494 张 / 1159 GT） | 原 80 类 **0.7800**，单类 **0.7800**，delta **0.0000** |
| opset 11 转换 | 8 个 Split 改写；与 opset 19 输出 **逐元素 0 差异** |

模型输出（NHWC，raw logits，无 Sigmoid/DFL）：

```
cls_80 [1,80,80,1]   box_80 [1,80,80,64]
cls_40 [1,40,40,1]   box_40 [1,40,40,64]
cls_20 [1,20,20,1]   box_20 [1,20,20,64]
```

## 目录

| 路径 | 说明 |
|---|---|
| `model/yolov8n_person1_nchw.onnx` | 验证用 NCHW 6 输出 |
| `model/yolov8n_person1_nhwc.onnx` | 官方同款 NHWC 布局 |
| `model/yolov8n_person1_op11.onnx` | **送 hb_mapper 的最终模型**（opset 11） |
| `tools/make_person_head.py` | head surgery |
| `tools/verify_parity.py` | 与原 80 类模型逐 anchor 对比 |
| `tools/evaluate_person.py` | person AP@0.5 |
| `tools/make_calibration.py` | 生成 `.rgbchw` 校准张量 |
| `config_x5.yaml` | hb_mapper 配置（bayes-e / nv12 / data_scale 1/255） |
| `convert.sh` | Docker 或本机 hb_mapper 编译 |
| `person.names` | 单类标签 |

## 复现 head surgery

```bash
python tools/make_person_head.py --src model/yolov8n.onnx \
  --dst model/yolov8n_person1_nhwc.onnx --layout nhwc
python tools/to_opset11.py --src model/yolov8n_person1_nhwc.onnx \
  --dst model/yolov8n_person1_op11.onnx
```

## 转换 X5 .bin

```bash
python tools/make_calibration.py \
  --images <coco>/images --labels <coco>/labels --out calib --num 32
./convert.sh
# 产物：bpu_model_output/yolov8n_person1_bayese_640x640_nv12.bin
```

要求：OE 1.2.8 / hb_mapper >= 1.24.3，输入 640×640 NV12。

### 本机实测（WSL，无 Docker）

`pip install rdkx5-yolo-mapper` 自带 `hbdk 3.49.15`，可以直接编译：

```bash
python3 -m venv venv
./venv/bin/pip install rdkx5-yolo-mapper
./venv/bin/pip install "setuptools<81"   # hbdk-config 需要 pkg_resources
export PATH="$PWD/venv/bin:$PATH"
hb_mapper makertbin --config config_x5.yaml --model-type onnx
```

两个坑（已踩过）：

1. `calib/` 里不能放 `calib_list.txt`，否则会被当成 168 元素的校准样本；
2. `hb_mapper` 调 `hbdk-config` 时依赖 PATH 里的 venv，必须显式 `export PATH`。

产物 `model/yolov8n_person1_bayese_640x640_nv12.bin`（3.7 MB，bayes-e）。
hb_mapper 给出的量化余弦相似度：

| 输出 | cosine |
|---|---|
| cls_80 / cls_40 / cls_20 | 0.99967 / 0.99859 / 0.99767 |
| box_80 / box_40 / box_20 | 0.99615 / 0.99662 / 0.99717 |

没有出现类别分数塌陷；head 本身也不在 INT8 图里做 sigmoid。

## 板端

```bash
# 1) 官方 runtime 先跑通（推荐，零自研代码）
git clone https://github.com/D-Robotics/rdk_model_zoo
cd rdk_model_zoo
python samples/vision/ultralytics_yolo/runtime/python/main.py \
  --platform x5 --family yolov8 --task detect \
  --model-path ~/yolov8n_person1_bayese_640x640_nv12.bin \
  --classes-num 1 --label-file ~/person.names \
  --test-img samples/vision/ultralytics_yolo/test_data/bus.jpg \
  --img-save-path /tmp/person.jpg
```

`--classes-num 1` 是必须的，否则 decoder 会按 80 类解释。

## 性能预期（640×640）

| 阶段 | X5 实测/估算 |
|---|---|
| BPU（1 类，6 输出） | ~7.5–8.0 ms |
| 预处理（NV12） | ~1.8 ms |
| CPU 后处理（C++ 优化） | <1 ms |
| 串行 | ~11 ms → **~90 FPS** |
| 三段流水线 | max(8, 1.8, 1) → **~120 FPS** |
| 双 BPU 线程 + 流水线 | **130–150 FPS** |

Python 后处理约 5 ms（80 类）；1 类 + 先筛后算可压到 ~1.5–2 ms，串行 ~90 FPS。
要稳定 100–150 必须 C++ 后处理 + 流水线。

## 待办

- [ ] 在 X5 上编译 `.bin` 并跑官方 runtime，记录实际 BPU/预处理/后处理耗时
- [ ] 1 类 C++ 后处理（先筛后算）+ 双线程流水线
- [ ] 若 640 不够：512 输入对照（BPU 约 5 ms）
- [ ] 用自有数据微调 person 头（可选，当前 COCO 权重已可用）
