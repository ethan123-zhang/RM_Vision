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
| `model/yolov8n_person1_512_op11.onnx` | 512 输入版本（仅改输入尺寸） |
| `model/yolov8n_person1_576_op11.onnx` | 576 输入版本（精度折中） |
| `model/yolov8n_person1_512_bayese_512x512_nv12.bin` | 512 输入 X5 模型，3.4 MB |
| `model/yolov8n_person1_576_bayese_576x576_nv12.bin` | 576 输入 X5 模型，3.5 MB |
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

512 版本：`tools/make_input_size.py` 改输入尺寸，`config_x5_512.yaml` + `calib512/`
编译，产物 `yolov8n_person1_512_bayese_512x512_nv12.bin`（3.4 MB）。

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

## 板端（已实测）

用自包含脚本 `board/yolov8_person1.py`（镜像自带 `hobot_dnn.pyeasy_dnn`）：

```bash
# 640
python3 yolov8_person1.py --model yolov8n_person1_bayese_640x640_nv12.bin \
  --img bus.jpg --bench 200 --cv-threads 4

# 512（推荐，100+）
python3 yolov8_person1.py --model yolov8n_person1_512_bayese_512x512_nv12.bin \
  --size 512 --img bus.jpg --bench 200 --pipeline --cv-threads 4
```

## 性能实测（bus.jpg 1080p，200 次）

| 配置 | total | FPS |
|---|---|---|
| 640 serial, cv_threads=4 | 13.54 ms | 73.9 |
| 640 pipeline（2 段） | 12.15 ms | 82.3 |
| 640 pipeline3（3 段） | 16.35 ms | 61.2 |
| **512 serial, cv_threads=4** | **9.76 ms** | **102.5** |
| **512 pipeline（2 段）** | **8.68 ms** | **115.2** |
| 512 pipeline3（3 段） | 8.96 ms | 111.6 |
| 640 pipeline + cpost | 13.50 ms | 74.0 |
| 640 pipeline3 + cpost | 11.46 ms | 87.2 |
| **512 pipeline + cpost** | **7.39 ms** | **135.3** |
| 512 pipeline3 + cpost | 7.70 ms | 130.0 |
| 576 pipeline + cpost | 10.97 ms | 91.1 |
| **576 pipeline3 + cpost** | **9.60 ms** | **104.1** |

**最终选择：576 输入 + 三段流水线 + C++ 后处理 = 104.1 FPS，person AP 0.7178。**
640 最好 87.2 FPS；512 虽 135 FPS，但 AP 只有 0.557。

512 的 BPU 4.41 ms、pre 2.28 ms、post 3.06 ms。512 + C++ 后处理 + 两段流水线
达到 **135.3 FPS**；640 的 C++ 后处理受 2 MB 输出拷贝拖累，最好 87.2 FPS。

C++ 后处理已实现（`board/postprocess.cpp`，ctypes 调用）：

```bash
bash board/build_post.sh
python3 board/yolov8_person1.py --model ... --img bus.jpg \
  --bench 200 --pipeline --cv-threads 4 --cpost
```

## 精度对照（COCO val 200 张，person AP@0.5）

| 输入 | AP@0.5 |
|---|---|
| 640 | **0.7244** |
| 576 | **0.7178**（只差 0.0066） |
| 512 | 0.5572 |

576 几乎不掉精度，适合作为 100+ FPS 的折中；512 掉 23%，只适合目标较大的场景。

## 待办

- [x] X5 `.bin` 编译（640 / 512）
- [x] 板端实测 60+ / 100+
- [x] 512 的 COCO person AP 对照（0.5572 vs 0.7244）
- [ ] 640 输入 + C++ 后处理，冲 120+ 且不掉精度（C++ 已实现，待板端实测）
- [ ] 用自有数据微调 person 头（可选）
