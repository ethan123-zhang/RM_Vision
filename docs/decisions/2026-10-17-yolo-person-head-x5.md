# 决策：RDK X5 用单类 person 切尾巴 YOLOv8n

- 日期：2026-10-17
- 状态：已采纳，待板端实测
- 相关：`x5_yolo/`

## 背景

RDK X5 上跑 YOLOv8n，目标 60 FPS 以上（自瞄场景希望 100~150）。
此前 RK3588 实测过：全 INT8 时检测头类别分数全部变 0，根因是主干量化误差
叠加 Sigmoid 饱和；只保检测头无效，保主干或切尾巴有效。

## 选项

| 选项 | 做法 | 问题 |
|---|---|---|
| A | COCO 权重把 `cv3.*` 切成 person 1 类；BPU 输出 raw logits，CPU 做 DFL/Sigmoid/NMS | 需要 6 输出 runtime；无需数据 |
| B | 检测头卷积也放 CPU（真正“head on CPU”） | 头卷积约 1.0–1.2 GMAC，A55 上 90–150 ms，100 FPS 不可能 |
| C | 保留 80 类，只解码 person | 头最大、后处理最贵，不满足“只检测几种” |

## 结论

采用 **A**。类别数不是下溢的机制：YOLOv8 类别分支是逐类 sigmoid，不是 softmax；
下溢由“主干量化误差 × Sigmoid 饱和”产生。把 Sigmoid/DFL 移出 INT8 图后，
BPU 只输出 logits，1 类只是顺带降低头与后处理开销。

## 依据

- 494 张 COCO val 子集 / 1159 个 person GT：AP@0.5 由 0.7800 → 0.7800，delta 0.0000。
- 逐 anchor parity：score 最大差 1.3e-7，box 最大差 1.5e-4 px。
- hb_mapper 1.24.3 量化 cosine：cls 0.9977~0.9997，box 0.9962~0.9972，无塌陷。
- 官方 X5 YOLOv8n 640：BPU 8.04 ms、预处理 1.78 ms、C++ 后处理 0.80 ms；
  1 类 + 流水线具备 100~150 FPS 的空间。

## 影响

- 模型输出 6 个 NHWC raw logits：`cls_80/box_80/cls_40/box_40/cls_20/box_20`。
- 板端必须用 `--classes-num 1`，并按 raw logits 解码（DFL + sigmoid 在 CPU）。
- 100~150 FPS 需要 C++ 后处理与三段流水线；Python 先做正确性基线。

## 复审条件

- 板端串行 <60 FPS 或流水线 <100 FPS：先降输入到 512，再考虑 C++ 重写。
- 自瞄需要更多类别（装甲板/机器人）：同一手术扩展到 N 类，不改架构。
