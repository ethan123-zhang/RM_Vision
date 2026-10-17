# RM_Vision

RoboMaster 2027 视觉方向的项目工作区。这里记录**当前进度、下一步、问题与解决过程、关键决策和实验记录**；跨项目可复用的领域知识沉淀在 [aiVault](https://github.com/ethan123-zhang/Ethan-s-aiVault)。

> 目标：让任何人（包括未来的自己）打开仓库就能回答三个问题——做到哪了、接下来做什么、为什么这样做。

## 当前状态

截至 2026-10-17：项目记忆与协作规则已建立，RM2027 官方资料已归档，尚未开始视觉方案开发。详见 [PROGRESS.md](PROGRESS.md)。

## 目录

| 路径 | 用途 |
| :--- | :--- |
| `PROGRESS.md` | 唯一“现在进行时”：状态、里程碑、下一步、阻塞 |
| `docs/conventions.md` | 双库协作与记录规则（本仓库的规则源） |
| `docs/environment.md` | 板卡、镜像、工具链版本与大文件校验 |
| `docs/issues/` | 问题记录：现象 → 排查 → 根因 → 解决 → 验证 |
| `docs/decisions/` | 决策记录：背景 → 选项 → 结论 → 影响 |
| `_extract/` | PDF 文本提取脚本与产物 |
| `相关文件/` | RM2027 官方 PDF（加密原件 + 未加密可 AI 读取版） |
| `RDK_X5镜像/` | RDK X5 系统镜像（**不入库**，见 `docs/environment.md`） |

## 资料

RM2027 V1.0.0（20261015）三份官方文档：

- 比赛规则手册
- 参赛手册
- 机器人制作规范手册

`相关文件/` 下同时保留加密原件与 `未加密可AI访问/` 解密版，用途与校验值见 [docs/environment.md](docs/environment.md)。

## 环境

- 板卡：RDK X5（D-Robotics）
- 镜像：`rdk-x5-ubuntu22-preinstalled-desktop-3.5.0-arm64.img`（Ubuntu 22.04 preinstalled desktop，约 9.91 GB，不入库）
- 校验值与获取方式：[docs/environment.md](docs/environment.md)

## 与 aiVault 的关系

- **RM_Vision**：项目记忆，允许 WIP，随项目演进。
- **aiVault**：领域知识库（嵌入式 / AI 量化 / 边缘推理），只收编译后的结论。
- 知识单向沉淀：RM 实践 → 提炼 → aiVault；aiVault 反向指导 RM。
- 两仓独立、互链，不做 submodule、不合并；本仓只放链接，不复述 aiVault 正文。

本项目直接相关的 aiVault 页面（固定到提交 `284c5d8`）：

- [YOLO_Head_Quantization_Collapse](https://github.com/ethan123-zhang/Ethan-s-aiVault/blob/284c5d8fd8a76023395135ba06eca8737d515560/wiki/concepts/YOLO_Head_Quantization_Collapse.md) — YOLO INT8 量化类别分数塌陷
- [Tail_Cut_Deployment](https://github.com/ethan123-zhang/Ethan-s-aiVault/blob/284c5d8fd8a76023395135ba06eca8737d515560/wiki/concepts/Tail_Cut_Deployment.md) — 切尾巴部署与真机实录
- [RKNN_Quantization_Strategy_vs](https://github.com/ethan123-zhang/Ethan-s-aiVault/blob/284c5d8fd8a76023395135ba06eca8737d515560/wiki/comparisons/RKNN_Quantization_Strategy_vs.qmd) — 量化策略对比
- [INDEX](https://github.com/ethan123-zhang/Ethan-s-aiVault/blob/284c5d8fd8a76023395135ba06eca8737d515560/INDEX.md) — 全局索引

## 工具

从 PDF 提取纯文本（仅标准库，Python 3）：

```powershell
python _extract/extractor.py "相关文件/未加密可AI访问/<文档>.pdf" "_extract/<文档>.txt"
```

## 维护

每次会话结束前更新 `PROGRESS.md` 并推送；改动了 aiVault 也要同步推送。完整规则见 [docs/conventions.md](docs/conventions.md)。
