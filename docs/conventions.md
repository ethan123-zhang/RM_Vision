# 双库协作与记录规则

> 2026-10-28 用户确认：修改文件后自检，按明确路径提交并推送；两仓分别处理，
> 不直接使用会整库暂存的 `push_ai.bat`。这是当前双库推送约定，优先于旧工具调用约定。
> 本次未修改 aiVault 自身文件，也未处理其 inbox。

> 本文件是 RM_Vision 的规则源；aiVault 内部规则以其 `AGENTS.md` 为准，本文只写两库交界处。
> 定稿：2026-10-17（与仓库初始化同批提交）。

## 1. 两个仓库的分工

| | RM_Vision（本仓） | aiVault |
| :--- | :--- | :--- |
| 定位 | 项目记忆 | 领域知识库 |
| 内容 | 进度、下一步、问题、决策、实验、代码 | 可复用的结论、原理、对比 |
| 状态 | 允许 WIP、半成品 | 只收编译后的稳定内容 |
| 规则 | 本文件 | `AGENTS.md` + `prompts/` |

知识单向流动：**RM 实践 → 提炼 → aiVault**。aiVault 反向指导 RM，但 RM 不复述 aiVault 正文，只放链接（带 commit SHA 的永久链接）。

两仓独立、互链；**不做 submodule、不合并**。

## 2. 目录与命名

```text
E:\RM\
├─ README.md
├─ PROGRESS.md
├─ docs/
│  ├─ conventions.md
│  ├─ environment.md
│  ├─ issues/
│  └─ decisions/
├─ _extract/
├─ 相关文件/
└─ RDK_X5镜像/        # 仅本地，不入库
```

- 新增文档：文件名英文 slug，正文中文；标题写清一句话结论。
- 日期用 `YYYY-MM-DD`。
- 不移动 `相关文件/`、`_extract/`、`RDK_X5镜像/` 的现有结构。

## 3. 记录什么、记在哪

### 3.1 进度 → `PROGRESS.md`

唯一“现在进行时”。每次会话结束必须更新：一句话状态、里程碑、正在做、下一步、阻塞。
历史进度不在本文件堆积，完成后在“最近变更”留一行即可。

### 3.2 问题 → `docs/issues/YYYY-MM-DD-slug.md`

适用：报错、跑不通、结果异常、硬件/环境问题。
必须包含：现象、复现步骤、排查过程、根因、解决、验证。
末尾写“可复用结论”：一句话说明这是项目特例还是可沉淀的通用知识；后者才考虑进 aiVault。

### 3.3 决策 → `docs/decisions/YYYY-MM-DD-slug.md`

适用：选型、路线、指标、接口约定等“以后可能被问为什么”的选择。
必须包含：背景、选项与取舍、结论、影响、复审条件。

### 3.4 实验

实验记录必须带环境指纹（镜像版本、SDK、板卡、相机、关键参数），否则不可复现。

## 4. Git 规则

- 分支：`main`；不 force push。
- 提交信息：`type(scope): 中文摘要`
  - type：`feat` / `fix` / `docs` / `chore` / `exp`
  - 例：`docs(progress): 初始化项目记忆与协作规则`
- 粒度：一个逻辑变更一个提交。RM 允许 WIP 提交，但需在 `PROGRESS.md` 标注“未完成”。
- 推送时机：每次会话结束、里程碑完成、用户要求时。
- 推前自检：
  1. `git status` 无意外文件；
  2. `git diff --stat` 符合预期；
  3. 无密钥、无 >100 MB 文件；
  4. 两仓分别检查，改过就推。
- aiVault 不推半成品；其 Ingest 待批期间不得推送 wiki 变更（见其 `AGENTS.md` §9）。
- `push_ai.bat` 保持原样，不改；但**不直接运行**（它会 `git add .`，可能带入 inbox 或大文件）。推送 aiVault 时按明确路径 `git add`。

## 5. 大文件

| 类型 | 策略 |
| :--- | :--- |
| 系统镜像（`.img` / `.iso`） | 不入库；`environment.md` 记录文件名、字节数、SHA256、获取方式 |
| 模型 / 权重（`.onnx` / `.pt` / `.rknn` / `.kmodel` 等） | 不入库；记录导出命令与校验值 |
| 官方 PDF | 入库（单文件 <100 MB）；保留加密原件与未加密可读版 |
| 数据集 | 不入库；记录来源、版本、校验值 |

GitHub 硬限制：单文件 100 MB，仓库建议 <1 GB。超过就走外部存储 + 校验值。

## 6. aiVault 纠错与新增

### 6.1 修正已有页面

不静默覆盖。在改动处插入：

```markdown
> [!warning] 修正 2026-10-17（来源：RM_Vision）
> - 原文：……
> - 改为：……
> - 依据：……（链接到 RM 的 issue / commit / 实测数据）
```

同时：

- 更新该页 frontmatter 的 `updated: 2026-10-17`；
- 在 `LOG.md` 追加 `## [YYYY-MM-DD HH:MM] correction | <页面路径>`，写清改动、原因、来源。

### 6.2 新增结论

- 走 aiVault 的 `prompts/ingest.md`（外部素材，需人工审批）或 `prompts/query.md`（库内追问沉淀，AI 自主）。
- 来自 RM 实践的结论按 query 沉淀；引用官方 PDF 等外部素材按 ingest 走审批。
- 页面 `sources:` 指向 RM 的 issue/decision 永久链接。

### 6.3 删除

删除页面/段落仍须人工审批（aiVault `AGENTS.md` 铁律）。纠错若必须删段落，先列清单与理由。

### 6.4 边界

- aiVault 只收“嵌入式 / AI 量化 / 边缘推理”范畴内的知识；比赛流程、队内事务留在 RM。
- 拿不准的结论先写进 RM 的 issue，不直接改 aiVault。

## 7. 每次会话的固定流程

1. **开始**：读 `PROGRESS.md`；需要领域知识时读 aiVault `INDEX.md` 对应条目。
2. **过程**：问题 → issues，决策 → decisions，进度变化 → PROGRESS。
3. **结束**：
   - 更新 `PROGRESS.md`；
   - RM：`git add` 明确路径 → commit → push；
   - aiVault：有改动则按规则落页/标记 → LOG → 按路径 commit → push；
   - 汇报：改了什么、推到哪、下一步。
4. **跨库**：新结论先判断“项目特例还是通用知识”；通用才沉淀，避免把流水账写进 aiVault。

## 8. 已确认的决策（2026-10-17）

| 事项 | 结论 |
| :--- | :--- |
| 仓库可见性 | 私有 |
| 约 9.7 GiB 镜像 | 不入库，记录 SHA256 与获取方式 |
| 官方 PDF | 加密原件 + 未加密版都入库 |
| aiVault 纠错 | 可直接修正并按 §6.1 标记；删除类先报批 |
| `push_ai.bat` | 保持原样，不修改 |
| aiVault `inbox/` | 本阶段不处理、不提交 |
| 互链 | 使用带 commit SHA 的永久链接 |
| RM 与 aiVault 关系 | 只放链接，不复述知识正文 |

## 9. 已知边界

- 本会话文件沙箱默认可写 `E:\RM`；写 aiVault 需要一次授权提升。
- 沙箱内 git 凭据助手受命名管道限制，推送需要一次权限提升；若失败，改用普通终端执行。
- 官方资料版本：RM2027 V1.0.0（20261015）；后续版本发布后需重新核对并更新 `environment.md`。
