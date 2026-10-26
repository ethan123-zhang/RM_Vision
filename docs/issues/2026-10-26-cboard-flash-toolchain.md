# C 板烧不进去：ULINK/器件包/CubeProgrammer 三重坑

- 日期：2026-10-26
- 环境：C 板 STM32F407IGH6、ST-Link V2 克隆（FW V2J37S7）、Keil MDK 5.38 破解版（AC5）、
  STM32CubeProgrammer 2.23（`D:\STM32软件`）、Windows 10
- 关联：`cboard_chassis/`、`cboard_chassis/README.md` §7、commit `6e8cce2` 之后

## 现象

C 板固件一直烧不进去，表现为四种不同的错误，容易互相误导：

```
# 1. UV4 -f（器件包缺失时）
（退出码 0，日志 0 字节，什么都不做）

# 2. CubeProgrammer
ST-LINK SN : 5 / Voltage : 3.27V / 然后 exit=1，没有 Device ID

# 3. UV4 -f（器件包装好之后）
Load "chassis_task\chassis_task.axf"
Internal DLL Error
Error: Flash Download failed - Target DLL has been cancelled

# 4. Keil 界面按 F8（Debug 选成 ULINK 时）
ULINK2/ME - Cortex-M Error: No ULINK2/ME Device found
```

## 排查过程（按时间，含被否定的方向）

| 假设 | 验证 | 结论 |
| :--- | :--- | :--- |
| SWD 线接错 | 用户按手册重接、换线、换 ST-Link、换三块板 | ❌ 全部无效，但不是原因 |
| 板子没上电 | RGB 灯亮、XTCC 上电 | ❌ |
| 缺 F4 器件包 | Pack Installer 只有 `STM32F1xx_DFP` | ✅ 真因之一（导致 `-f` 静默无动作） |
| 包下不了 | Pack Installer 报 `Cannot connect to server https://www.keil.com/pack/` | 机器不能上外网；后来用户从别处装上了 2.17.1 |
| CubeProgrammer 能判断硬件好坏 | 同一时刻 Keil 烧 C8T6 成功、Cube 仍报无 Device ID | ❌ **Cube 全程不可信** |
| UV4 -f 可以脚本化 | 多次运行：`Internal DLL Error` 或 `exit=0xC0000409`（栈溢出崩溃） | ❌ 破解版批处理模式不可用 |
| F8 报 ULINK 是线的问题 | 打开 Options → Utilities 看到 `-- Use Debug Driver --`，Debug 页选的是 ULINK2/ME | ✅ 真因之二 |

## 根因

三个独立问题叠在一起：

1. **Keil 缺 `Keil.STM32F4xx_DFP`** → 没有 `STM32F4xx_1024.FLM` 算法，`UV4 -f` 静默不动。
2. **Debug 选项卡选成了 `ULINK2/ME Cortex Debugger`** → F8 去找 ULINK，报 `No ULINK2/ME Device found`。
   Utilities 页显示 `-- Use Debug Driver --` 时，烧写驱动**跟随 Debug 页**，改 Utilities 没用。
3. **STM32CubeProgrammer 与这颗 ST-Link 不兼容** → 能读 SN/FW/电压，但建立不了 SWD，
   报"无 Device ID"，把排查方向带偏一整轮。

另外：**破解版 Keil 的 `UV4 -f` 批处理会崩**（`0xC0000409`），所以命令行烧写这条路本身不通。

## 解决

1. 装上 `Keil.STM32F4xx_DFP 2.17.1`（Pack Installer 或离线导入）。
2. `Options for Target → Debug → Use:` 改 **ST-Link Debugger**（不是 ULINK2/ME）。
3. 烧写**只用 Keil 界面按 F8**；VS Code 只负责编译（`.vscode/tasks.json` 的"编译"任务）。
4. 禁用 STM32CubeProgrammer，不再用它烧录，也不再用它的输出判断硬件好坏。

工作配置保存在 `chassis_task.uvoptx`（用户级），关键片段：

```
<pMon>STLink\ST-LINKIII-KEIL_SWO.dll</pMon>
... -N00("ARM CoreSight SW-DP") -D00(2BA01477)
    -FF0STM32F4xx_1024.FLM -FS08000000 -FL0100000
    -FP0($$Device:STM32F407IGHx$CMSIS\Flash\STM32F4xx_1024.FLM)
```

## 验证

Keil F8 输出：

```
Load "chassis_task\chassis_task.axf"
Erase Done.
Programming Done.
Verify OK.
Application running ...
```

烧写后 RGB 灯流动、固件可运行（阶段 0 待按 KEY 验证电调编号）。

## 可复用结论

- **先看工具链，再看接线**：同一个 ST-Link，Keil 能烧、CubeProgrammer 不能，
  说明"认不到芯片"不一定是硬件问题。
- **`UV4 -f` 静默无动作 = 缺器件包**；**报 `Internal DLL Error` = 驱动/算法配置不对**；
  **报 `No ULINK2/ME Device found` = Debug 页选错了调试器**。
- 这台机器上：**烧写 = Keil F8**，其他一切"命令行烧写"方案都不要碰。
