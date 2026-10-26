<#
  flash.ps1 —— 编译（+ 打开 Keil 按 F8 烧写）

  用法：
    pwsh -File cboard_chassis\tools\flash.ps1            # 编译 + 打开 Keil 工程
    pwsh -File cboard_chassis\tools\flash.ps1 -BuildOnly # 只编译
    pwsh -File cboard_chassis\tools\flash.ps1 -OpenOnly  # 只打开 Keil（不编译）

  为什么烧写要回 Keil 界面按 F8（2026-10-26 实测结论）：
    - 本机 Keil 是破解版，`UV4 -f` 批处理模式必崩（exit=0xC0000409，无日志）
    - STM32CubeProgrammer 在这台机器上无法与 ST-Link 建立 SWD 连接（Keil 能，它不能）
    - 实测唯一可靠的烧写路径：Keil 界面里按 F8
    - Keil 的 Debug 选项卡必须选 ST-Link Debugger，选成 ULINK2/ME 时 F8 会报
      "No ULINK2/ME Device found"

  ⚠ 不要用 STM32CubeProgrammer 烧录（含它的 DFU 模式）—— 见 README §7。
#>
param(
    [switch]$BuildOnly,
    [switch]$OpenOnly,
    [string]$Uv4 = $env:KEIL_UV4
)

$ErrorActionPreference = 'Stop'

$root   = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$prj    = Join-Path $root 'MDK-ARM\chassis_task.uvprojx'
$target = 'chassis_task'
$hex    = Join-Path $root 'MDK-ARM\chassis_task\chassis_task.hex'
$logBuild = Join-Path $root 'MDK-ARM\build.log'

function Find-Uv4 {
    param([string]$Hint)
    if ($Hint -and (Test-Path -LiteralPath $Hint)) { return $Hint }
    foreach ($c in @('D:\Keil5\UV4\UV4.exe', 'C:\Keil_v5\UV4\UV4.exe',
                     'C:\Keil\UV4\UV4.exe', 'D:\Keil_v5\UV4\UV4.exe',
                     'C:\Program Files\Keil_v5\UV4\UV4.exe')) {
        if (Test-Path -LiteralPath $c) { return $c }
    }
    return $null
}

$uv4 = Find-Uv4 -Hint $Uv4
if (-not $uv4) {
    Write-Host '找不到 UV4.exe（Keil）。用 -Uv4 指定，或设环境变量 KEIL_UV4。' -ForegroundColor Red
    exit 1
}

# ---------------------------------------------------------------- 编译
if (-not $OpenOnly) {
    # Keil 界面里按过 F8 之后会生成 MDK-ARM\DebugConfig\，会让 UV4 的 -b 批处理崩溃
    # （exit=0xC0000409）。它是调试会话的产物，删掉不影响工程，Keil 需要时会重建。
    $dbgCfg = Join-Path $root 'MDK-ARM\DebugConfig'
    if (Test-Path -LiteralPath $dbgCfg) {
        Remove-Item -LiteralPath $dbgCfg -Recurse -Force -ErrorAction SilentlyContinue
    }

    if (Test-Path -LiteralPath $logBuild) { Remove-Item -LiteralPath $logBuild -Force }
    Write-Host "--- 编译 ---" -ForegroundColor Cyan
    Write-Host "  UV4: $uv4"

    $uv4Args = '-b "{0}" -j0 -t "{1}" -o "{2}"' -f $prj, $target, $logBuild
    $p = Start-Process -FilePath $uv4 -ArgumentList $uv4Args -Wait -PassThru

    if (Test-Path -LiteralPath $logBuild) {
        Get-Content -LiteralPath $logBuild |
            Select-String -Pattern 'error|warning|Program Size|Error\(s\)' |
            ForEach-Object { '  ' + $_.Line.Trim() }
    }
    # UV4 退出码：0 无错无警 / 1 有警告 / 2 有错误 / 3 致命
    if ($p.ExitCode -ge 2) {
        Write-Host "编译失败（exit=$($p.ExitCode)），看 $logBuild" -ForegroundColor Red
        exit 1
    }
    Write-Host '编译通过' -ForegroundColor Green
}

if ($BuildOnly) {
    if (Test-Path -LiteralPath $hex) { Write-Host "产物：$hex" }
    exit 0
}

# ---------------------------------------------------------------- 打开 Keil 烧写
Write-Host "`n--- 打开 Keil 工程 ---" -ForegroundColor Cyan
Start-Process -FilePath $uv4 -ArgumentList ('"{0}"' -f $prj) | Out-Null

Write-Host ''
Write-Host '在 Keil 里按 F8 烧写（本机唯一可靠的路径）。' -ForegroundColor Green
Write-Host '  成功标志：Erase Done / Programming Done / Verify OK / Application running' -ForegroundColor DarkGray
Write-Host ''
Write-Host '注意：' -ForegroundColor Yellow
Write-Host '  - 不要用 STM32CubeProgrammer 烧录（本机与 ST-Link 不兼容）' -ForegroundColor Yellow
Write-Host '  - 不要用 UV4 -f 命令行烧写（破解版必崩）' -ForegroundColor Yellow
Write-Host '  - Debug 选项卡必须是 ST-Link Debugger，不是 ULINK2/ME' -ForegroundColor Yellow
