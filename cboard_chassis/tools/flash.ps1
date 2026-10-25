<#
  flash.ps1 —— 命令行编译 + SWD 下载（不用打开 Keil 的窗口）

  用法：
    pwsh -File cboard_chassis\tools\flash.ps1              # 编译 + 下载
    pwsh -File cboard_chassis\tools\flash.ps1 -BuildOnly   # 只编译
    pwsh -File cboard_chassis\tools\flash.ps1 -FlashOnly   # 只下载（用上次产物）
    pwsh -File cboard_chassis\tools\flash.ps1 -Probe       # 只探测 SWD 能不能认到芯片
    pwsh -File cboard_chassis\tools\flash.ps1 -Tool keil   # 强制用 Keil 下载
    pwsh -File cboard_chassis\tools\flash.ps1 -Tool cube   # 强制用 CubeProgrammer

  下载工具二选一（auto 时优先用找得到的 CubeProgrammer）：
    cube  STM32CubeProgrammer CLI —— 不依赖 Keil 器件包，推荐
    keil  UV4 -f —— 需要装 Keil.STM32F4xx_DFP（本机目前没装，见 README §7）
#>
param(
    [switch]$BuildOnly,
    [switch]$FlashOnly,
    [switch]$Probe,
    [ValidateSet('auto', 'cube', 'keil')]
    [string]$Tool = 'auto',
    [string]$Uv4 = $env:KEIL_UV4,
    [string]$CubeCli = $env:STM32_PROGRAMMER_CLI
)

$ErrorActionPreference = 'Stop'

$root  = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$prj   = Join-Path $root 'MDK-ARM\chassis_task.uvprojx'
$hex   = Join-Path $root 'MDK-ARM\chassis_task\chassis_task.hex'
$axf   = Join-Path $root 'MDK-ARM\chassis_task\chassis_task.axf'
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

function Find-Cube {
    param([string]$Hint)
    if ($Hint -and (Test-Path -LiteralPath $Hint)) { return $Hint }
    foreach ($c in @(
        'D:\STM32软件\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
        'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
        'C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
        'D:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe')) {
        if (Test-Path -LiteralPath $c) { return $c }
    }
    return $null
}

$uv4     = Find-Uv4  -Hint $Uv4
$cubeCli = Find-Cube -Hint $CubeCli

# ---------------------------------------------------------------- 探测
function Invoke-Probe {
    param($Cli)
    if (-not $Cli) { return $false }
    Write-Host "`n--- SWD 探测 ---" -ForegroundColor Cyan
    Write-Host "  $Cli"
    Push-Location (Split-Path -Parent $Cli)
    try {
        # 注意：这里必须 Write-Host，不能让输出进管道，否则会污染函数返回值
        & $Cli -c port=SWD 2>&1 | ForEach-Object { Write-Host ('  ' + $_) }
        $code = $LASTEXITCODE
    } finally { Pop-Location }

    if ($code -eq 0) {
        Write-Host 'SWD 正常：能认出芯片。' -ForegroundColor Green
        return $true
    }
    Write-Host 'SWD 失败：ST-Link 认到了，但芯片没应答。按顺序查：' -ForegroundColor Red
    Write-Host '  1. SWDIO / SWCLK 有没有接反（不能交叉），GND 是否共地；' -ForegroundColor Yellow
    Write-Host '  2. C 板是否上电（3.3V 灯亮），排线插的是 SWD 口不是串口口；' -ForegroundColor Yellow
    Write-Host '  3. 芯片跑飞了：按住 BOOT0 再点一下复位，让它停在 ROM 引导再重试；' -ForegroundColor Yellow
    Write-Host '  4. ST-Link 固件太旧：用 ST-LinkUpgrade 升级。' -ForegroundColor Yellow
    return $false
}

if ($Probe) {
    if (-not $cubeCli) { Write-Host '没找到 STM32_Programmer_CLI.exe。' -ForegroundColor Red; exit 1 }
    if (Invoke-Probe -Cli $cubeCli) { exit 0 } else { exit 1 }
}

# ---------------------------------------------------------------- 编译
if (-not $FlashOnly) {
    if (-not $uv4) {
        Write-Host '找不到 UV4.exe（Keil）。用 -Uv4 指定，或设环境变量 KEIL_UV4。' -ForegroundColor Red
        exit 1
    }
    if (Test-Path -LiteralPath $logBuild) { Remove-Item -LiteralPath $logBuild -Force }
    Write-Host "--- 编译 ---" -ForegroundColor Cyan
    Write-Host "  UV4: $uv4"

    $uv4Args = '-b "{0}" -j0 -o "{1}"' -f $prj, $logBuild
    $p = Start-Process -FilePath $uv4 -ArgumentList $uv4Args -Wait -PassThru

    if (Test-Path -LiteralPath $logBuild) {
        Get-Content -LiteralPath $logBuild |
            Select-String -Pattern 'error|warning|Program Size|Error\(s\)' |
            ForEach-Object { '  ' + $_.Line.Trim() }
    }
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

# ---------------------------------------------------------------- 下载
if (-not (Test-Path -LiteralPath $hex)) {
    Write-Host '没有 .hex，先编译一次（去掉 -FlashOnly）。' -ForegroundColor Red
    exit 1
}

$useCube = $false
if ($Tool -eq 'cube') { $useCube = $true }
elseif ($Tool -eq 'keil') { $useCube = $false }
else { $useCube = [bool]$cubeCli }

if ($useCube -and -not $cubeCli) {
    Write-Host '没找到 STM32_Programmer_CLI.exe，改用 Keil 下载。' -ForegroundColor Yellow
    $useCube = $false
}

if ($useCube) {
    Write-Host "`n--- 下载（STM32CubeProgrammer / SWD）---" -ForegroundColor Cyan
    Write-Host "  $cubeCli"
    # CubeProgrammer 必须在自己的 bin 目录下运行，否则找不到 Data_Base
    Push-Location (Split-Path -Parent $cubeCli)
    try {
        & $cubeCli -c port=SWD -w $hex -v -rst 2>&1 | ForEach-Object { '  ' + $_ }
        $code = $LASTEXITCODE
    } finally { Pop-Location }

    if ($code -ne 0) {
        Write-Host "下载失败（exit=$code）。先跑 -Probe 看 SWD 通不通。" -ForegroundColor Red
        exit 1
    }
    Write-Host '下载 + 校验完成，芯片已复位运行。' -ForegroundColor Green
    exit 0
}

# ---------------- Keil 下载（UV4 -f）----------------
if (-not $uv4) { Write-Host '找不到 UV4.exe。' -ForegroundColor Red; exit 1 }
$logFlash = Join-Path $root 'MDK-ARM\flash.log'
if (Test-Path -LiteralPath $logFlash) { Remove-Item -LiteralPath $logFlash -Force }

Write-Host "`n--- 下载（UV4 -f / ST-Link）---" -ForegroundColor Cyan
$uv4Args = '-f "{0}" -j0 -o "{1}"' -f $prj, $logFlash
$p = Start-Process -FilePath $uv4 -ArgumentList $uv4Args -Wait -PassThru

$logSize = 0
if (Test-Path -LiteralPath $logFlash) { $logSize = (Get-Item -LiteralPath $logFlash).Length }

if ($logSize -eq 0) {
    # 没有日志 = µVision 根本没走到下载这一步，通常是缺 STM32F4xx 器件包（没有 Flash 算法）
    Write-Host '  没有任何下载日志 —— 大概率是没装 Keil.STM32F4xx_DFP（没有 F4 的 Flash 算法）。' -ForegroundColor Yellow
    Write-Host '  改用 -Tool cube，或在 Pack Installer 里装 Keil::STM32F4xx_DFP。' -ForegroundColor Yellow
    exit 1
}

Get-Content -LiteralPath $logFlash | ForEach-Object { '  ' + $_.TrimEnd() }
if ($p.ExitCode -ne 0) {
    Write-Host "下载失败（exit=$($p.ExitCode)）" -ForegroundColor Red
    exit 1
}
Write-Host '下载完成。' -ForegroundColor Green
