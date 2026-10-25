<#
  flash.ps1 —— 命令行编译 + 烧写（不用打开 Keil 的界面，也不依赖 Keil 器件包）

  用法：
    pwsh -File cboard_chassis\tools\flash.ps1                # 编译 + 烧写（自动选路）
    pwsh -File cboard_chassis\tools\flash.ps1 -BuildOnly     # 只编译
    pwsh -File cboard_chassis\tools\flash.ps1 -FlashOnly     # 只烧写（用上次产物）
    pwsh -File cboard_chassis\tools\flash.ps1 -Tool dfu      # 强制 USB DFU
    pwsh -File cboard_chassis\tools\flash.ps1 -Tool keil     # 强制 UV4 -f（要 F4 器件包）
    pwsh -File cboard_chassis\tools\flash.ps1 -Probe         # 看 ST-Link 能不能认到芯片

  三条烧写路径：
    dfu   STM32CubeProgrammer 走 USB DFU —— 只用板子的 micro USB 口，
          **不需要 SWD、不需要 ST-Link、不需要 Keil 器件包**，推荐
    keil  UV4 -f —— 需要装了 Keil.STM32F4xx_DFP（本机没有，`-f` 会静默不动）
    cube  STM32CubeProgrammer 走 ST-Link/SWD —— 本机实测连不上，仅留作备选

  auto 规则：没有 F4 器件包 → 用 dfu；装了 → 用 keil。
#>
param(
    [switch]$BuildOnly,
    [switch]$FlashOnly,
    [switch]$Probe,
    [ValidateSet('auto', 'dfu', 'keil', 'cube')]
    [string]$Tool = 'auto',
    [string]$Uv4 = $env:KEIL_UV4,
    [string]$CubeCli = $env:STM32_PROGRAMMER_CLI
)

$ErrorActionPreference = 'Stop'

$root  = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$prj   = Join-Path $root 'MDK-ARM\chassis_task.uvprojx'
$target = 'chassis_task'
$hex   = Join-Path $root 'MDK-ARM\chassis_task\chassis_task.hex'
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

# F4 器件包（决定 keil 那条路能不能用）
$f4Pack = $null
if ($uv4) {
    $keilRoot = Split-Path -Parent (Split-Path -Parent $uv4)   # ...\Keil5
    $f4Pack = Join-Path $keilRoot 'ARM\PACK\Keil\STM32F4xx_DFP'
}
$hasF4Pack = ($f4Pack -and (Test-Path -LiteralPath $f4Pack))

# ---------------------------------------------------------------- 探测
if ($Probe) {
    if (-not $cubeCli) { Write-Host '没找到 STM32_Programmer_CLI.exe。' -ForegroundColor Red; exit 1 }
    Write-Host "`n--- ST-Link / SWD 探测 ---" -ForegroundColor Cyan
    Push-Location (Split-Path -Parent $cubeCli)
    try {
        & $cubeCli -c port=SWD 2>&1 | ForEach-Object { Write-Host ('  ' + $_) }
        $code = $LASTEXITCODE
    } finally { Pop-Location }
    if ($code -eq 0) { Write-Host 'SWD 正常。' -ForegroundColor Green; exit 0 }
    Write-Host 'SWD 不通。注意：本机的 CubeProgrammer 对 ST-Link 有兼容问题，' -ForegroundColor Yellow
    Write-Host '即使 Keil 能烧，这里也可能报错。烧写请直接用 -Tool dfu。' -ForegroundColor Yellow
    exit 1
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

    $uv4Args = '-b "{0}" -j0 -t "{1}" -o "{2}"' -f $prj, $target, $logBuild
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

# ---------------------------------------------------------------- 烧写
if (-not (Test-Path -LiteralPath $hex)) {
    Write-Host '没有 .hex，先编译一次（去掉 -FlashOnly）。' -ForegroundColor Red
    exit 1
}

# auto：优先 keil（如果装了 F4 包），否则 dfu
if ($Tool -eq 'auto') {
    if ($hasF4Pack) { $Tool = 'keil' }
    elseif ($cubeCli) { $Tool = 'dfu' }
    else {
        Write-Host '既没有 F4 器件包、也没有 CubeProgrammer，烧不了。' -ForegroundColor Red
        exit 1
    }
}

Write-Host "`n--- 烧写（$Tool）---" -ForegroundColor Cyan

switch ($Tool) {

    'dfu' {
        if (-not $cubeCli) { Write-Host '没找到 STM32_Programmer_CLI.exe。' -ForegroundColor Red; exit 1 }
        Write-Host "  $cubeCli"
        Write-Host '  要求：BOOT0 接 3V3、按过复位、micro USB 是数据线。' -ForegroundColor DarkGray

        Push-Location (Split-Path -Parent $cubeCli)
        try {
            & $cubeCli -c port=USB1 -w $hex -v 2>&1 | ForEach-Object { '  ' + $_ }
            $code = $LASTEXITCODE
        } finally { Pop-Location }

        if ($code -ne 0) {
            Write-Host 'DFU 失败。逐条查：' -ForegroundColor Red
            Write-Host '  1. BOOT0 是否真的接到 3V3（量一下是 3.3V）；' -ForegroundColor Yellow
            Write-Host '  2. 接完 BOOT0 有没有按复位；' -ForegroundColor Yellow
            Write-Host '  3. micro USB 是不是数据线（充电线不枚举）；' -ForegroundColor Yellow
            Write-Host '  4. 跑一次 STM32_Programmer_CLI -l，看 DFU Interface 下有没有设备。' -ForegroundColor Yellow
            exit 1
        }
        Write-Host '烧写 + 校验完成。' -ForegroundColor Green
        Write-Host '把 BOOT0 接回 GND，再按复位，程序才会运行。' -ForegroundColor Yellow
        exit 0
    }

    'keil' {
        if (-not $uv4) { Write-Host '找不到 UV4.exe。' -ForegroundColor Red; exit 1 }
        if (-not $hasF4Pack) {
            Write-Host '没装 Keil.STM32F4xx_DFP，UV4 -f 对 F4 是静默不动的（缺 Flash 算法）。' -ForegroundColor Red
            Write-Host '改用 -Tool dfu。' -ForegroundColor Yellow
            exit 1
        }
        $logFlash = Join-Path $root 'MDK-ARM\flash.log'
        if (Test-Path -LiteralPath $logFlash) { Remove-Item -LiteralPath $logFlash -Force }

        $uv4Args = '-f "{0}" -j0 -t "{1}" -o "{2}"' -f $prj, $target, $logFlash
        $p = Start-Process -FilePath $uv4 -ArgumentList $uv4Args -Wait -PassThru

        $logSize = 0
        if (Test-Path -LiteralPath $logFlash) { $logSize = (Get-Item -LiteralPath $logFlash).Length }
        if ($logSize -gt 0) {
            Get-Content -LiteralPath $logFlash | ForEach-Object { '  ' + $_.TrimEnd() }
        } else {
            Write-Host '  没有下载日志 —— µVision 没走到烧写这一步。' -ForegroundColor Yellow
            exit 1
        }
        if ($p.ExitCode -ne 0) { Write-Host "烧写失败（exit=$($p.ExitCode)）" -ForegroundColor Red; exit 1 }
        Write-Host '烧写完成。' -ForegroundColor Green
        exit 0
    }

    'cube' {
        if (-not $cubeCli) { Write-Host '没找到 STM32_Programmer_CLI.exe。' -ForegroundColor Red; exit 1 }
        Push-Location (Split-Path -Parent $cubeCli)
        try {
            & $cubeCli -c port=SWD -w $hex -v -rst 2>&1 | ForEach-Object { '  ' + $_ }
            $code = $LASTEXITCODE
        } finally { Pop-Location }
        if ($code -ne 0) {
            Write-Host 'CubeProgrammer 连不上 ST-Link（本机已知问题）。请改用 -Tool dfu。' -ForegroundColor Red
            exit 1
        }
        Write-Host '烧写完成。' -ForegroundColor Green
        exit 0
    }
}
