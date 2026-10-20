# run_tests.ps1 —— 全部离线检查（不需要板子、不需要 Keil）
#
#   pwsh -File car_slam_lap\tests\run_tests.ps1
#
# 四组：
#   1. C 单元测试：protocol.c（70 项）+ slcan.c（21 项）
#   2. 固件模块语法/类型检查：用 tests\hal_stub 的 HAL 替身跑 gcc -fsyntax-only
#   3. Python 单元测试：协议黄金向量 + 轨迹文件往返
#   4. Keil 工程一致性：文件在磁盘上有、在 uvprojx 里挂着、include 路径和 .ioc 参数对
$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$fw   = Join-Path $root 'firmware\C8T6_DT7_car'
$bld  = Join-Path $root 'tests\build'
$fail = 0

if (-not (Get-Command gcc -ErrorAction SilentlyContinue)) {
    throw '找不到 gcc（需要 MinGW 或任意 host 编译器）'
}
New-Item -ItemType Directory -Path $bld -Force | Out-Null

# ---------------------------------------------------------------- 1. C 单元测试
Write-Host '=== [1/5] C 单元测试 ===' -ForegroundColor Cyan

$cTests = @(
    @{ Name = 'protocol_test'; Src = 'tests\protocol_test.c'; Inc = "$fw\Hardware\LINK"; Lib = "$fw\Hardware\LINK\protocol.c" },
    @{ Name = 'slcan_test';    Src = 'tests\slcan_test.c';    Inc = "$fw\Hardware\CAN";  Lib = "$fw\Hardware\CAN\slcan.c" }
)
foreach ($t in $cTests) {
    $exe = Join-Path $bld "$($t.Name).exe"
    gcc -std=gnu99 -Wall -Wextra -Werror -O1 -I $t.Inc `
        (Join-Path $root $t.Src) $t.Lib -o $exe
    if ($LASTEXITCODE -ne 0) { throw "$($t.Name) 编译失败" }
    & $exe | Select-Object -Last 2
    if ($LASTEXITCODE -ne 0) { $fail = 1 }
}

# ---------------------------------------------------------------- 2. 固件语法检查
Write-Host "`n=== [2/5] 固件模块语法/类型检查（HAL 替身）===" -ForegroundColor Cyan
$stub = Join-Path $root 'tests\hal_stub'
$fwFiles = @(
    "$fw\Core\Src\main.c",
    "$fw\Core\Src\stm32f1xx_it.c",
    "$fw\Hardware\LINK\protocol.c",
    "$fw\Hardware\CAN\slcan.c",
    "$fw\Hardware\LINK\x5_link.c",
    "$fw\Hardware\CAN\can_bus.c",
    "$fw\Hardware\UART\DBUS.c",
    "$fw\Hardware\UART\uart_rx.c"
)
gcc -std=gnu99 -Wall -Wextra -Werror -fsyntax-only `
    -I $stub -I "$fw\Core\Inc" -I "$fw\Hardware\LINK" -I "$fw\Hardware\CAN" -I "$fw\Hardware\UART" `
    @fwFiles
if ($LASTEXITCODE -ne 0) { $fail = 1; Write-Host '  语法检查失败' }
else { Write-Host '  8 个模块（含 main.c / stm32f1xx_it.c）全部通过 -Wall -Wextra -Werror' }

# ---------------------------------------------------------------- 3. Python 测试
Write-Host "`n=== [3/5] Python 单元测试 ===" -ForegroundColor Cyan
foreach ($t in @('test_protocol.py', 'test_path_io.py')) {
    Write-Host "--- $t"
    & python (Join-Path $root "x5\car_bridge\test\$t") | Select-Object -Last 1
    if ($LASTEXITCODE -ne 0) { $fail = 1 }
}

Write-Host '--- 语法检查（不导入 rclpy）'
& python -m compileall -q (Join-Path $root 'x5\car_bridge\car_bridge')
if ($LASTEXITCODE -ne 0) { $fail = 1; Write-Host '  语法检查失败' } else { Write-Host '  所有 .py 语法 ok' }

# ---------------------------------------------------------------- 4. 工程一致性
Write-Host "`n=== [4/5] Keil 工程一致性 ===" -ForegroundColor Cyan
& python (Join-Path $root 'tests\check_keil_project.py')
if ($LASTEXITCODE -ne 0) { $fail = 1 }

# ---------------------------------------------------------------- 5. 真实编译
Write-Host "`n=== [5/5] Keil 编译（装了 UV4 才跑）===" -ForegroundColor Cyan
$uv4 = @('D:\Keil5\UV4\UV4.exe', 'C:\Keil_v5\UV4\UV4.exe', 'C:\Keil\UV4\UV4.exe') |
    Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $uv4) {
    Write-Host '  没找到 UV4.exe，跳过（本机无 Keil 时只能靠第 2、4 项兜底）'
} else {
    $prj = Join-Path $fw 'MDK-ARM\C8T6_DT7_car.uvprojx'
    $klog = Join-Path $bld 'keil_build.log'
    if (Test-Path $klog) { Remove-Item $klog -Force }
    # UV4 是 GUI 子程序，& 调用不会等待，必须 Start-Process -Wait
    $p = Start-Process -FilePath $uv4 -Wait -PassThru -ArgumentList @(
        '-b', $prj, '-j0', '-t', 'C8T6_DT7_car', '-o', $klog)
    $text = if (Test-Path $klog) { Get-Content $klog -Raw } else { '' }
    $sizeLine = ($text -split "`r?`n" | Where-Object { $_ -match 'Program Size' } | Select-Object -Last 1)
    $sumLine = ($text -split "`r?`n" | Where-Object { $_ -match '\d+ Error\(s\)' } | Select-Object -Last 1)
    if ($sizeLine) { Write-Host "  $($sizeLine.Trim())" }
    if ($sumLine) { Write-Host "  $($sumLine.Trim())" }
    if ($p.ExitCode -ne 0 -or $text -notmatch '- 0 Error\(s\)') {
        $fail = 1
        Write-Host '  编译失败，日志：' -ForegroundColor Red
        ($text -split "`r?`n" | Where-Object { $_ -match 'Error|error' } | Select-Object -First 8) |
            ForEach-Object { Write-Host "    $_" }
    }
}

if ($fail -ne 0) {
    Write-Host "`n结果: FAIL" -ForegroundColor Red
    exit 1
}
Write-Host "`n结果: PASS" -ForegroundColor Green
