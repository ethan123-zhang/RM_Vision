# run_tests.ps1 —— 离线测试：协议层 C + Python 两侧，不需要硬件
#
#   pwsh -File car_slam_lap\tests\run_tests.ps1
#
# 做两件事：
#   1. gcc 编译 firmware/.../LINK/protocol.c + tests/protocol_test.c，跑 70 项断言
#   2. python 跑 x5/car_bridge/test/test_protocol.py，跑 11 项（同一组黄金向量）
$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$inc  = Join-Path $root 'firmware\C8T6_DT7_car\Hardware\LINK'
$bld  = Join-Path $root 'tests\build'
$exe  = Join-Path $bld 'protocol_test.exe'
$fail = 0

if (-not (Get-Command gcc -ErrorAction SilentlyContinue)) {
    throw '找不到 gcc（需要 MinGW 或任意 host 编译器）'
}

Write-Host '=== [1/2] C 协议测试 ===' -ForegroundColor Cyan
New-Item -ItemType Directory -Path $bld -Force | Out-Null
gcc -std=gnu99 -Wall -Wextra -Werror -O1 -I $inc `
    (Join-Path $root 'tests\protocol_test.c') `
    (Join-Path $inc 'protocol.c') -o $exe
if ($LASTEXITCODE -ne 0) { throw 'C 编译失败' }
& $exe | Select-Object -Last 4
if ($LASTEXITCODE -ne 0) { $fail = 1 }

Write-Host "`n=== [2/2] Python 测试 ===" -ForegroundColor Cyan
$pyTests = @('test_protocol.py', 'test_path_io.py')
foreach ($t in $pyTests) {
    Write-Host "--- $t"
    & python (Join-Path $root "x5\car_bridge\test\$t") | Select-Object -Last 2
    if ($LASTEXITCODE -ne 0) { $fail = 1 }
}

Write-Host "`n=== [附加] Python 语法检查（不导入 rclpy，只查语法）===" -ForegroundColor Cyan
& python -m compileall -q (Join-Path $root 'x5\car_bridge\car_bridge')
if ($LASTEXITCODE -ne 0) { $fail = 1; Write-Host '语法检查失败' } else { Write-Host '  所有 .py 语法 ok' }

if ($fail -ne 0) {
    Write-Host "`n结果: FAIL" -ForegroundColor Red
    exit 1
}
Write-Host "`n结果: PASS" -ForegroundColor Green
