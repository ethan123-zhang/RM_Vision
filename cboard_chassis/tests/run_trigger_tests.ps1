# run_trigger_tests.ps1 —— 拨弹盘台架模式纯逻辑自测（不需要板子、不需要 Keil，只要 gcc）
#
#   pwsh -File cboard_chassis\tests\run_trigger_tests.ps1
#
# 被测文件：application/trigger_bench_logic.c
# 覆盖：档位表、36:1 换算、7 s 斜坡时序、电流环限幅、超速/失速判据。
#
$ErrorActionPreference = 'Stop'

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$src  = Join-Path (Split-Path -Parent $here) 'application'
$build = Join-Path $here 'build'
$test = Join-Path $here 'test_trigger_bench.c'

$gcc = (Get-Command gcc -ErrorAction SilentlyContinue)
if (-not $gcc) {
    Write-Host '  找不到 gcc，跳过拨弹盘纯逻辑自测' -ForegroundColor Yellow
    exit 2
}
$gcc = $gcc.Source

if (-not (Test-Path $build)) { New-Item -ItemType Directory -Force -Path $build | Out-Null }
$exe = Join-Path $build 'test_trigger_bench.exe'

$gccArgs = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-I', $src, $test,
             (Join-Path $src 'trigger_bench_logic.c'), '-o', $exe, '-lm')

Write-Host '--- 编译拨弹盘台架逻辑测试' -ForegroundColor Cyan
& $gcc @gccArgs
if ($LASTEXITCODE -ne 0) {
    Write-Host '  编译失败' -ForegroundColor Red
    exit 1
}

& $exe
if ($LASTEXITCODE -ne 0) {
    Write-Host 'RESULT: FAIL' -ForegroundColor Red
    exit 1
}

Write-Host 'RESULT: PASS' -ForegroundColor Green
exit 0