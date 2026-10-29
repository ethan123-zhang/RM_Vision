# run_open_loop_tests.ps1 —— 固定电流开环模式的纯逻辑自测（不需要板子/Keil，只要 gcc）
#
#   pwsh -File cboard_chassis\tests\run_open_loop_tests.ps1
#
$ErrorActionPreference = 'Stop'
$src = Join-Path $PSScriptRoot '..\application'
$exe = Join-Path $env:TEMP 'rm_chassis_open_loop_test.exe'
& gcc -std=c99 -Wall -Wextra -Werror -I $src (Join-Path $PSScriptRoot 'test_open_loop.c') (Join-Path $src 'chassis_bringup_map.c') -o $exe
if ($LASTEXITCODE -ne 0) { exit 1 }
& $exe
exit $LASTEXITCODE
