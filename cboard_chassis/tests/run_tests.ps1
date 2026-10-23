# run_tests.ps1 —— 底盘调车纯逻辑自测（不需要板子、不需要 Keil，只要 gcc）
#
#   pwsh -File cboard_chassis\tests\run_tests.ps1
#
# 同一份测试用四套标定表各编译一次，确认"换表只改数据、不改行为"：
#   default   1/2/3/4 + 全 +1
#   permuted  3/4/1/2 + {+,-,+,-}
#   dup       1/1/3/4      -> 必须判非法
#   badid     1/2/9/4      -> 必须判非法
#
$ErrorActionPreference = 'Stop'

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$src  = Join-Path (Split-Path -Parent $here) 'application'
$build = Join-Path $here 'build'
$test = Join-Path $here 'test_bringup_map.c'

$gcc = (Get-Command gcc -ErrorAction SilentlyContinue)
if (-not $gcc) {
    Write-Host '  找不到 gcc，跳过纯逻辑自测' -ForegroundColor Yellow
    exit 2
}
$gcc = $gcc.Source

if (Test-Path $build) { Remove-Item -Recurse -Force $build }
New-Item -ItemType Directory -Force -Path $build | Out-Null

$variants = @(
    @{ name = 'default';  defs = @('CONFIG_DEFAULT') },
    @{ name = 'permuted'; defs = @('CONFIG_PERMUTED',
                                  'CHASSIS_WHEEL_CAN_ID_0=3', 'CHASSIS_WHEEL_CAN_ID_1=4',
                                  'CHASSIS_WHEEL_CAN_ID_2=1', 'CHASSIS_WHEEL_CAN_ID_3=2',
                                  'CHASSIS_WHEEL_DIR_0=1', 'CHASSIS_WHEEL_DIR_1=-1',
                                  'CHASSIS_WHEEL_DIR_2=1', 'CHASSIS_WHEEL_DIR_3=-1') },
    @{ name = 'dup';      defs = @('CONFIG_DUP', 'CHASSIS_WHEEL_CAN_ID_1=1') },
    @{ name = 'badid';    defs = @('CONFIG_BADID', 'CHASSIS_WHEEL_CAN_ID_2=9') }
)

$failed = 0
foreach ($v in $variants) {
    $exe = Join-Path $build ("test_" + $v.name + '.exe')
    $gccArgs = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-I', $src, $test,
                 (Join-Path $src 'chassis_bringup_map.c'), '-o', $exe) +
               ($v.defs | ForEach-Object { "-D$_" })

    Write-Host ("--- [{0}] 编译" -f $v.name) -ForegroundColor Cyan
    & $gcc @gccArgs
    if ($LASTEXITCODE -ne 0) {
        Write-Host ("  [{0}] 编译失败" -f $v.name) -ForegroundColor Red
        $failed++
        continue
    }
    & $exe
    if ($LASTEXITCODE -ne 0) { $failed++ }
}

Write-Host ''
if ($failed -ne 0) {
    Write-Host ("RESULT: FAIL（{0} 组失败）" -f $failed) -ForegroundColor Red
    exit 1
}
Write-Host 'RESULT: PASS（4/4 组）' -ForegroundColor Green
