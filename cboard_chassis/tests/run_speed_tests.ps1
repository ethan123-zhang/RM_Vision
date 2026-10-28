$ErrorActionPreference = 'Stop'
$src = Join-Path $PSScriptRoot '..\application'
$exe = Join-Path $env:TEMP 'rm_chassis_speed_test.exe'
& gcc -std=c99 -Wall -Wextra -Werror -I $src (Join-Path $PSScriptRoot 'test_speed.c') (Join-Path $src 'chassis_bringup_map.c') -o $exe
if ($LASTEXITCODE -ne 0) { exit 1 }
& $exe
exit $LASTEXITCODE
