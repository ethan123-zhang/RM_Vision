# verify_view.ps1 — 建立 SSH 端口转发并验证 Odin1 网页预览端到端可用
#
#   pwsh -File x5_odin\pc\verify_view.ps1 -X5Ip 172.17.90.239
#
# 做的事：
#   1. plink -L 8080:127.0.0.1:8080（板端 8080 → 本机 127.0.0.1:8080）
#   2. 等端口起来
#   3. 拉 /stats（JSON：帧数、点数、距离范围、渲染耗时）
#   4. 拉 /rgb.jpg /depth.jpg /overlay.jpg 存到 x5_odin\demo\live_*.jpg
#   5. 打印结论；-KeepTunnel（默认）保留隧道方便浏览器直接看
param(
    [string]$X5Ip = '172.17.90.239',
    [int]$LocalPort = 8080,
    [switch]$StopTunnel
)
$ErrorActionPreference = 'Stop'

$here  = Split-Path -Parent $MyInvocation.MyCommand.Path
$repo  = Split-Path -Parent (Split-Path -Parent $here)
$plink = Join-Path $repo '_downloads\putty\plink.exe'
$hostkey = 'SHA256:K8oyEoddaZ4keNu39exAqMJLDEcncgoQGcRPQ898Bb4'
$demo  = Join-Path (Split-Path -Parent $here) 'demo'
$base  = "http://127.0.0.1:$LocalPort"

if (-not (Test-Path $plink)) { throw "找不到 plink: $plink" }

# 端口已有人监听就复用，避免两个 plink 抢同一个本地端口
$proc = $null
$busy = Test-NetConnection -ComputerName 127.0.0.1 -Port $LocalPort -InformationLevel Quiet -WarningAction SilentlyContinue
if ($busy) {
    Write-Host "tunnel: 127.0.0.1:$LocalPort 已在监听，直接复用"
} else {
    $argList = @('-batch', '-hostkey', $hostkey, '-pw', 'sunrise',
                 '-L', "$LocalPort`:127.0.0.1:8080", '-N', "sunrise@${X5Ip}")
    $proc = Start-Process -FilePath $plink -ArgumentList $argList -PassThru -WindowStyle Hidden
    Write-Host "tunnel: pid $($proc.Id)  127.0.0.1:$LocalPort -> ${X5Ip}:8080"
}

# 等端口监听
$up = $false
for ($i = 0; $i -lt 20; $i++) {
    Start-Sleep -Milliseconds 500
    if (Test-NetConnection -ComputerName 127.0.0.1 -Port $LocalPort -InformationLevel Quiet -WarningAction SilentlyContinue) { $up = $true; break }
}
if (-not $up) { throw "127.0.0.1:$LocalPort 没有起来，检查板端 viewer 是否在跑、IP 是否正确" }

$stats = & curl.exe -s --noproxy '*' "$base/stats"
Write-Host "stats: $stats"

New-Item -ItemType Directory -Path $demo -Force | Out-Null
foreach ($n in 'rgb', 'depth', 'overlay') {
    $dst = Join-Path $demo "live_$n.jpg"
    & curl.exe -s --noproxy '*' -o $dst "$base/$n.jpg"
    $len = (Get-Item $dst).Length
    Write-Host ("  {0,-8} {1,8:N0} bytes  -> demo/live_{0}.jpg" -f $n, $len)
}

$j = $stats | ConvertFrom-Json
if ($j.n_rgb -gt 0 -and $j.n_cloud -gt 0) {
    Write-Host ""
    Write-Host "OK  RGB $($j.n_rgb) 帧 / 点云 $($j.n_cloud) 帧，点数 $($j.pts)，距离 $($j.range[0])-$($j.range[1]) m，渲染 rgb $([math]::Round($j.ms[0])) / depth $([math]::Round($j.ms[1])) ms"
    Write-Host "    浏览器打开 $base/"
} else {
    Write-Warning "帧数为 0：viewer 起来了但没收到数据，先看板端 ~/odin_run.log 和 ~/web_viewer.log"
}

if ($StopTunnel -and $proc) { Stop-Process -Id $proc.Id -Force; Write-Host "tunnel stopped" }

