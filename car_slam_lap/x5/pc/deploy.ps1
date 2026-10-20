# deploy.ps1 —— 把 X5 侧代码打包传到板卡，可选顺带跑安装
#
#   pwsh -File car_slam_lap\x5\pc\deploy.ps1                     # 只传包
#   pwsh -File car_slam_lap\x5\pc\deploy.ps1 -Install            # 传完直接装
#   pwsh -File car_slam_lap\x5\pc\deploy.ps1 -X5Ip 172.17.90.239 -HostKey 'SHA256:...'
#
# 产物：car_slam_lap\x5\pc\car_slam_lap_deploy.zip（不入库）
# 板端落到：/home/<user>/car_slam_lap/x5/...（保持仓库内相对结构，
#           这样 scripts/env.sh 里的 CAR_SRC 才能算对）
param(
    [string]$X5Ip = '172.17.90.239',
    [string]$User = 'sunrise',
    [string]$Password = 'sunrise',
    [string]$HostKey = 'SHA256:K8oyEoddaZ4keNu39exAqMJLDEcncgoQGcRPQ898Bb4',
    [switch]$Install
)
$ErrorActionPreference = 'Stop'

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$src  = Split-Path -Parent $here              # x5\
$repo = Split-Path -Parent $src               # car_slam_lap\
$putty = Join-Path (Split-Path -Parent $repo) '_downloads\putty'
$pscp  = Join-Path $putty 'pscp.exe'
$plink = Join-Path $putty 'plink.exe'
$zip   = Join-Path $here 'car_slam_lap_deploy.zip'
$stage = Join-Path $env:TEMP ('car_slam_lap_' + [guid]::NewGuid().ToString('N').Substring(0, 8))

foreach ($exe in @($pscp, $plink)) {
    if (-not (Test-Path $exe)) { throw "找不到 $exe（x5_odin 里用过的那套 putty 工具）" }
}

# 只打包板端需要的：car_bridge 包、两份 Odin1 配置、脚本
New-Item -ItemType Directory -Path (Join-Path $stage 'x5') -Force | Out-Null
Copy-Item (Join-Path $src 'car_bridge') (Join-Path $stage 'x5') -Recurse
Copy-Item (Join-Path $src 'config')     (Join-Path $stage 'x5') -Recurse
Copy-Item (Join-Path $src 'scripts')    (Join-Path $stage 'x5') -Recurse
# 去掉本地测试产物
Get-ChildItem $stage -Recurse -Directory -Filter '__pycache__' | Remove-Item -Recurse -Force

if (Test-Path $zip) { Remove-Item $zip -Force }
Compress-Archive -Path (Join-Path $stage 'x5') -DestinationPath $zip
Remove-Item $stage -Recurse -Force
$kb = [math]::Round((Get-Item $zip).Length / 1KB, 1)
Write-Host "包: $zip  ($kb KB)"

$remoteHome = "/home/$User"
Write-Host "上传到 ${User}@${X5Ip}:$remoteHome/ ..."
& $pscp -batch -hostkey $HostKey -pw $Password $zip "${User}@${X5Ip}:$remoteHome/"

if (-not $Install) {
    Write-Host @"

包已上传。板端执行：
  cd ~ && rm -rf car_slam_lap && mkdir -p car_slam_lap && unzip -o car_slam_lap_deploy.zip -d car_slam_lap
  bash ~/car_slam_lap/x5/scripts/install_car.sh
"@
    exit 0
}

Write-Host '板端解包 + 安装 ...'
$cmd = "cd ~ && rm -rf car_slam_lap && mkdir -p car_slam_lap && " +
       "unzip -o car_slam_lap_deploy.zip -d car_slam_lap && " +
       "bash ~/car_slam_lap/x5/scripts/install_car.sh"
& $plink -batch -hostkey $HostKey -pw $Password "${User}@${X5Ip}" $cmd
if ($LASTEXITCODE -ne 0) { throw "远程安装失败（exit $LASTEXITCODE），看上面的输出" }
Write-Host '安装完成。'
