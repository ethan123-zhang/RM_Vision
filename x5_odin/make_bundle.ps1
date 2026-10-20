# make_bundle.ps1 — 把板端需要的文件打成一个 zip，一次 pscp 传到 X5
#
# 用法：  pwsh -File x5_odin\make_bundle.ps1
# 产物：  x5_odin\odin_deploy.zip （不入库，可随时重新生成）
#
# 板端解包后：bash ~/odin_deploy/install_odin.sh
$ErrorActionPreference = 'Stop'

$here  = Split-Path -Parent $MyInvocation.MyCommand.Path
$repo  = Split-Path -Parent $here
$zip   = Join-Path $here 'odin_deploy.zip'
$stage = Join-Path $env:TEMP ('odin_deploy_' + [guid]::NewGuid().ToString('N').Substring(0, 8))

$scripts = @(
  'install_odin.sh', 'probe_env.sh', 'probe_deps.sh', 'inspect_build.sh',
  'run_odin.sh', 'start_all.sh', 'stop_all.sh',
  'odin_web_viewer.py', 'capture_one_frame.py',
  '99-odin-usb.rules'
)
$driver = Join-Path $repo '_downloads\odin_ros_driver_main.zip'

New-Item -ItemType Directory -Path $stage -Force | Out-Null
foreach ($f in $scripts) {
  $src = Join-Path $here $f
  if (-not (Test-Path $src)) { throw "缺少 $src" }
  Copy-Item $src $stage -Force
}
if (-not (Test-Path $driver)) { throw "缺少驱动源码包 $driver（从 _downloads 取，不入库）" }
Copy-Item $driver $stage -Force

if (Test-Path $zip) { Remove-Item $zip -Force }
Compress-Archive -Path (Join-Path $stage '*') -DestinationPath $zip
Remove-Item $stage -Recurse -Force

$kb = [math]::Round((Get-Item $zip).Length / 1KB)
Write-Host "bundle -> $zip  ($kb KB, $($scripts.Count + 1) 个文件)"
