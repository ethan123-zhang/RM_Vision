param(
  [string]$A = "E:\RM\tb3_gazebo_nav2\shots\win_gazebo.png",
  [string]$B = "E:\RM\tb3_gazebo_nav2\shots\win_gazebo_after.png"
)
# 比较两张窗口截图的差异比例：差异 >0 说明 3D 画面确实随仿真在刷新
$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Drawing
$ia = [System.Drawing.Bitmap]::FromFile($A)
$ib = [System.Drawing.Bitmap]::FromFile($B)
if ($ia.Width -ne $ib.Width -or $ia.Height -ne $ib.Height) {
  "尺寸不同: A=$($ia.Width)x$($ia.Height) B=$($ib.Width)x$($ib.Height)"
  $ia.Dispose(); $ib.Dispose(); return
}
$w = $ia.Width; $h = $ia.Height
$diff = 0; $total = 0; $maxDelta = 0
$buckets = @{}
for ($y = 0; $y -lt $h; $y += 3) {
  for ($x = 0; $x -lt $w; $x += 3) {
    $ca = $ia.GetPixel($x, $y); $cb = $ib.GetPixel($x, $y)
    $d = [math]::Abs($ca.R-$cb.R) + [math]::Abs($ca.G-$cb.G) + [math]::Abs($ca.B-$cb.B)
    if ($d -gt 24) { $diff++ }
    if ($d -gt $maxDelta) { $maxDelta = $d }
    $k = [int]($d / 64); $buckets[$k] = 1 + [int]$buckets[$k]
    $total++
  }
}
"A: $A"
"B: $B"
"采样点: $total"
"明显变化像素占比: {0:P2}" -f ($diff/[double]$total)
"最大单点色差: $maxDelta (0-765)"
"差异分布(0..12 档, 每档色差64): " + (($buckets.Keys | Sort-Object | ForEach-Object { "$_=$($buckets[$_])" }) -join "  ")
$ia.Dispose(); $ib.Dispose()