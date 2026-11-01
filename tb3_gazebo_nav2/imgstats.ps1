param([string]$Image = "E:\RM\tb3_gazebo_nav2\shots\screen.png")
# 像素统计：无视觉引擎时可量化判断"窗口里到底有没有内容"
$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Drawing
$bmp = [System.Drawing.Bitmap]::FromFile($Image)
$w = $bmp.Width; $h = $bmp.Height
$colors = New-Object 'System.Collections.Generic.HashSet[int]'
$nonBlack = 0; $total = 0; $sumB = 0L
$step = 4
for ($y = 0; $y -lt $h; $y += $step) {
  for ($x = 0; $x -lt $w; $x += $step) {
    $c = $bmp.GetPixel($x, $y)
    $argb = $c.ToArgb()
    [void]$colors.Add($argb)
    $lum = [int](0.299*$c.R + 0.587*$c.G + 0.114*$c.B)
    $sumB += $lum
    if ($lum -gt 12) { $nonBlack++ }
    $total++
  }
}
$mean = [math]::Round($sumB / [double]$total, 1)
"文件: $Image"
"尺寸: ${w}x${h}   采样点: $total"
"不同颜色数: $($colors.Count)"
"非纯黑像素占比: {0:P1}" -f ($nonBlack / [double]$total)
"平均亮度: $mean"
# 4x4 分块亮度，看内容集中在屏幕哪个区域（判断窗口位置）
$rows = @()
for ($by = 0; $by -lt 4; $by++) {
  $line = @()
  for ($bx = 0; $bx -lt 4; $bx++) {
    $x0 = [int]($bx*$w/4); $y0 = [int]($by*$h/4)
    $s = 0L; $n = 0
    for ($y = $y0; $y -lt [math]::Min($y0 + $h/4, $h); $y += 12) {
      for ($x = $x0; $x -lt [math]::Min($x0 + $w/4, $w); $x += 12) {
        $c = $bmp.GetPixel($x, $y); $s += [int](0.299*$c.R + 0.587*$c.G + 0.114*$c.B); $n++
      }
    }
    $line += ("{0,5:N0}" -f ($s / [double]$n))
  }
  $rows += ("  " + ($line -join " "))
}
"4x4 分块平均亮度:"; $rows | ForEach-Object { $_ }
$bmp.Dispose()