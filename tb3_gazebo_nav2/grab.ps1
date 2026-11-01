param([string]$Out = "E:\RM\tb3_gazebo_nav2\shots\screen.png")
# 抓取 Windows 整个虚拟桌面（WSLg 的 Gazebo/RViz 窗口也在其中）
$ErrorActionPreference = "Stop"
New-Item -ItemType Directory -Force -Path (Split-Path $Out) | Out-Null
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
$vs = [System.Windows.Forms.SystemInformation]::VirtualScreen
$bmp = New-Object System.Drawing.Bitmap($vs.Width, $vs.Height)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen($vs.X, $vs.Y, 0, 0, $bmp.Size)
$bmp.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
$g.Dispose(); $bmp.Dispose()
$fi = Get-Item $Out
"SAVED {0}  {1}x{2}  {3} KB" -f $fi.FullName, $vs.Width, $vs.Height, [int]($fi.Length/1KB)