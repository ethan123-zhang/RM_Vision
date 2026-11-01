param(
  [string]$TitleMatch = "Gazebo Sim",
  [string]$Out = "E:\RM\tb3_gazebo_nav2\rm_field\shots\win_printwindow.png"
)
# 用 PrintWindow 抓指定窗口（不抢焦点，不影响用户正在用的窗口）
$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Drawing
Add-Type -Namespace PW -Name Api -MemberDefinition @'
[DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr hdc, uint flags);
[DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
[DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
public struct RECT { public int L, T, R, B; }
'@

$proc = Get-Process | Where-Object { $_.MainWindowHandle -ne 0 -and $_.MainWindowTitle -like "*$TitleMatch*" } | Select-Object -First 1
if (-not $proc) { "没找到标题含 '$TitleMatch' 的窗口"; exit 1 }
$h = $proc.MainWindowHandle
$r = New-Object PW.Api+RECT
[void][PW.Api]::GetWindowRect($h, [ref]$r)
$w = $r.R - $r.L; $ht = $r.B - $r.T
"窗口: $($proc.MainWindowTitle)  handle=$h  size=${w}x${ht}  visible=$([PW.Api]::IsWindowVisible($h))"

$bmp = New-Object System.Drawing.Bitmap($w, $ht)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$hdc = $g.GetHdc()
# flags: 0=普通, 2=PW_RENDERFULLCONTENT
$ok = [PW.Api]::PrintWindow($h, $hdc, 2)
$g.ReleaseHdc($hdc)
$g.Dispose()
New-Item -ItemType Directory -Force -Path (Split-Path $Out) | Out-Null
$bmp.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
$bmp.Dispose()
"PrintWindow(flag=2) -> $ok  saved=$Out  $([int]((Get-Item $Out).Length/1KB)) KB"