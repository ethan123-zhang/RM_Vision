param(
  [string]$TitleMatch = "Gazebo Sim",
  [string]$Out = "E:\RM\tb3_gazebo_nav2\rm_field\shots\gui_field_final.png",
  [int]$HoldMs = 1500,
  [switch]$NoRestoreFocus
)
# Find the WSLg Gazebo window, raise it, grab its rectangle, restore previous focus.
$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Drawing
Add-Type -Namespace GS -Name Api -MemberDefinition @'
public delegate bool EnumProc(IntPtr h, IntPtr l);
[DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc cb, IntPtr l);
[DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
[DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowText(IntPtr h, System.Text.StringBuilder s, int n);
[DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
[DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();
[DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
[DllImport("user32.dll")] public static extern bool BringWindowToTop(IntPtr h);
[DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int cmd);
[DllImport("user32.dll")] public static extern bool IsIconic(IntPtr h);
public struct RECT { public int L, T, R, B; }
'@

$script:cands = New-Object System.Collections.ArrayList
$cb = [GS.Api+EnumProc]{
  param($h, $l)
  if ([GS.Api]::IsWindowVisible($h)) {
    $sb = New-Object System.Text.StringBuilder 512
    [void][GS.Api]::GetWindowText($h, $sb, 512)
    if ($sb.ToString() -like "*$TitleMatch*") {
      $r = New-Object GS.Api+RECT
      [void][GS.Api]::GetWindowRect($h, [ref]$r)
      [void]$script:cands.Add([pscustomobject]@{
        H = $h; T = $sb.ToString(); L = $r.L; Tp = $r.T
        W = ($r.R - $r.L); Ht = ($r.B - $r.T); Iconic = [GS.Api]::IsIconic($h)
      })
    }
  }
  return $true
}
[void][GS.Api]::EnumWindows($cb, [IntPtr]::Zero)
if ($script:cands.Count -eq 0) { Write-Output "no Gazebo window found"; exit 1 }
foreach ($c in $script:cands) {
  Write-Output ("cand: {0} {1}x{2} at ({3},{4}) iconic={5}" -f $c.T, $c.W, $c.Ht, $c.L, $c.Tp, $c.Iconic)
}
# Prefer on-screen, non-iconic, largest
$win = $script:cands | Where-Object { -not $_.Iconic -and $_.L -gt -5000 -and $_.W -gt 300 } |
       Sort-Object W -Descending | Select-Object -First 1
if (-not $win) {
  $win = $script:cands | Where-Object { $_.W -gt 300 } | Sort-Object W -Descending | Select-Object -First 1
  if ($win) { [void][GS.Api]::ShowWindow($win.H, 9); Start-Sleep -Milliseconds 500 }
}
if (-not $win) { Write-Output "no usable window"; exit 2 }

$prev = [GS.Api]::GetForegroundWindow()
[void][GS.Api]::BringWindowToTop($win.H)
[void][GS.Api]::SetForegroundWindow($win.H)
Start-Sleep -Milliseconds $HoldMs

$r = New-Object GS.Api+RECT
[void][GS.Api]::GetWindowRect($win.H, [ref]$r)
$w = $r.R - $r.L; $ht = $r.B - $r.T
Write-Output ("capture: rect=({0},{1}) {2}x{3}" -f $r.L, $r.T, $w, $ht)
$bmp = New-Object System.Drawing.Bitmap($w, $ht)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen($r.L, $r.T, 0, 0, $bmp.Size)
New-Item -ItemType Directory -Force -Path (Split-Path $Out) | Out-Null
$bmp.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
$g.Dispose(); $bmp.Dispose()
if (-not $NoRestoreFocus) { [void][GS.Api]::SetForegroundWindow($prev) }
$kb = [int]((Get-Item $Out).Length / 1KB)
Write-Output ("SAVED {0}  {1}x{2}  {3} KB" -f $Out, $w, $ht, $kb)