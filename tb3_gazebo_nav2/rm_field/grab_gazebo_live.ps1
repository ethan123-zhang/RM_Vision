param(
  [string]$TitleMatch = "Gazebo Sim",
  [string]$Out = "E:\RM\tb3_gazebo_nav2\rm_field\shots\win_gazebo_live.png",
  [int]$HoldMs = 1500,
  [switch]$ListOnly
)
# Capture the WSLg Gazebo window (title contains "Gazebo Sim") after raising it,
# then give focus back to whatever window the user had in front.
$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Drawing
Add-Type -Namespace GW -Name Api -MemberDefinition @'
public delegate bool EnumProc(IntPtr h, IntPtr l);
[DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc cb, IntPtr l);
[DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
[DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowText(IntPtr h, System.Text.StringBuilder s, int n);
[DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
[DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();
[DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
[DllImport("user32.dll")] public static extern bool BringWindowToTop(IntPtr h);
[DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int cmd);
[DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(IntPtr h, out uint pid);
public struct RECT { public int L, T, R, B; }
'@

$found = New-Object System.Collections.ArrayList
$cb = [GW.Api+EnumProc]{
  param($h, $l)
  if ([GW.Api]::IsWindowVisible($h)) {
    $sb = New-Object System.Text.StringBuilder 512
    [void][GW.Api]::GetWindowText($h, $sb, 512)
    $t = $sb.ToString()
    if ($t -like "*$TitleMatch*") {
      $r = New-Object GW.Api+RECT
      [void][GW.Api]::GetWindowRect($h, [ref]$r)
      $wp = 0
      [void][GW.Api]::GetWindowThreadProcessId($h, [ref]$wp)
      $pn = (Get-Process -Id $wp -ErrorAction SilentlyContinue).ProcessName
      $area = ($r.R - $r.L) * ($r.B - $r.T)
      [void]$found.Add([pscustomobject]@{
        Handle = $h; Title = $t; Proc = $pn; Pid = $wp; Area = $area
        L = $r.L; T = $r.T; W = ($r.R - $r.L); H = ($r.B - $r.T)
      })
    }
  }
  return $true
}
[void][GW.Api]::EnumWindows($cb, [IntPtr]::Zero)
if ($found.Count -eq 0) { Write-Output "no window matching '$TitleMatch'"; exit 1 }
foreach ($f in ($found | Sort-Object Area -Descending)) {
  Write-Output ("{0,-10} pid={1,-6} {2,4}x{3,-4} at ({4},{5})  area={6}  '{7}'" -f `
    $f.Proc, $f.Pid, $f.W, $f.H, $f.L, $f.T, $f.Area, $f.Title)
}
if ($ListOnly) { exit 0 }

$win = $found | Where-Object { $_.Area -gt 300000 -and $_.Proc -notmatch 'chrome|msedge|Code' } |
       Sort-Object Area -Descending | Select-Object -First 1
if (-not $win) { Write-Output "no large candidate window"; exit 1 }

$prev = [GW.Api]::GetForegroundWindow()
[void][GW.Api]::ShowWindow($win.Handle, 9)
[void][GW.Api]::BringWindowToTop($win.Handle)
[void][GW.Api]::SetForegroundWindow($win.Handle)
Start-Sleep -Milliseconds $HoldMs

$bmp = New-Object System.Drawing.Bitmap($win.W, $win.H)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen($win.L, $win.T, 0, 0, $bmp.Size)
New-Item -ItemType Directory -Force -Path (Split-Path $Out) | Out-Null
$bmp.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
$g.Dispose(); $bmp.Dispose()
[void][GW.Api]::SetForegroundWindow($prev)
$kb = [int]((Get-Item $Out).Length / 1KB)
Write-Output ("SAVED {0}  {1}x{2}  {3} KB  focus-restored" -f $Out, $win.W, $win.H, $kb)