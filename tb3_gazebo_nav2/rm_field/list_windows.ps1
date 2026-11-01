param([string]$ProcFilter = "msrdc")
# List every top-level window owned by the given process name, with rect and title.
$ErrorActionPreference = "Stop"
Add-Type -Namespace GW2 -Name Api -MemberDefinition @'
public delegate bool EnumProc(IntPtr h, IntPtr l);
[DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc cb, IntPtr l);
[DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
[DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowText(IntPtr h, System.Text.StringBuilder s, int n);
[DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
[DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(IntPtr h, out uint pid);
[DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetClassName(IntPtr h, System.Text.StringBuilder s, int n);
public struct RECT { public int L, T, R, B; }
'@
$pids = @(Get-Process | Where-Object { $_.ProcessName -like "*$ProcFilter*" } | Select-Object -ExpandProperty Id)
"proc '$ProcFilter' pids: $($pids -join ',')"
$rows = New-Object System.Collections.ArrayList
$cb = [GW2.Api+EnumProc]{
  param($h, $l)
  $wp = 0
  [void][GW2.Api]::GetWindowThreadProcessId($h, [ref]$wp)
  if ($pids -contains [int]$wp) {
    $sb = New-Object System.Text.StringBuilder 512
    [void][GW2.Api]::GetWindowText($h, $sb, 512)
    $cs = New-Object System.Text.StringBuilder 256
    [void][GW2.Api]::GetClassName($h, $cs, 256)
    $r = New-Object GW2.Api+RECT
    [void][GW2.Api]::GetWindowRect($h, [ref]$r)
    [void]$rows.Add([pscustomobject]@{
      Handle = $h; Vis = [GW2.Api]::IsWindowVisible($h); Title = $sb.ToString()
      Class = $cs.ToString(); L = $r.L; T = $r.T
      W = ($r.R - $r.L); H = ($r.B - $r.T)
    })
  }
  return $true
}
[void][GW2.Api]::EnumWindows($cb, [IntPtr]::Zero)
$rows | Sort-Object W -Descending | Format-Table -AutoSize
"count=$($rows.Count)"