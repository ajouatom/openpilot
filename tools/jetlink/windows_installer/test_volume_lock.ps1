$ErrorActionPreference = 'Stop'
# Exercise the actual lock function with simulated Win32 outcomes; never open disks.
$tokens=$null; $errors=$null
$ast = [Management.Automation.Language.Parser]::ParseFile(
  (Join-Path $PSScriptRoot '../apply_offline_hotfix_windows.ps1'),[ref]$tokens,[ref]$errors)
if ($errors.Count) { throw ($errors | Out-String) }
$definition = $ast.Find({param($node) $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Lock-OfflineVolume'},$true)
. ([scriptblock]::Create($definition.Extent.Text))
Add-Type @'
using System;
using System.Collections.Generic;
using System.ComponentModel;
public sealed class TestVolumeHandle : IDisposable {
  public bool Closed;
  public void Dispose() { Closed = true; }
}
public static class CarrotOfflineNative {
  public static Queue<int> Errors = new Queue<int>();
  public static List<uint> Calls = new List<uint>();
  public static List<TestVolumeHandle> Handles = new List<TestVolumeHandle>();
  public static TestVolumeHandle Open(string path) {
    var result = new TestVolumeHandle(); Handles.Add(result); return result;
  }
  public static void Control(TestVolumeHandle handle, uint code) {
    Calls.Add(code);
    int error = Errors.Count > 0 ? Errors.Dequeue() : 0;
    if (error != 0) throw new Win32Exception(error);
  }
  public static void Reset() { Errors.Clear(); Calls.Clear(); Handles.Clear(); }
}
'@
function Start-Sleep { param($Seconds) $script:waits++ }
function Reset-Case([int[]]$Codes) {
  [CarrotOfflineNative]::Reset()
  foreach ($code in $Codes) { [CarrotOfflineNative]::Errors.Enqueue($code) }
  $script:waits=0; $script:checks=0
}
$check = { $script:checks++ }
Reset-Case @(5,32,0,0)
$handle = Lock-OfflineVolume 'fake-volume' 'partition=16' $check 3
if ($script:checks -ne 3 -or $script:waits -ne 2 -or $handle.Closed) { throw 'Retry did not retain successful lock' }
if (-not [CarrotOfflineNative]::Handles[0].Closed -or -not [CarrotOfflineNative]::Handles[1].Closed) { throw 'Failed handle leaked' }
if (([CarrotOfflineNative]::Calls -join ',') -ne '589848,589848,589848,589856') { throw 'Dismount ran before lock success' }
$handle.Dispose()
Write-Output 'PASS: busy lock retries, closes failures, dismounts only after lock, retains success'

foreach ($case in @(@(5,5,5),@(21),@(0,5,0,5,0,5))) {
  Reset-Case $case
  $failure=$null
  try { $null = Lock-OfflineVolume 'fake-volume' 'partition=16' $check 3 } catch { $failure=$_.Exception.Message }
  if (-not $failure -or $failure -notmatch 'partition=16.*Win32=' -or $failure -notmatch 'operation=') { throw 'Missing actionable native error' }
  if (@([CarrotOfflineNative]::Handles | Where-Object { -not $_.Closed }).Count) { throw 'Failed lock leaked a handle' }
  $expected = if ($case[0] -eq 21) { 1 } else { 3 }
  if ($script:checks -ne $expected) { throw 'Wrong retry bound' }
}
Write-Output 'PASS: permanent busy and dismount errors fail closed; unrelated errors fail immediately'

Reset-Case @(5)
$failure=$null
try {
  $null = Lock-OfflineVolume 'fake-volume' 'partition=16' {
    $script:checks++
    if ($script:checks -eq 2) { throw 'identity changed' }
  } 3
} catch { $failure=$_.Exception.Message }
if ($failure -ne 'identity changed' -or [CarrotOfflineNative]::Handles.Count -ne 1 -or -not [CarrotOfflineNative]::Handles[0].Closed) { throw 'Changed target reopened' }
Write-Output 'PASS: disk identity checked again before each reopen'
