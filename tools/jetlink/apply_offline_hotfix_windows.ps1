#Requires -RunAsAdministrator
param(
  [Parameter(Mandatory=$true)][int]$DiskNumber,
  [Parameter(Mandatory=$true)][AllowEmptyString()][string]$SerialNumber,
  [string]$UniqueId,
  [Parameter(Mandatory=$true)][long]$DiskBytes,
  [Parameter(Mandatory=$true)][string]$Python,
  [Parameter(Mandatory=$true)][ValidatePattern('^[0-9a-f]{64}$')][string]$ManifestSha256,
  [string]$Manifest = "$PSScriptRoot\offline-usbc.json",
  [ValidateSet('offline_hotfix.py', 'offline_boot_patch.py')][string]$Engine = 'offline_hotfix.py',
  [switch]$VerifyOnly,
  [Parameter(Mandatory=$true)][string]$Log
)
$ErrorActionPreference = 'Stop'
Start-Transcript -LiteralPath $Log -Force
$locks = [System.Collections.Generic.List[System.IDisposable]]::new()
try {
  $pythonPath = (Get-Command $Python -ErrorAction Stop).Source
  if ((Get-FileHash -LiteralPath $Manifest -Algorithm SHA256).Hash.ToLowerInvariant() -ne $ManifestSha256) {
    throw 'Release manifest checksum mismatch'
  }
  $info = & $pythonPath "$PSScriptRoot\$Engine" --manifest $Manifest --manifest-sha256 $ManifestSha256 --inspect
  if ($LASTEXITCODE -ne 0) { throw 'Invalid patch manifest' }
  $patch = $info | ConvertFrom-Json
  function Assert-Target {
    $disk = Get-Disk -Number $DiskNumber
    if ([string]::IsNullOrWhiteSpace($SerialNumber) -and [string]::IsNullOrWhiteSpace($UniqueId)) { throw 'Stable disk identity required' }
    if ($disk.IsBoot -or $disk.IsSystem -or $disk.IsReadOnly -or $disk.IsOffline -or $disk.BusType -ne 'USB' -or
        ([string]$disk.SerialNumber).Trim() -ne $SerialNumber.Trim() -or ($UniqueId -and $disk.UniqueId -ne $UniqueId) -or
        $disk.Size -ne $DiskBytes -or $disk.Size -lt $patch.image_bytes) {
      throw 'USB disk identity/capacity/system-disk guard failed'
    }
  }
  Assert-Target
  if (-not ('CarrotOfflineNative' -as [type])) { Add-Type -TypeDefinition @'
using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
public static class CarrotOfflineNative {
  [DllImport("kernel32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
  static extern SafeFileHandle CreateFile(string name, uint access, uint share, IntPtr security, uint creation, uint flags, IntPtr template);
  [DllImport("kernel32.dll", SetLastError=true)]
  static extern bool DeviceIoControl(SafeFileHandle file, uint control, IntPtr input, uint inputBytes, IntPtr output, uint outputBytes, out uint returned, IntPtr overlapped);
  public static SafeFileHandle Open(string path) {
    var handle = CreateFile(path, 0xC0000000, 3, IntPtr.Zero, 3, 0, IntPtr.Zero);
    if (handle.IsInvalid) throw new Win32Exception(Marshal.GetLastWin32Error());
    return handle;
  }
  public static void Control(SafeFileHandle handle, uint code) {
    uint count;
    if (!DeviceIoControl(handle, code, IntPtr.Zero, 0, IntPtr.Zero, 0, out count, IntPtr.Zero))
      throw new Win32Exception(Marshal.GetLastWin32Error());
  }
}
'@
  }
  foreach ($partition in (Get-Partition -DiskNumber $DiskNumber)) {
    foreach ($path in $partition.AccessPaths) {
      if ($path -like '\\?\Volume{*') {
        $handle = [CarrotOfflineNative]::Open($path.TrimEnd('\'))
        $locks.Add($handle)
        [CarrotOfflineNative]::Control($handle, 0x00090018)
        [CarrotOfflineNative]::Control($handle, 0x00090020)
      }
    }
  }
  Assert-Target
  # Keep volume locks throughout the Python process. No drive letters, mounted
  # filesystems, base image file or other partitions are written by the patcher.
  $arguments = @("$PSScriptRoot\$Engine", '--target', "\\.\PhysicalDrive$DiskNumber",
                 '--manifest', $Manifest, '--manifest-sha256', $ManifestSha256)
  if ($VerifyOnly) { $arguments += '--verify-only' }
  & $pythonPath @arguments
  if ($LASTEXITCODE -ne 0) { throw "Offline hotfix failed with exit code $LASTEXITCODE" }
  Write-Output 'OFFLINE_HOTFIX_COMPLETE (first boot installation still needs verification)'
  Set-Content -LiteralPath ($Log + '.success') -Value $ManifestSha256 -Encoding ASCII
} catch {
  Write-Output ('OFFLINE_HOTFIX_FAILED: ' + $_.Exception.Message)
  Set-Content -LiteralPath ($Log + '.failed') -Value $_.Exception.Message -Encoding UTF8
  throw
} finally {
  foreach ($item in $locks) { $item.Dispose() }
  Stop-Transcript
}
