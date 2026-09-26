#Requires -RunAsAdministrator
param(
  [Parameter(Mandatory=$true)][string]$Image,
  [Parameter(Mandatory=$true)][ValidatePattern('^[0-9a-fA-F]{64}$')][string]$Sha256,
  [Parameter(Mandatory=$true)][int]$DiskNumber,
  [Parameter(Mandatory=$true)][string]$SerialNumber,
  [Parameter(Mandatory=$true)][long]$DiskBytes,
  [string]$SetupJson,
  [Parameter(Mandatory=$true)][string]$Log
)
$ErrorActionPreference = 'Stop'
Start-Transcript -LiteralPath $Log -Force
$locks = [System.Collections.Generic.List[System.IDisposable]]::new()
$device = $null
$inputImage = $null
try {
  $imagePath = (Resolve-Path -LiteralPath $Image).Path
  $imageFile = Get-Item -LiteralPath $imagePath
  if ($imageFile.Length -lt 1GB -or $imageFile.Length % 512 -ne 0) { throw 'Invalid image size' }
  if ((Get-FileHash -LiteralPath $imagePath -Algorithm SHA256).Hash -ne $Sha256) { throw 'Image checksum mismatch' }
  $setupBytes = $null
  if ($SetupJson) {
    $setupBytes = [System.IO.File]::ReadAllBytes((Resolve-Path -LiteralPath $SetupJson).Path)
    if ($setupBytes.Length -gt 65536) { throw 'Setup configuration is too large' }
    $null = [System.Text.Encoding]::UTF8.GetString($setupBytes) | ConvertFrom-Json
  }
  $disk = Get-Disk -Number $DiskNumber
  if ($disk.IsBoot -or $disk.IsSystem -or $disk.IsReadOnly -or $disk.BusType -ne 'USB' -or
      $disk.SerialNumber.Trim() -ne $SerialNumber.Trim() -or $disk.Size -ne $DiskBytes -or $imageFile.Length -gt $disk.Size) {
    throw 'Disk identity, capacity or system-disk guard failed'
  }
  Write-Output "Verified USB target disk $DiskNumber serial $SerialNumber size $DiskBytes"
  Add-Type -TypeDefinition @'
using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
public static class CarrotSdNative {
  [DllImport("kernel32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
  public static extern SafeFileHandle CreateFile(string name, uint access, uint share, IntPtr security, uint creation, uint flags, IntPtr template);
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
  $partitions = Get-Partition -DiskNumber $DiskNumber
  foreach ($partition in $partitions) {
    foreach ($path in $partition.AccessPaths) {
      if ($path -like '\\?\Volume{*') {
        $handle = [CarrotSdNative]::Open($path.TrimEnd('\'))
        $locks.Add($handle)
        [CarrotSdNative]::Control($handle, 0x00090018) # FSCTL_LOCK_VOLUME
        [CarrotSdNative]::Control($handle, 0x00090020) # FSCTL_DISMOUNT_VOLUME
      }
    }
  }
  # Recheck identity immediately before the first destructive write.
  $disk = Get-Disk -Number $DiskNumber
  if ($disk.IsBoot -or $disk.IsSystem -or $disk.Size -ne $DiskBytes -or $disk.SerialNumber.Trim() -ne $SerialNumber.Trim()) {
    throw 'Disk identity changed before write'
  }
  $handle = [CarrotSdNative]::Open("\\.\PhysicalDrive$DiskNumber")
  $device = [System.IO.FileStream]::new($handle, [System.IO.FileAccess]::ReadWrite, 4MB, $false)
  $inputImage = [System.IO.File]::OpenRead($imagePath)
  $buffer = New-Object byte[] (4MB)
  $total = [long]0
  $nextReport = [long]1GB
  while (($count = $inputImage.Read($buffer, 0, $buffer.Length)) -gt 0) {
    $device.Write($buffer, 0, $count)
    $total += $count
    if ($total -ge $nextReport) { Write-Output "WRITE $total / $($imageFile.Length)"; $nextReport += 1GB }
  }
  $device.Flush($true)
  $inputImage.Dispose(); $inputImage = $null
  if ($total -ne $imageFile.Length) { throw 'Short image write' }
  $null = $device.Seek(0, [System.IO.SeekOrigin]::Begin)
  $hash = [System.Security.Cryptography.SHA256]::Create()
  $remaining = $imageFile.Length
  $nextReport = [long]1GB
  while ($remaining -gt 0) {
    $count = $device.Read($buffer, 0, [int][Math]::Min($buffer.Length, $remaining))
    if ($count -le 0) { throw 'Short read during verification' }
    $null = $hash.TransformBlock($buffer, 0, $count, $null, 0)
    $remaining -= $count
    $verified = $imageFile.Length - $remaining
    if ($verified -ge $nextReport) { Write-Output "VERIFY $verified / $($imageFile.Length)"; $nextReport += 1GB }
  }
  $null = $hash.TransformFinalBlock([byte[]]::new(0), 0, 0)
  $readbackHash = ([BitConverter]::ToString($hash.Hash)).Replace('-', '').ToLowerInvariant()
  $hash.Dispose()
  if ($readbackHash -ne $Sha256.ToLowerInvariant()) { throw 'SD readback checksum mismatch' }
  Write-Output "READBACK_VERIFIED $readbackHash"
  $device.Dispose(); $device = $null
  foreach ($item in $locks) { $item.Dispose() }; $locks.Clear()
  Update-Disk -Number $DiskNumber
  Update-HostStorageCache
  if ($setupBytes) {
    $setupPartition = $null
    for ($attempt = 0; $attempt -lt 10 -and -not $setupPartition; $attempt++) {
      $setupPartition = Get-Partition -DiskNumber $DiskNumber -PartitionNumber 16 -ErrorAction SilentlyContinue
      if (-not $setupPartition) { Start-Sleep -Milliseconds 500 }
    }
    if (-not $setupPartition) { throw 'Written image verified, but setup partition did not enumerate' }
    if ($setupPartition.Size -ne 64MB) { throw 'Unexpected setup partition' }
    if (-not $setupPartition.DriveLetter) {
      $setupPartition | Add-PartitionAccessPath -AssignDriveLetter
      $setupPartition = Get-Partition -DiskNumber $DiskNumber -PartitionNumber 16
    }
    $volume = $setupPartition | Get-Volume
    if ($volume.FileSystemLabel -ne 'CARROTSETUP') { throw 'Unexpected setup volume' }
    $setupPath = "$($setupPartition.DriveLetter):\setup.json"
    $setupStream = [System.IO.File]::Open($setupPath, [System.IO.FileMode]::Create, [System.IO.FileAccess]::Write, [System.IO.FileShare]::Read)
    try { $setupStream.Write($setupBytes, 0, $setupBytes.Length); $setupStream.Flush($true) }
    finally { $setupStream.Dispose() }
    Write-Output 'PRIVATE_SETUP_WRITTEN (contents intentionally omitted)'
  }
  Write-Output 'SD_WRITE_COMPLETE_AND_VERIFIED'
  Set-Content -LiteralPath ($Log + '.success') -Value $readbackHash -Encoding ASCII
} catch {
  Write-Output ('SD_WRITE_FAILED: ' + $_.Exception.Message)
  Set-Content -LiteralPath ($Log + '.failed') -Value $_.Exception.Message -Encoding UTF8
  throw
} finally {
  if ($inputImage) { $inputImage.Dispose() }
  if ($device) { $device.Dispose() }
  foreach ($item in $locks) { $item.Dispose() }
  Stop-Transcript
}
