#Requires -RunAsAdministrator
param(
  [Parameter(Mandatory=$true)][int]$DiskNumber,
  [AllowEmptyString()][string]$SerialNumber,
  [Parameter(Mandatory=$true)][string]$UniqueId,
  [Parameter(Mandatory=$true)][long]$DiskBytes,
  [Parameter(Mandatory=$true)][int]$SourceDisk,
  [Parameter(Mandatory=$true)][string]$Support
)
$ErrorActionPreference = 'Stop'
$assignedPath = $null
try {
  . "$PSScriptRoot\disks.ps1"
  $meta = Get-Content -LiteralPath "$Support\boot-patch-release.json" -Raw -Encoding UTF8 | ConvertFrom-Json
  $manifest = Join-Path $Support 'boot-patch.json'
  $payload = Join-Path $Support 'carrot-boot-update.zip'
  if ((Get-FileHash -LiteralPath $manifest -Algorithm SHA256).Hash.ToLowerInvariant() -ne $meta.patch_sha256 -or
      (Get-FileHash -LiteralPath $payload -Algorithm SHA256).Hash.ToLowerInvariant() -ne $meta.payload_sha256) { throw 'Patch package checksum mismatch' }
  $index = Get-Content -LiteralPath $manifest -Raw -Encoding UTF8 | ConvertFrom-Json
  if ($index.payload_sha256 -ne $meta.payload_sha256) { throw 'Patch payload identity mismatch' }
  $selected = [pscustomobject]@{ Number=$DiskNumber; Size=$DiskBytes; UniqueId=$UniqueId; SerialNumber=$SerialNumber }
  function Assert-CopyTarget {
    Assert-SameDisk (Get-Disk -Number $DiskNumber) $selected $meta.image_bytes $SourceDisk
    $partition = Get-Partition -DiskNumber $DiskNumber -PartitionNumber 16
    if ($partition.Offset -ne $index.setup_offset -or $partition.Size -ne $index.setup_bytes) { throw 'Unexpected SETUP partition layout' }
    $volume = $partition | Get-Volume
    if ($volume.FileSystemLabel -ne 'CARROTSETUP' -or $volume.FileSystem -ne 'FAT32') { throw 'Expected CARROTSETUP FAT32 volume' }
    return $partition
  }
  $partition = Assert-CopyTarget
  $volume = $partition | Get-Volume
  if ($volume.SizeRemaining -lt ($index.payload_bytes * 2 + 1048576)) { throw 'Not enough free space in SETUP' }
  if (-not $partition.DriveLetter) {
    Add-PartitionAccessPath -DiskNumber $DiskNumber -PartitionNumber 16 -AssignDriveLetter
    $partition = Assert-CopyTarget
    $assignedPath = ([string]$partition.DriveLetter) + ':\'
  }
  $setupRoot = ([string]$partition.DriveLetter) + ':\'
  if ($setupRoot -notmatch '^[A-Za-z]:\\$') { throw 'SETUP drive letter unavailable' }
  $destination = Join-Path $setupRoot 'carrot-boot-update.zip'
  $temporary = $destination + '.new'
  Copy-Item -LiteralPath $payload -Destination $temporary -Force
  $stream = [IO.File]::Open($temporary,[IO.FileMode]::Open,[IO.FileAccess]::ReadWrite,[IO.FileShare]::Read)
  try { $stream.Flush($true) } finally { $stream.Dispose() }
  if ((Get-FileHash -LiteralPath $temporary -Algorithm SHA256).Hash.ToLowerInvariant() -ne $meta.payload_sha256) { throw 'Payload readback failed' }
  $null = Assert-CopyTarget
  Move-Item -LiteralPath $temporary -Destination $destination -Force
  if ((Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash.ToLowerInvariant() -ne $meta.payload_sha256) { throw 'Payload final readback failed' }
  Write-Host 'BOOT_PAYLOAD_VERIFIED'
} catch {
  Write-Host $_.Exception.Message -ForegroundColor Red
  exit 1
} finally {
  if ($assignedPath) {
    # Remove only the letter created by this child, after verifying identity again.
    $null = Assert-CopyTarget
    Remove-PartitionAccessPath -DiskNumber $DiskNumber -PartitionNumber 16 -AccessPath $assignedPath
  }
}
# Process termination releases any provider/directory handles before raw locking.
