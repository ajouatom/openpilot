param([Parameter(Mandatory=$true)][string]$WorkDirectory)
$ErrorActionPreference = 'Stop'
$null = New-Item -ItemType Directory -Path $WorkDirectory -Force
$work = (Resolve-Path -LiteralPath $WorkDirectory).Path
$support = Join-Path $work 'support'
$setup = Join-Path $work 'setup'
$null = New-Item -ItemType Directory -Path $support,$setup -Force
Copy-Item -LiteralPath "$PSScriptRoot\disks.ps1" -Destination $support
# Run the actual body against local files and fake Storage cmdlets. Strip only the
# elevation directive: these tests must never access a real disk or require admin.
$body = Get-Content -LiteralPath "$PSScriptRoot\copy_boot_payload.ps1" -Raw -Encoding UTF8
[IO.File]::WriteAllText((Join-Path $support 'copy.ps1'),$body.Replace('#Requires -RunAsAdministrator',''),[Text.UTF8Encoding]::new($true))
[IO.File]::WriteAllText((Join-Path $support 'carrot-boot-update.zip'),'verified payload')
$payloadSha = (Get-FileHash "$support\carrot-boot-update.zip").Hash.ToLowerInvariant()
@{setup_offset=2048;setup_bytes=4096;payload_bytes=16;payload_sha256=$payloadSha} | ConvertTo-Json | Set-Content "$support\boot-patch.json" -Encoding UTF8
$manifestSha = (Get-FileHash "$support\boot-patch.json").Hash.ToLowerInvariant()
@{patch_sha256=$manifestSha;payload_sha256=$payloadSha;image_bytes=8192} | ConvertTo-Json | Set-Content "$support\boot-patch-release.json" -Encoding UTF8

function Get-Disk { param($Number)
  [pscustomobject]@{ Number=123456; Size=16384; BusType='USB'; IsBoot=$false; IsSystem=$false;
    IsReadOnly=$false; IsOffline=$false; UniqueId=$global:copyTest_currentId; SerialNumber='' }
}
function Get-Partition { param($DiskNumber,$PartitionNumber)
  if ($DiskNumber -ne 123456 -or $PartitionNumber -ne 16) { throw 'Unexpected partition' }
  [pscustomobject]@{ Offset=2048; Size=4096; DriveLetter=$global:copyTest_letter }
}
function Get-Volume { [CmdletBinding()]param([Parameter(ValueFromPipeline=$true)]$Partition)
  process { [pscustomobject]@{FileSystemLabel='CARROTSETUP';FileSystem='FAT32';SizeRemaining=2000000} }
}
function Add-PartitionAccessPath { param($DiskNumber,$PartitionNumber,[switch]$AssignDriveLetter)
  $global:copyTest_assigned++; $global:copyTest_letter='Q'
}
function Remove-PartitionAccessPath { param($DiskNumber,$PartitionNumber,$AccessPath)
  if ($DiskNumber -ne 123456 -or $PartitionNumber -ne 16 -or $AccessPath -ne 'Q:\') { throw 'Wrong letter removal' }
  if ($global:copyTest_cleanupFails) { throw 'simulated cleanup failure' }
  $global:copyTest_removed++; $global:copyTest_letter=''
}
function Join-Path { param($Path,$ChildPath)
  if ($Path -eq 'Q:\') { $Path=$setup }
  Microsoft.PowerShell.Management\Join-Path $Path $ChildPath
}
$arguments = @{DiskNumber=123456;UniqueId='fake-usb';DiskBytes=16384;SourceDisk=0;Support=$support}
foreach ($initialLetter in @('Q','')) {
  $global:copyTest_letter=$initialLetter; $global:copyTest_currentId='fake-usb'; $global:copyTest_assigned=0; $global:copyTest_removed=0
  & "$support\copy.ps1" @arguments
  if ((Get-FileHash "$setup\carrot-boot-update.zip").Hash.ToLowerInvariant() -ne $payloadSha) { throw 'Payload mismatch' }
  $expected = if ($initialLetter) { 0 } else { 1 }
  if ($global:copyTest_assigned -ne $expected -or $global:copyTest_removed -ne $expected -or $global:copyTest_letter -ne $initialLetter) { throw 'Drive letter ownership not preserved' }
}
Write-Output 'PASS: verified copy, overwrite retry, empty serial, own letter removed, existing letter preserved'
$global:copyTest_letter=''; $global:copyTest_cleanupFails=$true
$failure=$null
try { & "$support\copy.ps1" @arguments } catch { $failure=$_.Exception.Message }
if ($failure -ne 'simulated cleanup failure') { throw 'Cleanup failure did not stop the copy stage' }
$global:copyTest_cleanupFails=$false
Write-Output 'PASS: temporary-letter cleanup failure is not reported as successful completion'
$global:copyTest_currentId='different-usb'
$global:copyTest_letter=''; $global:copyTest_assigned=0
& "$support\copy.ps1" @arguments
if ($LASTEXITCODE -ne 1 -or $global:copyTest_assigned -ne 0) { throw 'Changed disk accepted' }
Write-Output 'PASS: changed disk fails before mounting or copying'
$global:copyTest_currentId='fake-usb'
[IO.File]::WriteAllText((Join-Path $support 'carrot-boot-update.zip'),'corrupt')
& "$support\copy.ps1" @arguments
if ($LASTEXITCODE -ne 1 -or $global:copyTest_assigned -ne 0) { throw 'Corrupt payload accepted' }
if ((Get-FileHash "$setup\carrot-boot-update.zip").Hash.ToLowerInvariant() -ne $payloadSha) { throw 'Failed retry changed existing verified payload' }
Write-Output 'PASS: corrupt input leaves existing verified payload unchanged'
exit 0
