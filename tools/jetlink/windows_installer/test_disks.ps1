$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\disks.ps1"
$good = [pscustomobject]@{ Number=7; Size=128000000000; BusType='USB'; IsBoot=$false; IsSystem=$false; IsReadOnly=$false; IsOffline=$false; UniqueId='card-1'; SerialNumber='' }
if (-not (Test-InstallDisk $good 25769803776 0)) { throw 'Eligible USB card rejected' }
foreach ($case in @(@('IsBoot',$true),@('IsSystem',$true),@('IsReadOnly',$true),@('IsOffline',$true),@('BusType','NVMe'),@('Size',1024),@('UniqueId',''))) {
  $disk = $good.PSObject.Copy()
  $disk.($case[0]) = $case[1]
  if (Test-InstallDisk $disk 25769803776 0) { throw "Unsafe disk accepted: $($case[0])" }
}
if (Test-InstallDisk $good 25769803776 7) { throw 'Installer source disk accepted' }
Assert-SameDisk $good $good 25769803776 0
foreach ($case in @(@('Number',8),@('Size',64000000000),@('UniqueId','card-2'),@('SerialNumber','changed'),@('BusType','SATA'))) {
  $disk = $good.PSObject.Copy()
  $disk.($case[0]) = $case[1]
  $rejected = $false
  try { Assert-SameDisk $disk $good 25769803776 0 } catch { $rejected = $true }
  if (-not $rejected) { throw "Replaced disk accepted: $($case[0])" }
}
foreach ($file in @('launcher.ps1','disks.ps1','../write_sd_windows.ps1')) {
  $tokens=$null; $errors=$null
  $null = [System.Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $file),[ref]$tokens,[ref]$errors)
  if ($errors.Count) { throw ($errors | Out-String) }
}
Write-Output 'PASS: 15 disk guards and PowerShell syntax; no disk opened or written'
