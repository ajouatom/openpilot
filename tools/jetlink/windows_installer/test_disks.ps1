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
foreach ($file in @('launcher.ps1','disks.ps1','messages.ps1','sd_nvme_patch.ps1','boot_update_patch.ps1','copy_boot_payload.ps1','../write_sd_windows.ps1','../apply_offline_hotfix_windows.ps1')) {
  $tokens=$null; $errors=$null
  $body = Get-Content -LiteralPath (Join-Path $PSScriptRoot $file) -Raw -Encoding UTF8
  $null = [System.Management.Automation.Language.Parser]::ParseInput($body,[ref]$tokens,[ref]$errors)
  if ($errors.Count) { throw ($errors | Out-String) }
}
Write-Output 'PASS: 15 disk guards and PowerShell syntax; no disk opened or written'

foreach ($answer in @('설치','INSTALL',' INSTALL ')) {
  if (-not (Test-EraseConfirmation $answer)) { throw 'Valid explicit erase confirmation rejected' }
}
foreach ($answer in @('', 'yes', 'Y', '7', 'install', 'CANCEL')) {
  if (Test-EraseConfirmation $answer) { throw 'Ambiguous erase confirmation accepted' }
}
. "$PSScriptRoot/messages.ps1"
$prepareText = (Show-InstallerIntro Prepare 6>&1 | Out-String)
$installText = (Show-InstallerIntro Install 6>&1 | Out-String)
$finishText = (Show-JetsonConnectionSteps 6>&1 | Out-String)
foreach ($item in @(@($prepareText,'5-15 min'), @($prepareText,'설치 준비'), @($installText,'30-90 min'), @($installText,'INSTALL'), @($finishText,'disconnect its power supply'), @($finishText,'전원 공급'))) {
  if (-not $item[0].Contains($item[1])) { throw 'Missing bilingual installer guidance' }
}
Write-Output 'PASS: explicit Korean/English erase confirmation and bilingual guidance; no disk opened'

$pair = @(Write-InstallerPair '한글 설명' 'English explanation' 6>&1 | ForEach-Object { $_.ToString() })
if ($pair.Count -ne 2 -or $pair[0].Trim() -ne '한글 설명' -or $pair[1].Trim() -ne 'English explanation') { throw 'Expected Korean then English on separate lines' }
Write-Output 'PASS: Korean-first, separate-line console layout'
