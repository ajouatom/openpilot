param([switch]$Elevated)
$ErrorActionPreference = 'Stop'
$selected = $null
try {
  [Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
  $env:PYTHONUTF8 = '1'
  $env:PYTHONIOENCODING = 'utf-8'
  . "$PSScriptRoot\messages.ps1"
  . "$PSScriptRoot\disks.ps1"
  Write-InstallerHeading 'CARROT JETSON | 자동 업데이트 전환 패치' 'One-time automatic update patch'
  Write-InstallerPair '기존 USB 저장장치를 검사하고 작은 패치를 적용합니다. 포맷하거나 이미지를 다시 설치하지 않습니다.' 'Checks your existing USB boot storage and applies a small patch. No formatting or image reinstall.'
  Write-InstallerPair 'Jetson을 정상 종료하고 전원을 분리한 뒤, 부팅용 저장장치를 PC에 연결하세요.' 'Shut down and unplug Jetson, then connect its boot storage to the PC.'
  Write-InstallerPair '압축을 모두 풀고 이 파일만 실행하세요. 별도 Python·SSH 설치는 필요 없습니다.' 'Extract all files and run this file only. No Python or SSH installation is needed.'
  Write-InstallerPair '예상 5~20분. 대부분 기존 설치 검사 시간이며 느린 저장장치는 더 걸릴 수 있습니다.' 'Allow about 5–20 minutes, mostly for checking the installation; slow storage may take longer.'
  Write-InstallerPair 'R2 또는 SD/NVMe v2 설치용 시험 패치입니다. 실제 저장장치 적용·부팅 검증은 아직 완료 전입니다.' 'Preview for R2 and SD/NVMe v2 installations. Physical media patching and boot validation are pending.' Yellow
  Write-InstallerPair '중요한 내용은 먼저 백업하세요. 완료 표시 전에는 분리하거나 창을 닫지 마세요.' 'Back up important data first. Do not unplug or close the window before completion.' Yellow
  $root = Split-Path -Parent $PSScriptRoot
  $python = Join-Path $PSScriptRoot 'python\python.exe'
  if (-not (Test-Path -LiteralPath $python)) { throw 'Extract the entire patch ZIP first.' }
  if ($root -notmatch '^[A-Za-z]:\\') { throw 'Extract to a local PC drive, such as C:.' }
  $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
  $principal = [Security.Principal.WindowsPrincipal]::new($identity)
  if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    $null = Read-InstallerInput 'Enter를 누른 뒤 관리자 권한 창에서 예를 선택하세요' 'Press Enter, then choose Yes in the administrator prompt'
    $child = Start-Process powershell.exe -Verb RunAs -Wait -PassThru -ArgumentList @('-NoProfile','-ExecutionPolicy','Bypass','-File',('"{0}"' -f $PSCommandPath),'-Elevated')
    exit $child.ExitCode
  }
  $meta = Get-Content -LiteralPath "$PSScriptRoot\boot-patch-release.json" -Raw -Encoding UTF8 | ConvertFrom-Json
  Write-Host ("PATCH_VERSION: {0}" -f $meta.version)
  $manifest = Join-Path $PSScriptRoot 'boot-patch.json'
  $payload = Join-Path $PSScriptRoot 'carrot-boot-update.zip'
  if ((Get-FileHash -LiteralPath $manifest -Algorithm SHA256).Hash.ToLowerInvariant() -ne $meta.patch_sha256 -or
      (Get-FileHash -LiteralPath $payload -Algorithm SHA256).Hash.ToLowerInvariant() -ne $meta.payload_sha256) { throw 'Patch package checksum mismatch; download it again.' }
  $index = Get-Content -LiteralPath $manifest -Raw -Encoding UTF8 | ConvertFrom-Json
  if ($index.payload_sha256 -ne $meta.payload_sha256) { throw 'Patch payload identity mismatch' }
  $source = @(Get-Partition -DriveLetter $root.Substring(0,1))
  if ($source.Count -ne 1) { throw 'Cannot identify the source disk. Extract to C: and retry.' }
  $choices = @(Get-Disk | Where-Object { Test-InstallDisk $_ $meta.image_bytes $source[0].DiskNumber })
  if ($choices.Count -eq 0) { throw "대상 USB 저장장치가 없습니다. 연결을 확인하세요.`nNo eligible USB storage. Check the reader/enclosure." }
  foreach ($disk in $choices) { Write-Host ("[{0}] {1} / {2:N1} GB / ID: {3}" -f $disk.Number,$disk.FriendlyName,($disk.Size/1e9),$disk.SerialNumber) }
  $answer = Read-InstallerInput '패치할 디스크 번호 입력' 'Enter the disk number to patch'
  $number = 0
  if (-not [int]::TryParse($answer,[ref]$number)) { throw 'Cancelled; nothing written.' }
  $selectedDisks = @($choices | Where-Object Number -eq $number)
  if ($selectedDisks.Count -ne 1) { throw 'Number not listed; nothing written.' }
  $selected = $selectedDisks[0]
  Write-Host ("{0} / {1:N1} GB" -f $selected.FriendlyName,($selected.Size/1e9))
  $answer = Read-InstallerInput '이 장치를 확인했다면 PATCH 입력' 'Type PATCH to confirm this device (anything else cancels)'
  if ($answer.Trim() -cne 'PATCH') { throw 'Cancelled; nothing written.' }
  function Assert-BootTarget {
    try { Assert-SameDisk (Get-Disk -Number $number) $selected $meta.image_bytes $source[0].DiskNumber }
    catch { throw "저장장치 연결 정보가 바뀌었습니다. 확인 후 패치를 다시 실행하세요.`nStorage identity changed. Check the connection and rerun this patch." }
    $partition = Get-Partition -DiskNumber $number -PartitionNumber 16
    if ($partition.Offset -ne $index.setup_offset -or $partition.Size -ne $index.setup_bytes) { throw 'Unexpected SETUP partition layout' }
    $volume = $partition | Get-Volume
    if ($volume.FileSystemLabel -ne 'CARROTSETUP' -or $volume.FileSystem -ne 'FAT32') { throw 'Expected CARROTSETUP FAT32 volume' }
    return $partition
  }
  $null = Assert-BootTarget
  $logs = Join-Path $root 'logs'
  $null = New-Item -ItemType Directory -Path $logs -Force
  $stamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
  $common = @{ DiskNumber=$number; SerialNumber=[string]$selected.SerialNumber; UniqueId=[string]$selected.UniqueId;
    DiskBytes=$selected.Size; Python=$python; Manifest=$manifest; ManifestSha256=$meta.patch_sha256; Engine='offline_boot_patch.py' }
  Write-InstallerHeading '1 / 3 · 기존 설치 검사' 'Verify existing installation'
  $log = Join-Path $logs ($stamp + '-check.txt')
  & "$PSScriptRoot\apply_offline_hotfix_windows.ps1" @common -VerifyOnly -Log $log
  if (-not (Test-Path -LiteralPath ($log + '.success'))) { throw 'Installation verification failed; nothing patched.' }
  Write-InstallerHeading '2 / 3 · 업데이트 준비 파일 복사' 'Copy update preparation files'
  $null = Assert-BootTarget
  # Finish all filesystem activity in a child process before taking raw volume locks.
  # A fresh child also prevents PowerShell provider handles surviving into stage 3.
  $copyArguments = @('-NoProfile','-ExecutionPolicy','Bypass','-File',"$PSScriptRoot\copy_boot_payload.ps1",
    '-DiskNumber',$number,'-UniqueId',([string]$selected.UniqueId),'-DiskBytes',$selected.Size,
    '-SourceDisk',$source[0].DiskNumber,'-Support',$PSScriptRoot)
  # Windows PowerShell drops empty native-command arguments; omit an absent serial.
  if (-not [string]::IsNullOrWhiteSpace([string]$selected.SerialNumber)) {
    $copyArguments += @('-SerialNumber',([string]$selected.SerialNumber))
  }
  & powershell.exe @copyArguments
  if ($LASTEXITCODE -ne 0) { throw 'Payload copy/cleanup failed. Close target USB windows and rerun this patch.' }
  Write-InstallerHeading '3 / 3 · 부팅 패치 및 기록 확인' 'Patch boot helper and verify readback'
  $null = Assert-BootTarget
  $log = Join-Path $logs ($stamp + '-patch.txt')
  & "$PSScriptRoot\apply_offline_hotfix_windows.ps1" @common -Log $log
  if (-not (Test-Path -LiteralPath ($log + '.success'))) { throw 'Patch completion not verified; check logs.' }
  Write-InstallerHeading 'PC 패치 기록·검증 완료' 'PC patch written and verified'
  Write-InstallerPair '안전하게 제거 → 전원이 분리된 Jetson에 재장착 → C4 USB·인터넷 연결 → 시동' 'Safely eject > Reinstall with Jetson power disconnected > Connect C4 USB and Internet > Power on'
  Write-InstallerPair 'C4도 최신 버전으로 업데이트하세요. 다음 부팅부터 필요한 Jetson 업데이트를 자동으로 확인합니다.' 'Update C4 too. From the next boot, required Jetson updates are checked automatically.'
  Write-InstallerPair '이 완료 표시는 PC 패치 검증 결과입니다. Jetson 프로그램 다운로드·적용은 다음 부팅에서 진행됩니다.' 'This confirms the PC patch, not runtime activation. Jetson downloads/applies its required runtime on the next boot.'
  Write-InstallerPair '첫 연결은 정차 상태에서 확인하세요. 웹의 최초 업데이트 대기나 SSH 접속은 필요 없습니다.' 'Verify the first connection while parked. No Web first-update wait or SSH connection is needed.'
} catch {
  Write-Host $_.Exception.Message -ForegroundColor Red
  Write-Host '실패했다면 포맷하지 말고 로그를 확인하세요. 작업 중 끊겼다면 같은 패치를 다시 실행하세요.' -ForegroundColor Yellow
  Write-Host 'Do not format after a failure. Check logs; after interruption, rerun this same patch.'
  if ($Elevated) { Read-Host 'Enter' }
  exit 1
}
if ($Elevated) { Read-Host 'Enter' }
