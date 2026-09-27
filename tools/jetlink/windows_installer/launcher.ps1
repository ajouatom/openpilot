param([ValidateSet('Prepare','Install')][string]$Stage)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$elevated = $false
try {
  [Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
  $env:PYTHONUTF8 = '1'
  $env:PYTHONIOENCODING = 'utf-8'
  . "$PSScriptRoot\messages.ps1"
  Show-InstallerIntro $Stage
  if ($root -notmatch '^[A-Za-z]:\\') { throw "ZIP을 PC의 C: 또는 D: 같은 로컬 드라이브에 풀어 주세요.`nExtract to a local PC drive such as C: or D:." }
  $drive = Get-Volume -DriveLetter $root.Substring(0,1)
  if ($drive.FileSystem -notin @('NTFS','ReFS','exFAT')) { throw "NTFS 또는 exFAT 드라이브에 압축을 풀어 주세요.`nExtract to an NTFS or exFAT drive." }
  if ($Stage -eq 'Prepare') {
    & "$PSScriptRoot\python\python.exe" "$PSScriptRoot\prepare.py"
    if ($LASTEXITCODE -ne 0) { throw "설치 준비 실패. 위 안내를 확인한 뒤 01을 다시 실행하세요.`nPreparation failed. Check the message above and retry 01." }
    exit 0
  }
  $image = Join-Path $root 'prepared.img'
  if (-not (Test-Path -LiteralPath $image)) { throw "먼저 01_설치준비.cmd를 실행하세요.`nRun 01 first." }
  $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
  $principal = [Security.Principal.WindowsPrincipal]::new($identity)
  $elevated = $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
  if (-not $elevated) {
    $null = Read-InstallerInput '안내를 읽은 뒤 Enter: 관리자 권한 요청으로 진행' 'Press Enter to open the administrator prompt'
    $arguments = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', ('"{0}"' -f $PSCommandPath), '-Stage', 'Install')
    $child = Start-Process powershell.exe -Verb RunAs -ArgumentList $arguments -Wait -PassThru
    exit $child.ExitCode
  }
  . "$PSScriptRoot\disks.ps1"
  $release = Get-Content -LiteralPath "$PSScriptRoot\release.json" -Raw -Encoding UTF8 | ConvertFrom-Json
  $source = @(Get-Partition -DriveLetter $root.Substring(0,1))
  if ($source.Count -ne 1) { throw "설치 파일이 저장된 디스크를 확인할 수 없습니다. C:에 풀어서 실행하세요.`nCannot identify the source disk. Extract to C: and retry." }
  $sourceDisk = [int]$source[0].DiskNumber
  $choices = @(Get-Disk | Where-Object { Test-InstallDisk $_ $release.image_bytes $sourceDisk })
  if ($choices.Count -eq 0) { throw "설치할 USB SD카드가 없습니다. 카드 리더를 PC에 연결하고 다시 실행하세요.`nNo eligible USB SD card. Connect the card reader and retry." }
  Write-Host ''
  Write-InstallerPair '설치할 SD카드를 선택하세요. 기존 내용은 모두 지워집니다.' 'Select the SD card. All its contents will be erased.' Yellow
  foreach ($disk in $choices) {
    Write-Host ("[{0}] {1} / {2:N1} GB / ID: {3}" -f $disk.Number,$disk.FriendlyName,($disk.Size/1e9),$disk.SerialNumber)
  }
  $answer = Read-InstallerInput '위 목록의 카드 번호 입력' 'Enter the card number shown above (other input cancels)'
  $number = 0
  if (-not [int]::TryParse($answer, [ref]$number)) { throw "설치를 취소했습니다. SD카드는 변경하지 않았습니다.`nCancelled; SD card unchanged." }
  $selected = @($choices | Where-Object Number -eq $number)
  if ($selected.Count -ne 1) { throw "목록에 없는 번호입니다. SD카드는 변경하지 않았습니다.`nNumber not listed; SD card unchanged." }
  $selected = $selected[0]
  Write-Host ("{0} / {1:N1} GB: 모든 내용을 지우고 설치합니다.`n  All contents on this card will be erased." -f $selected.FriendlyName,($selected.Size/1e9)) -ForegroundColor Yellow
  $confirmation = Read-InstallerInput '삭제 후 설치하려면 설치 또는 INSTALL 입력' 'Type INSTALL (or 설치) to erase and install'
  if (-not (Test-EraseConfirmation $confirmation)) { throw "설치를 취소했습니다. SD카드는 변경하지 않았습니다.`nCancelled; SD card unchanged." }
  Assert-SameDisk (Get-Disk -Number $number) $selected $release.image_bytes $sourceDisk
  $logDir = Join-Path $root 'logs'
  $null = New-Item -ItemType Directory -Force -Path $logDir
  $log = Join-Path $logDir (Get-Date -Format 'yyyyMMdd-HHmmss-fff')
  Write-InstallerPair '기록·검증 중입니다. 카드 분리·창 닫기 금지.' 'Writing and verifying. Do not unplug the card or close this window.'
  & "$PSScriptRoot\write_sd_windows.ps1" -Image $image -Sha256 $release.prepared_sha256 `
    -DiskNumber $number -SerialNumber ([string]$selected.SerialNumber) -UniqueId ([string]$selected.UniqueId) `
    -DiskBytes $selected.Size -Log ($log + '.txt')
  if (-not (Test-Path -LiteralPath ($log + '.txt.success'))) { throw "기록 완료를 확인하지 못했습니다. logs 폴더를 확인하세요.`nCompletion not verified. Check the logs folder." }
  Write-Host ''
  Show-JetsonConnectionSteps
} catch {
  Write-Host ''
  Write-Host $_.Exception.Message -ForegroundColor Red
  if ($elevated) { $null = Read-InstallerInput 'Enter를 누르면 창을 닫습니다' 'Press Enter to close' }
  exit 1
}
if ($elevated) { $null = Read-InstallerInput 'Enter를 누르면 창을 닫습니다' 'Press Enter to close' }
