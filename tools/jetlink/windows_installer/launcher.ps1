param([ValidateSet('Prepare','Install')][string]$Stage)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$elevated = $false
try {
  [Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
  $env:PYTHONUTF8 = '1'
  $env:PYTHONIOENCODING = 'utf-8'
  if ($root -notmatch '^[A-Za-z]:\\') { throw 'ZIP을 PC의 C: 또는 D: 같은 로컬 드라이브에 풀어 주세요.' }
  $drive = Get-Volume -DriveLetter $root.Substring(0,1)
  if ($drive.FileSystem -notin @('NTFS','ReFS','exFAT')) { throw 'NTFS 또는 exFAT 드라이브에 압축을 풀어 주세요.' }
  if ($Stage -eq 'Prepare') {
    & "$PSScriptRoot\python\python.exe" "$PSScriptRoot\prepare.py"
    if ($LASTEXITCODE -ne 0) { throw '설치 준비 실패. 위 안내를 확인한 뒤 01을 다시 실행하세요.' }
    exit 0
  }
  $image = Join-Path $root 'prepared.img'
  if (-not (Test-Path -LiteralPath $image)) { throw '먼저 01_설치준비.cmd를 실행하세요.' }
  $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
  $principal = [Security.Principal.WindowsPrincipal]::new($identity)
  $elevated = $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
  if (-not $elevated) {
    $arguments = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', ('"{0}"' -f $PSCommandPath), '-Stage', 'Install')
    $child = Start-Process powershell.exe -Verb RunAs -ArgumentList $arguments -Wait -PassThru
    exit $child.ExitCode
  }
  . "$PSScriptRoot\disks.ps1"
  $release = Get-Content -LiteralPath "$PSScriptRoot\release.json" -Raw -Encoding UTF8 | ConvertFrom-Json
  $source = @(Get-Partition -DriveLetter $root.Substring(0,1))
  if ($source.Count -ne 1) { throw '설치 파일이 저장된 디스크를 확인할 수 없습니다. C:에 풀어서 실행하세요.' }
  $sourceDisk = [int]$source[0].DiskNumber
  $choices = @(Get-Disk | Where-Object { Test-InstallDisk $_ $release.image_bytes $sourceDisk })
  if ($choices.Count -eq 0) { throw '설치할 USB SD카드가 없습니다. 카드 리더를 PC에 연결하고 다시 실행하세요.' }
  Write-Host ''
  Write-Host '설치할 SD카드를 선택하세요. 선택한 카드의 기존 내용은 모두 지워집니다.' -ForegroundColor Yellow
  foreach ($disk in $choices) {
    Write-Host ("[{0}] {1} / {2:N1} GB / 일련번호: {3}" -f $disk.Number,$disk.FriendlyName,($disk.Size/1e9),$disk.SerialNumber)
  }
  $answer = Read-Host '위 목록의 SD카드 번호 입력 (그 외 입력은 취소)'
  $number = 0
  if (-not [int]::TryParse($answer, [ref]$number)) { throw '설치를 취소했습니다. SD카드는 변경하지 않았습니다.' }
  $selected = @($choices | Where-Object Number -eq $number)
  if ($selected.Count -ne 1) { throw '목록에 없는 번호입니다. SD카드는 변경하지 않았습니다.' }
  $selected = $selected[0]
  Write-Host ("{0} / {1:N1} GB의 모든 내용을 지우고 설치합니다." -f $selected.FriendlyName,($selected.Size/1e9)) -ForegroundColor Yellow
  if ((Read-Host '진행하려면 설치 입력') -cne '설치') { throw '설치를 취소했습니다. SD카드는 변경하지 않았습니다.' }
  Assert-SameDisk (Get-Disk -Number $number) $selected $release.image_bytes $sourceDisk
  $logDir = Join-Path $root 'logs'
  $null = New-Item -ItemType Directory -Force -Path $logDir
  $log = Join-Path $logDir (Get-Date -Format 'yyyyMMdd-HHmmss-fff')
  Write-Host '기록과 검증을 시작합니다. 완료될 때까지 카드를 빼거나 창을 닫지 마세요.'
  & "$PSScriptRoot\write_sd_windows.ps1" -Image $image -Sha256 $release.prepared_sha256 `
    -DiskNumber $number -SerialNumber ([string]$selected.SerialNumber) -UniqueId ([string]$selected.UniqueId) `
    -DiskBytes $selected.Size -Log ($log + '.txt')
  if (-not (Test-Path -LiteralPath ($log + '.txt.success'))) { throw '기록 완료를 확인하지 못했습니다. logs 폴더를 확인하세요.' }
  Write-Host ''
  Write-Host '설치 완료! SD카드를 안전하게 제거하여 Jetson에 꽂고 전원을 켜세요.' -ForegroundColor Green
  Write-Host 'Wi-Fi 정보는 연결한 콤마에서 자동으로 받습니다.'
} catch {
  Write-Host ''
  Write-Host $_.Exception.Message -ForegroundColor Red
  if ($elevated) { $null = Read-Host 'Enter를 누르면 창을 닫습니다' }
  exit 1
}
if ($elevated) { $null = Read-Host 'Enter를 누르면 창을 닫습니다' }
