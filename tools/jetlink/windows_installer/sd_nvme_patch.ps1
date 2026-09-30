param([switch]$Elevated)
$ErrorActionPreference = 'Stop'
try {
  [Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
  $env:PYTHONUTF8 = '1'
  $env:PYTHONIOENCODING = 'utf-8'
  . "$PSScriptRoot\messages.ps1"
  . "$PSScriptRoot\disks.ps1"
  Write-InstallerHeading 'CARROT JETSON | SD·NVMe 공용 패치 v2' 'Common microSD / M.2 NVMe patch v2'
  Write-InstallerPair 'v1 부팅 오류를 수정합니다. 이미 기록·패치한 SSD는 01·02 없이 이 파일만 다시 실행하세요.' 'Fixes the v1 boot failure. For an already written/patched SSD, rerun only this step; skip 01 and 02.'
  Write-InstallerPair '기존 R2 이미지의 부팅 파일만 수정합니다. 이미지를 다시 받거나 만들지 않습니다.' 'Patches boot files in the existing R2 image. No image rebuild or download.'
  Write-InstallerPair '예상 시간 5~20분. 느린 리더에서는 더 걸릴 수 있습니다.' 'Estimated time: 5–20 minutes; slow readers may take longer.'
  Write-InstallerPair '실제 NVMe 부팅은 아직 검증 전인 시험 패치입니다. 정상 동작하던 카드는 보관하세요.' 'Test patch: physical NVMe boot is unverified. Keep your working card as a fallback.' Yellow
  Write-InstallerPair '대상 내용을 먼저 백업하세요. 작업 중 분리·창 닫기·전원 끄기 금지.' 'Back up first. Do not unplug, close this window or power off during patching.' Yellow
  Write-InstallerPair '처음 설치라면 기존 설치파일의 01 → 02 완료 후, Jetson에 넣기 전에 실행하세요.' 'For a fresh install, finish the existing installer steps 01 and 02 before running this patch.'
  Write-InstallerPair 'SSD는 NVMe용 USB 외장 케이스로 PC에 연결하세요. PC 내부 SSD는 선택되지 않습니다.' 'Connect the NVMe through a USB NVMe enclosure. Internal PC SSDs are excluded.'
  $python = Join-Path $PSScriptRoot 'python\python.exe'
  if (-not (Test-Path -LiteralPath $python)) { throw "패치를 기존 CarrotJetson 폴더에 풀어 support 폴더를 합쳐 주세요.`nExtract this patch into the existing CarrotJetson folder, merging support folders." }
  $root = Split-Path -Parent $PSScriptRoot
  if ($root -notmatch '^[A-Za-z]:\\') { throw 'Extract to a local PC drive.' }
  $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
  $principal = [Security.Principal.WindowsPrincipal]::new($identity)
  if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    $null = Read-InstallerInput 'Enter를 누른 뒤 관리자 권한 창에서 예를 선택하세요' 'Press Enter, then choose Yes in the administrator prompt'
    $child = Start-Process powershell.exe -Verb RunAs -Wait -PassThru -ArgumentList @('-NoProfile','-ExecutionPolicy','Bypass','-File',('"{0}"' -f $PSCommandPath),'-Elevated')
    exit $child.ExitCode
  }
  $meta = Get-Content -LiteralPath "$PSScriptRoot\sd-nvme-release.json" -Raw -Encoding UTF8 | ConvertFrom-Json
  $source = @(Get-Partition -DriveLetter $root.Substring(0,1))
  if ($source.Count -ne 1) { throw 'Cannot identify the source disk. Extract to C: and retry.' }
  $choices = @(Get-Disk | Where-Object { Test-InstallDisk $_ $meta.image_bytes $source[0].DiskNumber })
  if ($choices.Count -eq 0) { throw "대상 USB 카드/SSD가 없습니다. USB 리더/케이스를 확인하세요.`nNo eligible USB card/SSD. Check the USB reader/enclosure." }
  Write-InstallerPair '패치할 카드/SSD를 선택하세요.' 'Select the card/SSD to patch.' Cyan
  foreach ($disk in $choices) { Write-Host ("[{0}] {1} / {2:N1} GB / ID: {3}" -f $disk.Number,$disk.FriendlyName,($disk.Size/1e9),$disk.SerialNumber) }
  $answer = Read-InstallerInput '대괄호 안 디스크 번호 입력' 'Enter the disk number in brackets'
  $number = 0
  if (-not [int]::TryParse($answer,[ref]$number)) { throw 'Cancelled; nothing written.' }
  $selected = @($choices | Where-Object Number -eq $number)
  if ($selected.Count -ne 1) { throw 'Number not listed; nothing written.' }
  $selected = $selected[0]
  Write-Host ("{0} / {1:N1} GB" -f $selected.FriendlyName,($selected.Size/1e9))
  $answer = Read-InstallerInput '이 장치를 패치하려면 PATCH 입력' 'Type PATCH to patch this device (anything else cancels)'
  if ($answer.Trim() -cne 'PATCH') { throw 'Cancelled; nothing written.' }
  Assert-SameDisk (Get-Disk -Number $number) $selected $meta.image_bytes $source[0].DiskNumber
  $logs = Join-Path $root 'logs'
  $null = New-Item -ItemType Directory -Path $logs -Force
  $log = Join-Path $logs ((Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '-sd-nvme.txt')
  & "$PSScriptRoot\apply_offline_hotfix_windows.ps1" -DiskNumber $number -SerialNumber ([string]$selected.SerialNumber) -UniqueId ([string]$selected.UniqueId) -DiskBytes $selected.Size -Python $python -Manifest "$PSScriptRoot\sd-nvme-patch.json.gz" -ManifestSha256 $meta.patch_sha256 -Log $log
  if (-not (Test-Path -LiteralPath ($log + '.success'))) { throw 'Patch completion not verified; check logs.' }
  Write-InstallerHeading '패치 기록·검증 완료' 'Patch written and read back'
  Write-InstallerPair '안전하게 제거 → Jetson 정상 종료·전원 분리 → SD 또는 NVMe 장착 → 전원 연결' 'Safely eject > Shut down and unplug Jetson > Install SD or NVMe > Power on'
  Write-InstallerPair '복제된 SD와 NVMe를 동시에 꽂지 마세요. 사용할 매체 하나만 장착하세요.' 'Do not connect cloned SD and NVMe together. Install only the boot medium you will use.' Yellow
  Write-InstallerPair '나중에 02로 이미지를 다시 기록하면 이 패치도 다시 실행해야 합니다.' 'If you rewrite the original image with step 02 later, apply this patch again.'
  Write-InstallerPair '첫 시험은 안전한 정차 상태에서 부팅·Wi-Fi·모델 연결을 확인하세요.' 'For the first test, verify boot, Wi-Fi and model connection while safely parked.'
} catch {
  Write-Host $_.Exception.Message -ForegroundColor Red
  if ($Elevated) { Read-Host 'Enter' }
  exit 1
}
if ($Elevated) { Read-Host 'Enter' }
