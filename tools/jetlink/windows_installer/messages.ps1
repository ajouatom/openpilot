function Write-InstallerPair([string]$Korean, [string]$English, [string]$Color = 'White') {
  Write-Host "  $Korean" -ForegroundColor $Color
  Write-Host "  $English" -ForegroundColor DarkGray
}

function Write-InstallerHeading([string]$Korean, [string]$English) {
  Write-Host ''
  Write-Host '  ------------------------------------------------------------' -ForegroundColor DarkGray
  Write-InstallerPair $Korean $English Cyan
  Write-Host '  ------------------------------------------------------------' -ForegroundColor DarkGray
  Write-Host ''
}

function Read-InstallerInput([string]$Korean, [string]$English) {
  Write-Host ''
  Write-InstallerPair $Korean $English Cyan
  Read-Host '  >'
}

function Show-InstallerIntro([string]$Stage) {
  if ($Stage -eq 'Prepare') {
    Write-InstallerHeading 'CARROT JETSON  |  01. 설치 준비' 'Prepare your installation image'
    Write-InstallerPair '예상 시간  5~15분' 'Estimated time: 5-15 min; slower PCs may take longer.'
    Write-Host ''
    Write-InstallerPair '이미지 검사 → 압축 해제 → 필요한 수정 자동 반영 → 최종 검사' 'Check image > Extract > Include required fixes > Verify'
    Write-Host ''
    Write-InstallerPair '입력할 내용은 없습니다. 완료되면 02_SD카드설치.cmd를 실행하세요.' 'No answers needed. When finished, run 02_SD카드설치.cmd.'
    Write-InstallerPair 'PC에 파일을 준비합니다. SD카드 기록은 다음 단계입니다.' 'This step prepares files on the PC. Card writing is the next step.'
  } else {
    Write-InstallerHeading 'CARROT JETSON  |  02. SD카드 설치' 'Write and verify your SD card'
    Write-InstallerPair '예상 시간  30~90분' 'Estimated time: 30-90 min; slow cards or readers may take longer.'
    Write-InstallerPair '선택한 카드 삭제 → 이미지 기록 → 전체 기록 검사' 'Erase selected card > Write image > Verify the entire written image'
    Write-Host ''
    Write-InstallerPair '화면에서 이렇게 답하세요' 'How to answer' Cyan
    Write-InstallerPair '① 관리자 권한 창: 예를 선택하세요.' '1. Windows administrator prompt: choose Yes.'
    Write-InstallerPair '② 카드 선택: 대괄호 안 번호만 입력하고 Enter를 누르세요.' '2. Card selection: type the number in brackets, then press Enter.'
    Write-InstallerPair '③ 삭제 확인: 설치 또는 INSTALL을 입력하고 Enter를 누르세요.' '3. Erase confirmation: type INSTALL, then press Enter.'
    Write-InstallerPair '카드 번호나 삭제 확인이 맞지 않으면 취소됩니다.' 'An invalid card number or erase confirmation cancels installation.'
    Write-Host ''
    Write-InstallerPair '주의  선택한 카드의 모든 파일이 지워집니다. 먼저 백업하세요.' 'Caution: all files on the selected card will be erased. Back up first.' Yellow
    Write-InstallerPair '진행 중 카드·리더를 빼거나 창을 닫거나 PC 전원을 끄지 마세요.' 'Do not unplug the card/reader, close this window or power off the PC.' Yellow
  }
  Write-Host ''
}

function Show-JetsonConnectionSteps {
  Write-InstallerHeading '설치 완료  |  이제 Jetson에 연결하세요' 'Installation complete — connect your Jetson'
  Write-InstallerPair '1. PC에서 SD카드를 안전하게 제거하세요.' 'Safely eject the SD card from the PC.'
  Write-Host ''
  Write-InstallerPair '2. Jetson을 정상 종료하고 전원 공급을 분리하세요.' 'Shut down Jetson, then disconnect its power supply.' Yellow
  Write-Host ''
  Write-InstallerPair '3. 전원이 완전히 꺼진 뒤 카드를 꽂고 콤마에 연결하세요.' 'Once fully powered off, insert the card and connect to comma.'
  Write-Host ''
  Write-InstallerPair '4. 보드에 맞는 전원을 연결하고 Jetson을 켜세요.' 'Reconnect the correct power supply and turn Jetson on.'
  Write-Host ''
  Write-InstallerPair '켜진 상태에서 SD카드를 넣거나 빼지 마세요.' 'Never insert or remove the SD card while powered on.' Yellow
  Write-InstallerPair '차량은 안전하게 주차하고 주행 보조를 해제한 상태로 작업하세요.' 'Work only while safely parked with driver assistance disengaged.'
  Write-Host ''
  Write-InstallerPair 'Wi-Fi 정보는 연결한 콤마에서 자동으로 받습니다.' 'Wi-Fi settings come automatically from the connected comma.'
}
