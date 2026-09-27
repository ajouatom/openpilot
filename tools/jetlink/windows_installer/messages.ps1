function Show-InstallerIntro([string]$Stage) {
  Write-Host ''
  if ($Stage -eq 'Prepare') {
    Write-Host '01 설치 준비 / Prepare installation' -ForegroundColor Cyan
    Write-Host '이미지 검사, 압축 해제, USB-C 수정, 최종 검사를 자동으로 합니다.'
    Write-Host 'Automatically checks, extracts and prepares the image with the USB-C fix.'
    Write-Host '예상 약 5~15분. PC 속도에 따라 더 걸릴 수 있습니다. / About 5-15 min; slower PCs may take longer.'
    Write-Host '질문 없이 진행합니다. 완료되면 02를 실행하세요. / No answers needed. Run 02 when finished.'
    Write-Host 'PC 파일만 준비하며 SD카드는 변경하지 않습니다. / Prepares PC files only; does not write the SD card.'
  } else {
    Write-Host '02 SD카드 설치 / Install to SD card' -ForegroundColor Cyan
    Write-Host '선택한 SD카드를 지우고 이미지를 기록한 뒤 전체 내용을 검사합니다.'
    Write-Host 'Erases the selected SD card, writes the image and verifies the entire written image.'
    Write-Host '예상 약 15~60분. 느린 카드/리더는 더 걸립니다. / About 15-60 min; slow cards/readers take longer.'
    Write-Host '답변 순서 / How to answer:'
    Write-Host '  1) 관리자 권한 창: 예 / Yes in the Windows administrator prompt.'
    Write-Host '  2) 카드 목록: 대괄호 안 번호만 입력 후 Enter / Type only the card number in brackets, then Enter.'
    Write-Host '  3) 삭제 확인: 설치 또는 INSTALL 입력 후 Enter / Type INSTALL (or 설치), then Enter to confirm erasing.'
    Write-Host '번호/삭제 확인에서 다른 입력은 취소입니다. / Invalid selection or confirmation cancels installation.'
    Write-Host '중요한 파일은 먼저 백업하세요. / Back up important files first.' -ForegroundColor Yellow
    Write-Host '작업 중 카드·리더를 빼거나 창을 닫거나 PC 전원을 끄지 마세요.' -ForegroundColor Yellow
    Write-Host 'Do not unplug the card/reader, close this window or turn off the PC during installation.' -ForegroundColor Yellow
  }
  Write-Host ''
}

function Show-JetsonConnectionSteps {
  Write-Host ''
  Write-Host '설치 완료 / Installation complete' -ForegroundColor Green
  Write-Host '1. PC에서 SD카드를 안전하게 제거하세요. / Safely eject the SD card from the PC.'
  Write-Host '2. Jetson을 정상 종료하고 전원 공급을 분리하세요. / Shut down Jetson, then disconnect its power supply.'
  Write-Host '3. 전원이 완전히 꺼진 뒤 SD카드를 꽂고 콤마 연결을 준비하세요.'
  Write-Host '   Insert the SD card and connect to comma only after Jetson is fully powered off.'
  Write-Host '4. 보드에 맞는 전원을 연결하고 켜세요. / Reconnect the correct power supply and turn Jetson on.'
  Write-Host '켜진 상태에서 SD카드를 넣거나 빼지 마세요. / Never insert or remove the SD card while powered on.' -ForegroundColor Yellow
  Write-Host '차량은 안전하게 주차하고 주행 보조를 해제한 상태에서 작업하세요.'
  Write-Host 'Work only while safely parked with driver assistance disengaged.'
  Write-Host 'Wi-Fi 정보는 연결한 콤마에서 자동으로 받습니다. / Wi-Fi settings come automatically from the connected comma.'
}
