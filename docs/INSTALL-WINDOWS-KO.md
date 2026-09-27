# Jetson 설치 / Installation

**[설치파일 받기 / Download installer](https://upload.shind0.synology.me/downloads/jetson/v0.3.1-windows-preview/carrot-jetson-windows.zip)**

1. PC에 **모두 압축 풀기** / **Extract all** on the PC.
2. **`01_설치준비.cmd`** — 이미지 준비·검사, 약 **5~15분**, 질문 없이 진행. / Prepare and verify, about **5–15 min**, no answers needed.
3. SD카드 연결 후 **`02_SD카드설치.cmd`** — 기록·검사, 약 **15~60분**. / Connect the SD card; write and verify, about **15–60 min**.
   화면 안내대로 **Enter → 예/Yes → 카드 번호 → 설치 또는 INSTALL**을 입력합니다. / Answer **Enter → Yes → card number → INSTALL (or 설치)**.
4. 완료 후 PC에서 **안전하게 제거**합니다. **Jetson 정상 종료 → 전원 분리 → 카드 삽입·콤마 연결 → 전원 켜기** 순서입니다.
   / **Safely eject**. **Shut down Jetson → disconnect power → insert card/connect comma → power on**.

**나머지는 배치 파일이 알아서 합니다. / The batch files handle the rest.**
시간은 대략적인 예상이며 PC·카드·리더 속도에 따라 더 걸릴 수 있습니다. / Slow hardware may take longer.

**주의 / Safety:** 선택한 카드 내용은 지워지므로 먼저 백업하세요. 작업 중 카드/리더를 빼거나 창을 닫거나 PC 전원을 끄지 마세요.
Jetson이 켜진 상태에서는 카드를 넣거나 빼지 마세요. 차량은 안전하게 주차하고 주행 보조를 해제하세요.
/ Back up the card before erasing. Do not unplug, close the window or power off the PC during installation.
Never insert/remove the card while Jetson is on. Work while safely parked with assistance disengaged.

Windows 10/11 x64, PC 여유 공간 약 45GB, USB 리더, 64GB 이상 microSD, Orin Nano Super 기본 보드용입니다.
/ Requires Windows 10/11 x64, about 45 GB free, USB reader, 64 GB+ microSD and Orin Nano Super reference carrier.
콤마는 Jetson 통합 이후 `carrot-wip`를 사용하세요. / Use `carrot-wip` with Jetson support on comma.

시험판: 실제 카드 기록·첫 부팅 시험은 아직 남아 있습니다. / Preview: physical card write/first-boot testing is pending.
