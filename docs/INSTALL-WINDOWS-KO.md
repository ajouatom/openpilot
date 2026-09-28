# 🥕 Jetson 설치 안내

*Carrot Jetson · Installation guide*

**파일을 받고, 01 → 02 순서로 실행하세요. 나머지는 배치 파일이 알아서 합니다.**

> Download, extract, then run 01 followed by 02. The scripts handle the rest.

### 📦 [설치파일 받기](https://upload.shind0.synology.me/downloads/jetson/v0.4.0-boot-preview/carrot-jetson-windows.zip)

*Download installer · 10.1 GB*

Windows 10/11 64비트 · PC 여유 공간 약 70GB · 64GB 이상 microSD와 USB 리더를 준비하세요.

> Requires 64-bit Windows 10/11, about 70 GB free, a 64 GB+ microSD card and a USB reader.

---

### ① 압축을 모두 풀기

*Extract all files*

다운로드한 파일에서 **모두 압축 풀기**를 선택하고, **CarrotJetson** 폴더를 엽니다.
폴더의 `설치안내.html`을 열면 보기 편한 안내 화면이 나옵니다.

> Choose **Extract all**, then open the **CarrotJetson** folder. Open **설치안내.html** for the visual step-by-step guide.

### ② `01_설치준비.cmd` 실행

*Prepare the installation image · About 5–15 min*

**예상 약 5~15분.** 두 번 클릭하고 기다리세요. 수정사항이 포함된 이미지 검사·압축 해제·최종 검증은 자동입니다. 별도 핫픽스는 필요 없습니다. **입력할 내용은 없습니다.**

> Double-click and wait. Image checks, extraction and final verification run automatically. No separate hotfix is needed. **No answers needed.**

### ③ `02_SD카드설치.cmd` 실행

*Write and verify your SD card · About 30–90 min*

**예상 약 30~90분.** 카드 리더를 PC에 연결하고 두 번 클릭하세요. 안내를 읽고 **Enter**를 누르면 관리자 권한 창이 열립니다.

> Connect the card reader and double-click. Read the introduction and press **Enter** to open the administrator prompt.

**창에서 이렇게 답하세요**

> **How to answer**

1. 관리자 권한 창 → **예**
   > Administrator prompt → **Yes**
2. 카드 이름·용량 확인 → **대괄호 안 번호** 입력 → **Enter**
   > Check the card name and capacity → type its **number in brackets** → **Enter**
3. 삭제 확인 → **설치** 또는 **INSTALL** 입력 → **Enter**
   > Erase confirmation → type **INSTALL** → **Enter**

카드 번호나 확인 문구가 맞지 않으면 취소됩니다.

> An invalid card number or confirmation cancels installation.

### ④ 전원을 끄고 Jetson에 연결

*Power off before connecting to Jetson*

**PC에서 카드 안전하게 제거 → Jetson 정상 종료·전원 분리 → 완전히 꺼진 뒤 카드·콤마 연결 → 보드에 맞는 전원 연결 후 켜기**

> **Safely eject the card → shut down Jetson and disconnect power → once fully off, insert the card and connect comma → reconnect the correct power supply and turn on.**

Wi-Fi 정보는 연결한 콤마에서 자동으로 받습니다. 시스템은 읽기 전용으로 구성하고, 로그·임시 파일은 메모리에 저장합니다.

> Wi-Fi settings come automatically from the connected comma. The system is configured read-only; logs and temporary files use RAM.

---

### ⚠️ 시작 전, 이것만 확인하세요

*A few important precautions*

- **선택한 카드의 모든 파일이 지워집니다.** 중요한 파일은 먼저 백업하세요.
  > **All files on the selected card will be erased.** Back up important files first.
- 설치 중 카드·리더를 빼거나 창을 닫거나 PC 전원을 끄지 마세요.
  > Do not unplug the card/reader, close the window or power off the PC during installation.
- **Jetson이 켜진 상태에서는 카드를 넣거나 빼지 마세요.** 차량은 안전하게 주차하고 주행 보조를 해제하세요.
  > **Never insert or remove the card while Jetson is on.** Work while safely parked with assistance disengaged.

---

Orin Nano Super 기본 보드용입니다. 콤마는 Jetson 지원이 포함된 `carrot-wip`를 사용하세요.

> For the Orin Nano Super reference carrier. Use `carrot-wip` with Jetson support on comma.

시간은 예상치이며 PC·카드 속도에 따라 더 걸릴 수 있습니다. **공개 시험판:** 카드 전체 검증 완료·사용자 정상 동작 확인. 자동 업데이트 전환·Wi-Fi 복구·전원 차단 시험은 별도 확인 전입니다.

> Times depend on your hardware. **Public preview:** full card verification passed; owner reports normal operation. Update activation, Wi-Fi recovery and power-cut tests remain pending.
