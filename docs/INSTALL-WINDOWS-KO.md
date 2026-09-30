# 🥕 Jetson 설치 안내

*Carrot Jetson · Installation guide*

**파일을 받고, 01 → 02 순서로 실행하세요. 나머지는 배치 파일이 알아서 합니다.**

> Download, extract, then run 01 followed by 02. The scripts handle the rest.

### 📦 [설치파일 받기](https://upload.shind0.synology.me/downloads/jetson/v0.4.0-boot-preview/carrot-jetson-windows.zip)

*Download installer · 10.1 GB*

Windows 10/11 64비트 · PC 여유 공간 약 70GB · 64GB 이상 microSD와 USB 리더를 준비하세요.

> Requires 64-bit Windows 10/11, about 70 GB free, a 64 GB+ microSD card and a USB reader.

**M.2 NVMe SSD에 설치할 분:** 같은 설치파일을 사용하고, **01 → 02 → 03 공용 패치** 순서로 진행하세요. 아래에 패치 다운로드와 실행 방법이 있습니다. 이미지를 새로 받을 필요는 없습니다.

> **Installing to M.2 NVMe?** Use the same installer, then **01 → 02 → 03 common patch**. See the patch download and instructions below. No new image download is needed.

**9월 30일 수정 — 공용 패치 v2:** v1에 초기 부팅을 막는 오류가 있어 교체했습니다. 이미 SSD에 설치·패치했다면 **새 패치를 기존 폴더에 덮어 풀고 03만 다시 실행**하세요. 01·02나 이미지 재기록은 필요 없습니다.

> **September 30 fix — common patch v2:** v1 contained an early-boot defect. For an already installed/patched SSD, **extract the new patch over the existing folder and rerun only 03**. Skip 01 and 02; no image rewrite is needed.

---

### ① 압축을 모두 풀기

*Extract all files*

다운로드한 파일에서 **모두 압축 풀기**를 선택하고, **CarrotJetson** 폴더를 엽니다.
폴더의 `설치안내.html`을 열면 보기 편한 안내 화면이 나옵니다.

> Choose **Extract all**, then open the **CarrotJetson** folder. Open **설치안내.html** for the visual step-by-step guide.

### ② `01_설치준비.cmd` 실행

*Prepare the installation image · About 5–15 min*

**예상 약 5~15분.** 두 번 클릭하고 기다리세요. 수정사항이 포함된 이미지 검사·압축 해제·최종 검증은 자동입니다. 기존 USB-C 수정은 포함되어 있습니다. **입력할 내용은 없습니다.** SSD용 공용 패치는 02 완료 후 적용합니다.

> Double-click and wait. Image checks, extraction and final verification run automatically. The USB-C fix is included. **No answers needed.** Apply the common SSD patch after step 02.

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

### SSD 사용 시 추가 · `03_SD_SSD공용패치.cmd`

*Additional step for NVMe · Common microSD/NVMe patch · About 5–20 min*

**[공용 패치파일 받기 · 약 32MB](https://upload.shind0.synology.me/downloads/jetson/v0.4.0-sd-nvme-patch-v2-preview/carrot-jetson-windows.zip)**

> **Download the common patch · About 32 MB**

패치 내용을 기존 **CarrotJetson 폴더 안에** 풀고 `support` 폴더를 합쳐 주세요. **02가 끝난 뒤**, Jetson에 넣기 전에 **03_SD_SSD공용패치.cmd**를 실행합니다. 관리자 권한 **예** → 대상 **디스크 번호** → **PATCH** 입력. 나머지 검사·패치·기록 확인은 자동입니다.

> Extract the patch inside your existing **CarrotJetson folder**, merging `support` folders and replacing matching files. **After 02**, before installing the medium in Jetson, run **03_SD_SSD공용패치.cmd**. Choose administrator **Yes**, enter the **disk number**, then type **PATCH**. Verification, patching and readback are automatic.

SSD는 **64GB 이상 M.2 NVMe + NVMe용 USB 외장 케이스**를 사용해 PC에서 기록·패치한 뒤 Jetson의 NVMe 슬롯에 장착합니다. 02의 이름은 SD카드 설치이지만 USB 케이스의 SSD도 선택할 수 있습니다. PC 내부 SSD는 선택되지 않습니다.

> Use a **64 GB+ M.2 NVMe SSD and a USB NVMe enclosure** for PC writing and patching, then install it in Jetson's NVMe slot. Step 02 retains its SD-card filename but also lists eligible USB-enclosed SSDs. Internal PC SSDs are excluded.

**이 페이지의 R2 이미지 전용 시험 패치입니다.** microSD에도 같은 패치를 적용할 수 있습니다. 이미 R2를 기록했다면 다시 기록하지 않고 03만 실행하세요. 실제 패치 후 SD·NVMe 부팅은 아직 검증 전이므로 정상 작동하던 SD는 보관하고, 복제된 SD와 NVMe를 동시에 장착하지 마세요. [자세한 패치 안내](jetson_sd_nvme_patch.md)

> **Experimental patch for this page's R2 image only.** The same patch also applies to microSD. Already recorded R2 media needs only step 03. Physical patched SD/NVMe boot is unverified: keep your working SD as a fallback and never install both clones together. See the detailed patch guide linked above.

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
