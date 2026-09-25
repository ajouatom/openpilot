# Jetson 배포 자동화 검토 — 2026-09-25

대상은 `carrot-jetlink` 실험 브랜치의 Orin Nano Super 구성이다.
현재 차량에 설치된 JetPack/L4T 36.4.7, TensorRT 10.3 구성의 수동 설치와
소프트웨어 재부팅은 확인했지만, 새 장치의 완전 자동 설치와 자동 업데이트는
아직 검증하지 않았다. Mac은 Apple Silicon용 실행 스크립트만 있으며 실기 검증 전이다.

## 현재 제공하는 코드

| 단계 | 코드 | 현재 범위 |
|---|---|---|
| 호스트 소스 배포 | `tools/jetlink/build_host_bundle.py` | 커밋된 허용 경로만 묶고 `SOURCE_COMMIT`, SHA-256 파일, 배포 JSON 생성. 수정/미추적 호스트 소스가 있으면 거부 |
| 설치 상태 점검 | `tools/jetlink/inspect_jetson.py` | 읽기 전용 JSON 보고서. Jetson/패키지/소스 버전/캐시 계약/원본 해시/서비스/전원 모드 확인 |
| 모델 다운로드 | `tools/jetlink/download_model.py` | NAS manifest와 브랜치의 크기·해시·URL 대조, 임시 파일 다운로드 후 검증. 다른 기존 모델은 덮어쓰지 않음 |
| 엔진 준비 | `server.py --build` / `prepare_cache.py` | 해당 Jetson에서 원본 ONNX로 생성하거나 검증된 로컬 엔진을 가져오기 |
| 추론 자동 시작 | `install_server.sh` | USB 권한, systemd 서비스, 재시작 및 연결 대기 |
| USB 화면 | `install_hud.sh`, `install_headless.sh` | 표시 서비스, NVIDIA Xorg, 로그인 없는 실행 환경 |
| 정격 성능 모드 | `install_performance.sh` | 이미 선택된 MAXN_SUPER 확인 후 `jetson_clocks` 자동 실행 |
| Mac | `run_mac.sh` | Apple Silicon 확인, venv, 모델 검증, CoreML 준비 및 실행. 상주 서비스 설치는 미구현 |

새 점검기는 엔진을 실행하거나 서비스를 바꾸지 않는다. `static_checks_ok`는
정적 설치 자료의 검사 결과이며, 서비스 상태는 별도 필드다. TensorRT 엔진의
실제 로드 성공, GPU 호환성, USB 연결 품질 및 50 ms 충족을 보증하지 않는다.
SHA-256 sidecar도 다운로드 손상 확인용이며 배포자 서명은 아니다.

## 준비된 장치에 배포할 때의 절차

1. 실험 브랜치의 검증한 커밋에서 다음을 실행한다. 차량 기록·인증 정보는 포함하지 않는다.

   ```sh
   python tools/jetlink/build_host_bundle.py /path/to/carrot-jetlink-host.tar.gz
   ```

   결과는 tar.gz, tar.gz.sha256, tar.gz.json 세 파일이다. 세 파일을 SSH/SCP로
   Jetson에 복사하고 내려받은 디렉터리에서 `sha256sum -c carrot-jetlink-host.tar.gz.sha256`로 확인한다.
   호스트 번들은 차량 전체 저장소나 ONNX/TensorRT 엔진을 포함하지 않는다.

2. **차량 USB를 분리한 정비 상태**에서 소스를 준비한다. 최초 설치의 런타임은
   `RUNTIME/{carrot,venv,cache}` 구조이고, 번들은 `RUNTIME/carrot` 아래에 푼다.
   기존 설치 업데이트는 실행 중인 `carrot` 위에 덮어 풀지 않는다. 새 버전은
   별도 디렉터리에 풀고 기존 소스/venv/서비스 파일을 보존한 뒤 전환한다.

3. JetPack의 TensorRT/CUDA를 그대로 사용하는 `--system-site-packages` venv에
   `requirements-jetson.txt`를 설치한다. 시스템 Python이나 JetPack 드라이버를
   일반 pip TensorRT 패키지로 교체하지 않는다. 시스템의 venv, USB, GLFW/OpenGL,
   Xorg/xauth, FFmpeg 의존성은 새 장치의 배포판에 맞춰 별도 준비해야 한다.

4. 원본을 캐시의 `models/09d080f36965bb2a.onnx`로 다운로드하고 검증한 다음,
   **최초 설치/정비 중에** 엔진을 생성한다. 예시의 `RUNTIME`은 설치 경로로 바꾼다.

   ```sh
   RUNTIME=/home/USER/carrot-jetlink
   "$RUNTIME/venv/bin/python" "$RUNTIME/carrot/tools/jetlink/download_model.py" \
     --output "$RUNTIME/cache/models/09d080f36965bb2a.onnx"
   "$RUNTIME/venv/bin/python" "$RUNTIME/carrot/tools/jetlink/server.py" \
     --backend trt --cache "$RUNTIME/cache" \
     --build "$RUNTIME/cache/models/09d080f36965bb2a.onnx"
   "$RUNTIME/venv/bin/python" "$RUNTIME/carrot/tools/jetlink/inspect_jetson.py" \
     --runtime "$RUNTIME" --verify-model
   ```

   기존 엔진이 있다는 이유만으로 사용하지 않는다. 모델 계약, GPU, TensorRT 버전과
   실제 로드/추론을 확인한다. `--build`는 존재하는 엔진을 건너뛸 수 있으므로
   그 명령의 성공만으로 엔진 검증을 대체할 수 없다. 새 릴리스에서 모델이 바뀌면
   위 고정 파일명도 해당 릴리스의 manifest를 따라야 한다.

5. 기존 [설치 절차](jetlink_experiment.md#jetson-installation)에 따라 성능/추론/
   화면/headless 서비스를 설치한다. 현재 스크립트는 서비스를 즉시 시작하고,
   headless 설치는 데스크톱을 종료하므로 처음부터 SSH로 작업한다. SSH/Wi-Fi
   접속과 재부팅 후 자동 연결을 확인한 뒤 차량 USB를 연결하여 정차 시험한다.

6. 차량과 호스트의 소스 버전, 실제 표시 이름, USB 5 Gbit/s, 모델 신원,
   DM/USB 화면을 포함한 추론 지연·프레임 누락·pose validity를 확인한다.
   실패하면 보존한 이전 **소스+venv+서비스 구성**으로 되돌린다.
   전원/화면 설정을 변경했다면 저장된 Xorg/default-target/전원 설정도 별도로 복구한다.

현재 설치 스크립트의 경로/계정 인자를 systemd 파일에 직접 넣으므로, 기존 절차는
공백·개행·systemd 특수 문자가 없는 전용 설치 경로와 일반 Linux 계정명을 사용한다.
배포 자동화에서 이를 입력 검증 및 systemd escaping으로 강제해야 한다.

## 통합 자동화의 권장 구성과 미구현 사항

`prepare → validate → activate → health-check → rollback`을 분리하는 것이 적합하다.

* **prepare:** Jetson/JetPack/디스크/계정 사전 점검, 설치 잠금, 의존성 설치,
  버전별 소스·venv 준비, 검증된 모델 다운로드와 엔진 생성. 네트워크 실패는
  제한된 재시도와 이어받기를 제공하고, 서비스 파일은 아직 바꾸지 않는다.
* **validate:** 모델 전체 해시/입출력 계약/엔진 실제 로드/유한 출력과 기준 결과 비교.
  `inspect_jetson.py`의 정적 점검은 이 중 일부만 수행한다.
* **activate:** 추론 USB가 분리된 정비 상태를 확인한 뒤 서비스를 멈추고,
  버전 디렉터리의 `current` 링크와 서비스 구성을 전환한다. 현재 고정 경로
  서비스는 이 릴리스 구조로 한 번 마이그레이션해야 한다.
* **health-check / rollback:** 단순 `systemctl is-active` 외에 엔진 준비 상태,
  서비스 재시작 횟수 및 지정 시간 내 준비 완료를 확인한다. 실패 시 직전
  소스·venv·서비스로 자동 복구하고 원인을 남긴다. USB/차량 검증은 별도로 수행한다.
* **boot:** 이미 준비한 엔진으로 즉시 서버를 띄우고 C3/C4 연결을 기다린다.
  부팅 경로에 모델 다운로드, pip 설치, TensorRT 컴파일을 넣지 않는다.
* **update:** 시동 중 자동 `git pull`/pip/서비스 재시작을 하지 않는다. 업데이트는
  미리 내려받아 준비한 뒤 USB 분리 상태에서 적용한다. 시동 전원이 끊길 때의
  업데이트 중단과 복구도 검증해야 한다.

현재 통합 bootstrap, 원자적 버전 전환, 자동 복구, 다운로드 이어받기,
부팅 health gate, Mac launchd 설치는 **미구현**이다. 완전 무인 배포가 가능하다고
보고해서는 안 된다. 다음 구현은 준비/전환을 분리한 설치 도구부터 시작하고,
사용 중인 장치와 다른 빈 런타임에서 실패 주입 및 반복 설치를 검증한다.

## 장치 이름 표시와 검증 범위

호스트 hello의 `carrot_host`로 `jetSON`/`MAC`을 구분한다. 이전 서버의 Orin/TRT,
CoreML 정보도 인식하고 알 수 없는 호스트는 `Jetlink`로 남긴다. 주행 화면은
활성/준비/대기/오류를 구분하며, USB 계기판은 활성 장치 이름만 표시한다.
eGPU가 활성화된 경우 기존 eGPU 표시가 우선이다. 표시 이름은 모델 선택이나
USB 대역폭/스케줄링 정책을 변경하지 않는다. eGPU 전용 전원·펌웨어 진단 메뉴는
실제 eGPU 기능이므로 이름을 Jetson/Mac으로 바꾸지 않는다.

데스크톱에서 장치 식별/오래된 상태 제거/계기판 표시/기존 HUD 테스트 및 실제
계기판 렌더러의 배지 이미지 확인을 수행했다. 새 호스트 코드의 차량 배포,
새 Jetson 설치, Mac 실물 확인은 차량/장치가 다시 준비된 뒤 수행해야 한다.
