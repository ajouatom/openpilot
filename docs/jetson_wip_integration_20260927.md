# Jetson 기능의 carrot-wip 통합 — 2026-09-27

사용자 요청에 따라 `carrot-jetlink`의 전체 이력(끝 커밋 `b9950442ca`)을
최신 `carrot-wip`(`c55e1e8346` 기준)에 병합했습니다. 병합 커밋은 `af9fdea29b`입니다. 파일 일부만 복사한 통합이 아닙니다.
이제 콤마의 배포·유지보수 기준은 `carrot-wip`이며, 기존 Jetson 전용 저장소
[`ajouatom/carrot-jetson`](https://github.com/ajouatom/carrot-jetson)은 그대로 유지합니다.

## 함께 들어오는 기능

- Jetson USB 추론 연결과 늦게 켜진 장치의 재연결, 기존 연결 실패 처리
- USB 계기판·네비 영상 전달과 수신 정체 개선
- 연결한 콤마의 Wi-Fi 정보 전달 및 변경 반영
- Carrot Web의 Jetson IP·온도·오류 상태와 차량 화면의 장치 표시
- 콤마 버전에 맞는 서명된 Jetson 업데이트 지정, 다운로드·검증·다음 부팅 적용
- 개인정보 없는 SD 이미지 제작, 기기별 초기 설정, USB-C 역할 핫픽스 도구
- Jetson 저장소에서 추가한 PC 오프라인 핫픽스 도구와 한글 설치 안내 연결

첫 설치 순서는 **NAS 이미지 다운로드 → PC에서 SD 기록 → 필요한 오프라인 핫픽스 → Jetson 첫 부팅**입니다.
자세한 프로그램·버튼·명령은 [초보자 한글 안내](https://github.com/ajouatom/carrot-jetson/blob/main/docs/INSTALL-WINDOWS-KO.md)에 있습니다.
일반 프로그램·모델 업데이트는 SD를 다시 기록하는 방식이 아닙니다.

## 모델과 기존 동작의 관계

| 실행 경로 | 통합 후 기준 |
|---|---|
| 콤마 내부 GPU | 기존 내부 모델 유지 |
| USB eGPU | 기존 Cinque v3 선택·실행 경로 유지 |
| Jetson | 별도로 고정한 Cinque v2 ONNX·입출력 계약 유지 |
| Jetson 업데이트 | 기존 서명된 `f2b22dc` 실행 패키지 지정 유지 |
| OS | 기존 carrot-wip AGNOS 요구 버전 유지; Jetson은 L4T 36.4.7 / TensorRT 10.3.0 |

차량 브랜치 통합 때문에 Jetson의 모델을 Cinque v3로 바꾸거나 NAS stable 채널을 자동 승격하지 않습니다.
기존 Jetson에 새 이미지가 필요해지는 변경도 아닙니다.
기존 eGPU가 선택된 경우의 우선순위, CAN·레이더 처리, 카메라·포즈 유효성 기준,
실제 연결 실패 시 오류/해제 정책은 통합 과정에서 완화하지 않습니다.

Jetson 연결 전에는 기존 내부 모델이 실행됩니다. 연결 전환은 신선하고 유효한 차량 상태와
기존 정차/물리적 조향 개입 조건을 지킵니다. 활성 추론이 실패하면 기존 `commIssue` 경로로
오류를 알리고 내부 모델로 복귀합니다. 단순히 선택 장치가 없는 정상 대기 상태와 실제 고장은 구분합니다.

## 배포와 한글 설명

GitHub 첫 화면, 초보자 설치 안내와 현재 릴리스 설명은 한글을 기본으로 제공합니다.
명령, 파일명, 프로그램이 출력하는 원문 오류는 정확한 복사·문의가 가능하도록 유지하고 한글로 뜻을 설명합니다.
Carrot Web의 알려진 연결 상태 사유도 선택한 언어로 표시합니다.
영문 실험 기록은 과거 측정 근거로 남기며 현재 설치 안내를 대신하지 않습니다.

실제 장치 이미지와 모델은 NAS, 코드·작은 도구·안내는 GitHub에 둡니다.
개인 Wi-Fi·SSH 개인키·사용 중 카드의 기기 식별 정보를 공용 배포물에 포함하지 않습니다.

## 검증 범위

Windows 통합 작업본에서 다음을 확인했습니다.

- Jetson USB·복귀·상태·Wi-Fi·업데이트·이미지·오프라인 핫픽스: **141 통과, Linux 전용 등 17 건너뜀**
- 네비 전달·계기판 장면: **119 통과**
- 기존 대형 모델·카메라 동기·일반 모델 런타임·워프·USB 대기·사전 컴파일: **111 통과, 1 건너뜀**
- 웹 빌드 성공. 전체 794개 중 792개 통과 후, 잘못 선택된 Python의 NumPy 부재는 올바른 실행기로 재시험하여 해결했습니다.
  Jetson·한글 상태·투영 관련 8개는 모두 통과했습니다. 남은 `logs_player_transport`의 뒤로가기 검사는
  이전 코드의 `history.back()`을 기대하지만 현재 기준 코드가 `goToSettingParent`를 사용하여 실패합니다.
  해당 구현·시험 파일은 기준 커밋 `c55e1e8346`과 동일하며 이 통합의 변경이 아닙니다.
- Windows에서 `test_dmonitoring_artifact.py`는 tinygrad 의존성 부재로 수집할 수 없었습니다.
  내부/DM 모델 파일과 선택·OS 요구 버전 파일은 이 통합으로 바뀌지 않습니다.
- 핫픽스 Python 소스는 Windows에서도 LF 줄바꿈을 유지하여 원본 해시·패키지 재현성을 보존합니다.

### Linux와 공개 배포 확인

- [전체 openpilot 빌드 및 통합 검사](https://github.com/ajouatom/openpilot/actions/runs/36311993748):
  병합 커밋 `af9fdea29b`의 SCons 빌드, 카메라 SOF·노출 검사, 모델·DM·관련 회귀 **629개** 모두 성공.
  Windows에서 수집하지 못했던 DM 아티팩트 검사도 이 Linux 실행에 포함됩니다.
- [Jetson 통합 CI](https://github.com/ajouatom/openpilot/actions/runs/36312101784): **158 + 119 = 277개 통과**.
  첫 실행에서 빠진 렌더러·비동기 시험 의존성을 추가한 뒤 전체 성공했습니다. 차량 실행 코드를 바꾼 수정은 아닙니다.
- [Jetson 저장소 CI](https://github.com/ajouatom/carrot-jetson/actions/runs/36312044436): 한글 안내 커밋 `2c07bcd` 성공.
- [NAS 이미지 CI](https://github.com/ajouatom/openpilot/actions/runs/36311993753): **291 + 33 + 254 + 881 = 1,459개 통과**.
- NAS 예약 업데이트가 `af9fdea29b32c8a1b5f6ce90e399f80142dd46cd`를 배포했습니다.
  공개 서비스의 `sourceCommit`, 실제 업로드 결과 페이지, 재계산 레이더 **1,196프레임**을 직접 확인했습니다.
  코드 지문은 `7d1a58c4bdcf7ab2041f`, 재계산 해시는
  `c3b2d2a21a4d2c1112a3c5e93815676fbaf74cb4bd4212f4c7373255cc9d2a41`로 배포 전 검사와 일치했습니다.
  후속 CI·문서 커밋은 NAS 실행 코드에 영향을 주지 않습니다.
- 공개 SD 다운로드의 크기·앞/뒤 Range 응답·메타데이터와 NAS 원본 일치를 확인했습니다.
  이미지와 핫픽스 ZIP은 재생성하지 않았고 기존 배포 해시를 유지합니다.
- 공개 릴리스 4개의 제목·본문을 한글화했고, 기본 이미지 릴리스의 첨부 설치 안내가 저장소 최신본과 일치합니다.
  실행 파일·서명 manifest·모델 핀은 안내 번역으로 바꾸지 않았습니다.

기존 `carrot-jetlink`의 C4 정차 시험과 통합된 `carrot-wip`의 실물·주행 시험은 별개입니다.
PC 오프라인 핫픽스 카드 첫 부팅, 다른 실물 콤마로 이동, C3와 주행 조건은 아직 통합 검증 완료로 간주하지 않습니다.
기존 이미지/USB-C 정책 시험은 [Jetson 업데이트 조사 기록](jetlink_updates.md)에 있습니다.
