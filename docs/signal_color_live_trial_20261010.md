# 신호 색 모델의 실시간 비교 기록 — 2026-10-10

사용자가 새 신호 분류 모델을 실제 주행 영상에 연결해 시험하도록 요청했다.
`carrot-signal-shadow`에 별도 `signalcolord` 프로세스를 추가한다.
이 프로세스는 원본 road 카메라를 읽고 `logMessage`만 기록한다.
기존 modeld·모델 출력·정지/출발 조건·조향/가감속·이전 정책 비교기는 그대로다.
화면 표시나 주행 제어를 추가하는 변경은 아니다.

## 실행과 기록

manager는 시동 중이고 `/data/signal-color-shadow/enabled`가 `1`일 때 실행한다.
기본은 OFF이며 기기 로컬에 명시적으로 설치한 모델/manifest가 필요하다.
프로세스는 새 Python 인터프리터로 시작하고 CPU 0~3, SCHED_OTHER, nice19,
ONNX Runtime CPU 단일 스레드, busy spinning OFF를 사용한다.
정상 동작 중 최대 1Hz 및 평균 CPU 시간 한 코어의 25% 이하를 목표로 하며
느린 프레임을 따라잡으려 하지 않는다. 시작 시 import/모델 준비는 이 duty 제한 밖이다.
이는 코어 격리나 OS CPU quota가 아니다. 다른 프로세스와 메모리/CPU 경합은 가능하다.

입력은 conflated VisionIPC road NV12이다. stride와 UV offset, 유효 크기를 확인하고
BT.601 limited-range 행렬로 RGB를 만든다. 원본 버퍼를 변경하지 않고 읽은 뒤 frame ID가
바뀌었으면 폐기한다. 300ms보다 오래된 입력은 건너뛰고 결과가 1.5초보다 오래되면
`unknown`, `usable=false`로 기록한다. `usable`은 점수/시간 조건일 뿐 신호의 정확성,
진행 차로와의 연관성, 출발 허용을 뜻하지 않는다.

1344x760 기준 화면 상단 60%, 중앙 90%의 667개 창을 이전 오프라인 실험과 같은
방법으로 처리한다. 각 창은 독립적으로 bilinear 96x32 변환 후 32개씩 ONNX에 입력한다.
배치 atlas는 Python↔NumPy 변환 횟수만 줄이고 개별 입력 픽셀은 원래 실험과 같다.
최대 적/녹 점수 0.8을 넘으면 그 색을 기록한다. 점수는 보정된 신뢰도가 아니다.
최고 창과 겹침이 적은 후순위 후보를 최대 3개 남기며 후순위 정리는 최고 판정을 바꾸지 않는다.
차로 연관성이나 신호등 몸체 검출을 새로 해결한 모델은 아니다.

`signalColorShadow` 이벤트는 다음을 담는다.

- 모델 SHA256, 카메라 stream, frame ID, EOF timestamp, 결과 시각.
- 원본/기준 화면 크기, 각 후보 위치·색·3클래스 점수.
- 최종 적색/녹색/unknown, 점수·시간 조건 통과 여부와 이유.
- 입력/결과 나이, 전체 처리 시간, CPU 시간, 다음 대기 시간.

`signalColorShadowLoaded`는 시작/모델 정보, `signalColorShadowSkipped`는 오래되거나
재사용된 버퍼를 기록한다. 실제 신호 전환 시간과 기존 x/v 예측은 frame ID/EOF를 통해
영상, roadEncodeIdx, 원래 modelV2와 결합해 비교한다. 새 출력은 modelV2에 들어가지 않는다.
로그/카메라 전송이 끊기면 샘플이 빠질 수 있으며 실제 약 1.7초 샘플 간격은 빠른 전환의
정밀 지연 측정을 제한한다. 녹색을 읽는 것만으로 안전한 출발을 판단하지 않는다.

중복 실행은 파일 잠금으로 막는다. 일반적인 모델/카메라/실행 오류는 한 번 로그에 남기고
프로세스가 대기하여 manager의 반복 재시작을 피한다. OFF 또는 시동 재시작 후 다시 시작한다.
OS 강제 종료 등은 manager의 기존 프로세스 정책을 따른다. 상태 파일 `latest.json`은 마지막
관측이며 현재 살아 있음을 보장하지 않는다. 기록 시각과 새 이벤트를 함께 확인해야 한다.

## 사전 실기기 검증

P단, standstill, |vEgo|<0.01m/s, fresh/valid carState·selfdriveState,
disabled/inactive를 지속 확인했다. 정지 기준 15초와 관찰기를 포함한 65초를 비교했다.

| 항목 | 기준 | 관찰기 시험 구간 |
|---|---:|---:|
| modelV2 수신/유효 프레임 | 301/301 | 1301/1301 |
| 모델 주기 | 20.064Hz | 19.999Hz |
| frame ID 누락 / 최대 drop 비율 | 0 / 0% | 0 / 0% |
| 모델 실행 평균 | 24.931ms | 25.180ms |
| 모델 실행 최대 | 28.286ms | 35.528ms |

관찰기 시작 이벤트 1개와 결과 26개를 수신했다. 결과 간격 중앙값 1.751초,
전체 처리 중앙값 497.3ms, 최대 721.9ms이다. 고정 주차 장면에서 unknown 20개,
녹색 6개를 기록했다. 녹색 후보는 출입문/차량 반사 영역에 해당한다. 이 오감지도
그대로 보존하며 정상 신호 인식이라고 주장하지 않는다. 결과는 기록기가 돌아감을
검증한 것이고 주행 중 부하나 신호 정확성의 검증은 아니다.

NV12 원본 한 장을 FFmpeg 기본 NV12→RGB 변환과 비교한 평균 채널 차이는 1.566/255,
99백분위 4, 최대 10이었다. 크로마 보간/반올림 차이가 있어 픽셀 동일 재현은 아니며
압축 영상과 실제 작은 신호에서의 색 차이는 추가 검토 대상이다. 이전 영상은
HEVC yuv420p(tv) 1344x760이었다.

Windows 25개 focused tests는 모델 식별/기본 OFF, 패딩 NV12 색, 입력 불변,
잘못된 버퍼/확률 거부, 기존 crop 픽셀 일치, 최고 후보 보존, stale 결과 거부,
부하 간격 및 오류 대기 경로와 기존 정책 관찰기를 검증한다. Ruff가 통과했다.
최초 수동 실행에서는 차량 pydeps 경로 누락을 수정했고, 유한 시험의 종료 대기 시간도
수정했다. 첫 원시 trial JSON의 `execution_ms` 필드는 실제 seconds였으며 최종
`trial_summary.json`과 위 표는 ×1000 보정했다. 재현 스크립트도 수정했다.

## 기기 로컬 선택과 산출물

모델은 [색 특징 실험](signal_roi_trial_20261010.md)의 987-byte ONNX를 그대로 사용한다.
SHA256 `c091fad2b08d9a59e0e061413ec62ec181a77f371791d6f78e6a5bd61b148d06`을
코드와 설치 manifest에서 확인한다. 별도 runtime은 기존 `/data/signal-model-shadow/runtime/`를
사용하며 시스템 Python 설치를 수정하지 않는다. 개인 모델/영상은 Git에 포함하지 않는다.

```sh
cd /data/openpilot
python -m openpilot.selfdrive.modeld.signal_color_shadow status
python -m openpilot.selfdrive.modeld.signal_color_shadow off
python -m openpilot.selfdrive.modeld.signal_color_shadow on
```

정상 부팅 환경의 Python/PYTHONPATH에서 실행한다. 최초 코드 설치는 manager 재시작이
필요하다. 이후 on/off는 manager가 읽고 시동 중 반영한다. 이 선택은 개인 시험용 파일이며
일반 설정 메뉴/백업/공개 배포 모델 선택을 추가하지 않는다.

로컬 재현/검증 자료는 `.analysis/archive/2026-10-10-signal-color-live/`에 보관한다.
영구 설치 후 부팅·rlog 저장 검증 결과는 이 문서에 추가한다.
