# 신경망 신호 관찰기 연결과 차량 GPU 비교 준비

이 문서는 최초 연결 중단 시점의 기록이다. 이후 실제 차량 설치, GPU 수치 비교,
내부/eGPU 전환 결과는 [후속 실측](signal_gpu_dual_model_20261011.md), 사용자가
추가 요청한 출발 차단은 [정차 적색 차단](signal_start_guard_20261011.md)을 따른다.

사용자는 표시 유지 수정만 설치된 상태와 새 검출기 미연결 상태를 구분해
설명하도록 요구했고, 시동 없이 카메라 시험을 진행하고 GPU 속도도 비교하도록
요청했다. 이 문서는 `signal_continuity_20261011.md`의 오프라인 검출기를 실제
카메라 관찰 경로에 연결하는 후속 작업이다.

## 구현

`signal_neural_shadow.py`는 기존 manager의 `signalcolord` 진입점에서
`/data/signal-color-shadow/neural_enabled=1`일 때 선택된다. 별도 opt-in이
없으면 기존 경로를 유지한다. `neural_backend`는 `cpu` 또는 `qcom`이며 누락 시
CPU, 잘못된 값은 오류다. 백엔드 전환은 관찰기를 껐다 켜서 적용한다.

- 기존 관찰기와 같은 파일 잠금을 사용하므로 표시 producer가 중복되지 않는다.
- 원본 모델과 독립된 두 고정 ONNX를 `neural/detector.onnx`,
  `neural/classifier.onnx`에서 읽고 SHA256을 확인한다. 파일을 Git에 넣지 않는다.
- 초기화/워밍업 후 ROAD VisionIPC의 최신 프레임을 받는다. 입력 age 150 ms,
  버퍼 재사용 확인, 영상 크기 검사를 유지한다. 단절/역순 시 추적을 초기화한다.
- 상단 ROI 검출과 프레임별 위치 추적/현재 이미지 색상 분류를 수행한다.
  `signalNeuralShadow`에 시간, frame ID, backend, 원래 조합 label/score를 기록한다.
- 화면에는 점수 0.5 이상 단독 green을 녹색, red 포함 조합을 적색으로 보낸다.
  yellow/unknown/낮은 점수는 unknown이다. 이는 표시 정책이며 차로/진행 방향을
  판정하지 않는다. 색상 후보는 확정 출발 허가로 표시하지 않는다.
- 기존 tmpfs 표시 endpoint를 쓰고 `engine=neural_cpu/neural_qcom`을 기록한다.
  앞서 구현한 최대 0.8초 이전 판독 표시를 공유한다.
- 제어 관찰 endpoint를 전혀 호출하지 않는다. 별도 assist flag가 켜져 있어도
  새 관찰기는 제어 입력을 발행하지 않는다. 기존 주행 ONNX/x/v는 수정하지 않는다.
- little cores/nice19, 최대20 Hz 및 CPU duty 목표50%의 기존 제한을 유지한다.
  이것은 실제20 Hz를 보장하지 않는다. 영상은 현재1344×760이며 원본2688×1520
  camerad 버퍼 연결은 이 변경에 포함하지 않는다.

`NeuralSignal`은 CPU ONNX Runtime과 tinygrad QCOM 실행기를 선택할 수 있다.
QCOM은 별도 프로세스의 `DEV=QCOM`, 고정 형태 입력, TinyJit, 동기 출력 복사를
사용한다. GPU 실패를 CPU로 숨기는 자동 fallback은 없다. 기존 내부 주행 모델의
QCOM 실행과 자원을 공유하므로 동시 부하 검증 전 GPU 기본값으로 바꾸지 않는다.

## 비교 도구와 검증 범위

`tools/signal_analysis/benchmark_neural_signal.py`는 같은 NPZ 입력을 각 backend의
별도 프로세스에서 처리한다. 세 번의 워밍업을 분리하고, 입력 전달과 출력 읽기를
포함한 반복 시간 및 전체 출력 NPZ를 저장한다. 카메라/전처리 시간은 별도이며
추론 속도만으로 전체 live 처리 주기를 설명하면 안 된다.

최근1098--6 영상의600/700/820/835/871/980 프레임에서 같은 검출 입력과 동일한
신호 영역 분류 입력을 준비했다. CPU/GPU 출력 수치 비교뿐 아니라 검출 상자,
색상 전환 결과 및 기존 modelV2의 지연/누락을 확인해야 GPU 적용을 판단할 수 있다.

관련 데스크톱 시험은 **130 passed, 1 skipped**다. 신규 시험은 명시적 선택,
원래 경로 유지, 색상 표시 변환, 실제 worker 루프의 VisionIPC 처리,
버퍼 재사용 결과 폐기 및 assist가 켜져 있어도 제어 미발행을 검증한다.
Windows Params만 대체했고 실제 QCOM 실행/물리 화면/차량 반응 검증은 아니다.

## 차량 실측 상태

차량192.168.0.178에서 offroad, Panda noOutput/controlsAllowed=false,
ORT1.23.2(CPU/Azure provider), OpenCV4.13, tinygrad OnnxRunner를 확인했다.
ONNX Runtime에 QCOM provider가 없으므로 위의 tinygrad 별도 경로를 사용한다.

이후 SSH가 반복적으로 연결 시간 초과를 보였고 최초 시험 자료 전송은
detector.onnx 약4.56MB까지만 진행된 뒤 끊겼다. 부분 모델은 해시 검사에서
거부되며 설치/실행 완료로 취급하지 않는다. 이어받기 전송과 측정 스크립트는
비공개 작업 폴더에 준비했다. **현재 GPU 속도 결과, 새 검출기의 차량 활성화,
강제 카메라 시험 성공은 아직 없다.** 마지막 확인 차량 HEAD는fc7588d67a이며
이는 이전 판독 표시 유지 기능의 설치다.

시동 없이 카메라·추론을 실행하는 시험은 가능하지만, 전원/네트워크 연결은
필요하다. 실제 시험에서는 offroad/noOutput을 확인한 뒤 카메라 및 관찰기와
필요한 모델 프로세스만 시간 제한으로 실행하고 복구한다. 화면 표시, 추론
속도, 기존 주행 모델 영향, 주행 인식 정확도는 각각 구분해 보고한다.

모델·입력·연결 실패 기록 등 비공개 재현 자료는
`.analysis/scratch/2026-10-11-signal-live/`에 있다. 연결 복구 후 같은 자료로
검증을 이어가며, 미완료 차량 시험을 완료로 기록하지 않는다.
