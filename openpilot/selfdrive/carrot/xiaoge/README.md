# Xiaoge Vision: V-ASM and Lane Detection

## 中文

### 概述

Xiaoge Vision 是 CarrotPilot 的本地视觉扩展。它使用 comma3 的 VisionIPC 相机帧运行两个 ONNX
模型，并通过一个版本化的 `xiaogeVision` JSON 消息写入 `customReservedRawData0`。`card.py` 接收该
消息后，将车道线类型和视觉盲区结果安全地合并到 `carState`，供现有 UI 与变道逻辑使用。

受管理入口是 `openpilot.selfdrive.carrot.xiaoge_data`。只有启用现有的 `ShareData` 参数时，管理器才会
启动此进程，同时提供本地视觉识别和原有 TCP 7711 数据服务。相机推理需要 `camerad` 运行。
请不要同时手动启动 `v_asm_server.py`，否则会
与入口进程争用 8082 端口并产生重复的视觉推理进程。

### 启用

在 Carrot Web（端口 7000）的 **设置 → 驾驶控制 → 转向 → 变道 (自动转向) → ONNX 车道与盲点识别** 中开关。它使用现有 `ShareData` 参数，默认关闭并保留已有值。关闭后停止视觉和原有 TCP 7711 服务；车辆 OEM BSD 保持有效。

### 功能和原理

| 功能 | 相机与模型 | 工作方式 |
|---|---|---|
| 车道线识别 | 前向道路相机 `VISION_STREAM_ROAD`、`assets/lane.onnx` | 始终运行。直接从 NV12 的 Y 平面进行居中正方形裁切，缩放至 416x416 灰度图，复制为三个归一化通道后输入 YOLOv8-Seg 模型。输出左/右车道线：`1` 实线、`0` 虚线、`-1` 未知。 |
| V-ASM 盲区识别 | 广角道路相机 `VISION_STREAM_WIDE_ROAD`、`assets/v_asm_model.onnx` | 仅在必要时运行。根据当前变道方向只检测目标侧的用户标注 ROI，减少 CPU 使用并提高目标侧响应速度。 |
| 车载合并 | `customReservedRawData0` -> `card.py` | 车道类型只覆盖模型已识别的一侧，同时保留车辆原有颜色编码。视觉盲区与 OEM `leftBlindspot/rightBlindspot` 做 OR 合并，视觉结果永远不会清除 OEM 盲区。 |

V-ASM 仅在以下条件全部满足时推理：

1. 车速为 30-120 km/h。
2. `modelV2.meta.laneChangeDirection` 明确为左或右。
3. 目标侧的 `laneWidthLeft` 或 `laneWidthRight` 不低于 3.0 m。

车道结果超过 4 秒、盲区结果超过 1.5 秒未更新时，`card.py` 自动忽略它们。这样相机、模型或进程异常
时不会将过期视觉信息继续用于 UI 或变道判断。

### mici 行车显示

启用 `ShareData` 后，mici 右下角显示 VISION 状态、最近一次车道推理耗时和左右实线/虚线图标。
`?` 表示无法识别；等待或过期结果不会保留已识别图标。BSD 区分待机、指定侧未检测到和检测到；
只有当前已评估的侧才会显示未检测到，另一侧不会被当作已检查。
相机画面隐藏时仍保留状态卡和黄色侧边 BSD 提示。显示道路模型时，车道使用与 c3 相同的虚线分段规则，
BSD 使用黄色侧边路障。车道位置来自驾驶模型，`lane.onnx` 只提供实线/虚线分类。
OEM BSD 提示不依赖 `ShareData`；驾驶告警仍在最上层。V-ASM 的速度、方向和车道宽度条件保持不变。

### Web 调试页面

在 Carrot Web 的工具菜单中打开“ONNX 车道与盲点诊断”，或访问 `http://<comma-ip>:7000/xiaoge/`。
7000 端口提供页面并转发到本机 8082 诊断接口，推理仍由独立服务运行。功能关闭时页面会提示开启 ONNX 设置并自动重连。
直接访问 `http://<comma-ip>:8082` 仍然可用。

- 默认适配手机浏览器；宽度达到 900px 时自动显示为左侧车道线、右侧 V-ASM 的电脑双栏布局。
- 支持中文/English/한국어 切换，并沿用工具菜单的语言；显示模型状态、推理耗时、频率、检测置信度和原始状态 JSON。
- 相机就绪后自动加载第一帧；首次加载失败会重试，并显示加载状态。之后点击对应“刷新画面”按钮获取新快照，不会连续播放视频。
- V-ASM 页面显示门控条件、目标侧和当前车道宽度；可在广角图像上修改左右盲区多边形。
- 未保存配置时使用内置的 1928x1208 广角默认多边形；“恢复默认标注”会删除本地覆盖配置。

网页中的阈值、平滑时间和推理间隔用于调试和验证。车道线识别在服务运行时固定启用，避免因网页操作中断
系统车道线结果。

`Latency` 是一次图像预处理、模型推理和后处理的实际耗时，也包含线程等待；不是网页网络延迟。
`Interval` 只限制推理启动间隔，缩短它不会让模型本身计算更快。车载相机和推理线程使用普通工作核心 CPU 0–3，
OpenCV 最多使用两个 CPU 计算线程，避免与实时驾驶核心争用。
相机以非阻塞方式接收，空闲时在 Python 中短暂休眠，避免 VisionIPC 等待持有 GIL 并阻塞其他线程。
原始状态 JSON 的 `lane.inference.threadCpuMs` 和 `inference.threadCpuMs` 分别记录车道和 V-ASM
推理线程的 CPU 时间。它们不包含等待或其他工作线程的 CPU 时间，因此与 `latencyMs` 的差值不能直接视为 GIL 等待时间。

### 安装和手动启动

正常启动会从 `third_party/wheels` 将已固定版本的 OpenCV 安装到可写的 `pydeps`，无需联网，也不会修改只读系统虚拟环境或替换 NumPy。下面的命令仅用于手动验证。

在 comma3 的 openpilot 根目录运行：

```bash
python -m pip install --no-index --no-deps --find-links third_party/wheels \
  --target pydeps opencv-python-headless==4.13.0.92
PYTHONPATH="$PWD/pydeps:$PWD${PYTHONPATH:+:$PYTHONPATH}" python -m openpilot.selfdrive.carrot.xiaoge_data
```

`camerad` 必须运行。V-ASM 多边形配置保存在本目录的 `v_asm_config.json`，该文件已被 Git 忽略。

---

## English

### Overview

Xiaoge Vision is a local CarrotPilot vision extension. It runs two ONNX models on comma3 VisionIPC
frames and publishes one versioned `xiaogeVision` JSON payload through `customReservedRawData0`.
`card.py` safely merges the resulting lane-marking types and visual blindspot state into `carState`,
where the existing UI and lane-change logic consume them.

The managed entry point is `openpilot.selfdrive.carrot.xiaoge_data`. The manager starts it only when
the existing `ShareData` parameter is enabled, providing both local vision and the existing TCP 7711
data service. Camera inference requires `camerad` to be running. Do not also launch
`v_asm_server.py` manually: it would conflict on port 8082 and create a second vision process.

### Enabling the feature

Use **Carrot Web (port 7000) → Settings → Driving → Steering → Lane Change (Auto Turn) → ONNX Lane and BSD Detection**. The toggle uses the existing `ShareData` parameter, defaults to off, and preserves saved values. Turning it off stops vision and the existing TCP 7711 service; OEM BSD remains available.

### Features and operation

| Feature | Camera and model | Operation |
|---|---|---|
| Lane detection | `VISION_STREAM_ROAD`, `assets/lane.onnx` | Always enabled. The NV12 Y plane is center-square cropped, resized to a 416x416 grayscale image, copied into three normalized channels, and passed to the YOLOv8-Seg model. Lane types are `1` solid, `0` dashed, and `-1` unknown. |
| V-ASM blindspot detection | `VISION_STREAM_WIDE_ROAD`, `assets/v_asm_model.onnx` | Runs only when needed. It evaluates only the user-annotated ROI for the target lane-change side, reducing CPU use and improving target-side responsiveness. |
| Vehicle-state merge | `customReservedRawData0` -> `card.py` | A recognized lane type replaces only its corresponding type digit while preserving the vehicle color code. Visual blindspot state is OR-merged with OEM `leftBlindspot/rightBlindspot`; vision can never clear an OEM blindspot. |

V-ASM runs only when all conditions hold:

1. Speed is between 30 and 120 km/h.
2. `modelV2.meta.laneChangeDirection` is explicitly left or right.
3. The target-side `laneWidthLeft` or `laneWidthRight` is at least 3.0 m.

`card.py` discards lane results older than four seconds and blindspot results older than 1.5 seconds.
This prevents stale visual state from affecting the UI or lane-change decision after a camera, model,
or process failure.

### mici on-road display

With `ShareData` enabled, a card at the lower right shows VISION status, the latest lane inference
time, and left/right solid or dashed icons. `?` means unknown; waiting or stale results do not retain
recognized icons. BSD distinguishes standby, no detection on the evaluated side, and detection.
Only a freshly evaluated side can show no detection; the other side is not assumed to have been checked.
The card and amber side warnings remain visible with the camera hidden. With the road model visible,
lanes use the same dash geometry as c3 and BSD adds amber roadside barriers. Lane positions still come
from the driving model; `lane.onnx` supplies only solid/dashed classification.
OEM BSD warnings also work with `ShareData` off. Driving alerts remain on top. The existing V-ASM
speed, direction, and lane-width conditions are unchanged.

### Web diagnostics

In Carrot Web, open Tools → ONNX Lane / BSD, or visit `http://<comma-ip>:7000/xiaoge/`.
Port 7000 serves the page and forwards its diagnostic requests to local port 8082; inference stays in its separate service.
When detection is off, the page explains how to enable the ONNX setting and reconnects automatically.
Direct access at `http://<comma-ip>:8082` still works.

- The interface is mobile-first. At 900px or wider it automatically becomes a desktop two-column
  view with lanes on the left and V-ASM on the right.
- Korean, Chinese, and English are available, following the Tools menu language. The page shows model state, inference latency/rate,
  confidence, and raw status JSON.
- The first snapshot loads automatically when its camera is ready, with retries and visible loading
  status. Use the matching refresh button for later snapshots; the images are not a continuous video feed.
- The V-ASM panel shows its activation gate, selected side, and lane width. Its wide-camera view
  supports editing the left and right blindspot polygons.
- Built-in 1928x1208 wide-camera polygons are used until a local override is saved. Resetting the
  annotations removes that override.

The web threshold, smoothing, and interval controls are intended for validation. Lane detection
remains enabled while the service runs so web actions cannot interrupt system lane results.

`Latency` measures one preprocessing, model inference, and postprocessing pass, including thread
waits; it is not web network latency. `Interval` only limits how often inference starts, so reducing
it does not speed up the model. On the device, camera and inference threads use background CPU cores
0–3, with OpenCV limited to two compute threads, avoiding the realtime driving cores. Camera receivers poll
without blocking and briefly sleep in Python when idle, so VisionIPC waits cannot hold the GIL and
stall other threads. Raw status JSON exposes `lane.inference.threadCpuMs` and `inference.threadCpuMs`
for the lane and V-ASM inference threads. These exclude waiting and CPU time on other worker threads;
their difference from `latencyMs` is not a direct measurement of GIL waiting.

### Installation and manual launch

Normal startup installs pinned OpenCV from `third_party/wheels` into writable `pydeps` without internet access. It leaves the read-only system venv and its NumPy intact. The commands below are for manual validation only.

From the comma3 openpilot root:

```bash
python -m pip install --no-index --no-deps --find-links third_party/wheels \
  --target pydeps opencv-python-headless==4.13.0.92
PYTHONPATH="$PWD/pydeps:$PWD${PYTHONPATH:+:$PYTHONPATH}" python -m openpilot.selfdrive.carrot.xiaoge_data
```

`camerad` must be running. Local V-ASM polygon overrides are saved as `v_asm_config.json` in this
directory and are ignored by Git.

---

## 한국어

### 개요

Xiaoge Vision은 CarrotPilot용 로컬 비전 확장 기능입니다. comma3 VisionIPC 카메라 프레임에서 두 개의
ONNX 모델을 실행하고, 하나의 버전 관리된 `xiaogeVision` JSON 메시지를
`customReservedRawData0`으로 발행합니다. `card.py`는 차선 종류와 비전 사각지대 상태를 `carState`에
안전하게 병합하며, 기존 UI와 차선 변경 로직이 이를 사용합니다.

관리형 시작점은 `openpilot.selfdrive.carrot.xiaoge_data`입니다. 기존 `ShareData` 파라미터가 켜져 있을
때만 관리자가 실행하며, 로컬 비전 인식과 기존 TCP 7711 데이터 서비스를 함께 제공합니다.
카메라 추론에는 `camerad` 실행이 필요합니다. `v_asm_server.py`를 별도로
실행하지 마십시오. 8082 포트가 충돌하고 두 번째 비전 프로세스가 실행됩니다.

### 기능 켜기

**Carrot Web(7000번 포트) → 설정 → 주행 제어 → 차량 조향 → 차로 변경 (자동 턴) → ONNX 차선·BSD 인식**에서 켜고 끕니다. 기존 `ShareData` 파라미터를 사용하며 기본값은 꺼짐이고 저장값은 유지합니다. 끄면 비전과 기존 TCP 7711 서비스가 중지되고 차량 자체 BSD는 유지됩니다.

### 기능 및 동작 원리

| 기능 | 카메라 및 모델 | 동작 |
|---|---|---|
| 차선 인식 | `VISION_STREAM_ROAD`, `assets/lane.onnx` | 항상 활성화됩니다. NV12 Y 평면을 중앙 정사각형으로 자르고 416x416 그레이스케일로 조정한 뒤, 정규화된 동일한 세 채널로 복사하여 YOLOv8-Seg 모델에 입력합니다. `1`은 실선, `0`은 점선, `-1`은 알 수 없음입니다. |
| V-ASM 사각지대 인식 | `VISION_STREAM_WIDE_ROAD`, `assets/v_asm_model.onnx` | 필요한 경우에만 실행됩니다. 차선 변경 대상 측의 사용자가 지정한 ROI만 평가하므로 CPU 사용량을 줄이고 대상 측 응답을 높입니다. |
| 차량 상태 병합 | `customReservedRawData0` -> `card.py` | 인식된 차선 종류는 차량 색상 코드를 보존한 채 해당 종류 자리만 갱신합니다. 비전 사각지대 상태는 OEM `leftBlindspot/rightBlindspot`과 OR 병합되므로 비전 결과가 OEM 사각지대 상태를 해제할 수 없습니다. |

V-ASM은 다음 조건을 모두 만족할 때만 실행됩니다.

1. 속도가 30-120 km/h입니다.
2. `modelV2.meta.laneChangeDirection`이 명시적으로 left 또는 right입니다.
3. 대상 측 `laneWidthLeft` 또는 `laneWidthRight`가 3.0m 이상입니다.

`card.py`는 4초가 지난 차선 결과와 1.5초가 지난 사각지대 결과를 무시합니다. 따라서 카메라, 모델 또는
프로세스 장애 후 오래된 비전 상태가 UI나 차선 변경 판단에 사용되지 않습니다.

### mici 주행 화면

`ShareData`가 켜져 있으면 오른쪽 아래에 VISION 상태, 최근 차선 추론 시간, 좌우 실선·점선 아이콘이 나옵니다.
`?`는 차선 종류를 알 수 없다는 뜻이며, 결과를 기다리거나 오래된 결과만 남으면 인식 아이콘을 유지하지 않습니다.
BSD는 ‘대기’, 검사한 쪽의 ‘미감지’, ‘감지’를 구분합니다. 방금 검사한 쪽만 미감지로 표시하고,
검사하지 않은 반대쪽을 검사한 것으로 간주하지 않습니다.
카메라 화면을 숨겨도 상태 표시와 노란색 좌우 BSD 경고는 남습니다. 도로 모델이 보일 때는 c3와 같은
점선 구간을 사용하고 BSD를 노란색 도로 옆 표시로 그립니다. 차선 위치는 주행 모델에서 가져오며,
`lane.onnx`는 실선·점선 종류만 제공합니다.
차량 자체 BSD 경고는 `ShareData`가 꺼져 있어도 표시하며, 주행 경고는 가장 위에 나옵니다.
V-ASM의 기존 속도·방향·차선 폭 조건은 유지합니다.

### 웹 진단 페이지

Carrot Web의 도구 → ONNX 차선·BSD 진단을 누르거나 `http://<comma-ip>:7000/xiaoge/`를 여십시오.
7000번 서버가 페이지를 제공하고 진단 요청을 장치 내부 8082번으로 전달하며, 추론은 기존 별도 서비스에서 실행합니다.
기능이 꺼져 있어도 페이지가 열리고 ONNX 설정을 켜는 방법을 안내하며 자동으로 다시 연결합니다.
`http://<comma-ip>:8082`로 직접 접속하는 방법도 유지됩니다.

- 모바일 우선 UI이며, 너비가 900px 이상이면 왼쪽 차선/오른쪽 V-ASM의 데스크톱 2열 보기로 자동 전환됩니다.
- 도구 메뉴의 언어를 이어받으며 한국어·영어·중국어 전환을 지원합니다. 모델 상태, 추론 시간/주기, 신뢰도, 원시 상태 JSON을 표시합니다.
- 카메라가 준비되면 첫 스냅샷을 자동으로 가져오며, 처음 실패하면 재시도하고 로딩 상태를 표시합니다. 이후에는 각 새로고침 버튼으로 새 사진을 가져옵니다. 연속 재생 영상은 아닙니다.
- V-ASM 패널은 실행 조건, 대상 측, 차선 폭을 표시하며 광각 영상에서 좌우 사각지대 다각형을 편집할 수 있습니다.
- 로컬 설정을 저장하기 전에는 내장 1928x1208 광각 기본 다각형을 사용합니다. 주석을 초기화하면 로컬 설정이 제거됩니다.

웹의 임계값, 평활 시간, 추론 간격은 검증용입니다. 서비스가 실행되는 동안 차선 인식은 항상 활성화되어 웹
동작 때문에 시스템 차선 결과가 중단되지 않습니다.

`Latency`는 영상 전처리, 모델 계산, 후처리 한 번에 걸린 실제 시간이며 스레드 대기도 포함합니다. 웹 통신
지연이 아닙니다. `Interval`은 추론 시작 간격만 제한하므로 줄여도 모델 계산 자체가 빨라지지는 않습니다.
장치의 카메라·추론 스레드는 일반 작업용 CPU 0~3번을 사용하고, OpenCV 계산 스레드는 최대 두 개로 제한합니다.
실시간 주행 작업이 사용하는 코어와의 경쟁을 피하기 위한 배치입니다. 카메라는 대기 없이 수신을 확인하고, 프레임이 없으면
Python에서 잠시 쉬어 VisionIPC 대기가 GIL을 잡고 다른 스레드를 막지 않도록 합니다.
원시 상태 JSON의 `lane.inference.threadCpuMs`와 `inference.threadCpuMs`는 각각 차선 및 V-ASM 추론
스레드가 사용한 CPU 시간입니다. 대기와 다른 작업 스레드의 CPU 시간은 제외하므로 `latencyMs`와의 차이가
그대로 GIL 대기 시간은 아닙니다.

### 설치 및 수동 시작

정상 시작 시 `third_party/wheels`에 포함된 고정 버전 OpenCV를 쓰기 가능한 `pydeps`에 자동 설치합니다. 인터넷 없이 준비하며 읽기 전용 시스템 가상환경과 기존 NumPy는 변경하지 않습니다. 아래 명령은 수동 검증용입니다.

comma3 openpilot 루트에서 실행합니다.

```bash
python -m pip install --no-index --no-deps --find-links third_party/wheels \
  --target pydeps opencv-python-headless==4.13.0.92
PYTHONPATH="$PWD/pydeps:$PWD${PYTHONPATH:+:$PYTHONPATH}" python -m openpilot.selfdrive.carrot.xiaoge_data
```

`camerad`가 실행 중이어야 합니다. 로컬 V-ASM 다각형 설정은 이 디렉터리의 `v_asm_config.json`에
저장되며 Git에서 무시됩니다.
