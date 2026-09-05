# Xiaoge Vision: V-ASM and Lane Detection

## 中文

### 概述

Xiaoge Vision 是 CarrotPilot 的本地视觉扩展。它使用 comma3 的 VisionIPC 相机帧运行两个 ONNX
模型，并通过一个版本化的 `xiaogeVision` JSON 消息写入 `customReservedRawData0`。`card.py` 接收该
消息后，将车道线类型和视觉盲区结果安全地合并到 `carState`，供现有 UI 与变道逻辑使用。

受管理入口是 `openpilot.selfdrive.carrot.xiaoge_data`。进程在 onroad 时自动运行；保留
`ShareData` 时也会继续提供原有 TCP 7711 数据服务。请不要同时手动启动 `v_asm_server.py`，否则会
与入口进程争用 8082 端口并产生重复的视觉推理进程。

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

### Web 调试页面

打开 `http://<comma-ip>:8082`：

- 默认适配手机浏览器；宽度达到 900px 时自动显示为左侧车道线、右侧 V-ASM 的电脑双栏布局。
- 支持中文/English 切换，显示模型状态、推理耗时、频率、检测置信度和原始状态 JSON。
- 相机图像仅在点击对应“刷新画面”按钮时请求一帧，不会自动刷新或重载网页。
- V-ASM 页面显示门控条件、目标侧和当前车道宽度；可在广角图像上修改左右盲区多边形。
- 未保存配置时使用内置的 1928x1208 广角默认多边形；“恢复默认标注”会删除本地覆盖配置。

网页中的阈值、平滑时间和推理间隔用于调试和验证。车道线识别在服务运行时固定启用，避免因网页操作中断
系统车道线结果。

### 安装和手动启动

在 comma3 的 openpilot 根目录运行：

```bash
./tools/op.sh venv
python -m pip install --no-cache-dir -r openpilot/selfdrive/carrot/xiaoge/requirements.txt
python -m openpilot.selfdrive.carrot.xiaoge_data
```

`camerad` 必须运行。V-ASM 多边形配置保存在本目录的 `v_asm_config.json`，该文件已被 Git 忽略。

---

## English

### Overview

Xiaoge Vision is a local CarrotPilot vision extension. It runs two ONNX models on comma3 VisionIPC
frames and publishes one versioned `xiaogeVision` JSON payload through `customReservedRawData0`.
`card.py` safely merges the resulting lane-marking types and visual blindspot state into `carState`,
where the existing UI and lane-change logic consume them.

The managed entry point is `openpilot.selfdrive.carrot.xiaoge_data`. It runs onroad and preserves
the existing TCP 7711 data service when `ShareData` is enabled. Do not also launch
`v_asm_server.py` manually: it would conflict on port 8082 and create a second vision process.

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

### Web diagnostics

Open `http://<comma-ip>:8082`.

- The interface is mobile-first. At 900px or wider it automatically becomes a desktop two-column
  view with lanes on the left and V-ASM on the right.
- Chinese and English are available. The page shows model state, inference latency/rate,
  confidence, and raw status JSON.
- Camera frames are fetched only when the matching refresh button is pressed; the page does not
  reload and camera images do not auto-refresh.
- The V-ASM panel shows its activation gate, selected side, and lane width. Its wide-camera view
  supports editing the left and right blindspot polygons.
- Built-in 1928x1208 wide-camera polygons are used until a local override is saved. Resetting the
  annotations removes that override.

The web threshold, smoothing, and interval controls are intended for validation. Lane detection
remains enabled while the service runs so web actions cannot interrupt system lane results.

### Installation and manual launch

From the comma3 openpilot root:

```bash
./tools/op.sh venv
python -m pip install --no-cache-dir -r openpilot/selfdrive/carrot/xiaoge/requirements.txt
python -m openpilot.selfdrive.carrot.xiaoge_data
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

관리형 시작점은 `openpilot.selfdrive.carrot.xiaoge_data`입니다. onroad에서 실행되며,
`ShareData`가 활성화된 경우 기존 TCP 7711 데이터 서비스도 유지합니다. `v_asm_server.py`를 별도로
실행하지 마십시오. 8082 포트가 충돌하고 두 번째 비전 프로세스가 실행됩니다.

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

### 웹 진단 페이지

`http://<comma-ip>:8082`를 여십시오.

- 모바일 우선 UI이며, 너비가 900px 이상이면 왼쪽 차선/오른쪽 V-ASM의 데스크톱 2열 보기로 자동 전환됩니다.
- 중국어와 영어 전환을 지원하며 모델 상태, 추론 시간/주기, 신뢰도, 원시 상태 JSON을 표시합니다.
- 각 새로고침 버튼을 눌렀을 때만 해당 카메라 프레임을 가져옵니다. 페이지를 다시 로드하거나 카메라 영상을 자동 갱신하지 않습니다.
- V-ASM 패널은 실행 조건, 대상 측, 차선 폭을 표시하며 광각 영상에서 좌우 사각지대 다각형을 편집할 수 있습니다.
- 로컬 설정을 저장하기 전에는 내장 1928x1208 광각 기본 다각형을 사용합니다. 주석을 초기화하면 로컬 설정이 제거됩니다.

웹의 임계값, 평활 시간, 추론 간격은 검증용입니다. 서비스가 실행되는 동안 차선 인식은 항상 활성화되어 웹
동작 때문에 시스템 차선 결과가 중단되지 않습니다.

### 설치 및 수동 시작

comma3 openpilot 루트에서 실행합니다.

```bash
./tools/op.sh venv
python -m pip install --no-cache-dir -r openpilot/selfdrive/carrot/xiaoge/requirements.txt
python -m openpilot.selfdrive.carrot.xiaoge_data
```

`camerad`가 실행 중이어야 합니다. 로컬 V-ASM 다각형 설정은 이 디렉터리의 `v_asm_config.json`에
저장되며 Git에서 무시됩니다.
