# 설정 이해하기

[English](../en/settings.md)

> [!NOTE]
> 이 문서는 `carrot-wip` 코드와 함께 관리하는 사용자 설명서 원본입니다. 사용자 동작이 바뀌면 관련 코드·테스트와 같은 변경에서 이 문서도 갱신합니다.

carrotpilot의 세부 설정은 **Carrot Web에서 모두 확인하고 변경하는 것**을 기준으로 합니다. 장치 자체의 기본 설정 화면은 Wi-Fi, 장치 정보, openpilot 기본 토글과 소프트웨어 업데이트에 사용하고, 아래에서 설명하는 `carrot_settings.json`의 파라미터는 Carrot Web의 **설정** 화면에서 관리합니다.

> [!IMPORTANT]
> **현재 지원 상태**
>
> - 설정 변경에 사용하는 인터페이스: **Carrot Web**
> - 당근맨(CarrotMan): 현재 사용자용 앱·외부 연동으로 지원하지 않음
> - CarrotLink: 현재 지원하지 않음
> - 예전 문서의 CarrotMan·CarrotLink 접속·설정 안내는 현재 사용 방법이 아님
>
> 코드나 메시지 내부에 `carrotMan`이라는 이름이 남아 있을 수 있지만, 사용자용 CarrotMan 앱을 지원한다는 뜻은 아닙니다.

## Carrot Web 접속

1. comma 장치와 휴대폰 또는 PC를 같은 네트워크에 연결합니다.
2. 브라우저에서 `http://장치-IP:7000`을 엽니다.
3. 상단 메뉴에서 **설정**을 선택합니다.

예를 들어 장치 IP가 `192.168.0.25`라면 다음 주소를 사용합니다.

    http://192.168.0.25:7000

접속 문제와 다른 화면의 설명은 [Carrot Web](https://github.com/ajouatom/openpilot/wiki/Guide-Carrot-Web)을 참고하세요.

## 설정 화면 사용법

Carrot Web 설정 화면에서는 다음 기능을 사용할 수 있습니다.

- **분류 탐색**: 대분류, 중분류, 세부 구역 순서로 설정을 찾습니다.
- **검색**: 화면 제목이나 파라미터 이름으로 원하는 항목을 찾습니다.
- **즐겨찾기**: 설정 항목을 길게 눌러 추가하거나 제거합니다.
- **프로필**: 현재 설정 묶음을 이름을 붙여 저장하고 다시 적용합니다.
- **설정 비교**: 프로필이나 백업의 값과 현재 값을 비교한 뒤 적용합니다.
- **초기화**: 전체 설정을 카탈로그 기본값으로 되돌립니다.
- **파일·QR 백업**: 도구 화면에서 설정을 보관하거나 복원합니다.

전체 초기화나 다른 차량의 백업 적용은 많은 값을 한꺼번에 바꿉니다. 먼저 현재 설정을 별도 프로필과 파일로 보관하세요.

### 항목별 상세 설명

설정 항목의 값 조절 부분을 제외한 본문을 누르면 해당 항목의 상세 화면으로 이동합니다. 기존 설정 박스 아래의 정보 박스에는 **설명**, **인기값**, **이력** 탭이 항상 표시되며 **설명**이 기본으로 열립니다. 데이터가 없는 탭도 사라지지 않고 빈 상태를 표시합니다.

- **설명**은 GitHub에서 검증·게시된 현재 언어의 설정 설명을 우선 표시합니다.
- 게시 인덱스와 설명 페이지의 해시가 모두 맞는 내용만 표시합니다.
- 요청이 실패하거나 내용이 올바르지 않으면 마지막으로 검증된 브라우저 저장본을 사용합니다.
- 검증된 원격 정상본이나 저장본이 없으면 장치 내부 문서로 대체하지 않고 빈 상태를 표시합니다.
- **인기값**은 수집된 차량의 값 분포를 참고용으로 표시하며 권장값을 뜻하지 않습니다.
- **이력**은 최근 변경 3건을 먼저 표시하고, 기록이 더 있으면 전체 이력을 열 수 있습니다.
- 네트워크나 문서 처리에 문제가 생겨도 설정값 확인·변경 기능은 계속 사용할 수 있습니다.
- Carrot Web이 한국어이면 한국어 원문, 영어이면 영어 원문을 표시합니다.
- 중국어 상세 원문이 아직 없으면 중국어 제목·짧은 설명은 유지하고 상세 설명만 영어로 표시합니다.
- GitHub Wiki의 우측 `사용 설명서 > 설정 이해하기 > 전체 설정`에서는 Carrot Web과
  같은 한국어 메뉴 계층·순서로 개별 설정 페이지를 찾을 수 있습니다. 중앙 전체 설정
  문서에서는 한국어·영어·중국어 목록을 함께 제공합니다.

가로·세로 화면 모두 기존 설정 박스 다음에 정보 박스가 세로로 이어집니다. 배율과 기본값 버튼의 위치와 동작은 기존 설정 박스 안에서 유지됩니다.

검증된 게시 설명은 현재 설정 코드의 기본값·범위·선택지를 기준으로 작성됩니다. 화면의 짧은 설명과 상세 설명이 다르면 현재 브랜치의 설정 코드와 함께 문제를 확인하세요.

## 숫자와 기본값 읽는 법

`carrot_settings.json`의 각 항목에는 다음 정보가 들어 있습니다.

| 필드 | 의미 |
|---|---|
| `name` | 코드와 Params에서 사용하는 고유 이름 |
| `title` | Carrot Web에 표시되는 한국어 제목 |
| `descr` | 값의 방향, 모드 번호와 주의사항 |
| `min` / `max` | 입력 가능한 범위 |
| `unit` | `+`와 `-` 버튼으로 한 번에 움직이는 단위 |
| `default` | 설정 카탈로그가 사용하는 초기 기준값 |

### 현재 값이 가장 우선입니다

업데이트 후에도 장치에 저장된 영구 설정은 유지될 수 있습니다. JSON의 `default`, 제목 괄호 안 숫자, 다른 사용자의 설정값보다 **내 장치의 Carrot Web에 표시된 현재 값**을 먼저 기록하세요.

`default`는 모든 차량에 권장되는 튜닝값이라는 뜻이 아닙니다. 차종 기본 튜닝, 기존 Params와 브랜치에 따라 실제 시작값이 다를 수 있습니다.

### 배율과 단위를 확인합니다

| 표시 예 | 저장값 | 실제 의미 |
|---|---:|---:|
| `TFollowGap1 x0.01s` | `110` | 1.10초 |
| `SteerActuatorDelay` | `30` | 0.30초 |
| `LateralTorqueAccelFactor x0.001` | `2500` | 2.500 |
| `StopDistanceCarrot` | `600` | 6.00m(600cm) 기준 값 |
| 비율 `%` | `105` | 105% |

항목 이름과 설명에 적힌 `x0.01`, `x0.001`, `cm`, `km/h`, `%`를 생략하면 값의 크기를 잘못 이해할 수 있습니다.

## 전체 설정 지도

현재 `carrot-wip`의 `carrot_settings.json`에는 **171개 파라미터**가 있으며, 모든 항목이 아래 메뉴에 연결되어 있습니다.

| 대분류 | 항목 수 | 중분류 |
|---|---:|---|
| 주행 제어 | 107 | 시작·오토, 버튼·프리셋, 차량 조향, 속도·감속, 크루즈·차간 |
| 차량·하드웨어 | 16 | 현대·기아, CANFD·HDA, 레이더, 운전자 모니터링, 차량 보조, 기기 하드웨어 |
| 화면 표시 | 37 | 정보 표시, 경로 표시, 밝기·주행화면, 외부 HUD |
| 시스템 | 11 | 녹화·전원, 네트워크·지도, 사운드, 소프트웨어 |

## 주행 제어

주행 제어는 차량 움직임에 영향을 줄 수 있는 107개 항목입니다. 한 번에 여러 값을 변경하지 마세요.

<a id="start-auto"></a>
### 시작·오토 — 8개

| 세부 구역 | 파라미터 | 용도 |
|---|---|---|
| 시작 동작 | `AlwaysLateral`, `AutoEngage`, `DisableMinSteerSpeed` | 상시 조향, 주행 시작 시 자동 활성화, 저속 조향 제한 |
| 오토크루즈 | `AutoCruiseControl`, `AutoGasTokSpeed`, `AutoGasCancelSpeed`, `AutoGasSyncSpeed`, `CruiseOnDist` | 크루즈 자동 활성화와 가속 페달 입력 시 동작 |

- `AlwaysLateral`: 크루즈가 켜져 있지 않아도 조향 제어를 허용합니다.
- `AutoEngage`: `0` 끄기, `1` 조향 ON, `2` 조향 ON과 크루즈 대기입니다.
- `AutoCruiseControl`: 현대·기아 차량용 오토크루즈와 소프트홀드 관련 설정입니다.
- `DisableMinSteerSpeed`: SMDPS 장착 차량의 저속 조향 제한과 관련된 차량별 설정입니다.

### 버튼·프리셋 — 15개

아래 표의 **세부 구역 제목을 누르면** 실제 코드 기준의 버튼 상태기계, 속도 단위와 적용 조건을 설명한 페이지로 이동합니다.

| 세부 구역 | 파라미터 | 용도 |
|---|---|---|
| [버튼 모드](buttons-presets.md#button-modes) | `CruiseButtonMode`, `CancelButtonMode`, `LfaButtonMode`, `PaddleMode` | 크루즈, 캔슬, LFA와 패들 버튼의 동작 지정 |
| [속도 단위](buttons-presets.md#speed-units) | `CruiseSpeedUnit`, `CruiseSpeedUnitBasic`, `CruiseButtonLongDelay` | 짧게·길게 누를 때의 속도 변경 단위와 판정 시간 |
| [버튼 테스트](buttons-presets.md#button-spam) | `CruiseButtonTest1`, `CruiseButtonTest2`, `CruiseButtonTest3` | 순정 SCC 목표속도 동기화용 버튼 메시지 값 |
| [속도 프리셋](buttons-presets.md#speed-presets) | `CruiseSpeed1`, `CruiseSpeed2`, `CruiseSpeed3`, `CruiseSpeed4`, `CruiseSpeed5` | 사용자3 모드에서 사용하는 속도 테이블 |

버튼 설정은 순정 SCC 사용 여부와 차량 버튼 메시지에 따라 체감이 크게 다릅니다. 버튼이 예상과 다르게 작동하면 사용자 모드보다 `CruiseButtonMode=0`의 일반 동작에서 먼저 확인하세요.

<a id="vehicle-steering"></a>
### 차량 조향 — 36개

| 세부 구역 | 파라미터 | 용도 |
|---|---|---|
| 중앙 보정 | `PathOffset`, `CameraYawTrimDeg` | 레인모드 경로의 좌우 위치와 카메라 YAW 미세 보정 |
| 조향감 | `SteerActuatorDelay`, `LatSmoothSec`, `LatSuspendAngleDeg`, `CustomSR`, `SteerRatioRate` | 조향 시점, 평활화, 일시중지 각도와 조향비 |
| [차로 변경](lane-change.md)·자동 턴 | `LaneChangeNeedTorque`, `LaneChangeDelay`, `LaneChangeBsd`, `LaneLineCheck`, `AutoTurnControl`, `AutoTurnControlSpeedTurn`, `AutoTurnControlTurnEnd`, `AutoTurnMapChange` | 차로 변경 진입 조건과 ATC 동작 |
| 레인모드 | `LatMpcPathCost`, `LatMpcMotionCost`, `LatMpcAccelCost`, `LatMpcJerkCost`, `LatMpcSteeringRateCost`, `LatMpcInputOffset`, `UseLaneLineSpeed`, `UseLaneLineCurveSpeed`, `AdjustLaneOffset` | 레인모드 MPC 가중치와 차선 사용 조건 |
| 고급 토크·토크 계수 | `LateralTorqueCustom`, `LateralTorqueAccelFactor`, `LateralTorqueFriction`, `LateralTorqueKpV`, `LateralTorqueKiV`, `LateralTorqueKf`, `LateralTorqueKd` | 커스텀 토크 제어 계수 |
| 고급 토크·조향 제한 | `CustomSteerMax`, `CustomSteerDeltaUp`, `CustomSteerDeltaDown`, `CustomSteerDeltaUpLC`, `CustomSteerDeltaDownLC` | 최대 조향 토크와 토크 변화율 제한 |

`SteerActuatorDelay`는 높을수록 더 일찍 조향하도록 보상하고, `LatSmoothSec`는 높을수록 부드러워지는 대신 반응이 늦어질 수 있습니다. 두 값을 동시에 바꾸면 원인을 구분하기 어렵습니다.

`SteerRatioRate`의 기본값 `100%`는 학습된 조향비를 그대로 적용합니다. `CustomSR=0`일 때 사용되며, 저장된 비율이 허용 범위(`30~200%`)를 벗어나면 안전하게 `100%`로 대체됩니다.

`LateralTorqueCustom`과 `CustomSteer*` 계열은 차량의 기본 조향 튜닝과 안전 제한에 영향을 줄 수 있는 고급 항목입니다. 차종별 검증값과 복구 방법이 없으면 변경하지 마세요.

### 속도·감속 — 22개

아래 표의 **세부 구역 제목을 누르면** 카메라 감속 계산, 제한속도 연동과 커브·신호 로직을 설명한 페이지로 이동합니다.

| 세부 구역 | 파라미터 | 용도 |
|---|---|---|
| [과속카메라](speed-deceleration.md#speed-camera) | `AutoNaviSpeedCtrlMode`, `AutoNaviSpeedCtrlEnd`, `AutoNaviSpeedDecelRate`, `AutoNaviSpeedSafetyFactor`, `AutoNaviCountDownMode`, `VehicleNaviCanControl`, `VehicleNaviSchoolZoneControl`, `VehicleSpeedCameraControlMode`, `VehicleSpeedCameraDistanceTime` | 안전운전 이벤트의 대상, 순정 내비 CAN, 감속 시점과 목표 속도 |
| [도로 제한속도](speed-deceleration.md#road-speed-limit) | `AutoRoadSpeedLimitOffset`, `AutoRoadSpeedAdjust`, `AutoSpeedUptoRoadSpeedLimit` | 도로 제한속도에 맞춘 목표 속도 조절 |
| [과속방지턱](speed-deceleration.md#speed-bump) | `AutoNaviSpeedBumpTime`, `AutoNaviSpeedBumpSpeed` | 방지턱 감속 완료 시점과 통과 속도 |
| [커브·턴](speed-deceleration.md#curve-turn) | `AutoCurveSpeedFactor`, `AutoCurveSpeedLowerLimit`, `TurnSpeedControlMode`, `MapTurnSpeedFactor`, `ModelTurnSpeedFactor`, `ApplyModelSpeed` | 모델 곡률과 경로를 이용한 커브·턴 속도 |
| [신호감지](speed-deceleration.md#traffic-light) | `TrafficLightDetectMode`, `TrafficStopDistanceAdjust` | 신호 정지·출발 감지, 정지 위치 및 정지차 기준 자동 보정 |

`AutoNaviSpeedCtrlMode`는 `0` 미사용, `1` 과속카메라, `2` 과속카메라+방지턱, `3` 과속카메라+방지턱+이동식카메라입니다.

`VehicleSpeedCameraControlMode=2`는 차량 수신 카메라의 실제 감속이 시작된 뒤 새로 가속페달을 밟으면 현재 이벤트를 무시하려는 의사로 판단합니다. 감속구간에서 가속으로 도달한 최고속도를 하한으로 유지하고 이벤트가 끝나면 초기화하며, 감속 전부터 계속 밟은 입력은 오버라이드를 시작하지 않습니다.

`AutoNaviSpeedDecelRate`는 값이 낮을수록 더 먼 거리에서 감속을 시작하며, `AutoNaviSpeedSafetyFactor`는 감속 목표에 적용하는 제한속도 비율입니다. 감속이 이상하면 값부터 바꾸지 말고 이벤트 종류, 제한속도와 남은 거리가 정상 수신되는지 먼저 확인하세요.

`TrafficLightDetectMode`는 `0` 미사용, `1` 정지만 감지, `2` 정지와 출발을 모두 감지합니다. 모델 판단에 의존하므로 운전자가 항상 직접 확인해야 합니다.

### 크루즈·차간 — 전체 31개, 현대·기아·제네시스 28개

아래 표의 **세부 구역 제목을 누르면** 실제 코드 기준의 계산 방식, 값의 방향과 주의사항을 설명한 페이지로 이동합니다.

| 세부 구역 | 파라미터 | 용도 |
|---|---|---|
| [가속 성향·드라이브 모드](cruise-gap.md#driving-mode) | `MyDrivingMode`, `MyDrivingModeAuto` | 연비, 안전, 일반, 고속 모드와 자동 전환 |
| [가속 성향·속도별 가속값](cruise-gap.md#acceleration-table) | `CruiseMaxVals0`, `CruiseMaxVals1`, `CruiseMaxVals2`, `CruiseMaxVals3`, `CruiseMaxVals4`, `CruiseMaxVals5`, `CruiseMaxVals6` | 속도 구간별 최대 가속 성향 |
| [정차·재출발](cruise-gap.md#stop-resume) | `StopDistanceCarrot`, `StoppingAccel`, `VEgoStopping`, `AChangeCostStarting` | 정지 위치, 정지 진입과 재출발 특성 |
| [가감속 튜닝](cruise-gap.md#longitudinal-tuning) | `LongTuningKpV`, `LongTuningKiV`, `LongTuningKf`, `LongActuatorDelay` | 현기차는 Kp/Ki/Kf `100/0/100` 고정·숨김, 다른 브랜드는 조정 가능 |
| [차간거리](cruise-gap.md#following-gap) | `TFollowGap1`, `TFollowGap2`, `TFollowGap3`, `TFollowGap4`, `DynamicTFollow`, `DynamicTFollowLC`, `EnableSpeedTF`, `TFollowDecelBoost` | 차간 단계별 시간, 동적 차간과 감속 여유 |
| [선행차 반응](cruise-gap.md#lead-response) | `LeadAccelResponse`, `JLeadFactor3`, `RadarReactionFactor` | TF1 앞차 가속과 선행차 변화에 대한 반응 특성 |
| [당근 크루즈](cruise-gap.md#carrot-cruise) | `CruiseEcoControl`, `CarrotCruiseDecel`, `CarrotCruiseAtcDecel` | 연비 제어와 당근 크루즈 감속 특성 |

`MyDrivingMode`는 `1` 연비, `2` 안전, `3` 일반, `4` 고속 모드입니다. 고속 모드는 신호 감지를 무시하고 가속 성향을 높이므로 모드 이름만 보고 선택하지 말고 설명을 확인하세요.

`TFollowGap1`~`TFollowGap4`는 저장값에 `0.01초`를 곱한 시간 간격입니다. 값을 줄이면 선행차와 가까워집니다. `DynamicTFollow` 관련 기능은 고정 차간에서 기준 동작을 확인한 다음 적용하세요.

`LeadAccelResponse`는 차간 1단계에서만 앞차의 출발·가속을 따라가는 MPC 민첩성을 0~5단계로 조절합니다. 1단계는 약한 반응, 2단계는 완만한 반응, 3단계는 일상적으로 조금 급한 경쾌함, 4단계는 급한 추종, 시험용 5단계는 최대 추종입니다. 단계가 높을수록 MPC의 활성 가속변화 비용과 jerk 비용을 낮춰 `vTargetNow`와 `aTarget`이 함께 더 빠르게 상승하며, `CruiseMaxVals`·곡선·끼어들기·위험거리 상한은 그대로 유지합니다. 설정 TF에 도달하거나 앞차 가속이 끝나면 즉시 기존 MPC 비용과 감속 제어로 돌아갑니다. 적용 조건과 단계별 비용은 [선행차 반응 설명](cruise-gap.md#lead-response)을 확인하세요.

`LongTuning*`, `LongActuatorDelay`, `StoppingAccel`은 openpilot이 가감속을 제어하는 차량에서 직접적인 영향을 줄 수 있는 고급 항목입니다. 현대·기아·제네시스에서는 `LongTuningKpV`, `LongTuningKiV`, `LongTuningKf`가 안전값 `100/0/100`으로 고정되어 설정 화면에 나오지 않으며, 순정 ACC 차량에서는 관련 없는 항목도 있습니다.

<a id="vehicle-hardware"></a>
## 차량·하드웨어

차량·하드웨어 16개 항목은 차종, 하네스와 기기 하드웨어 구성을 결정하는 설정입니다. 화면 표시 설정처럼 시험 삼아 켜면 안 됩니다.

| 중분류 | 파라미터 | 용도 |
|---|---|---|
| 현대·기아 | `HyundaiCameraSCC`, `IsLdwsCar`, `HapticFeedbackWhenSpeedCamera` | SCC 연결 방식, LDWS 차량과 카메라 구간 햅틱 |
| CANFD·HDA | `CanfdHDA2`, `CanfdDebug`, `HDPuse` | HDA2 차량과 CAN FD 디버그·HDP 기능 |
| 레이더 | `EnableRadarTracks`, `EnableCornerRadar`, `CarrotRadarMode`, `CarrotRadarCutInSensitivity` | SCC 레이더, 레이더 트랙, 코너 레이더와 당근레이더 처리·컷인 감도 |
| 운전자 모니터링 | `DisableDM`, `MuteDoor`, `MuteSeatbelt` | 운전자 모니터링과 일부 차량 경고음 처리 |
| 차량 보조 | `MaxAngleFrames`, `SpeedFromPCM` | 최대 조향각 관련 프레임과 순정 SCC 속도 제어 방식 |
| 기기 하드웨어 | `HardwareC3xLite` | 스피커가 없는 C3X Lite의 알림음과 프로세스 구성 |

> [!CAUTION]
> `HyundaiCameraSCC`, `CanfdHDA2`, `EnableRadarTracks`, `CarrotRadarMode`, `CarrotRadarCutInSensitivity`, `SpeedFromPCM`은 잘못 설정하면 차량 인식, SCC, 레이더와 가감속 동작이 달라질 수 있습니다. 차종, 연식, HDA 구성, 하네스 연결 위치와 순정 ACC 사용 여부를 확인한 뒤 변경하세요.

- `HyundaiCameraSCC`: 현대·기아 차량의 롱컨, 크루즈 동기화와 CAN FD 배선 구성에 따라 모드가 달라집니다.
- `CanfdHDA2`: HDA2 차량에서만 활성화합니다.
- `EnableRadarTracks`: `-2`는 비전 전용 시험, `-1`은 비전 매칭 없이 SCC를 항상 사용, `0`은 SCC-비전 매칭, `1`은 SCC 없이 프런트 레이더-비전 매칭, `2`는 프런트 레이더와 저속 SCC-비전 매칭, `3`은 프런트 레이더-비전 매칭 실패 시 SCC 강제 사용입니다. 매칭 모드는 실패 시 확률 `0.40` 이상인 중앙 비전을 사용하고, `-1`·`3`은 SCC가 없을 때만 비전으로 전환합니다. 강제 SCC는 횡좌표를 무시합니다. 레거시 Mando 레이더의 32·64슬롯 차이는 자동 처리합니다.
- `CarrotRadarMode`: 전방·코너 레이더로 차량의 움직임을 계속 추적해 끼어드는 차량을 감지하고, 카메라 영상과 레이더 정보를 새로운 방식으로 맞춰 앞차를 선택합니다. 코너 레이더와 레이더 트랙 기능이 모두 없는 차량에서는 기존 방식과 동일하게 동작합니다. 가감속 동작이 달라질 수 있으므로 검증을 마친 동일 차량에서만 켭니다. 변경값은 다음 OnRoad가 시작될 때 고정되므로, 변경 후 현재 주행을 끝내고 차량을 재시동하거나 기기를 재부팅해야 적용됩니다. 기존 `RadarMotionMode` 값은 업데이트 후 처음 시작할 때 새 이름으로 한 번 자동 이관됩니다.
- `CarrotRadarCutInSensitivity`: 당근레이더모드 전용 CUT-IN 감도입니다. `0`은 사용 안 함, `1`은 둔감, `3`은 보통(기본값), `5`는 아주 민감이며 `2`와 `4`는 중간 단계입니다. 단계 `1`~`5`는 실제 측정 움직임이 각각 `0.50`, `0.40`, `0.35`, `0.25`, `0.20초` 계속될 때 확정하며, 물리 미래 예측시간은 5.0초로 고정합니다. 전방 레이더의 최근 실측 이력에서 0.50m 이상 강한 단방향 진입이 확인되면 timestamp 양자화로 확정을 놓치지 않도록 최대 20Hz 레이더 한 프레임만 반영하며, 작은 인접 차로 흔들림에는 적용하지 않습니다. 기존 레이더모드와 `EnableCornerRadar`에는 영향을 주지 않습니다. 다음 OnRoad 시작 때 읽으므로 변경 후 차량을 재시동하거나 기기를 재부팅해야 적용됩니다.
- `DisableDM`: 운전자 모니터링을 비활성화할 수 있는 안전 관련 항목이며 재부팅이 필요합니다.
- `SpeedFromPCM`: 비롱컨 순정 SCC의 버튼 스패밍과 커브·카메라 감속 방식에 영향을 줍니다.

`HardwareC3xLite`는 일반 C3/C3X에서는 반드시 꺼 두고 C3X Lite에서만 켠 뒤 기기를 재부팅하세요. 이 설정을 켜면 존재하지 않는 앰프를 초기화하지 않아 I2C 재시도로 인한 시작 지연을 없애고, 경고음을 GPIO 부저로 출력합니다. 또한 `micd`, `soundd`, `loggerd`를 실행하지 않고 `RecordAudio`를 끄므로 이 하드웨어 모드에서는 일반 주행 로그 기록을 사용할 수 없습니다.

<a id="display"></a>
## 화면 표시

화면 표시에는 37개 항목이 있습니다. 일반 화면 항목은 비교적 되돌리기 쉽지만, 외부 HUD는 별도 하드웨어와 성능 설정을 포함합니다.

| 중분류 | 파라미터 | 용도 |
|---|---|---|
| 정보 표시 | `ShowDebugUI`, `ShowTpms`, `ShowDateTime`, `ShowPathEnd`, `ShowDeviceState`, `ShowLaneInfo`, `ShowRadarInfo`, `ShowRouteInfo`, `ShowPlotMode` | 주행 화면의 디버그, 타이어, 시간, 차선, 레이더와 경로 정보 |
| 경로 표시 | `ShowPathMode`, `ShowPathColor`, `ShowPathColorCruiseOff`, `ShowPathModeLane`, `ShowPathColorLane` | 레인리스·레인모드·크루즈 OFF 상태의 경로 모양과 색상 |
| 밝기·주행화면 | `ShowCustomBrightness`, `ShowModelView`, `ShowCameraWithCluster` | 주행 중 밝기, 카메라·모델 표시 조합과 외부 HUD 연결 중 본체 카메라 표시 |
| 외부 HUD·기본 | `ClusterHud`, `ClusterHudBrightness`, `ClusterHudOrientation`, `ClusterHudMirror`, `ClusterHudTheme`, `ClusterNaviMapTheme`, `ClusterNaviMapType`, `ClusterNaviMapFps` | TURZX 외부 HUD, 밝기, 화면 회전, 미러링과 지도 테마 |
| 외부 HUD·화면·카메라 | `ClusterHudEncoder`, `ClusterHudLiveFps`, `ClusterHudScreenMode`, `ClusterHudPanelLayout`, `ClusterHudCameraViewMode` | 인코더, 전송 FPS와 화면·카메라·좌우 패널 구성 |
| 외부 HUD·레이더 표시 | `ClusterHudRadarInfo`, `ClusterHudRadarDisplay`, `ClusterHudRadarSourceColor` | 외부 HUD의 레이더 정보와 색상 |
| 외부 HUD·성능·디버그 | `ClusterHudCoreMode`, `ClusterHudPriority`, `ClusterHudDebug` | CPU 코어, 프로세스 우선순위와 진단 정보 |

`ShowRouteInfo` 설명에 남아 있는 APN 표기는 경로 정보 입력 상태를 뜻합니다. 이를 CarrotMan 또는 CarrotLink 지원 안내로 해석하면 안 됩니다.

`ShowCustomBrightness=0`은 주변 밝기에 따른 자동 조절이고, `ShowModelView`는 카메라와 모델 표시 조합을 선택합니다. `ShowCameraWithCluster=0`은 외부 HUD 연결 중 본체 카메라를 숨기는 기존 기본 동작이고, `1`은 본체 카메라 영상을 표시합니다. `ClusterHud` 계열은 지원되는 외부 HUD를 연결한 경우에만 사용하세요.

`ClusterHudBrightness=0`은 카메라 노출값을 따르는 자동 밝기이고, `1~100`은 고정 밝기입니다. `ClusterHudOrientation`은 `0`(0도)과 `2`(180도)만 지원하며 `1`, `3`은 무시합니다. 실행 중인 TURZX 프로세스는 두 저장값을 100ms마다 확인합니다. 밝기는 실행 중 적용되고, 관리형 H.264의 회전값이 바뀌면 HUD가 자동 재시작되어 캡처와 동일한 스트림 설정 절차로 적용됩니다.

`ClusterHudPanelLayout=0`은 `ClusterHudCameraViewMode`가 선택한 주행 화면을 왼쪽에, 화면 모드·디버그·내비 상태에 따라 선택되는 정보 패널을 오른쪽에 배치합니다. `1`은 두 영역을 서로 바꿔 정보 패널을 왼쪽에, 주행 화면을 오른쪽에 표시합니다. 실행 중 약 1초 안에 적용되며 재시작하지 않습니다. 전체화면 그래프와 전체화면 내비처럼 좌우 영역이 없는 모드는 바뀌지 않습니다. `ClusterHudDebug`은 HUD 상시 출력과 디버그 UI·내비 입력을 강제로 켜는 설정이며, 그 결과 표시되는 디버그·내비 정보 패널도 선택한 좌우 배치를 따릅니다.

외부 HUD가 USB로 연결된 동안 `ShowCameraWithCluster=0`이면 일반 C3/C3X와 mici의 본체 주행 화면은 검은 배경으로 전환하고 카메라 영상과 모델 경로 렌더링을 생략합니다. `1`이면 이 연결 전용 억제를 해제해 본체 카메라 영상과 기존 주행화면 렌더 경로를 사용합니다. 값은 주행 중에도 최대 약 5초 안에 반영됩니다. 어느 값에서도 속도·제한속도·운전자 상태·경고·주행 상태 테두리 등 본체 HUD는 계속 표시되며, 외부 HUD 연결이 끊기면 이 옵션은 화면에 영향을 주지 않습니다. `0`에서도 `camerad`와 모델 입력은 계속 동작하고 본체 화면의 중복 렌더링만 줄입니다.

`ClusterHudScreenMode`의 최종 화면 구성은 다음과 같습니다.

- `-1`은 카메라 뷰 `0`, `1`의 3D 화면에서만 정보 패널과 월드 이동을 제거하고 전체 폭을 사용합니다. 왼쪽 HUD는 기존 여백을 유지하고, 오른쪽 게이지·TPMS는 물리 화면 오른쪽 여백에 맞추며, 시계·월드·방향지시등은 전체 화면 중앙축을 사용합니다. 로드카메라 뷰 `2`에서는 내비 유무에 따른 자동 주행 리포트와 `ClusterHudPanelLayout`까지 `0`과 완전히 동일하게 동작합니다.
- `0`은 기본 화면입니다. 내비가 수신되면 내비 패널을, 수신되지 않으면 주행 리포트를 자동으로 표시합니다.
- `1`은 라이브 지연·토크·조향·횡방향 계획을 2×2 카드로 묶은 일반 디버그 화면입니다. 3D 카메라 뷰에서는 모드 `0`처럼 1124px 주행 영역에 월드와 주행 HUD를 맞추고, 반대편 792px 정보 영역 전체를 네 개의 라이브 디버그 카드가 사용합니다. 가속·조향·연료·요소수 게이지와 TPMS는 주행 영역의 오른쪽 경계 안에 유지됩니다.
- `2`는 고정 시스템 디버그 화면입니다. 상세 카드에는 네트워크·출력 해상도/프레임률·카메라 상태·메모리 용량·코어별 CPU 사용률을 표시하고, 시스템 헬스 카드에는 CPU·온도·메모리·디스크 사용량을 2×2 원형 게이지로 표시하며 디바이스 피치/요 타깃과 각도 숫자도 함께 보여 줍니다. 3D 카메라 뷰에서는 모드 `0`처럼 1124px 주행 영역에 월드와 주행 HUD를 맞추고 두 카드는 반대편 792px 정보 영역을 사용합니다. 내비 연결·라이브 안내·내비 디버그 상태와 관계없이 이 화면을 유지하며, 내비·`NAVI DISCONNECTED`·경로 오버레이·주행 리포트로 자동 전환하지 않습니다.
- `3`은 주행 장면을 끄고 `ShowPlotMode` 그래프를 크게 표시합니다.
- `4`는 3D 월드와 주행 HUD를 모드 `0`과 같은 1124px 주행 영역에 맞추고, 반대편 792px 정보 영역 전체에 같은 그래프를 표시합니다. 가속·조향·연료·요소수 게이지와 TPMS는 주행 영역의 오른쪽 경계 안에 유지되며, 좌우 패널을 바꾸면 그래프와 주행 영역이 서로 함께 교환됩니다.
- `5`는 주행 리포트를 항상 표시합니다.

`ClusterHudScreenMode=5`는 정보 패널에 실시간 주행 리포트를 표시합니다. 기본 화면 모드(`0`)에서도 실시간 내비가 수신되지 않는 동안에는 이 리포트가 자동으로 표시되고, 수신이 시작되면 내비 패널로 돌아갑니다. 리포트 배경·카드·테두리·기본/보조 글자와 값이 없는 상태의 색상은 현재 `ClusterHudTheme`를 따라 자동·다크·라이트 모드에 맞게 전환됩니다. 넓은 카드에는 주행 시간·거리·평균/최고 속도·자동주행 비율·최대 가감속·급가감속/급코너 횟수를 큰 글씨로 표시합니다. 작은 카드는 CPU·온도·메모리·디스크 사용량을 2×2 원형 게이지로 표시하고, 하단 타깃은 저장된 디바이스 피치(P)와 요(Y)를 정상 중심점 대비 상하·좌우 방향으로 시각화하면서 각도 숫자도 함께 표시합니다. 주행 화면 아래에는 브랜치·네트워크 주소·프레임률 상태를 계속 표시하고, 리포트와 겹치는 코어 사용량 문구만 생략합니다. 로드카메라 화면에서는 감지 차량을 내부가 투명한 라운드 사각 프레임으로 감싸고 기존 감지 색상을 테두리에 적용합니다. 그룹화되지 않은 레이더 감지점은 같은 방식의 작은 소스 색상 표식으로 표시합니다. 차량 프레임은 가벼운 단일 테두리로 그리고, 화면 경계에서 일부만 투영되거나 레이더 방위값 때문에 비정상적으로 길어지는 프레임은 표시하지 않습니다.

외부 HUD는 기기의 `LanguageSetting`을 따라 주행 리포트, 주행 모드와 내비 상태 문구를 한글(`ko`) 또는 영어(`en`)로 실시간 전환합니다. 중국어를 포함한 그 밖의 언어값은 영문으로 표시합니다. `IsMetric`이 켜져 있으면 현재/설정/제한 속도, 내비, 레이더 라벨과 주행 리포트를 `km/h`, `m`, `km`로 표시하고, 꺼져 있으면 `mph`, `ft`, `mi`로 변환합니다. 가속도와 온도는 각각 `m/s²`, `°C`를 유지합니다. 두 설정은 약 1초마다 확인하므로 HUD를 다시 시작하지 않아도 적용됩니다.

기본 화면과 로드카메라 화면의 TPMS는 같은 위치에 표시됩니다. 기존 압력 글자 크기는 유지하며, 네 압력값을 단순한 장난감 자동차의 큰 바퀴 안에 각각 배치합니다. 네 값이 모두 없을 때만 전체가 숨겨지고, 일부 값만 없으면 해당 바퀴에 `--`를 표시하며 31 psi 미만은 빨간색으로 표시합니다.

속도 표시 옆의 주행 모드 문구는 배경과 테두리 없이 속도 숫자의 마지막 자리 위에 확대 표시합니다. 연비는 녹색, 안전은 주황색, 일반은 흰색, 고속은 빨간색입니다. 기어 배지도 내부 배경 없이 글자와 테두리만 표시합니다. 왼쪽 위의 조향/LFA·Wi-Fi·시계는 화면 가장자리 및 아이콘 사이 여백을 넓히고, 제한속도 표지는 약간 왼쪽으로 배치합니다.

`ClusterHudTheme=1`(다크)은 일반 HUD의 빈 배경을 지도와 `NAVI DISCONNECTED` 상태 영역에 사용하는 것과 같은 순수 검정으로 표시합니다. 자동(`0`)도 18:00~06:00에는 같은 다크 팔레트를 사용합니다. 도로, 게이지와 일반 정보 패널은 구분과 가독성을 위해 서로 다른 어두운 음영을 유지합니다.

CAN FD 현대·기아 하이브리드 차량에서 외부 HUD의 초록색 `EV` 표시는 ECAN에 `0xFA`와 `0x230`이 모두 DLC32로 존재할 때만 활성화됩니다. `0x230`의 4비트 하이브리드 동력 흐름 모드를 해석해 확인된 모터 주행·회생 모드 1, 2, 6에서 `EV`를 표시합니다. 일반 HUD에서만 현재 속도와 설정 속도 사이에 표시하며, 전체 내비 화면에서는 표시하지 않습니다. 그 밖의 모드이거나 지원 조건 또는 샘플이 없거나, 유효하지 않거나, 오래된 경우에도 표시하지 않습니다.

일반 외부 HUD는 현재 속도 위의 교통 상태 점 옆에 현재 주행 모드도 표시합니다. `MyDrivingMode` 값에 따라 `1` 연비는 초록색, `2` 안전은 주황색, `3` 일반은 흰색, `4` 고속은 빨간색 배지로 표시합니다. `longitudinalPlan`이 없거나 유효하지 않거나 오래되었거나 모드 값이 범위를 벗어나면 배지를 숨기며, 전체 내비 화면에서는 표시하지 않습니다. 바로 왼쪽의 빨강·초록 점은 주행 모드가 아니라 모델의 교통 상태를 나타내는 별도 표시입니다.

일반 화면과 도로 카메라 화면의 TPMS는 가속·조향·연료·요소수 게이지 아래에서 같은 고정형 차량 그림과 위치를 사용합니다. 네 압력값이 모두 없을 때만 전체 표시를 숨기며, 일부 값만 없으면 해당 위치에 `--`를 표시합니다. 31 psi 미만은 빨간색이고 차량 그림 바깥에는 별도 배경이나 외곽선을 그리지 않습니다. 외부 내비가 활성화됐거나 내비 대시보드가 연결되면 기존 우측 하단 `NAVI` 대신 Wi-Fi 아이콘 아래에 초록색 `NAV`를 표시합니다. 중앙 시계와 EV·연료·요소수 표시는 그대로 유지됩니다.

### Carrot Vision AR과 리플레이 내비 이벤트

Carrot Vision에는 `carrot_settings.json` 카탈로그와 별도로 **AR 표시**와 **AR 디버그** 설정이 있습니다. **AR 표시**를 켜면 Vision 영상 위에 주행 안내를 표시하며, 켜져 있는 동안에만 추가 실시간 데이터를 요청합니다. **AR 디버그**는 표지·앵커·렌더 수, 현재 차단 사유와 복사 가능한 변화 이력을 진단 패널에 표시합니다.

리플레이 이벤트 타임라인은 당근내비 연결과 경로 세션 변경, 현재·다음 주행 안내, 권장 차선, 도로 안전 안내, 구간단속, 신호와 교차로 안내의 변화를 구분해 표시합니다. 이 항목들은 리플레이에 기록된 상태 변화를 검토하기 위한 이벤트 라벨입니다.

<a id="system"></a>
## 시스템

시스템에는 녹화, 전원, 네트워크, 지도, 소리와 소프트웨어 메뉴를 다루는 11개 항목이 있습니다.

| 중분류 | 파라미터 | 용도 |
|---|---|---|
| 녹화·전원 | `RecordRoadCam`, `MaxTimeOffroadMin` | 도로 카메라 저장과 시동 OFF 후 자동 전원 종료 시간 |
| YouTube 라이브 | `CarrotYouTubeLive`, `CarrotYouTubeQuality`, `CarrotYouTubeTimestamp` | 카메라 영상 송출, 품질과 타임스탬프 |
| 네트워크·지도 | `HotspotOnBoot`, `MapboxStyle` | 부팅 시 핫스팟과 지도 배경 스타일 |
| 사운드 | `SoundLanguageSetting`, `SoundVolumeAdjust`, `SoundVolumeAdjustEngage` | 안내음 언어와 일반·인게이지 볼륨 |
| 소프트웨어 | `SoftwareMenu` | Carrot Web의 소프트웨어 메뉴 활성화 |

- `RecordRoadCam`: `0` 녹화 안 함, `1` 일반 카메라, `2` 일반+광각 카메라입니다. 저장 공간 사용량을 확인하세요.
- `MaxTimeOffroadMin`: 시동이 꺼진 뒤 장치가 자동으로 꺼질 때까지의 시간입니다.
- `CarrotYouTubeLive`: 네트워크 사용량, 발열과 개인정보 노출 가능성을 함께 확인하세요.
- `HotspotOnBoot`: USIM을 장착한 장치에서 자동 핫스팟을 사용할 때의 설정입니다.
- `SoftwareMenu`: 메모리 문제가 있을 때 끌 수 있는 Carrot Web 메뉴 설정입니다.

## 위험도별로 접근하기

### 비교적 쉽게 되돌릴 수 있는 설정

- 화면 정보, 경로 모양·색상
- 밝기와 사운드
- 즐겨찾기와 프로필

### 주행 동작을 바꾸는 설정

- 상시 조향과 오토인게이지
- 버튼 모드와 자동 턴
- 커브·카메라·신호 감속
- 차간거리, 정지와 가감속 튜닝
- 조향 지연, 평활화와 토크 계수

### 차량 구성을 확인해야 하는 설정

- `HyundaiCameraSCC`, `CanfdHDA2`
- `EnableRadarTracks`, `EnableCornerRadar`, `CarrotRadarMode`, `CarrotRadarCutInSensitivity`
- `SpeedFromPCM`, `DisableMinSteerSpeed`
- `LateralTorqueCustom`, `CustomSteer*`
- `DisableDM`

## 안전한 변경 순서

1. Carrot Web에서 현재 값을 확인합니다.
2. 기준 프로필과 파일 백업을 모두 만듭니다.
3. 변경할 파라미터 이름, 이전 값과 변경 이유를 기록합니다.
4. 한 번에 한 항목만 `unit` 한 단계만큼 바꿉니다.
5. 재부팅 안내가 나오면 차량이 꺼진 오프로드 상태에서 재부팅합니다.
6. 허용된 시험 환경의 같은 조건에서 결과를 반복 확인합니다.
7. 나빠지거나 판단하기 어렵다면 즉시 이전 값 또는 기준 프로필로 복원합니다.

조향과 가감속의 실제 조정 순서는 [튜닝 입문](https://github.com/ajouatom/openpilot/wiki/Guide-Tuning)을 참고하세요.
