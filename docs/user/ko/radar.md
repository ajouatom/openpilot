# 레이더트랙·코너레이더

[English](../en/radar.md)

> [!NOTE]
> 이 문서는 `carrot-wip` 코드와 함께 관리하는 사용자 설명서 원본입니다. 사용자 동작이 바뀌면 관련 코드·테스트와 같은 변경에서 이 문서도 갱신합니다.

레이더가 차량에 장착되어 있어도 carrotpilot이 필요한 CAN 메시지를 읽을 수 있다는 뜻은 아닙니다. 같은 차종도 연식, 옵션, 레이더 부품, 펌웨어, 메시지 그룹과 하네스 연결 위치에 따라 결과가 달라집니다.

> [!WARNING]
> 검증되지 않은 레이더 설정은 경고등, CAN 오류, 선행차 오인식 또는 잘못된 끼어들기 판단을 일으킬 수 있습니다. 변경 전 원래 값을 기록하고 정확히 같은 차량 구성이 검증됐을 때만 한 단계씩 시험하세요.

<a id="front-radar"></a>
## 전방 레이더 트랙

`EnableRadarTracks`는 전방 선행차 정보의 원본과 추가 처리를 선택합니다.

| 값 | 현재 코드의 의미 | 사용 판단 |
|---:|---|---|
| `-2` | VOACC 비전 전용 시험 | 개발 시험용 |
| `-1` | SCC를 항상 사용 | 차량 구성 확인 필요 |
| `0` | 순정 SCC 레이더 사용 | 기본값 |
| `1` | 전방 레이더의 원시 트랙 사용 | 차량별 레이더 활성화와 메시지 지원 필요 |
| `2` | 레이더 트랙과 저속 SCC 병행 | 검증된 동일 구성에서만 시험 |
| `3` | 위 기능에 끼어들기·스텔스 차량 처리를 추가 | 오검출 가능성이 있는 실험 기능 |

비-CAN FD 현대·기아 차량에서 양수 값을 사용하면 시작 과정에서 레이더 트랙 활성화를 시도하고 결과를 `EnableRadarTracksResult`에 저장합니다. 물리적인 레이더가 있어도 활성화 결과와 실제 트랙 수신을 함께 확인해야 합니다.

<a id="corner-radar"></a>
## 코너 레이더

| `EnableCornerRadar` | 의미 |
|---:|---|
| `0` | 사용하지 않음 |
| `1` | 지원되는 코너 레이더 트랙 사용 |
| `2` | 코너 레이더를 이용한 끼어드는 차량 검출 추가 |

코너 레이더 객체는 차량 코드가 인식한 지원 메시지 그룹이 있을 때만 생성됩니다. 지원되는 0x430 메시지군도 전방 레이더가 아닌 코너 레이더 입력으로 구분됩니다. 현재 `2`의 끼어들기 처리는 현대 계열을 중심으로 동작하므로 다른 제조사에 같은 값을 일반화하면 안 됩니다.

### dPath 물리 레이더 처리

| `RadarDPathMode` | 의미 |
|---:|---|
| `0` | 기존 `radard` 선행차·끼어들기 처리 유지(기본값) |
| `1` | 독립 dPath RadarD가 front/SCC–vision leadOne을 먼저 계산하고, 물리 dPath CUT-IN만 leadTwo로 사용 |

`1`은 학습 모델을 사용하지 않으며 기존 `controls/radard.py`를 호출하거나 그 결과를 섞지 않습니다. 독립 프로세스가 삭제 전 `radard_model.py`와 같은 순서로 model lead zero를 전방/SCC 레이더와 먼저 매칭해 leadOne을 정합니다. 그 뒤 물리 predictor가 0.25초 동안 확인한 OUT→IN 차량 중 leadOne과 다른 객체만 leadTwo 후보가 됩니다. 최근 0.75초의 leadOne은 거리·횡위치·속도 연속성으로도 중복을 막아 비전 ID가 잠깐 바뀌었을 때 같은 차량을 leadTwo로 다시 내보내지 않습니다. leadOne보다 먼 차량과 `max(20 m, vEgo × 2초 + 10 m)` 밖의 차량은 종방향 제어 후보에서 제외합니다.

코너 레이더 포인트가 한 번이라도 확인되면 해당 실행 동안 코너 motion만 사용하고, 코너 포인트가 없는 구성에서는 `frontRadar` raw track만 사용합니다. 프레임마다 front와 corner를 교체하지 않으며 SCC는 dPath motion 입력으로 사용하지 않습니다. 잘못된 leadTwo는 실제 감속에 영향을 줄 수 있으므로 전체 shadow 검증을 마친 동일 차량 구성에서만 켜세요.

차로 변경 보조가 사용하는 `leadLeft`, `leadRight`와 측면 목록도 같은 motion 센서에서 보이는 좌·우 인접 차량 위치로 발행합니다. `|vLead| < 3 km/h` 포인트는 이 현재 위치에는 사용할 수 있지만 dPath 이력을 만들거나 leadTwo로 예측하지 않습니다.

<a id="lead-selection"></a>
## 선행차 선택과 검증

manager는 두 레이더 구현을 동시에 실행하지 않습니다. `RadarDPathMode=0`에서는 기존 `openpilot.selfdrive.controls.radard`만 실행하고 기존 leadOne/leadTwo 선택을 그대로 유지합니다. `RadarDPathMode=1`에서는 기존 프로세스를 중지하고 독립 `openpilot.selfdrive.carrot.radar.radard_dpath`만 실행합니다. 이 프로세스는 front/SCC–vision 매칭으로 leadOne을 먼저 계산한 뒤 아래 물리 predictor로 leadTwo를 계산해 `radarState`를 직접 발행합니다. 전방 레이더, SCC, 코너 레이더의 입력 역할과 소스 구분은 유지하며 학습형 레이더 리드 모델은 사용하지 않습니다.

headless 검증기는 기존 radard와 실험 중인 단순 물리 predictor를 별도로 보고할 수 있습니다. 화면 리플레이는 의도적으로 물리 predictor만 표시하며 기존 radard의 `leadOne`, `leadTwo`, CUT-IN 마커를 가져오지 않고 종방향 제어도 바꾸지 않습니다.

shadow predictor는 다음 원칙으로 동작합니다.

- `measured=true` 레이더 포인트만 사용합니다.
- 자차 뒤 5m부터 전방 100m까지의 포인트만 motion 예측에 사용합니다.
- 리플레이 레이더 포인트를 실측 상대속도로 model path 시점에 먼저 맞춘 뒤 같은 시점의 model path polyline에 수직 투영합니다. 중심선을 따라간 호 길이가 `S`, 중심선 법선 방향의 부호 있는 거리가 `dPath`입니다.
- model path 기준 고정 범위 `|dPath| ≤ 5.4 m`를 사용해 ego lane과 바로 좌·우 인접 차로만 처리합니다.
- 좌·우 각 인접 차로에서는 5m 이내 포인트와 5m 이상에서 가장 가까운 차량을 유지하고, 같은 방향에서 그 차량 뒤에 가려진 더 먼 차량은 검출 대상에서 제외합니다. 단, 범위 안의 실측 이력은 유지해 가려졌던 차량이 보이기 시작할 때 물리 연속성을 잃지 않습니다.
- 로그에 실측 코너레이더 데이터가 있으면 코너 motion만 사용하고, 없으면 `frontRadar` raw track만 사용합니다. SCC는 기존 radard 표시에는 남지만 motion predictor 입력으로 사용하지 않으며, 개별 프레임마다 소스를 바꾸지 않습니다.
- `|vLead| < 3 km/h` 포인트는 현재 위치만 표시하고 motion 이력을 만들거나 미래로 외삽하지 않습니다.
- 프레임마다 차선 중심과 model path를 바꿔 사용하지 않습니다.
- 레이더 포인트와 경로를 같은 시점·자차 좌표계로 맞춘 뒤에는 yawRate를 `dPath`에 다시 보정하지 않습니다.
- 재사용된 track ID와 짧은 단절은 물리적인 위치·속도 연속성으로 검사합니다.
- front와 corner의 이력과 파라미터를 분리합니다.
- raw `dRel`을 그대로 경로 거리로 보지 않고, 중심선에 투영한 `S`와 누적 자차 이동거리, 법선 방향 `dPath`로 2차원 경로좌표 이력을 만듭니다.
- 시간에 따른 횡이동이 아니라 대상의 실제 `S` 진행량에 대한 `dPath` 기울기를 사용하므로, 종방향으로 거의 진행하지 않은 포인트의 흔들림을 미래 CUT-IN으로 연장하지 않습니다.
- 긴 구간 `(S, dPath)` 이동 벡터와 model path 접선에 대한 각도 오차를 예측 평균으로 사용하고, 미래 외삽 거리가 관측한 공간 이력보다 길면 신뢰도를 낮추며, 짧은 구간과의 차이는 방향을 억지로 꺾지 않고 곡률과 불확실도를 키우는 데 사용합니다.
- 코너 레이더는 위치 이력으로 계산한 법선 방향 움직임과 레이더가 보고한 횡속도를 비교합니다. 차량 표면에서 반사 기준점이 옮겨간 경우처럼 두 값이 같은 물리 움직임을 설명하지 않으면 CUT-IN/CUT-OUT 신뢰도를 낮춥니다.
- 0.5, 1.0, 1.5, 2.0초의 미래 `dRel`과 `dPath`를 같은 시점으로 예측합니다.
- CUT-IN과 CUT-OUT 확률을 별도로 표시합니다.

현재 경로 겹침 판정에는 자차와 대상 차량의 반폭을 포함합니다. 처음 관측될 때부터 경로에 걸친 차량은 현재 `IN`으로 표시하며 새로운 shadow CUT-IN으로 취급하지 않습니다. 물리 이력으로 추적하던 차량의 `OUT → IN` 전이는 경계를 지난 뒤에도 진입 근거를 유지하므로, 겹침이 시작된 뒤 0.25초 확인을 마칠 수 있습니다. 경로 상태와 확정에 작은 hysteresis만 사용하며 경로·차종·장면별 예외는 추가하지 않습니다.

### PC 리플레이

`radar_lead_validation_review.py`는 같은 로그의 유지 중인 검증 사례를 묶어 40개 고유 로그를 순서대로 한 번씩 엽니다. 각 화면에는 동기화된 qcamera 비디오와 물리 predictor가 사용·검출한 포인트, 궤적, 확률, CUT-IN 이벤트만 표시합니다. 기존 radard의 `leadOne`, `leadTwo`, CUT-IN 포인트와 이벤트 마커는 표시하지 않습니다. 한 로그가 끝나면 창을 닫고 다음 로그를 자동으로 엽니다. `--front-only`는 리플레이 전에 코너 포인트를 제거합니다. `--prob`는 predictor의 표시·일시정지 기준만 바꾸며 물리 계산식은 바꾸지 않습니다.

물리 predictor가 0.25초 동안 새로운 CUT-IN을 확인했을 때만 재생이 멈춥니다. 화면은 한글을 지원하는 읽기 쉬운 글꼴과 한글 작업 문구를 사용합니다. bird's-eye 지도에서 회색선은 model lane line, 흰 점선은 화면 비교용 차선 중심, 파란선은 predictor가 유일한 corridor 기준으로 사용하는 model path입니다. 계산 중 차선 중심으로 바꾸지 않습니다. 기본으로 모든 track에 그려지는 소스 색의 흐려지는 채운 점·실선은 predictor가 실제로 사용하는 경로좌표 `(S, dPath)` 과거 이력입니다. 회색 또는 녹색 빈 원은 같은 이력으로 계산한 0.5/1.0/1.5/2.0초 미래 `(S, dPath)` 위치이고, 확정된 predictor CUT-IN만 주황색으로 바뀝니다. `H`로 이 계산 이력과 미래선을 표시하거나 숨깁니다. `A`를 누르면 ego 이동을 보정한 raw radar `(xRel, yRel)` 과거 관측을 회색으로 별도 겹쳐 볼 수 있습니다. 이 선택적 raw 관측선은 대상 차량의 ground truth도 아니고 predictor가 외삽하는 이력도 아닙니다. ego yaw는 이 선택적 raw 관측선을 한 좌표계에 정렬할 때만 쓰며 시점 동기화된 `dPath`에 다시 적용하지 않습니다. 가로 seek bar에는 확정된 predictor CUT-IN 진입만 주황색으로 표시하고 유지 중인 검증 구간은 bar 위쪽에 표시합니다. 기존 radard 마커는 없습니다. 레이더 지도는 qcamera 위에 투영한 원근 화면이 아니라 자차 좌표계의 bird's-eye 화면입니다. bar를 클릭해 이동할 수 있으며, Space는 일시정지, 좌우 키는 키보드 이동, 위아래 키는 재생 속도 변경, `M`은 predictor CUT-IN 마커 표시/숨김, `R`은 처음부터 다시 재생하면서 처리한 predictor 일시정지를 재활성화합니다. `I`, `C`, `S`는 CUT-IN, CLEAR, STATIONARY 라벨을 적용합니다. 유지 중인 검증 구간 안에서는 해당 사례를 갱신하고, 그 밖에서는 `radar_trajectory_labels.json`에 저장합니다.

`validate_radar_lead_model.py` 파일명은 기존 사용 명령과의 호환을 위해 유지하지만 더 이상 학습 모델을 불러오거나 검증하지 않습니다. `cutin_validation_cases.json`과 `radar_trajectory_labels.json` 전체를 재생해 기존 radard와 물리 shadow 결과를 따로 보고합니다. 모든 수동 라벨은 검증 전용이며 계산식, 임계값 조정이나 학습에 사용하지 않습니다.

## 레이더 감지음

openpilot이 활성화된 상태에서 새로운 끼어들기 차량이 확정되면 2단계 알림음이 재생됩니다. 같은 객체가 계속 추적되는 동안에는 한 번만 울립니다. 스피커가 없는 C3X Lite에서는 같은 이벤트를 GPIO 부저 패턴으로 알립니다. 이 알림음은 선택된 기존 radard 결과를 알려줄 뿐 선행차 선택이나 종방향 제어를 변경하지 않습니다. 더 높은 우선순위의 안전 경고가 발생하면 해당 경고음이 우선할 수 있습니다.

## 하네스 프리셋과의 관계

현재 최초 설정 프리셋은 다음처럼 시작합니다.

| 구성 | `EnableRadarTracks` | `EnableCornerRadar` |
|---|---:|---:|
| ADAS 모듈 하네스 | `0` | `1` |
| 카메라 하네스 | `0` | `0` |
| 순정 SCC 유지 | `0` | `0` |

ADAS 프리셋이 코너 레이더를 켠다는 것은 하네스에서 접근 가능한 구성을 뜻할 뿐, 모든 ADAS 차량의 코너 레이더 메시지가 검증됐다는 뜻은 아닙니다.

## 확인 순서

1. 차종, 연식, 트림, HDA 세대와 정확한 하네스 위치를 확인합니다.
2. 현재 값을 기록하고 기본값 `0`에서 정상 동작을 먼저 확인합니다.
3. 동일 차량 구성의 검증 기록에서 지원되는 레이더 메시지를 확인합니다.
4. 한 번에 한 설정만 변경하고 차량을 다시 시작합니다.
5. 정차 상태에서 경고등과 CAN 오류를 확인합니다.
6. 안전한 시험 환경에서 선행차 거리·상대속도와 끼어들기 오검출을 확인합니다.
7. 이상이 있으면 즉시 원래 값으로 복원합니다.

## 코드 분석 기준

- 설정 범위와 설명: `openpilot/selfdrive/carrot_settings.json`
- 기존 production 레이더 선행차 선택: `openpilot/selfdrive/controls/radard.py`
- 독립 dPath RadarD: `openpilot/selfdrive/carrot/radar/radard_dpath.py`
- front/SCC–vision leadOne 매칭: `openpilot/selfdrive/carrot/radar_motion/primary.py`
- leadOne 우선·leadTwo 후처리 순서: `openpilot/selfdrive/carrot/radar_motion/controller.py`
- 물리 shadow predictor: `openpilot/selfdrive/carrot/radar_motion/predictor.py`
- dPath leadTwo 선택: `openpilot/selfdrive/carrot/radar_motion/lead_selection.py`
- PC 리플레이: `openpilot/selfdrive/carrot/radar/tools/radar_validation_replay.py`
- 현대·기아 레이더 메시지 파싱: `opendbc_repo/opendbc/car/hyundai/radar_interface.py`
- 비-CAN FD 레이더 활성화: `opendbc_repo/opendbc/car/hyundai/interface.py`
- 최초 설정 프리셋: `openpilot/selfdrive/carrot/server/features/intro/presets.py`
