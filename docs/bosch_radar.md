# Bosch MRRevo14F 전방 레이더 provider

비-CAN FD 현대·기아 차량에서 Bosch MRRevo14F 전방 레이더가 보내는 객체 메시지를 passive로 받아
`liveTracks`로 발행하는 provider의 설계·검증 요약입니다. 사용자 설정 설명은
[radar.md](user/ko/radar.md#bosch-passive-reception)에 있습니다.

## 1. 범위와 감지

- **대상:** `values.BOSCH_MRREVO14F_CAR`
  - 실차 검증 차종: 아반떼 AD `HYUNDAI_ELANTRA` (레이더 OE `96400-F2000`, Bosch `0 203 300 251`)
  - 미검증 후보: i30 PD `HYUNDAI_ELANTRA_GT_I30`, 코나 `HYUNDAI_KONA`, 코나 일렉트릭 `HYUNDAI_KONA_EV`·`HYUNDAI_KONA_EV_2022`, K3 BD/Forte `KIA_FORTE`
- **감지:** `values.bosch_mrrevo14f_bus()`
  - `0x601~0x612` 18개가 한 수신 버스에서 모두 DLC 8로 보여야 합니다.
  - 카메라 버스를 먼저 보고, 다음에 ACAN을 봅니다.
  - 감지되면 `HyundaiExtFlags.BOSCH_RADAR`를 켜고, ACAN이면 `BOSCH_RADAR_BUS1`도 켭니다.
  - 하네스 커넥터 이름이 아니라 실제로 관측한 버스로 정합니다.
- **passive 수신:** `EnableRadarTracks > 0`일 때 다음 두 가지를 건너뜁니다.
  - 레이더 트랙 활성화 UDS 쓰기
  - SCC ECU 통신 비활성화(`disable_ecu`). 이 요청을 보내면 객체 메시지가 멈추기 때문입니다.
  - `EnableRadarTracksResult`는 갱신하지 않습니다.

## 2. 구조

```
can + carState(radarInput 배치) ─┐
livePose, modelV2 (context) ─────┤
                                 ▼
radarcan.py ── RadarInterface.update_carrot / set_bosch_context
                                 ▼
radar_bosch.BoschRadarProvider
  decode 0x601~0x612 → raw track → physical PID tracker
  → object grouping (strict pair, camera-extended, mature retention)
  → publication view (filters, alias, aLead) → liveTracks 20 Hz
```

- **radarcan:** Bosch provider가 있을 때만 `livePose`·`modelV2`를 구독합니다. 배치 수신 시각(`receiveMonoTime`)을 now로 씁니다. 발행은 `frame % 5`, 즉 20 Hz입니다.
- **solver:** 배정 solver는 numpy 구현입니다(기기에는 scipy가 없습니다).
- **카메라 object CAN:** LDWS 카메라 object(실측 LSB 0.05)를 다음 용도에 씁니다.
  - 대형차 병합
  - 연관 판단(A0)
  - 신생 점 억제
- **발행 필터:** 모두 발행 단계에서만 동작하고, 추적 상태는 바꾸지 않습니다.
  - road-edge 밖 정지물 억제
  - 벽 거울(mirror birth·M3) ghost 억제
  - native surface가 지지하지 않는 신생 점 억제
  - OEM `0x601` validation veto
  - B5 신생 defer
  - companion(B1) deferral
  - burst multi-return defer
  - P91 지속 clone 억제
  - corner 억제는 dRel ≥ 100 m에만 적용합니다.
- **radard_dpath:** Bosch 활성 구성에서는 `modelV2`와 `liveTracks`를 둘 다 poll합니다.
  - 이유: 두 20 Hz 시계가 같은 경계로 drift하면 conflate 큐가 2:1로 alias되어 기록이 버려졌습니다. 이것이 `radarFault`의 원인이었습니다.
  - 다른 차량은 단일 poll 그대로입니다.

## 3. 주요 결정과 근거

모든 근거는 기록 입력을 쓴 lockstep faithful radarcan replay입니다. 기준은 결과를 보기 전에 고정했습니다.
상세 기준과 결과는 저장소 밖 연구 폴더에 있습니다(7절).

### 3.1 context 시각 규칙 (2026-10-08)

**문제:** 배치 시각보다 몇 ms라도 늦게 stamp된 modelV2를 받으면 다음 일이 생겼습니다.
- road-edge 모델이 무효화됨
- path와 vision cue가 비워짐
- livePose가 늦게 stamp되면 yaw가 None이 됨

IPC 전달 순서에 따라 road-edge 필터가 간헐적으로 꺼졌고, 길가 정지물이 깜빡이며 발행됐습니다.

**변경:** now보다 늦게 stamp된 메시지는 아직 도착하지 않은 것으로 봅니다. 직전에 받아들인 메시지를 유지하고, 그 stamp에 도달한 첫 호출에서 새 메시지를 씁니다. 200 ms 신선도 규칙은 그대로라서, 오래됐거나 없는 context는 이전처럼 무효화됩니다.

**근거:**
- 결정성(같은 입력 기준)
  - 0 vs −3 ms: 99.96 % (변경 전 83.0 %)
  - 0 vs −10 ms: 99.27 % (변경 전 82.0 %)
- 160구간 lead_one·lead_two·cut-in risk 차이 0
- 회랑 안에서 추가로 제거된 점 6장면을 영상으로 확인했습니다. 가드레일, 화단, 대향차였고 실차 손실은 0입니다.
- −10 ms 문턱은 사후 기준으로 바꿨습니다(사용자 승인).

### 3.2 대형차 multi-return 병합 (2026-10-08~09)

카메라가 대형차 하나로 보는 차량이 2–3개 점으로 갈라져 발행되는 문제를 위험이 작은 순서로 고쳤습니다.
- 대상: 2f1/2f2/2f4 주행, 갈라진 scan 579개
- 각 단계마다 추적 출력(scan digest) 동일, 카메라가 두 대로 보는 쌍의 새 병합 0, lead가 멀어지거나 사라진 경우 0을 확인했습니다.

| 단계 | 변경 | 갈라진 scan |
|---|---|---|
| 1 | camera-extended 성숙 연속성 경계 확대: 거리 `1.0+2.5dt²`, 횡 `+0.005·d_rel`. 이미 성숙한 group은 coast 중 모든 멤버가 측정되고 대표가 최근접 멤버일 때만 유지 | 598 → 478 |
| 2 | class-6 truck-P2 거리 간격 상한 9.0 → 12.0 m | → 357 |
| 3 | strict pair 전용 footprint 연관: 먼 멤버가 bearing만 실패(A0 UNRESOLVED)하고, 뒷면에서 12 m 안·횡 `min(1.75, w/2+0.5)` 안·`|dv| ≤ 0.5`이면 같은 차로 봄. 내보내는 A0 판정은 그대로 | → 232 |
| 4 | camera class-1 set은 대표가 최근접 멤버일 때 1구간 만에 성숙 | → 199 |
| 5 | truck-P2 확인 횟수 5 → 3 | → 182 |
| 6 | truck-P2 폭 하한 2.40 → 2.25 m (885구간: 3,494 → 3,174) | — |

- **기각한 변형:**
  - 간격 하한 5.0 m: 주차 차와 그 뒤 차를 합침
  - class-1 거리 16 m: 중형 트럭 앞 차를 합침
  - 카메라는 차 길이가 아니라 폭만 알려 줍니다.
- **남은 위험:**
  - 카메라가 못 보는, 같은 속도로 짧은 대형차 12 m 앞을 가는 차가 합쳐질 수 있습니다. 대표는 최근접 면이라 lead_one은 영향이 없고, lead_two는 가려질 수 있습니다.
  - car carrier는 PID 수명이 약 0.2 s라 깜빡임이 남습니다.

### 3.3 mature group retention (2026-10-03 이식, 10-04 기본 ON)

- **동작:** 2 s 이상 strict pair로 성숙했던 exact family는 최대 2 s 동안 retention edge로 묶을 수 있습니다.
- **조건:** 거리·횡·속도 지름이 커지지 않아야 하고, outsider가 없어야 하며, 멤버 누락·gap·shape 성장이 없어야 합니다. 조건이 깨지면 인증서를 취소합니다.
- **하지 않는 것:** PID history를 복사하지 않고, 문턱도 완화하지 않습니다.
- **world-robust surface guard:** 아래를 모두 만족해야 합니다. 하나라도 어기면 strict split으로 돌아갑니다.
  - 발행 면이 family 최근접 멤버에서 0.25 m 안
  - 숨긴 멤버보다 0.25 m/s 넘게 빠르지 않음
  - `|dPath|`가 `max(숨긴 멤버 |dPath|, 0.5 m)` 이하
  - model path 필수, 최대 0.5 s 유지
- **기본값:** `BoschRadarProvider(..., mature_group_retention=True)`. bare `BoschGroupingConfig`는 OFF이고, 명시적 opt-out이 가능합니다.
- **근거:**
  - 1,413구간에서 lead_one·lead_two·cut-in이 OFF보다 덜 보수적인 frame 0
  - 경로 회랑에서 숨겨진 더 가까운 면 613 → 0
  - 정답 가상 주입 시험: DIFFERENT 6 시나리오 악화 0, 알려진 −2 m/s² 제동에서 lead 없는 frame 560 → 128
  - 원래 G5(SCC 기울기 aLead) 기준은 사용자 승인으로 제동 시험으로 바꿨습니다.
- **한계:** 짧은 family(성숙 0.4 s 등)는 대상이 아닙니다. 묶인 family의 물리적 동일성은 직접 관측하지 못했습니다. guard 덕분에 동일성이 어느 쪽이든 split보다 나쁘지 않게 발행합니다.

### 3.4 성능과 메모리

- **CPU:** modelV2를 메시지당 한 번만 읽고, scan·발행 경로의 반복 계산을 줄였습니다. 같은 점 수 구간 기준 radarcan CPU −15.7 %이고, 발행은 비트 동일입니다.
- **메모리:** 발행마다 pycapnp `.schema.fields`가 순환 참조를 만들었습니다. gc가 꺼진 radarcan에서 185 MB/h씩 늘었습니다.
  - 수정: schema별 field tuple 캐시
  - 회귀 시험: `test_append_points_leaves_no_reference_cycle`
  - 실차 결과: 1.1 / 0.66 MB/h
- **solver closure·camera snapshot 소유권:** gc가 꺼진 상태에서 누수와 공유 문제가 있던 것도 고쳤습니다.

## 4. 하지 않기로 한 것

- **side-pass camera-still arm(`cea76210`), 2026-10-08 revert**
  - 목적: 추월당하는 옆 차의 반사점이 안쪽으로 미끄러져 생기던 거짓 cut-in 사전 감속(45/50)을 줄이는 것이었습니다.
  - 기하 진입 검사(실제 진입 468건)에서 실제 cut-in 2건의 risk가 0.15 s, 0.5 s 늦어졌습니다. 원인은 카메라 lateral이 한 scan 바깥으로 튄 것을 "카메라 정지"로 읽은 것입니다.
  - 다시 시도한다면: 어느 방향이든 lateral이 튀는 카메라로는 arm하지 않고, 기하 진입 검사를 고정 기준에 넣어야 합니다.
  - 거짓 predecel 수정은 provider 안에서만 하고, 당근 planner는 건드리지 않습니다.
- **PID 연속성 history 승계, NO-GO(2026-09-23)**
  - 저속 PID 분절의 원인은 raw return이 group에 들어갔다 나오는 ownership 변화였습니다.
  - 이전 PID를 되살리면 현재 group owner와 충돌할 수 있습니다.
  - sequential DIFFERENT GT가 없어서 false-stitch 안전성을 평가할 수 없었습니다.
- **`trackState` 값 변경:** Bosch는 `trackState=0`을 냅니다. 그래서 `trackState ≥ 2`를 요구하는 당근 관문(cut-out 완화 등)은 꺼집니다.
  - 반사실 replay(15.7 h): 켜도 0.55회/h, 각 ≤ 0.25 s, 완화 ≤ 5.3 m 수준이었습니다.
  - 같은 값이 다른 관문까지 함께 켜기 때문에 바꾸지 않았습니다.
- **추가 진단 로그:** PR 전에 브랜치 전용 진단 출력을 제거했습니다.
  - 제거한 것: research logger, mirror·road-edge·B1·burst·side-pass 이벤트 출력, `radard_input_health_invalid_transition`
  - 남긴 것: 추적·발행 동작과 replay 점검용 내부 상태(`last_events`, `last_decisions`)

## 5. 검증 범위

- **시험:** `opendbc/car/hyundai/tests/`의 6개 파일입니다. CI `tests` job의 "Bosch provider regression tests" 단계에서 돕니다.
  - `test_radar.py`, `test_bosch_camera_extended.py`, `test_bosch_mature_retention.py`, `test_bosch_mirror_birth.py`, `test_bosch_mirror_m3.py`, `test_bosch_road_edge.py`
- **실차 GO 평가(2026-10-12, 사후 기준):**
  - 노출량 단위: Bosch 지문(`radar_bosch.py`, `radar_interface.py`, `radarcan.py`, `radard_dpath.py`의 blob)
  - 노출량: longActive 2.13 h, route 5개, 3일(그중 지문 동일 주행 0.36 h). 과거 빌드 주행은 발행이 비트 동일한 구간만 이어받았습니다.
  - 기기 재현 99.997 % / 100 %
  - radarcan RSS 1.1 / 0.66 MB/h, radarFault 0
  - 영상 확인 기준 실차 손실 0, Bosch 고스트 제동 0
  - 실제 cut-in 8건 모두 차로 중심선 통과 1.08 s 전 ~ 0.07 s 뒤 사이에 lead로 채택
- **검증 범위 밖:**
  - 장면: 공사 구간 콘, 분기·합류, 터널
  - 주행감(jerk cost 변경과 겹침)
  - 아반떼 AD 외 후보 차종
- **알려진 공용부 현상:** `radard_dpath` RSS 33–43 MB/h(RadarState 빌더 재사용, 당근 영역)

## 6. 결정 기록

| 날짜 | 결정 |
|---|---|
| 2026-10-04 | retention guard와 기본 ON 승인. G5를 정답 제동 시험으로 교체 |
| 2026-10-08 | context 시각 결정성 수정 승인(−10 ms 사후 기준) |
| 2026-10-08 | 대형차 분열을 위험 순서로 수정(1–5단계), 각 단계 기준을 먼저 고정 |
| 2026-10-08 | side-pass camera arm revert, 재시도 조건 고정 |
| 2026-10-09 | truck-P2 폭 하한 2.25 m(6단계) |
| 2026-10-10 | 발행 schema 순환 누수 수정 |
| 2026-10-12 | R1 노출량을 Bosch 지문 단위로 계산(사후 기준) |
| 2026-10-12 | 감지 허용 목록을 MRRevo14F 후보 차종으로 확장(아반떼 AD 외 미검증) |

## 7. 상세 자료

- 이 문서는 아래 7개 문서를 합친 것입니다. 원문과 전체 개발 이력(203 커밋)은 fork `heatagain/openpilot`의 태그 `backup/bosch-radar-tracks-history-20261012`에 있습니다. 원문은 그 이력의 커밋 `7b721cba` 아래 `docs/`에 있습니다.
  - `bosch_context_timing_20261008.md`
  - `bosch_group_retention_20261003.md`
  - `bosch_truck_split_maturity_20261008.md`
  - `bosch_sidepass_camera_20261008.md`
  - `bosch_pid_continuity_20260923.md`
  - `bosch_pr_logging_cleanup_20260929.md`
  - `bosch_development_notes.md`
- 고정 기준, replay 결과, 영상 검토 자료는 저장소 밖 연구 폴더(`analysis/2026MMDD_*`)에 로컬로 보관합니다. 원격 백업은 아닙니다.
  - context 시각: `20261008_ctx_determinism`
  - 대형차: `20261008_truck_fix`, `20261008_lv_c3`
  - retention: `20261004_opus55_bosch_group_retention_enable`
  - 실차 GO 평가: `20261012_carry_forward`
