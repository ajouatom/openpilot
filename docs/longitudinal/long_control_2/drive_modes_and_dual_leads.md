# 주행모드와 복수 선행차

## 기능 목표

주행모드는 평상시 가속과 jerk 감각만 바꾸고, approaching 거리 보존과 위험 제약은 모든 모드에서 같게 한다.

| 값 | 화면 표시 | 내부 enum | 역할 |
|---:|---|---|---|
| 1 | `STBL` | `Smooth` | 가장 부드러운 평상시 가감속과 jerk |
| 2 | `BALN` | `Balanced` | 중간 응답 |
| 3 | `RUSH` | `Sync` | 빠른 평상시 응답 |
| 4 | `HIGH` | `High` | RUSH 계열 응답과 기존 고속 모드 기능 |

## 공통 안전 입력

다음 값은 모드에 따라 바꾸지 않는다.

- `aLead×1.5`, `aLeadTau×0.5`, `jLead×0.25`
- approaching 진입·해제 기준과 거리 앵커
- 접근용 MPC TF 상한 `2.0`
- `comfortBrake=2.4`
- `leadDangerFactor=0.8`
- predicted danger에 따른 기존 jerk/cost 안전 완화

lead motion은 관측된 세계의 예측이므로 모드에 따라 다르게 왜곡하지 않는다. STBL의 편안함은 약한 lead 안전 입력이 아니라 기존 nominal jerk와 정상 가속 한계에서 만든다.

## 현재 모드 차이

- `CarrotPlanner.get_carrot_accel()`은 모드별 정상 최대가속 factor를 사용한다.
- `jerk_factor`는 Smooth와 나머지 모드의 정상 jerk 응답을 구분한다.
- 위험도가 올라가면 `lead_safety_jerk_factor()`가 모든 모드에서 같은 안전 쪽으로 수렴시킨다.
- 차선변경 중 `DynamicTFollowLC`는 기존 사용자 설정으로 유지한다.
- 예전 `DynamicTFollow`의 `jLead→TF` 가감은 제거했다. approaching TF는 새 거리 앵커 한 곳에서만 계산한다.

`leadResponseMode` 로그 필드는 호환 진단값으로 남지만 더 이상 `prev_a` FF 제어권을 갖지 않는다.

## leadOne/leadTwo 동등 처리

1. 두 lead 모두 같은 motion 적분 함수를 사용한다.
2. 각 radar track id별로 같은 거리 앵커 상태를 갖는다.
3. 접근용 TF는 두 lead가 요구하는 보정 중 더 보수적인 값을 사용한다.
4. obstacle은 기존처럼 horizon 각 단계에서 lead0/lead1/cruise/traffic-stop 최솟값을 사용한다.
5. predicted danger margin과 FCW도 두 lead를 모두 평가한다.

배열 순서가 아니라 radar track id로 앵커를 연결하므로 leadOne/leadTwo 순서가 바뀌어도 같은 결과를 만든다. track id가 없는 vision lead에는 해당 슬롯을 사용하고, 사라지는 즉시 상태를 폐기한다.

## 테스트

- 두 radar track의 입력 순서를 바꿔도 `tFollow`와 거리 floor가 같은지 확인한다.
- track id 변경 시 이전 거리 floor를 재사용하지 않는지 확인한다.
- 한 번도 approaching하지 않은 lead가 stale floor를 만들지 않는지 확인한다.
- closing 해제 뒤 설정 TF로 서서히 복귀하는지 확인한다.
- 설정 TF가 작은 정상 추종에서는 값이 바뀌지 않는지 확인한다.

## 코드 일치 확인

| 설명 | 현재 코드 |
|---|---|
| 화면/내부 모드 | `STBL/BALN/RUSH/HIGH`, `Smooth/Balanced/Sync/High` |
| 모드별 정상 가속 | `CarrotPlanner.get_carrot_accel()` |
| 모드별 nominal jerk | `CarrotPlanner._get_base_t_follow()`의 `jerk_factor` |
| 공통 approaching 안전 | `ApproachDistanceController` |
| 두 lead 공통 motion | `extrapolate_lead_motion()` |
| 구형 lead 동적 TF | `dynamic_t_follow()`에서 제거됨 |

## 작업내역

### 2026-08-31: 책임 분리

- 안전 결과가 모드별 `aLead` gain에 따라 달라지지 않도록 lead motion profile을 공통화했다.
- 모드 차이를 정상 가속과 jerk에 남겼다.
- cut-in/radar/lane 관련 기존 변경은 수정하지 않았다.

### 2026-08-31: 두 lead 상태 구현

- radar track id 기반 거리 앵커를 구현했다.
- lead 순서 교환, track 변경, stale floor 회귀 테스트를 추가했다.
- 기존 `equal_lead_t_follow_adjustment()`를 런타임 경로에서 제거했다.
