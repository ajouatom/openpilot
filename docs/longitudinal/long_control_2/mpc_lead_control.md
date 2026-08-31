# MPC 선행차 제어

## 기능 목표

- 작은 선행차 감속에는 작고 빠르게 반응한다.
- `vRel<0` 접근 중에는 감속 전 차간거리를 가능한 한 보존하며 closing을 없앤다.
- 상대속도가 안정되면 사용자가 설정한 짧은 TF로 천천히 돌아간다.
- lead motion을 MPC obstacle에 한 번만 표현하고 최종 명령은 MPC 해 하나로 만든다.

## 채택 구조

```text
radarState leadOne/leadTwo
  ├─ aLeadK/aLeadTau/jLead
  │    └─ lead motion 적분 ─> lead0/lead1 obstacle
  └─ dRel/vRel/track id
       └─ approaching 거리 앵커 ─> MPC tFollow

lead0/lead1/cruise/traffic-stop obstacle의 단계별 최솟값
이전 MPC 해 ─> params[:,3]
고정 danger factor + 런타임 comfortBrake
  └─ LongitudinalMpc ─> 단일 종방향 명령
```

외부 lead acceleration reference가 `params[:,3]`을 대체하던 경로는 제거했다.

## 접근 거리 앵커

설정 TF는 안정 추종 목표로 유지한다. 접근 시작은 `closing_speed=max(-vRel, 0)`가 `0.20 m/s` 이상이 되는 시점이다.

```text
d_floor = max(d_desired(base_tf), dRel_at_approach_start)
tf_mpc = clip(base_tf + max(d_floor - d_desired(base_tf), 0) / vEgo,
              base_tf, 2.0)
```

- 접근 중에는 `d_floor`가 줄지 않는다.
- closing speed가 `0.05 m/s` 이하가 되면 거리 floor를 초당 `2.0 m`씩 설정 목표로 되돌린다.
- radar track id가 바뀌거나 lead가 사라지면 이전 앵커를 재사용하지 않는다.
- 한 번도 approaching하지 않은 리드는 목표거리 변화만으로 앵커를 만들지 않는다.
- `vEgo<=0.5 m/s`에서는 TF 환산을 적용하지 않지만 내부 floor 상태는 정상적으로 해제한다.

## lead motion 입력

두 lead에 다음 공통값을 적용한다.

| 입력 | 적용 |
|---|---:|
| `aLeadK` | `×1.5` |
| `aLeadTau` | `×0.5`, 결과 범위 `0.25..3.0` |
| `jLead` | `×0.25`, `m/s³`로 시간 적분 |

가속도 궤적은 다음 형태다.

```text
a(t) = 1.5·aLead·exp(-(0.5·tau)·t²/2) + integral(jLead trajectory)
v(t) = vLead + integral(a(t))
x(t) = dRel + integral(v(t))
```

`jLead`를 가속도에 직접 더하지 않고 시간 적분해 단위를 보존한다.

## cruise에서 lead로의 전환

접근 앵커로 `tFollow`가 올라가면 cruise obstacle의 여유거리가 늘고, 예측 lead obstacle이 단계별 최솟값으로 더 일찍 선택될 수 있다. 별도 source 전환 조건이나 후단 감속 명령은 없다.

## comfortBrake와 비용

- 현재 런타임 기준은 `comfortBrake=2.4`이다.
- `2.3` 아래를 동적으로 사용하는 후보는 간격만 키우고 closing 제거가 일관되지 않아 폐기했다.
- `A_CHANGE_COST=200`, `X_EGO_OBSTACLE_COST=5`, `V_EGO_COST=0`은 변경하지 않았다.
- `leadDangerFactor=0.8`을 유지한다.

## 폐기·보류 후보

| 후보 | 판단 |
|---|---|
| 외부 lead `prev_a` FF | 제거: 강한 A-change 항으로 MPC를 사실상 우회 |
| `comfortBrake<2.3` | 폐기: gap 증가 대비 closing 개선 불충분 |
| obstacle 위치 preview | 폐기: 먼 거리 불필요 감속과 후반 closing 반복 |
| `V_EGO_COST=0.05` | 보류: 효과가 작고 큰 값은 jerk/overshoot 증가 |
| `aLead×1.75` 이상 | 폐기: e50/e51 최소 명령 과도 |
| closing speed × 0.5/1.0/1.5초 거리 추가 | 폐기: e51/e50 일부 개선, e4d 최소 vRel 악화 |
| 전역 cost 변경 | 보류: 입력 구조 실차 검증 뒤 별도 작업 |

## 코드 일치 확인

| 설명 | 현재 코드 |
|---|---|
| 거리 앵커와 lead motion | `lead_mpc_input.py` |
| obstacle 생성과 MPC 입력 | `LongitudinalMpc.process_lead()` / `update()` |
| acceleration reference | `self.params[:,3] = self.prev_a` |
| 접근용 TF | `ApproachDistanceController.update()` 결과 |
| danger factor | `LEAD_DANGER_FACTOR=0.8` |
| 비용 | 기존값 유지 |

## 작업내역

### 2026-08-31: 입력 후보 비교

- e51/e50에서 비용, `comfortBrake`, `aLead/jLead`, obstacle preview, velocity reference를 분리했다.
- `aLead×1.5/tau×0.5/jLead×0.25`가 과도한 1.75 후보보다 보수적이었다.
- 작은 설정 TF에서는 lead motion 강화만으로 approaching 초반 closing을 충분히 제거하지 못했다.

### 2026-08-31: 거리 앵커 구현

- 선행차 감속 전 실제 `dRel`을 MPC용 TF로 환산하는 상태를 추가했다.
- lead가 멀어지는 중에도 TF가 남는 초기 상태 오류를 회귀 테스트로 고정했다.
- 분석 구현과 제품 구현의 TF/obstacle 결과가 3,463프레임에서 완전히 같음을 확인했다.
