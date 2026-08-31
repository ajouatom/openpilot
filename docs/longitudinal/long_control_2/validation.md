# 로그 재생과 검증

## 검증 목적

동일한 실제 ego/lead 입력에서 후보 MPC 명령을 비교하고, 단순 폐루프 근사로 불안정 후보를 미리 배제한다. 로그 재생은 실차 결과를 보장하지 않으며 통과 후보만 `carrot-egpu-tg` 시험 대상으로 올린다.

## 기준 로그

| 로그 | 주요 목적 | 분석 구간 |
|---|---|---:|
| e51 | STBL, TF 0.5, 지속 approaching | `37.5..59.5 s` |
| e50 | 강한 연속 감속과 closing | `8.9..32.9 s` |
| e42 s45 | 긴 안정 track, 근거리 추종 | `0..60 s` |
| e4d s5 | 먼 거리에서 근거리로 접근 | `0..31.7 s` |
| e4f s2 | track 획득·소실·전환과 강한 closing | `17.4..60 s` |

e51 메타데이터:

```text
vehicle: HYUNDAI_IONIQ_5_PE 07b62e389ed26c81
segment: 00000e51--2272031698--1
commit: 772e743e68
mode: STBL(1)
TFollow: 0.5
leadDangerFactor: 0.8
vCruise: 80 km/h
```

모든 rlog는 full cereal schema로 읽었다.

## 현행 구조 shadow 재현

- NumPy Gauss-Newton shadow가 e51 현행 `aTarget`을 RMSE `0.055 m/s²`, MAE `0.023 m/s²`, 상관계수 `0.993`으로 재현했다.
- 현행 외부 FF의 stage-0 reference 재구성 RMSE는 `0.0039 m/s²`였다.
- 아래 후보 비교는 같은 근사 solver와 실제 프레임 입력을 사용한다.

## 채택 제품 후보 same-input 결과

채택 후보는 실제거리 앵커, 공통 `aLead×1.5/tau×0.5/jLead×0.25`, `comfortBrake=2.4`, 기존 cost 조합이다.

| 로그 | approaching `aTarget-aLead` | 더 감속한 비율 | far 음의 명령 면적 | 최소 명령 | command jerk p01/p99 |
|---|---:|---:|---:|---:|---:|
| e51 | `+0.162` | `8.9%` | `-0.715` | `-3.01` | `-6.04 / +4.24` |
| e50 | `+0.200` | `34.6%` | `-0.790` | `-3.72` | `-8.65 / +4.97` |
| e42 s45 | `-0.240` | `77.8%` | `-0.206` | `-2.26` | `-2.65 / +1.15` |
| e4d s5 | `+0.209` | `0.0%` | `-0.058` | `-2.94` | `-5.23 / +6.73` |
| e4f s2 | `+0.451` | `31.8%` | `-3.593` | `-3.46` | `-4.97 / +8.61` |

same-input에서는 기록된 ego 경로가 그대로 유지되므로 후보가 먼저 감속해 바꿀 미래 `vRel/dRel`은 반영되지 않는다. 따라서 이 표는 즉시 명령과 과제동 배제에 사용하고, closing 제거 여부는 아래 폐루프 근사와 실차에서 판단한다.

## 폐루프 근사 선별

기록된 lead 속도를 외생 입력으로 두고, 기록된 `aEgo`에 맞춘 1차 actuator 근사로 ego 경로를 적분했다.

| 로그 | 후보 | closing 회복시간 | 접근 거리 손실 | 최소 vRel | 최소 aEgo |
|---|---|---:|---:|---:|---:|
| e51 | MPC-only | `15.66 s` | `18.16 m` | `-1.97` | `-2.05` |
| e51 | 채택 후보 | `9.04 s` | `10.43 m` | `-1.29` | `-1.41` |
| e50 | MPC-only | `13.86 s` | `21.41 m` | `-3.10` | `-3.11` |
| e50 | 채택 후보 | `4.85 s` | `8.50 m` | `-2.92` | `-1.74` |
| e4d s5 | MPC-only | `13.26 s` | `25.85 m` | `-3.35` | `-2.10` |
| e4d s5 | 채택 후보 | `7.40 s` | `25.47 m` | `-2.14` | `-2.13` |

e50 actuator 보정 오차는 크므로 수치 승인의 근거로 쓰지 않고 발산·충돌 후보 배제에만 사용한다. e51도 `dRel` RMSE가 `3.92 m`이므로 실차 검증이 필요하다.

## 응답시간 거리 추가 후보

거리 앵커에 `closing_speed×0.5/1.0/1.5초`를 더하는 후보를 비교했다.

- e51 회복시간은 `9.04→8.29초`로 소폭 줄었다.
- e50은 `4.85→4.31초`로 줄었다.
- e4d 최소 `vRel`은 `-2.14→-2.32 m/s`로 나빠졌다.
- e4f far 음의 명령 면적도 증가했다.

로그별 방향이 일관되지 않아 제품 코드에는 넣지 않았다.

## 제품 코드와 분석 코드 일치

`verify_product_candidate.py`로 다섯 로그 3,463프레임을 비교했다.

```text
maximum_t_follow_error = 0.0
maximum_obstacle_error = 0.0
```

따라서 문서의 채택 후보 수식과 현재 `lead_mpc_input.py` 구현이 일치한다.

## 단위·정적 검증

실행한 집중 테스트:

```text
pytest -q -c NUL --confcutdir=openpilot/selfdrive/carrot/tests \
  test_lead_response.py test_lead_mpc_input.py test_t_follow.py
```

결과는 `62 passed`이다. 새 입력 관련 확인 항목은 다음과 같다.

- 안정 추종 설정 TF 불변
- approaching 시작 실제거리 캡처
- closing 해제 뒤 거리 floor 복귀
- non-closing stale floor 방지
- track 변경 초기화
- lead 순서 교환 동등성
- 동일 track 중복 관측의 순서 독립성
- `aLead/aLeadTau/jLead` 물리 적분
- 기존 TF ramp 동작

Windows에는 빌드된 `params_pyx`와 acados 생성물이 없어 저장소 전체 pytest와 실제 acados replay는 이 환경에서 실행할 수 없다.

## 분석 산출물

- `.tmp_long_analysis/long_control_2/e51_approach_tf_anchor.png`
- `.tmp_long_analysis/long_control_2/e50_approach_tf_anchor.png`
- `.tmp_long_analysis/long_control_2/e42_s45_approach_tf_anchor.png`
- `.tmp_long_analysis/long_control_2/e4d_s5_approach_tf_anchor.png`
- `.tmp_long_analysis/long_control_2/e4f_s2_approach_tf_anchor.png`
- 같은 stem의 CSV/JSON summary

`.tmp_long_analysis`는 untracked 분석 자료이며 제품 커밋 대상이 아니다.

## 실차 필수 확인

1. 먼 거리에서 lead 감속 시작 시 작은 선행 반응
2. `vRel<0` 회복시간과 감속 전 거리 손실
3. lead brake release 중 재접근 여부
4. 정지·재출발에서 TF 상한 및 floor 해제
5. leadOne/leadTwo 전환과 radar track 변경
6. 불안정 vision lead 및 cut-in
7. STBL/BALN/RUSH/HIGH 공통 안전 결과와 정상 jerk 차이

## 작업내역

### 2026-08-31: 교차 로그 검증

- e42/e4d/e4f 원본 rlog를 추가해 e50/e51과 같은 입력 비교를 수행했다.
- 과도한 `prev_a` reference와 큰 velocity cost 후보를 배제했다.
- 실제거리 앵커가 세 안정 track 로그에서 closing 회복을 줄이는 방향임을 확인했다.

### 2026-08-31: 구현 일치 및 회귀 검증

- 분석과 제품 lead trajectory가 완전히 같은지 확인했다.
- non-closing stale floor 오류를 발견해 수정하고 테스트를 추가했다.
- 응답시간 거리 추가는 일관성이 없어 최종 구현에서 제외했다.
