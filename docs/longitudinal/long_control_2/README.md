# Long Control 2 작업 인덱스

## 목적

`carrot-egpu-tg`에서 짧은 설정 `TFollow`를 유지하면서도 선행차 감속과 접근속도에 빠르고 부드럽게 대응하는 MPC 중심 종방향 제어를 만든다.

이 디렉터리는 계획과 작업내역을 기능별로 나눈다. 코드가 바뀔 때마다 관련 문서의 코드 일치 항목과 검증 결과를 함께 갱신한다.

## 고정 원칙

- 제품 코드와 실차 시험 대상은 `carrot-egpu-tg`이다.
- 최종 종방향 명령은 `LongitudinalMpc` 결과 하나만 사용한다.
- MPC 후단이나 `carcontroller`에 lead 보정 명령을 추가하지 않는다.
- `leadOne`과 `leadTwo`는 같은 수식으로 처리한다.
- `leadDangerFactor=0.8`과 런타임 `comfortBrake=2.4`를 유지한다.
- 안전 입력은 주행모드와 무관하게 같게 하고, 모드 차이는 정상 가속과 jerk에 둔다.
- 설정 TF는 안정 추종 목표이고, approaching 중 MPC TF만 감속 전 실제거리를 보존하도록 환산한다.
- 코드 반영 전후에 실제 로그 비교와 코드-분석 일치 검사를 수행한다.

## 기능별 문서

- [MPC 선행차 제어](mpc_lead_control.md): 구조, 거리 앵커, lead motion 입력과 폐기 후보
- [주행모드와 복수 선행차](drive_modes_and_dual_leads.md): 모드 책임 분리와 두 lead 동등 처리
- [로그 재생과 검증](validation.md): 기준 로그, 수치, 그래프, 테스트와 한계

## 현재 구현 상태

| 항목 | 상태 | 근거 |
|---|---|---|
| 대상 브랜치 | 확정 | `carrot-egpu-tg` |
| 작업 시작 HEAD | 확인 | `9cb66c5464` |
| 새 MPC 입력 모듈 | 구현 | `lead_mpc_input.py` |
| 외부 lead `prev_a` FF | 제거 | `params[:,3] = self.prev_a` |
| approaching 거리 앵커 | 구현 | 진입 `0.20 m/s`, 해제 `0.05 m/s`, 거리 복귀 `2.0 m/s` |
| lead motion 예측 | 구현 | `aLead×1.5`, `aLeadTau×0.5`, 물리 적분 `jLead×0.25` |
| 최대 접근용 MPC TF | 구현 | `2.0 s`; 설정 TF 자체는 변경하지 않음 |
| lead 기반 구형 `dynamic_t_follow` | 제거 | 차선변경 TF 기능만 유지 |
| 비용 변경 | 없음 | obstacle/A-change/velocity cost 기존값 유지 |
| 단위 테스트 | 통과 | lead-response/새 입력/기존 TF 집중 테스트 `62 passed` |
| 제품-분석 일치 | 통과 | 5개 로그 3,463프레임, TF/obstacle 최대 오차 `0` |
| 실차 시험 | 대기 | 커밋·푸시 및 차량 적용 전 |

## 코드 일치 확인

- 확인일: 2026-08-31
- 기준 브랜치: `carrot-egpu-tg`
- `LEAD_DANGER_FACTOR`는 `0.8`이다.
- lead 감속은 obstacle 예측과 접근용 `tFollow`로만 MPC에 들어간다.
- lead 가속도 reference가 `params[:,3]`을 바꾸는 경로는 없다.
- `comfortBrake<2.3`, velocity cost, obstacle cost, 응답시간 거리 추가는 채택하지 않았다.
- 화면 표시는 `STBL/BALN/RUSH/HIGH`, 내부 enum은 `Smooth/Balanced/Sync/High`이다.

## 작업내역

### 2026-08-31: 구조 조사와 현행 재현

- e51 현행 MPC를 shadow solver로 재현했다. `aTarget` RMSE는 `0.055 m/s²`, 상관계수는 `0.993`이었다.
- 기존 `lead_response.py` FF가 `A_CHANGE_COST=200`인 `prev_a` 항을 통해 MPC를 강하게 끌고 있음을 확인했다.
- 비용, `comfortBrake`, obstacle preview, velocity reference, `aLead/jLead` 후보를 분리 비교했다.

### 2026-08-31: 접근 TF 정의 정정

- 짧은 설정 TF에서는 approaching 초반 obstacle 압력이 늦어짐을 확인했다.
- 기존 짧은 목표거리가 아니라 선행차 감속 직전의 실제 `dRel`을 거리 앵커로 정했다.
- 한 번도 approaching하지 않은 리드가 오래된 목표거리를 유지하는 상태 오류를 찾아 제거했다.

### 2026-08-31: 제품 코드 구현

- `lead_mpc_input.py`에 거리 앵커와 물리적인 lead motion 적분을 분리 구현했다.
- `long_mpc.py`에서 외부 acceleration FF와 관련 상태·가중치 경로를 제거했다.
- lead `jLead`로 TF를 직접 더하고 빼던 구형 동적 TF를 제거하고 차선변경 기능만 남겼다.
- 0.5/1.0/1.5초 closing 거리 추가는 로그별 효과가 일관되지 않아 폐기했다.

## 남은 작업

1. 변경 후 가능한 기존 회귀 테스트와 정적 검사를 완료한다.
2. diff와 문서-코드 일치를 다시 확인한다.
3. `carrot-egpu-tg`에 커밋·푸시한다.
4. 실차 로그에서 approaching 회복, brake release, cut-in, track 변경을 확인한다.
5. 입력만으로 부족한 구간이 반복되면 그때 최소 MPC cost 변경을 별도 후보로 검토한다.
