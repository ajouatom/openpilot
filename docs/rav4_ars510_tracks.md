# RAV4 2022 ARS 레이더 트랙 연결

Toyota RAV4 TSS2 2022의 RADAR_ACC 구성은 기존 Toyota 레이더 DBC가 없어
`radarUnavailable=True`로 처리된다. 이 변경은 해당 모델의 bus 1 객체 레코드를
ARS 수신기로 해석하고 기존 `liveTracks`와 Carrot `MyTrack` 경로에 전달한다.
다른 Toyota의 기존 파서와 현대 Bosch의 문맥·주기 처리는 유지한다.

## 옆 차선 표시와 제어의 분리

원형 fused의 경로 필터는 신선한 요레이트가 있을 때 15m 밖·예측 경로에서 2.5m
벗어난 비ACC 객체를 제외한다. 근거리 정지 객체에는 더 좁은 범위를 사용한다.
이 제어용 목록만 표시에도 사용하면 옆 차선 차량을 감지해도 표시가 빠질 수 있다.

`RadarData.displayPoints`는 별도 표시용 디코더의 결과다. 동일한 프레임 검증을 거쳐
원형 fused의 `path_gate_m`만 표시에서 해제하고, 최소 나이·CRC·속도 품질·60m 밖의
비ACC 객체 제외는 유지한다. `points`, MyTrack, DPath, 선행차·끼어들기·계획 입력에는
합치지 않으며 표시 객체 번호와 복구 이력도 제어와 독립적이다. 표시 오류가 제어
수신기의 상태나 객체 번호를 바꾸지 않도록 분리한다.

Konik 기본 주행 화면은 `liveTracks`를 구독해 `ShowRadarInfo=3`에서 이 목록으로
기존 레이더 정보 그룹을 대체한다. `hasDisplayData`는 빈 정상 목록과 오류에도 지원을
알리므로 이전 제어 목록으로 돌아가 오래된 표시를 남기지 않는다. 수신·게시 시각,
유효성·오류·측정 여부·유한 값을 검사하고, 1/2 또는 미지원 수신기는 기존 표시를 유지한다.
좌우 반전은 게시 사본의 두 목록에 한 번씩 적용하며 UI는 다시 반전하지 않는다.
화면의 기존 `radarState`·`modelV2` 유효성 및 차선 높이·투영 조건은 유지한다.

메시지 필드는 기존 번호를 바꾸지 않고 뒤에 추가했다. 구버전 메시지의 표시 지원
표지는 기본 false다. 실제 적용에는 스키마·수신기·UI의 동시 업데이트와 정상 빌드,
새 프로세스 시작이 필요하다. 웹과 mici 표시 및 실제 장치 화면 검증은 이 변경의 범위 밖이다.

## 커브·지도 감속과 PCM 모드 0

도로 곡률과 지도 경로의 속도 제한은 이미 브랜드 공통이다. `carrot_serv`가
각 제한의 최소값을 선택하고 `CarrotPlanner`가 기본 설정속도보다 낮은 목표를
적용한다. Toyota 허용 조건을 이 경로에 추가할 필요는 없다.

별도 입력 문제로, RAV4의 순정 SET/RES는 버튼 이벤트 대신 PCM 설정속도로
전달되는데 모드 0은 이를 동기화하지 않아 이전 Carrot 속도에 남을 수 있었다.
`ToyotaPcmSetSpeedSync`는 정확한 `TOYOTA_RAV4_TSS2_2022`·Toyota·PCM·직접 가감속·
모드 0·유효 CAN 조건에서 관측한 속도 변경과 크루즈 재개만 기본 목표에 반영한다.
첫 활성 표본은 기준만 저장하고 같은 주기의 로컬 버튼·블루투스 입력이 우선한다.
다른 차종과 모드 1/2/3, 더 낮은 커브·지도·앞차 목표는 기존 경로를 유지한다.
이 변경은 SET/RES 메시지를 송신하거나 종방향 제어를 활성화하지 않는다.

## 전방 끼어들기와 측면 코너레이더의 구분

ARS 점은 `frontRadar`로 전달된다. 기존 `DPathRadarController`는 코너 객체가
없을 때 전방 트랙의 시간에 따른 횡방향 이동을 사용해 끼어들기를 추정한다.
`leadCutInRisk`의 선감속, 확정된 끼어들기 표시, 두 번째 선행차 선택은 각각
다른 조건이다. 코너 설정 0/1/2는 이 Toyota 전방 경로를 차단하지 않는다.

`corner_radar_enabled()`의 Hyundai 조건은 측면 센서 우선 선택 조건이다.
RAV4에 이 조건만 추가하면 실제 측면 객체 없이 코너 모드에 들어가 전방
끼어들기 후보에 추가 제한이 걸린다. 따라서 해당 조건은 변경하지 않았다.

현재 Toyota `0x3F6`의 사각지대 신호는 좌우 경고 비트이며 거리·상대속도·객체
번호를 제공하지 않는다. ARS의 `0x235/0x237`도 Hyundai의 코너 객체 프로토콜과
다르다. **Toyota 측면 코너레이더 지원은 이 PR에 포함되지 않는다.** 이를 추가하려면
ECU 식별 자료와 검증된 거리 객체 신호 정의, 자연스러운 주행에서 이미 수집한
CAN·carState·modelV2·liveTracks·radarState 동시 로그가 필요하다.

## 수신 계약

- 지원 조건은 Toyota, `TOYOTA_RAV4_TSS2_2022`, RADAR_ACC 플래그의 조합이다.
- `0x80` 객체 목록을 조립한 뒤 CRC, 순서, 길이, 시각, 유한 값과 객체 번호 범위를 검사한다.
- 디코더는 [ars510-radar의 fb5b29c2](https://github.com/ds-sebastian/ars510-radar/tree/fb5b29c2b5999f34afeb6f7ac494a73949d66c67)의
  원문 9개와 MIT 라이선스다. 원형 `fused`의 필터 계수와 객체 나이 기준을 유지한다.
- 각 점의 거리·상대속도·지면 속도를 실제 `RadarData`와 `MyTrack`에 전달한다.
  정상 레코드마다 전체 점 집합을 교체하므로 사라진 객체는 남기지 않는다.
- 누락·CRC·순서·시각 오류는 이전 점과 필터 이력을 폐기한다. 호스트 입력 오류로
  수신기를 재생성한 뒤에도 새 정상 레코드 전까지 오류를 유지하고 객체 번호를 재사용하지 않는다.
- 지원 모델의 레이더 처리 능력과 주기만 선언한다. 종방향 제어·안전 설정의 원래 결정과
  ECU 초기화 코드를 유지하며 이 수신기는 CAN을 송신하지 않는다.

## 검증 범위

한국 RAV4 Hybrid 2022의 제공 CAN을 오프라인으로 재생해 실제 객체 갱신이
MyTrack에 전달되는 것을 확인했다. 사용자도 차량에서 정상 작동했다고 보고했다.
사용자 보고와 자동 검증은 별도 근거이며 ECU 부품 번호·펌웨어 목록을 확정한 결과가 아니다.

객체 나이가 충분하지 않으면 원형 fused는 점을 보류한다. 레코드가 정상이어도
트랙이 없는 구간을 임의 객체나 느슨한 필터로 채우지 않는다. 표시된 선행차가
비전인지 레이더인지와 실제 종방향 제어 상태는 별도로 확인해야 한다.

새 회귀 검사는 CRC·순서·시각·누락·객체 재시작·Int32 번호 경계, MyTrack 전달,
입력 장애 후 복구 및 일반 Toyota의 기존 생성자 보존을 다룬다. 빌드된 Linux에서:

```bash
pytest opendbc_repo/opendbc/car/toyota/tests/test_ars510_carrot.py
pytest openpilot/selfdrive/carrot/tests/test_radarcan_process.py
pytest openpilot/selfdrive/car/tests/test_toyota_pcm_mode0.py openpilot/selfdrive/car/tests/test_toyota_pcm_speed.py
pytest openpilot/selfdrive/carrot/tests/test_rav4_radar_slowdown.py
pytest opendbc_repo/opendbc/car/toyota/tests/test_ars510_display.py
pytest openpilot/selfdrive/ui/tests/test_radar_display.py openpilot/selfdrive/carrot/tests/test_radar_display_flip.py
```

미빌드 수신기 검사에는 `tools/ars510_validation/offline_io.py`를 명시적으로
호출할 수 있다. 이 보조 코드는 미빌드 Params 생성자만 차단하고 디코더·스키마·MyTrack을
대체하지 않는다. 호스트 IPC 검사는 빌드된 msgq/Params 런타임이 필요하다.

감속 회귀 검사는 합성 CAN → 실제 ARS 수신기 → DPath 끼어들기 추정과
감속 상한 전달을 다룬다. 별도로 실제 커브 속도 계산, 지도 제한 최소값,
만료 입력 거절, 가속 페달·제어 비활성 시 선감속 억제를 검사한다.
일부 호스트 메서드는 소스에서 추출해 실행하고 MPC 계산 직전에 중단하므로,
이 검사는 네이티브 MPC·실제 CAN 송신·차량 제동이나 주행 안전성 시험이 아니다.
합성 객체는 처음부터 원형 fused 나이 조건을 충족하도록 만든 입력이며,
새로 관측한 객체의 초기 보류나 실제 측면 센서 검출 성능을 해결했다는 뜻이 아니다.

기존 종방향 ECU 진단 정책은 그대로이므로 모든 펌웨어에서 제어 중 객체 송신이
유지된다고 보장하지 않는다. 실제 가감속·PCS/AEB·회생제동과 차량 안전성 검증은 별도다.
