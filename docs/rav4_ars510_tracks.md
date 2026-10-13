# RAV4 2022 ARS 레이더 트랙 연결

Toyota RAV4 TSS2 2022의 RADAR_ACC 구성은 기존 Toyota 레이더 DBC가 없어
`radarUnavailable=True`로 처리된다. 이 변경은 해당 모델의 bus 1 객체 레코드를
ARS 수신기로 해석하고 기존 `liveTracks`와 Carrot `MyTrack` 경로에 전달한다.
다른 Toyota의 기존 파서와 현대 Bosch의 문맥·주기 처리는 유지한다.

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
```

미빌드 수신기 검사에는 `tools/ars510_validation/offline_io.py`를 명시적으로
호출할 수 있다. 이 보조 코드는 미빌드 Params 생성자만 차단하고 디코더·스키마·MyTrack을
대체하지 않는다. 호스트 IPC 검사는 빌드된 msgq/Params 런타임이 필요하다.

기존 종방향 ECU 진단 정책은 그대로이므로 모든 펌웨어에서 제어 중 객체 송신이
유지된다고 보장하지 않는다. 실제 가감속·PCS/AEB·회생제동과 차량 안전성 검증은 별도다.
