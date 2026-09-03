# 7713 / 7714 내비 감속 적용 비교

## 확인 기준

- 저장소: `ajouatom/openpilot`
- 브랜치: `origin/thftgr/navi-stream`
- 커밋: `4488bd591b26dad1b9d44a6f3890336a0ba25ec8`
- 확인일: 2026-07-16
- 범위: 수신, 파싱, `CarrotServ` 상태 반영, 감속 목표 선택, 종방향 계획 반영, UI 표시

## 핵심 결론

7713과 7714는 수신 및 상태 관리 방식은 다르지만, 최종적으로는 같은 `CarrotServ._update_sdi()`와
`CarrotServ.update_navi()`를 통해 `carrotMan.desiredSpeed`와 `desiredSource`를 만든다. 따라서 같은
`xSpdType/xSpdLimit/xSpdDist`, TBT, 경로 상태에 도달한 뒤의 감속 공식과 UI source 표시는 같다.

그러나 두 포트가 동시에 활성화될 때의 명시적 우선순위나 소스별 상태 분리는 없다. 양쪽이 같은
`CarrotServ` 필드를 직접 갱신하므로 마지막으로 실제 필드를 쓴 입력이 이긴다. 7714의 동일 sequence
heartbeat는 SDI 필드를 다시 쓰지 않기 때문에, 그 사이 들어온 7713 값이 다음 7714 speed sequence까지
남을 수 있다.

실제 운용 전제는 두 포트를 동시에 사용하지 않는 것이다. 각 포트를 단독 사용하면 7713의 핵심 감속
항목에 대응하는 7714 코드 경로는 존재한다. 그러나 **7714 primary type 22 방지턱은 수신만으로 감속이
보장되지 않는다.** 두 비교 브랜치 모두 `lane_current.road_category` 처리 순서와 기본값 때문에
`xSpdType=22` 후보가 생성되지 않을 수 있다. 특히 lane item에 `road_category`가 빠지면 cereal 변환값이
0이 되고, `roadcate > 1` gate에서 방지턱이 계속 탈락한다. 이는 7713은 감속하지만 7714는 감속하지 않는
현장 증상을 코드상 재현한다.

## 7713 → 7714 핵심 감속 parity

| 7713 | 7714 | parity 판정 |
|---|---|---|
| `nRoadLimitSpeed` | `speed.road_limit_kph` | 정상 1..200 km/h는 적용. 7713의 200 초과 legacy encoding decode와 0→30 fallback은 7714에 없음 |
| `nSdiType/SpeedLimit/Dist` | `speed.sdi.type/speed_limit_kph/distance_m` | 동일 적용 |
| `nSdiBlockType/Dist` | `speed.sdi.block_type/block_distance_m` | 동일 적용 |
| `nSdiBlockSpeed` | `speed.sdi.block_speed_kph` | 양쪽 모두 수신/저장만 하고 감속 제한속도로 사용하지 않음 |
| `nSdiPlusType/Dist`의 type 22 | `speed.sdi_secondary.type/distance_m`의 type 22 | primary 감속 분기가 없을 때 동일 적용 |
| `nSdiPlusSpeedLimit`, plus block fields | secondary speed/block fields | 양쪽 모두 감속 미적용 |
| `roadcate` | `lane_current.road_category` | 최종 gate는 같지만 갱신 순서가 다름. 7713은 같은 packet 값을 먼저 적용하고, 7714는 SDI 판정 후 적용 |
| 현재/다음 TBT type, distance | `guidance_current/next` type, distance | 동일 적용 |
| `nTBTNextRoadWidth` | 직접 대응 control mapping 없음 | 7714에서 누락. 다만 현재 코드는 감속 목표속도 공식에는 쓰지 않고 ATC type 전환 거리/표시에만 사용 |
| route/vrtx polyline | `route.polyline` | 동일 곡률 감속 적용. 7713 최대 4096점, 7714 최대 256점 |

일반 SDI의 지원 type과 최종 감속 공식은 같지만, 다음 차이는 7713/7714의 실제 동작을 갈라놓는다.

- 7713의 `nRoadLimitSpeed > 200` legacy encoding은 decode하지만 7714는 200 초과 road limit을 invalid로
  처리한다.
- 7713은 road limit 0 이하를 30으로 바꾸지만 7714는 invalid/no limit으로 처리한다.
- 7714는 `off_route=true`이면 SDI/TBT를 의도적으로 억제한다. 7713에는 이 안전 gate가 없다.
- 7714 route는 256점으로 잘리므로 점 간격이 매우 조밀하고 필요한 전방 구간이 256점 밖이면 7713보다
  route 곡률 정보가 짧아질 수 있다.

## 데이터 흐름

### 7713 legacy HTTP

1. `POST /api/navi/{tmap_version}`가 JSON을 수신한다.
2. `rgdata`만 `CarrotServ.update()`로 전달한다. 중첩된 `guidance`, `sdi`, `lane` 그룹은 flat key로
   보완된다.
3. 실제 내비 감속 필드 갱신은 payload에 `nRoadLimitSpeed` key가 있을 때만 수행된다.
4. route/vrtx는 별도 `handle_route()`로 경로를 저장한다.

### 7714 Carrot Navi v2

1. 상시 `carrot_navi` 프로세스가 WebSocket v2 스트림을 수신한다.
2. item별 presence, sequence와 session을 검사하고 `carrotNavi` cereal service로 2 Hz heartbeat를
   발행한다.
3. `carrot_man`이 `CarrotNaviControl`로 파싱한 뒤 기존 `CarrotServ` 필드로 변환한다.
4. speed item은 새 session, off-route 변경 또는 speed sequence 변경 시에만 다시 적용한다. 동일
   sequence heartbeat에서는 로컬에서 차감 중인 거리를 보존한다.

## SDI wire JSON 구조

### 7713 HTTP `rgdata`

`POST http://<device>:7713/api/navi/<tmap_version>` body는 다음 형태다. 실제 SDI 갱신 gate이므로
`rgdata.nRoadLimitSpeed` key를 반드시 포함해야 한다.

```json
{
  "timestamp_ms": 1710000000000,
  "rgdata": {
    "nRoadLimitSpeed": 50,
    "nSdiType": 1,
    "nSdiSpeedLimit": 50,
    "nSdiSection": -1,
    "nSdiDist": 420,
    "nSdiBlockType": -1,
    "nSdiBlockSpeed": 0,
    "nSdiBlockDist": 0,
    "nSdiPlusType": 22,
    "nSdiPlusSpeedLimit": 0,
    "nSdiPlusDist": 93,
    "nSdiPlusBlockType": -1,
    "nSdiPlusBlockSpeed": 0,
    "nSdiPlusBlockDist": 0,
    "roadcate": 8
  }
}
```

`guidance`, `sdi`, `lane` nested object도 flatten하지만, 그 안의 key도 위와 같은 legacy 이름이어야 한다.
충돌 시 `rgdata` 최상위 값이 우선하므로 전송은 평면 구조가 가장 명확하다.

### 7714 WebSocket v2 `speed` item

네트워크에서 앱이 보내는 `value` 내부 key는 snake_case다. cereal로 변환된 뒤의 camelCase
(`sdiPresent`, `sdiType` 등)를 앱에서 직접 보내면 안 된다. WebSocket path는
`/api/navi/ws/v2/json/<session_id>/speed`이고, `session_id`, `manifest_revision`, `stream_handle`은 control
협상에서 받은 manifest 값을 사용한다.

```json
{
  "type": "item_update",
  "protocol_version": 2,
  "session_id": "0123456789abcdef",
  "manifest_revision": 1,
  "schema_version": 1,
  "kind": "json",
  "name": "speed",
  "stream_handle": 6,
  "sequence": 101,
  "source_timestamp_ms": 1710000000000,
  "sent_at_ms": 1710000000010,
  "present": true,
  "value": {
    "current_kph": 48.0,
    "road_limit_kph": 50,
    "sdi": {
      "type": 1,
      "distance_m": 420,
      "speed_limit_kph": 50,
      "section_type": -1,
      "block_type": -1,
      "block_speed_kph": 0,
      "block_distance_m": 0
    },
    "sdi_secondary": {
      "type": 22,
      "distance_m": 93,
      "speed_limit_kph": 0,
      "section_type": -1,
      "block_type": -1,
      "block_speed_kph": 0,
      "block_distance_m": 0
    },
    "section": {
      "active": false,
      "speed_limit_kph": 0,
      "average_kph": 0.0,
      "overall_average_kph": 0.0,
      "remaining_distance_m": 0.0,
      "remaining_time_sec": 0,
      "progress": 0.0,
      "suspended": false,
      "off_route": false
    }
  }
}
```

방지턱 허용 판단의 `road_category`는 별도
`/api/navi/ws/v2/json/<session_id>/lane_current` item의 `value.road_category`로 보내야 한다. 같은 envelope
형태에서 `name`, `stream_handle`, `sequence`, `value`를 해당 stream에 맞게 바꾼다.

`road_category`는 현재 주행 링크의 도로 등급 코드다. 이 저장소에는 0~9 전체 enum 표가 없으며 제어
코드가 확정적으로 해석하는 의미는 `0/1 = 고속도로 계열`, `2 이상 = 그 밖의 도로`뿐이다. 따라서 제어
관점에서는 다음처럼 취급해야 한다.

| 값 | 현재 코드의 의미 | type 22 방지턱 |
|---:|---|---|
| 0 | 고속도로 | 차단 |
| 1 | 도시고속도로/자동차전용도로 계열 | 차단 |
| 2 이상 | 일반도로 계열로 취급 | 허용 |
| 누락 | lane item이 present이면 cereal에서 0으로 변환 | 차단 |

TMAP 데이터 정의의 `roadcate` 표는 0 고속국도, 1 도시고속화도로, 2 국도, 3 국가지원지방도,
4 지방도, 5 주요도로1, 6 주요도로2, 7 주요도로3, 8 기타도로1, 9 이면도로, 10 페리항로,
11 단지내도로, 12 이면도로2(세도로)다. 필드명과 0/1 의미가 코드 주석과 정확히 일치하므로 이 값의
원천으로 볼 수 있다. 다만 7714 protocol 문서 자체에는 enum이 선언되어 있지 않으며 openpilot 제어는
세부 등급을 구분하지 않고 `0/1` 대 `2 이상`만 사용한다. 초기값 8은 실제 지도 판정이 아니라 수신 전
fallback이고, 결과적으로 방지턱을 허용한다.

```json
{
  "type": "item_update",
  "protocol_version": 2,
  "session_id": "0123456789abcdef",
  "manifest_revision": 1,
  "schema_version": 1,
  "kind": "json",
  "name": "lane_current",
  "stream_handle": 4,
  "sequence": 33,
  "source_timestamp_ms": 1710000000000,
  "sent_at_ms": 1710000000010,
  "present": true,
  "value": {
    "road_category": 8
  }
}
```

SDI를 지울 때는 더 큰 sequence로 `present: false`, `value: null`, 비어 있지 않은 `reason`을 보낸다.

```json
{
  "type": "item_update",
  "protocol_version": 2,
  "session_id": "0123456789abcdef",
  "manifest_revision": 1,
  "schema_version": 1,
  "kind": "json",
  "name": "speed",
  "stream_handle": 6,
  "sequence": 102,
  "source_timestamp_ms": 1710000001000,
  "sent_at_ms": 1710000001010,
  "present": false,
  "value": null,
  "reason": "source_absent"
}
```

## 감속 기능별 비교

| 기능 | 7713 입력 | 7714 입력 | 실제 적용 조건과 결과 | 적용 source/UI label |
|---|---|---|---|---|
| 고정/일반 카메라 | `nSdiType` 0,1,2,3,4,8,75,76 + speed/dist | primary `sdi`의 같은 type + speed/dist | `AutoNaviSpeedCtrlMode > 0`, speed > 0. 안전계수와 감속률 적용 | `cam` / 주황 `cam` |
| 이동식 카메라 | type 7 | type 7 | mode 3에서만 적용. mode 1/2에서는 `_update_sdi()`가 limit/dist를 0으로 지움 | `cam` / 주황 `cam` |
| 구간단속(block) | `nSdiBlockType` 2/3, block distance | primary SDI block type 2/3, block distance | type을 4로 바꾸고 block distance 사용. 단, block speed는 사용하지 않고 primary SDI speed에 안전계수를 적용 | `section` / 주황 `section` |
| 7714 전용 section object | 없음 | `section.active`, speed limit, remaining distance | present + active + not suspended + section off-route 아님 + 전체 off-route 아님 + limit > 0일 때 type 4로 변환 | `section` / 주황 `section` |
| 방지턱 | primary/plus type 22 | primary/secondary type 22 | `roadcate > 1`, mode >= 2. payload speed는 무시하고 `AutoNaviSpeedBumpSpeed` 사용. 실제 감속 중 새 가속 입력이 들어오면 이벤트 종료까지 가속 최고속도를 하한으로 유지. 단, 7714는 road category 갱신 순서/기본값 문제로 type 22가 수신되어도 후보 생성에 실패할 수 있음 | `bump` / 주황 `bump`, 오버라이드 시 `gas` |
| 차량 수신 과속카메라 | `carState.speedLimit/speedLimitDistance` | 동일 | 차량 CAN에서 단속속도만 수신하며 Hyundai `CarState`가 `speedLimit × (VehicleSpeedCameraDistanceTime / 10)`으로 가상거리 생성. `VehicleSpeedCameraControlMode`에 따라 미사용·항상 적용·감속 중 가속 이벤트 무시·가속페달 입력 중 해제를 선택 | `hda` / 라벤더 `cam`, 오버라이드 시 `gas` |
| 차량 내비 CAN 정확거리/구간 | `carState.speedLimitDistance/speedBumpDistance/vehicleNaviSectionActive` | 동일 | `VehicleNaviCanControl`이 켜진 Hyundai CAN-FD에서 0x4BE 또는 PV5 CAN-FD wrapper의 alert spot Offset을 휠 주행거리로 추적. 카메라는 기존 `hda`, Value 6 방지턱은 별도 후보로 계산하고 실제 감속 중 새 가속 입력에는 이벤트 오버라이드를 적용. PV5의 구간단속은 주기 상태 신호가 검증될 때까지 비활성화 | `hda`, `hda_section`, `hda_bump` / 라벤더 `cam`, `section`, `bump`, 오버라이드 시 `gas` |
| 차량 내비 CAN 30 km/h 구간 | `carState.schoolZoneActive` | 동일 | `VehicleNaviSchoolZoneControl`이 켜지고 0x4BE 종류 7이 30 km/h를 알리면 30 km/h 후보 적용. 차량의 30 카메라 상태 종료, 비-30 종류 7, 경로 재계산 또는 1 km 주행 시 해제. 가속페달 동작은 `VehicleSpeedCameraControlMode`를 따름 | `school` / 라벤더 `school`, mode 2 하한 적용 시 `gas` |

차량 순정 내비의 0x4BA 곡률 프로파일은 경로·차선 연계 신뢰도가 충분하지 않아 파싱, 속도제어,
설정 및 화면 표시에서 사용하지 않는다. 0x4BE 기반 카메라·구간단속·방지턱·school 기능과는 별개다.
| 도로 제한속도 | `nRoadLimitSpeed` | `road_limit_kph` | `AutoRoadSpeedLimitOffset >= 0`, active >= 2, road limit valid일 때 limit+offset | `road` / 주황 `road` |
| 현재 TBT | `nTBTTurnType/nTBTDist` | `guidance_current.turn_type/distance_m` | 지원 turn type이 `xTurnInfo`로 변환되고 `AutoTurnControl`이 2 또는 3일 때 속도 목표 계산 | `atc` / 주황 `turn` |
| 다음 TBT | `nTBTTurnTypeNext/nTBTDistNext` | `guidance_next` | 현재 거리 + 다음 거리를 사용하고 같은 ATC 설정 적용 | `atc2` / 주황 `turn` |
| route 곡률 | `route`/`vrtx`, 최대 4096점 | route polyline, 최대 256점 | 동일 경로 곡률 계산. `TurnSpeedControlMode=2`는 TBT ±500 m, mode 3/4는 항상. 기본값 mode 1에서는 route 감속 미적용 | `route` / 주황 `route` |
| 신호등 | `sinf/ssinf` | `traffic_signal` | `TrafficLight` shared-memory param과 cluster/UI 표시에만 전달. `desiredSpeed`나 longitudinal stop target에는 직접 연결되지 않음 | 감속 source 없음 |

공통 감속 속도는 목표 지점의 안전속도와 안전시간을 기준으로
`sqrt(v_target^2 + 2 * decel_rate * decel_distance)` 형태로 계산한다. 모든 후보 중 최솟값을
`desiredSpeed`로 선택하고, planner가 cruise 속도와 다시 `min()`하여 실제 종방향 목표에 반영한다.

### 차량 수신 과속카메라 정책

Hyundai 일반 CAN의 `Navi_HU.SpeedLim_Nav_Cam == 1` 또는 CAN-FD의 `HDA_INFO_4A3.MapSource == 2`이면
`carState.speedLimit`에 차량이 수신한 과속카메라 단속속도를 넣는다. 별도 거리 신호가 없으므로
`VehicleSpeedCameraDistanceTime` 기본값 60(6.0초)을 사용하여 다음 가상거리를 만든다. 저장값은
0.1초 단위이므로 62는 6.2초다.

`speedLimitDistance(m) = speedLimit(km/h) × VehicleSpeedCameraDistanceTime / 10`

따라서 기본값에서 50 km/h는 300 m, 80 km/h는 480 m이며, 62로 설정하면 50 km/h에서 310 m다.
Hyundai `CarState`가 설정을 약 1초마다 다시 읽으며, 값이 바뀌면 현재 활성 과속카메라의 가상거리도
새 값으로 즉시 다시 계산한다.

`VehicleSpeedCameraControlMode` 기본값은 1이다.

| 값 | 동작 |
|---:|---|
| 0 | 차량 수신 과속카메라 후보를 만들지 않는다. 원시 `carState.speedLimit` 표시는 유지될 수 있다. |
| 1 | 가속페달 상태와 무관하게 후보를 항상 적용한다. 가속페달 속도 하한은 만들지 않는다. |
| 2 | 차량 수신 과속카메라가 최종 후보이고 계산 목표가 현재 차량속도보다 낮아 실제 감속을 요구할 때, 새로 가속페달을 밟으면 현재 이벤트를 무시하는 것으로 판단한다. 감속구간에서 가속으로 도달한 최고속도를 `gas_override_speed`에 저장하고 감속 목표 하한으로 유지한다. |
| 3 | 가속페달을 밟는 동안 차량 수신 과속카메라 후보만 제외하고, 페달을 놓으면 다시 적용한다. |

mode 2는 실제 감속 요구 전에 밟아 계속 유지한 가속페달로 하한을 미리 만들지 않는다. 실제 감속 중 새 가속 입력으로 오버라이드가 시작된 뒤에는
가속으로 도달한 최고속도까지 저장값을 높이고, 페달을 놓아도 이벤트가 끝날 때까지 그 하한을 유지한다.
하한은 source 변경, 정차, 브레이크 입력, 제한속도 변경 또는 정상 목표가 150 km/h를 넘을 때 초기화된다.
이 모드 선택은 차량 CAN의 `hda`, `hda_section`, `school`에 적용한다. 차량 CAN 방지턱 `hda_bump`와 내비 방지턱 `bump`는
모드와 무관하게 같은 감속 중 새 가속 입력·이벤트 최고속도 하한 정책을 사용한다. 그 외 source에서는 `road`/`vturn`/`route` 등의 기존 속도 하한을 유지하고,
`cam`/`section`/`police`는 기존과 같이 하한을 초기화한다.

### 차량 내비 CAN 정확거리 정책

`VehicleNaviCanControl`과 `VehicleNaviSchoolZoneControl`은 기본값이 꺼진 실험 기능이다. 둘 중
하나를 켜면 Hyundai CAN-FD `0x4BE` `ProfileType=16`(Alert Information Spot)을 해석한다.
전자는 카메라·방지턱 거리 제어, 후자는 종류 7의 30 km/h 구간 상한만 활성화한다.
확인된 값은 다음과 같다.

Kia PV5는 기존 8바이트 메시지를 그대로 보내지 않고 CAN-FD wrapper에 넣는다. E-CAN `0x364`
bytes 8–15는 `HDA_INFO_4A3`, E-CAN `0x093` bytes 8–15는 `NEW_MSG_4BE`와 같은 필드 배치로
해석한다. A-CAN `0x380` byte 3의 bit 6은 카메라 접근 중 켜지고 실제 통과 시 꺼지므로 기존
`MapSource=2` 카메라 상태 대신 정확거리 후보 종료에 사용한다. PV5에서는 일반 카메라와 Value 6
방지턱만 활성화하며, 종류 7의 구간단속·30 km/h 구간은 주기 위치·평균속도 메시지가 실로그로
검증될 때까지 무시한다.

- `Value=0x06`: 방지턱 후보. `carState.speedBumpDistance`로 전달하고
  `AutoNaviSpeedCtrlMode >= 2`일 때 `AutoNaviSpeedBumpSpeed/Time`을 적용한다.
- `Value=0xB1/0x71/0xD1/0x150`: 각각 50/30/60/100 km/h 카메라 후보다.
  확인된 형식은 하위 4비트가 종류, 나머지 상위 비트가 `(제한속도 / 5) + 1`이다.
- 카메라 종류는 실로그로 확인한 0, 1, 2만 제어에 사용하며 나머지는 무시한다.
- `Value=0x77/0xB7`, `Offset=0`: 계기판 제한속도 30/50 km/h 전환과 일치하는 종류 7의
  현재 도로 제한속도 이벤트다. `VehicleNaviSchoolZoneControl`이 켜진 경우 0x77에서
  `carState.schoolZoneActive`를 시작하고 `school` 속도 후보를 30 km/h로 제한한다. 0x77 수신 시
  `HDA_INFO_4A3`의 30 km/h 카메라 상태도 활성화돼 있었다면 그 상태가 끝날 때 함께 해제한다.
  단, `HDA_INFO_4A3 LinkClass=1/2/3`(Freeway/IC/JC) 또는
  `0x4B9 FuncRoadClass=1/2`(Freeway/Arterial-City freeway)로 고속도로·고속화도로가 확인되면
  `school` 30 km/h 후보를 생성하지 않으며 이미 활성화된 후보도 즉시 해제한다. 같은 도로에서는
  차량 내비 방지턱 후보도 생성하지 않고 기존 대기 후보를 제거한다. 차량 내비의 일반 30 km/h 카메라
  후보 역시 감속 거리를 만들지 않지만, 50 km/h 이상의 실제 카메라·구간단속 후보는 유지한다.
  명시적인 비-30 종류 7 이벤트, `0x4B9 CalculatedRoute=2` 경로 재계산, 또는 진입 후 1 km 주행도
  해제 조건으로 사용하여 종료 프레임 누락 시 고착되지 않게 한다.
  `VehicleSpeedCameraControlMode` 0/1/2/3은 각각 미사용/항상 적용/가속페달 속도 하한/가속 중 해제로 동작한다.
  별도의 어린이보호구역 비트가 확인된 것은
  아니므로 일반 30 km/h 제한구역에도 적용될 수 있다.
- `Value=0x157`, `Offset=0`: 100 km/h 구간단속에서 확인된 종류 7의 제한속도 구간이다.
  `VehicleNaviCanControl`과 차량 카메라 상태가 유지되는 동안 `hda_section` 후보를 100 km/h에
  `AutoNaviSpeedSafetyFactor`를 적용한 연속 상한으로 사용한다. 차량 카메라 상태 종료 또는 경로
  재계산에서 해제한다. 같은 로그의 `Value=0x150`, `Offset=1997`은 실제 구간 종료까지 휠 적산거리
  1959 m와 근접하므로 가상거리보다 우선하는 정확거리 카메라 후보로 사용한다.

각 이벤트는 수신 시점 누적 주행거리와 `Offset`을 더한 절대 위치로 보관하고, 이후 실제 주행거리를
차감한다. `0x4B9 CalculatedRoute=2`(off route/recalculating)를 받으면 거리 기반 경로 후보를 즉시 모두
삭제하고 현재 적용 중인 종류 7 제한속도 상태도 해제한다.
`0x4BE` 무효값 `0xffffffff`, Offset 8191, 2.5 km 밖 이벤트와 지원하지 않는 값은 무시한다.

`AutoNaviCountDownMode`가 켜져 있으면 유효한 차량 내비 이벤트의 `speedLimitDistance`도 기존
`leftSec` 음성 카운트다운에 사용한다. mode 2에서는 `speedBumpDistance`도 포함한다. 카운트다운 대상이
없어지면 `leftSec=100`을 다시 발행해 다음 카메라가 같은 초에서 시작하더라도 새 이벤트로 인식한다.
로컬 `soundd`와 web sound 경로는 `selfdriveState`뿐 아니라 `carrotMan` 갱신에도 즉시 반응한다.

차량 카메라 후보가 활성화되면 같은 종류의 기존 외부 내비 카메라 후보는 중복 적용하지 않는다. 차량
방지턱 후보가 활성화되면 기존 type 22 방지턱 후보 대신 `hda_bump`를 사용한다. 서로 다른 종류의 후보는
함께 최솟값 경쟁에 참여하므로, 예를 들어 차량 방지턱과 외부 내비 카메라는 동시에 비교된다. 차량 후보가
없거나 설정이 꺼져 있으면 기존 외부 내비 후보를 그대로 사용한다.

## 수신되지만 감속 제어에는 쓰이지 않는 값

### 양쪽 공통 legacy 상태에서 미사용

- `nSdiSection` / 7714 `sdi.section_type`: 저장만 하고 판단에 사용하지 않는다.
- `nSdiBlockSpeed`: 저장은 되지만 block/section 진입 시 `xSpdLimit`에 사용하지 않는다.
- `nSdiPlusSpeedLimit`, `nSdiPlusBlockType`, `nSdiPlusBlockSpeed`, `nSdiPlusBlockDist`: 저장만 한다.
- secondary SDI는 type 22 방지턱의 거리만 감속에 사용할 수 있다. 적용 가능한 primary
  카메라/section이 있으면 primary 분기가 먼저 선택되므로 secondary 방지턱도 별도 후보로 계산되지
  않는다. secondary 카메라나 secondary section은 제어에 적용되지 않는다.
- 지원 목록 밖의 primary SDI type은 설명이나 화면 표시는 가능해도 `_update_sdi()` 감속 대상이 아니다.
- type 22의 payload speed limit은 primary/secondary 모두 무시한다.
- `roadcate` 0/1에서는 방지턱 감속을 의도적으로 막는다.

### 7714에서 화면/상태용이지만 제어에 미사용

- speed `current_kph`
- section `average_kph`, `overall_average_kph`, `remaining_time_sec`, `progress`
- guidance `time_sec`, `road_name`, `mid_direction`, point 좌표
- navigation status의 `mode`, `route_present`; `guidance_active`는 control 활성 판정에는 포함되지만
  그 자체로 속도를 만들지 않는다.
- lane 대부분은 제어에 쓰이지 않고 `road_category`만 방지턱 허용 판단에 사용한다.
- traffic signal은 위와 같이 UI/shared-memory 전달만 한다.

7714의 explicit section object가 active이면 같은 speed item의 primary SDI보다 먼저 선택된다. 둘을
각각 감속 후보로 계산해 더 낮은 값을 고르는 구조가 아니다. 이후 공통 `_update_sdi()`에서도 primary
section/camera 분기가 secondary 방지턱보다 먼저다.

## 7713과 7714의 중요한 동작 차이

### Primary type 22 방지턱의 결정적 차이

7713 `update(json)`의 처리 순서는 다음과 같다.

1. `nSdiType/nSdiDist`를 저장한다.
2. 같은 JSON의 `roadcate`를 저장한다.
3. 마지막에 `_update_sdi()`를 호출한다.

따라서 같은 packet의 `roadcate=8`, `nSdiType=22`, `nSdiDist=93`은 곧바로
`xSpdType=22`, `xSpdLimit=AutoNaviSpeedBumpSpeed`, `xSpdDist=93`이 된다.

7714 `_update_carrot_navi()`는 반대 순서다.

1. speed sequence가 바뀌면 `_apply_carrot_navi_speed()`를 호출한다.
2. 그 안의 `_update_sdi()`가 **이전 `self.roadcate`**로 방지턱을 판정한다.
3. 그 뒤에야 현재 `navi.road_category`를 `self.roadcate`에 저장한다.

또한 lane sequence만 바뀐 경우 `_update_sdi()`를 다시 호출하지 않는다. 그래서 이전 road category가
0/1이면 같은 7714 snapshot에 새 `road_category=8`이 있어도 그 speed item은 일단 탈락한다. 다음 speed
sequence에서 다시 평가되어야 살아난다. 더 심각하게는 present인 `lane_current`에 `road_category` key가
없으면 `carrot_navi_cereal._lane()`이 기본값 0을 넣고 control parser가 이를 유효한 road category로
받는다. 이 상태에서는 이후 type 22 speed sequence도 계속 0을 보고 탈락한다.

두 브랜치의 실제 `_update_sdi()`를 추출해 같은 상태로 실행한 결과는 동일했다.

| 처리 | `xSpdType` | `xSpdLimit` | `xSpdDist` |
|---|---:|---:|---:|
| 7713 순서: 먼저 `roadcate=8`, 이후 type 22 판정 | 22 | 22 | 93 |
| 7714 순서: 이전 `roadcate=0`으로 판정, 이후 8 저장 | -1 | 0 | 0 |
| 그 다음 speed sequence에서 8로 재판정 | 22 | 22 | 93 |

이는 `origin/carrot-wip`과 `origin/thftgr/navi-stream` primary type 22에 공통이다. `navi-stream`의
secondary SDI/route 확장은 이 primary gate 문제를 고치지 않는다.

### 도로 제한속도

- 7713은 현재 저장값과 다른 제한속도가 6회 연속 들어와야 값을 바꾼다. 새 후보값 자체의 일치 여부는
  추적하지 않으므로 서로 다른 값들이 연속으로 들어와도 counter는 증가한다.
- 7713은 200 초과 값을 `(value - 20) / 10`으로 legacy decode하고, 120은 115로 보정하며,
  0 이하는 30으로 바꾼다.
- 7714는 1..200만 valid로 인정하여 즉시 적용한다. 120 보정이나 200 초과 legacy decode는 없다.

### off-route와 clear

- 7714는 전체 off-route에서 primary/secondary SDI, section, current/next TBT를 억제한다. 도로
  제한속도는 유효하면 남긴다.
- 7713에는 off-route gate가 없다. 앱이 clear 값이나 새 값을 보내거나 active timer가 끝날 때까지
  기존 값이 유지된다.
- 7714는 control WebSocket disconnect 또는 `connected=false`가 cereal로 전달되면 speed/TBT/route
  control 상태를 명시적으로 지운다.
- 7713 rgdata는 매 수신 시 `active_count=80`이다. 20 Hz loop 기준 약 4초 동안 새 rgdata가 없으면
  active가 꺼진다. `active_sdi_count=200`은 단독으로 active를 유지하지 않는다.
- 7714 receiver는 manifest에 stale timeout을 광고하지만 receiver 자체에서 JSON item을 시간으로
  expire하지는 않는다. control socket이 살아 있고 tombstone이 오지 않으면 같은 item을 heartbeat로
  계속 발행한다. 거리는 차량 이동량만큼 로컬 차감되지만 정차 중인 stale SDI/road limit은 남을 수 있다.

### 순서와 freshness

- 7713 rgdata는 양수 timestamp가 있을 때 하나의 전역 `last timestamp`보다 큰지만 검사한다. timestamp가
  없거나 0이면 검사하지 않으며, client 재접속 때 counter를 reset하지 않는다.
- 7714는 새 협상마다 session을 만들고 item stream별 strictly increasing sequence를 검사한다.
- 7714 cluster raw 패널은 별도로 speed 등 일반 item 6초, status 10초, route 30초 TTL을 적용하지만,
  이 UI TTL은 `CarrotServ` control parser에는 적용되지 않는다.

### route

- 7713 route는 `NavDestination`도 마지막 점으로 갱신한다.
- 7714 route bridge는 기존 route consumer만 갱신하고 `NavDestination`은 쓰지 않는다.
- 두 경로 모두 실제 route 감속은 공통 곡률 계산을 사용하므로 설정이 허용하고 route candidate가
  최솟값일 때만 `desiredSource=route`가 된다.

### 동시 수신 충돌

- 포트별 별도 `CarrotServ` state나 fixed priority가 없다.
- 7713 HTTP handler thread와 20 Hz control/update thread 사이에 `CarrotServ` state를 보호하는 lock도
  없다. 여러 attribute를 갱신하는 도중 main loop가 읽을 수 있어, 엄밀히는 단순 last-writer뿐 아니라
  서로 다른 입력의 type/limit/dist가 일시적으로 섞일 가능성도 있다.
- 7713 rgdata가 들어오면 shared speed/TBT 필드를 즉시 덮어쓴다.
- 7714는 새 speed sequence일 때만 speed 필드를 덮어쓴다. 동일 sequence heartbeat는 7713이 바꾼
  값을 복원하지 않는다.
- 7714가 연결되어 있지만 7714 speed에 valid road limit이 없으면, `carrot_navi_active=true` 때문에
  7713이 채운 road limit의 `road` 후보가 비활성화될 수 있다.
- 따라서 두 포트를 동시에 제어 입력으로 쓰는 것은 결정적 우선순위를 보장하지 않는다.

## UI 표시 확인

### 실제 감속 source 표시

`CarrotServ`가 후보 중 최솟값을 고른 뒤 다음 중 하나를 `desiredSource`로 발행한다.

`atc`, `atc2`, `cam`, `hda`, `hda_section`, `hda_bump`, `school`, `bump`, `section`, `police`, `waze`,
`road`, `vturn`, `route`, `model`, `gas`

on-road UI, mici UI, cluster live UI의 보조속도 영역은 선택된 감속 이유와 목표속도만 표시한다. 제어용
`desiredSource` 값 자체는 바꾸지 않으며, cruise가 꺼졌을 때 차량 내비 속도를 보조속도처럼 강제로 표시하지 않는다.

- `0 < desiredSpeed < 200`
- `desiredSpeed < 운전자 설정 cruise speed`
- 외부 내비 source는 실제 이유(`cam`, `section`, `bump`, `turn`, `route` 등)를 주황으로 표시
- 차량 CAN 내비 source는 실제 이유(`cam`, `section`, `bump`, `school`)를 라벤더로 표시
- `atc`/`atc2`는 실제 내비 회전 안내이므로 `turn`, 차선·곡률 기반 `vturn`은 `vturn`,
  모델 예측 감속은 `model`로 구분하고 나머지 source도 실제 이유(`gas` 등)와 주황색을 유지

내비 상태 표시는 보조속도와 별개다. 외부 내비가 연결되면 주황 `NAVI`를 차량 CAN 내비보다 우선하고,
그렇지 않은 상태에서 Hyundai CAN-FD `0x4BE`가 한 번이라도 수신되면 라벤더 `vNAVI`를 표시한다.
두 표시는 디바이스와 USB 클러스터에서 기존 위치보다 글자 한 칸 정도 왼쪽에 배치한다.
외부 내비 연결은 감속 상태인 `activeCarrot`로 추정하지 않는다. 7713 legacy 경로는 최근 송신자 주소인
`carrotMan.remote`, 7714 경로는 alive/valid인 `carrotNavi.connected`를 사용한다. 따라서 차량 CAN의
카메라·구간단속·방지턱·school 후보가 `activeCarrot=3/4/5/6`을 만들더라도 `vNAVI`가 `NAVI`로 바뀌지
않고 cluster도 외부 내비 화면으로 전환하지 않는다. 실제 외부 연결이 생기면 기존 내비 패널로 전환하고,
연결이 끊기면 주행 중 유지된 `vehicleNaviAvailable`에 따라 다시 `vNAVI`와 주행리포트로 복귀한다.

따라서 route 데이터가 존재하는 것만으로 `route`가 표시되는 것은 아니다. route 후보가 설정상
활성이고 다른 모든 후보보다 낮아 실제 winner가 되어야 한다. 방지턱도 같은 방식으로 `bump`가 winner일
때만 표시된다. `longitudinalPlan.cruiseTarget`의 eco 표시 조건이 먼저 참이면 `eco`가 우선 표시된다.
`VehicleSpeedCameraControlMode=2`에서 차량 수신 과속카메라 `hda`, 구간단속 `hda_section` 또는 30 km/h 구간 `school`이 winner이고
계산 목표가 현재 차량속도보다 낮아진 뒤 새로 가속페달을 밟으면 현재 감속 이벤트를 무시하는 오버라이드를 시작한다. 오버라이드 중
가속으로 도달한 최고속도가 하한이 되며, 이 하한이 계산 목표보다 높으면 최종 source가 `gas`로 바뀐다. 감속 전에 밟아 계속 유지한
가속페달은 하한을 만들지 않고, 이벤트 source가 끝나면 하한을 초기화한다.
다른 감속 source에는 이 모드 선택을 적용하지 않는다. 차량 CAN 방지턱 `hda_bump`와 내비 방지턱 `bump`는 카메라 모드와
무관하게 실제 감속 중 새 가속 입력으로 같은 이벤트 오버라이드를 시작한다. `school`에서 mode 2의 `gas`가 연속 3초 이상 유지되면 현재 school 구간을 억제하고,
차량의 `schoolZoneActive`가 끝난 뒤 다음 구간에서 다시 활성화한다.

카운트다운은 같은 종류의 다음 카메라·방지턱·회전까지 거리가 20 m 또는 현재 2초 주행거리보다 크게 증가하면 새 목표로
판정한다. 이전 이벤트의 초 값이 11 이하라면 `leftSec=100`을 한 프레임 발행해 soundd를 재무장한 뒤 새 목표의 초 값을
다시 발행한다. 따라서 앞 이벤트가 사라지는 idle 프레임 없이 다음 0x4BE 후보로 바로 넘어가도 같은 숫자의 안내음이
누락되지 않는다.

후보 속도가 완전히 같으면 list 순서상 `atc`, `atc2`, SDI 계열, `road`, `vturn`, `route`, `model`
순서로 먼저 등장한 source가 label이 된다.

### 방지턱에서 `route 30`이 표시되는 경우

`desiredSource`는 현재 들어온 이벤트 type을 그대로 표시하는 값이 아니라, 실제로 생성된 후보 중 가장
낮은 속도를 만든 후보의 이름이다. 따라서 `route 30`은 두 경우 모두 가능하다. (1) 정상 생성된 bump
후보보다 route가 낮거나, (2) 위 road category 문제로 bump 후보 자체가 생성되지 않은 경우다. 7714만
실제 감속하지 않았다는 관측까지 합치면 두 번째 경우를 먼저 의심해야 한다.

- 방지턱 후보는 `AutoNaviSpeedBumpSpeed`를 도착 목표로 하여 거리 기반 감속 속도를 계산한다. 목표속도가
  30 km/h여도 방지턱에 도달하기 전 계산값은 보통 30보다 조금 크다. 기본 목표는 35 km/h다.
- route 후보는 `max(route_speed * MapTurnSpeedFactor, AutoCurveSpeedLowerLimit)`이다. 기본
  `AutoCurveSpeedLowerLimit`는 30 km/h라서 경로 곡률 계산값이 낮으면 정확히 30에 고정된다.
- UI에 발행하는 `desiredSpeed`는 `int()`로 소수점을 버린다. 내부적으로 route가 30.0, bump가 30.x이면
  둘 다 화면에는 30처럼 보일 수 있지만 route가 엄밀히 더 낮아 source는 `route`가 된다.
- 속도가 완전히 같은 경우에는 후보 list에서 먼저 나오는 SDI/bump가 이긴다. 따라서 `route 30`은 내부
  route 값이 실제로 더 낮았거나, 방지턱이 `roadcate > 1`, mode >= 2 등의 조건을 통과하지 못했다는 뜻이다.
- route 곡률은 현재 위치부터 약 300 m 경로를 사용하고 뒤에서 앞으로 감속속도를 전파한다. 전방의 실제
  급커브뿐 아니라 polyline의 꺾임/노이즈도 route 값을 30까지 낮출 수 있으며, 방지턱 type 자체를 route로
  변환하는 로직은 없다.
- `TurnSpeedControlMode == 2`의 route 활성 조건은 현재 `-500 < xDistToTurn < 500`뿐이다. 유효한 TBT가
  없을 때의 `xDistToTurn == 0`도 이 조건을 통과하므로 route가 의도와 달리 활성화될 수 있다.
  mode 3/4에서는 route 후보가 항상 활성화된다.

현장에서 `carrotMan.xSpdType == 22`이고 `activeCarrot == 5`이면 방지턱 자체는 정상 인식된 상태에서
route가 더 낮아 이긴 것이다. `xSpdType == -1`이면 7714의 raw SDI가 UI에 보여도 방지턱 제어 후보는
없으며, `lane_current.road_category`, speed sequence, off-route를 먼저 확인해야 한다.

### 현장 설정 사례: bump 22 km/h, route/curve 하한 18 km/h

확인한 현장 Params의 관련 값은 다음과 같다.

| Param | 값 | 실제 영향 |
|---|---:|---|
| `AutoNaviSpeedCtrlMode` | 2 | 일반 카메라와 type 22 방지턱 활성. 이동식 type 7은 비활성 |
| `AutoNaviSpeedBumpSpeed` | 22 | payload의 bump speed 대신 최종 방지턱 목표 22 km/h 사용 |
| `AutoNaviSpeedBumpTime` | 3 | 22 km/h × 3초 = 약 18.33m를 safe distance로 사용 |
| `AutoNaviSpeedDecelRate` | 120 | 계산에서는 1.20 m/s² 사용 |
| `AutoNaviCountDownMode` | 2 | 방지턱 거리 카운트다운 포함 |
| `AutoNaviSpeedSafetyFactor` | 107 | 일반 camera/section에는 107% 적용하지만 bump에는 미적용 |
| `AutoNaviSpeedCtrlEnd` | 10 | 일반 camera에 사용하며 bump에는 미적용 |
| `TurnSpeedControlMode` | 2 | vision `vturn`과 조건부 route 후보 활성 |
| `AutoCurveSpeedLowerLimit` | 18 | vturn/route/model 후보 하한 18 km/h |
| `MapTurnSpeedFactor` | 102 | route 계산속도에 1.02 곱함 |
| `AutoCurveSpeedFactor` | 90 | vision 곡률 입력에 0.90 곱해 vturn 감속을 다소 완화 |
| `AutoTurnControl` | 1 | TBT `atc/atc2` 속도 후보는 비활성 |
| `ModelTurnSpeedFactor` | 0 | model turn speed가 200으로 유지되어 현재 후보 조건 `< 200`을 통과하지 않음 |
| `AutoRoadSpeedLimitOffset` | -1 | `road` 제한속도 후보 비활성 |

이 설정의 순수 방지턱 후보는 다음 공식이다.

여기서 `bump 후보`는 방지턱 자체의 제한속도가 아니라, 현재 남은 거리에서 planner에 넘길 **현재 허용
목표속도**다. `AutoNaviSpeedBumpSpeed=22`는 방지턱 도착 목표이고, 코드는 감속률과 거리를 역산하여
멀리서는 22보다 높은 속도를 허용한 뒤 접근할수록 22로 낮춘다. 최종 후보가 운전자 설정속도보다 높으면
아직 cruise를 제한하지 않으며, 그보다 낮아지는 지점부터 실제 감속 요구가 생긴다.

`safeDist = (22 / 3.6) * 3 = 18.33m`

`distance > 18.33m`이면
`bumpKph = 3.6 * sqrt((22 / 3.6)^2 + 2 * 1.2 * (distance - 18.33))`, 그 이하는 22
km/h다.

| 남은 방지턱 거리 | 내부 bump 후보 | publish `int()` 값 |
|---:|---:|---:|
| 150m | 67.67 | 67 |
| 100m | 54.99 | 54 |
| 93m | 52.98 | 52 |
| 60m | 42.19 | 42 |
| 50m | 38.33 | 38 |
| 40m | 34.03 | 34 |
| 30m | 29.10 | 29 |
| 20m | 23.15 | 23 |
| 18.33m 이하 | 22.00 | 22 |

따라서 route 후보가 30 km/h로 유지된다면 약 31.7m 바깥에서는 route 30이 bump 후보보다 낮아
`desiredSource=route`가 되고, 약 31.7m 안쪽부터 bump 후보가 30보다 낮아져 `bump`가 이긴다. route가
하한 18까지 내려가면 bump 최저목표 22보다 항상 낮아서 방지턱 전체 구간에서 route가 이길 수 있다.

추가 조건과 주의점:

- 7714 primary 또는 secondary SDI type 22가 있어야 한다. secondary type 22는 적용 가능한 primary
  camera/section이 있으면 무시된다.
- `road_category > 1`이어야 한다. lane item이 전혀 없으면 마지막 값/초기값 8이 남지만, lane item은
  present이고 key만 빠지면 0으로 변환되어 방지턱을 차단한다. 앱은 `lane_current.road_category`를 반드시
  명시하고 speed보다 먼저 또는 적어도 다음 speed sequence 전에 유효값이 반영되게 해야 한다.
- `navigation_status.off_route=true`이면 7714 SDI가 억제된다.
- `HapticFeedbackWhenSpeedCamera=1`은 현재 checkout에서 Param 등록/설정 UI 외에 읽는 코드가 없어
  방지턱 감속이나 실제 햅틱에 영향을 주지 않는다.
- bump에는 차량 수신 과속카메라용 가속페달 하한을 적용하지 않으므로, 가속페달 입력만으로 방지턱
  목표 source가 `gas`로 바뀌지 않는다.

### 실제 적용과 무관할 수 있는 별도 표시

- 일반 on-road HUD의 별도 `ROUTE` badge는 `carrotMan.navPathVertexCount`를 읽지만 해당 capnp field가
  존재하지 않고 publisher는 `naviPaths` text만 보낸다. 따라서 이 badge 경로는 현재 항상 0으로
  떨어져 표시되지 않는다. 실제 감속 winner의 `route` label은 별도 로직이므로 표시 가능하다.
- speed limit box는 방지턱(type 22)을 제외한 모든 `xSpdType`을 세분하지 않고 `CAM`으로 표시한다.
  section도 이 box에서는 CAM이다. winner source 표시에서는 `section`으로 구분된다.
- 7714 전용 cluster 내비 패널은 raw SDI를 `SDI/SDI 2`, raw section을 `SECTION`, route presence를
  `ROUTE`로 표시한다. 7713은 이 raw `carrotNavi` 패널의 입력이 아니다. 이 패널 표시는 실제
  `desiredSpeed` winner 여부와 무관하다.
- cluster의 raw SDI/section 표시는 전체 off-route를 함께 확인하지 않고, section은 `sectionActive`만
  확인하여 suspended/section off-route도 반영하지 않는다. 따라서 control parser가 감속을 억제해도
  패널에는 SDI 또는 SECTION이 보일 수 있다.

## 검증 근거

- 7714 control parser의 focused test 함수들은 현재 체크아웃에서 직접 실행하여 통과했다.
- 저장소의 `test_carrot_navi_serv.py`는 일반 7714 SDI/section/secondary 상태 반영, disconnect clear 및
  legacy 7713 경로를 단언하지만, primary type 22의 road-category 순서를 end-to-end로 단언하지 않는다.
- 두 브랜치의 실제 `_update_sdi()` 함수로 `roadcate=8`인 7713 순서와 이전값 0인 7714 순서를 별도로
  실행하여 각각 `(22,22,93)`과 `(-1,0,0)`을 재현했다.
- `test_carrot_navi_route_bridge.py`는 양쪽 route 경로와 7714 tombstone/disconnect ownership을 단언한다.
- Windows 환경에는 전체 openpilot Linux 의존성과 pytest가 없어 전체 test suite는 실행하지 못했다.
