# carrotpilot 사용자 설명서 원본

`docs/user/ko`와 `docs/user/en`은 `carrot-wip`의 공개 사용자 설명서 원본입니다. 사용자에게 보이는 동작을 바꾸는 코드 수정은 관련 테스트와 **한국어·영어 문서 쌍**을 같은 변경에서 갱신합니다. Wiki는 문서 안내와 커뮤니티 자료를 제공하고, 상세 기능 설명은 이 원본 문서로 연결합니다.

## 문서 목록

| 주제 | 한국어 | English |
|---|---|---|
| Carrot Web | [Carrot Web 사용 설명서](ko/carrot-web.md) | [Carrot Web User Guide](en/carrot-web.md) |
| 분석 로그 전송 | [분석용 대시캠 로그 전송](ko/dashcam-log-sharing.md) | [Sending Dashcam Logs for Analysis](en/dashcam-log-sharing.md) |
| 설정 | [설정 이해하기](ko/settings.md) | [Understanding Settings](en/settings.md) |
| 버튼·프리셋 | [버튼·프리셋](ko/buttons-presets.md) | [Buttons and Presets](en/buttons-presets.md) |
| 속도·감속 | [속도·감속](ko/speed-deceleration.md) | [Speed and Deceleration](en/speed-deceleration.md) |
| 크루즈·차간거리 | [크루즈·차간거리](ko/cruise-gap.md) | [Cruise and Following Gap](en/cruise-gap.md) |
| 레이더 | [레이더트랙·코너레이더](ko/radar.md) | [Radar Tracks and Corner Radar](en/radar.md) |
| Tesla | [Tesla 차량 연동](ko/tesla.md) | [Tesla Vehicle Integration](en/tesla.md) |

## 코드 변경 시 문서 확인

코드와 문서의 연결은 [`docs_map.json`](docs_map.json)에 선언합니다. 브랜치의 기준 커밋과 비교하려면 저장소 루트에서 다음을 실행합니다.

```bash
python tools/docs/check_user_docs.py --base origin/carrot-wip
```

검사기는 연결표에 등록된 코드가 바뀌었는데 관련된 한국어·영어 문서 쌍이 함께 수정되지 않은 경우 실패합니다. 경로가 여러 사용자 기능에 공통으로 쓰이면 실제 동작과 관련된 주제의 문서 쌍 하나 이상을 갱신합니다.

사용자 동작이 바뀌지 않은 리팩터링, 로그·성능·테스트 전용 수정은 문서를 억지로 고치지 않습니다. Pull Request 본문에 다음처럼 구체적인 이유를 남깁니다.

```text
Docs-Not-Needed: 사용자 동작은 동일하며 내부 함수만 분리함
```

빈 사유나 `N/A` 같은 형식적인 값은 예외로 인정하지 않습니다.

## 공개 범위

이 디렉터리는 공개 저장소에 게시됩니다. 비공개 기능, 내부 전용 사용법, 인증정보, 개인 식별정보와 공개 권한이 확인되지 않은 커뮤니티 자료는 넣지 않습니다. 이런 자료는 별도의 비공개 저장 위치에서 관리합니다.

## English policy summary

Every public user document has a Korean and English file with the same name. A user-visible code change must update both files in the same change. The documentation checker accepts a topic only when one complete `ko`/`en` pair from `docs_map.json` changed. Internal-only or non-public material must not be added here.
