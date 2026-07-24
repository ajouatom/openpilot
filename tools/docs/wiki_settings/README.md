# Wiki 설정 설명 규격

이 디렉터리는 GitHub Wiki에서 작성한 설정별 설명을 검증하고 Carrot Web에 표시하기 위한 규격 원본이다. 대화 세션이나 Wiki 안내 페이지는 규격 원본이 아니다.

## 책임 분리

| 대상 | 책임 |
|---|---|
| `openpilot/selfdrive/carrot_settings.json` | 현재 설정 목록과 기본값·범위·단위·선택지 |
| `docs/user/ko`, `docs/user/en` | 공개 장문 사용자 가이드 |
| GitHub Wiki | 설정별 짧은 설명의 작성 공간 |
| GitHub Wiki 자동 영역 | 검증을 통과한 Carrot Web 조회 원본 |
| 이 디렉터리 | 작성·구조·렌더링·검증 규격 |

Wiki의 설정별 설명은 장문 가이드를 복사하지 않는다. 필요한 경우 `docs/user`의 관련 가이드로 연결한다.

## 파일

- `AUTHORING_GUIDE.md`: 사람이 따르는 Wiki 작성 규칙
- `RENDERING_CONTRACT.md`: Wiki와 Carrot Web의 의미·컴포넌트 대응 규칙
- `content.schema.json`: 동기화 콘텐츠의 기계 검증 규격
- `generate.py`: 설정마다 언어별 독립 Wiki 페이지와 해시 인덱스를 만드는 생성기
- `ci_check.py`: 생성 결과를 검증하고 PR용 요약·Diff Artifact를 만드는 읽기 전용 검사기
- `validate.py`: 동기화 JSON과 Wiki Markdown의 공용 검사기
- `examples/valid`: 정상 규격 예제
- `examples/invalid`: 검사기가 거부해야 하는 예제

## 기준

- 현재 규격 버전은 `1`이다.
- 문서 생성과 삭제는 `carrot-wip`의 현재 설정 목록만 기준으로 한다.
- `carrot-wip`에서 삭제된 설정은 Wiki 생성 결과·인덱스에서 삭제하며 별도 보관하지 않는다.
- 설정별 파일명·페이지 제목·목록과 메뉴 계층은 Carrot Web이 각 언어에서 실제로
  표시하는 `title`, `etitle`, `ctitle`과 `menu` 구조를 기준으로 한다.
- 기존 수동 Wiki 문서는 수정하지 않는다. `_Sidebar.md`에는 기존 구조를 유지한 채
  `사용 설명서 > 설정 이해하기 > 전체 설정` 아래에 Carrot Web과 같은 순서의
  한국어 메뉴 계층과 설정 링크를 생성기가 관리한다.
- 로케일 제목이 바뀌면 파라미터를 기준으로 페이지를 이동하고 `MANUAL` 영역을
  보존한다. 같은 이름의 비관리 Wiki 페이지가 있으면 덮어쓰지 않고 실패한다.
- Wiki 편집은 자유롭게 할 수 있지만 Carrot Web에는 생성·검증된 자동 영역만 표시한다.
- 한국어와 영어는 필수이며 중국어 상세 설명이 없으면 영어로 대체한다.
- `contentHash`는 `source.contentHash` 필드를 제외한 정규화 콘텐츠의 SHA-256 값이다.

## 변경 절차

1. 작성 규칙이나 콘텐츠 토큰을 바꾼다.
2. `content.schema.json`과 렌더링 계약을 함께 갱신한다.
3. 정상·오류 예제를 갱신한다.
4. 공용 검사기를 통과시킨다.
5. 호환되지 않는 변경이면 `schemaVersion`과 마이그레이션을 함께 추가한다.

## 검증

추가 Python 패키지 없이 저장소 루트에서 실행한다.

```bash
python tools/docs/wiki_settings/generate.py
python tools/docs/wiki_settings/ci_check.py --wiki-dir <Wiki 체크아웃> --output-dir <보고서 디렉터리>
python tools/docs/wiki_settings/validate.py <동기화 JSON 또는 Wiki Markdown>
python -m unittest discover -s tools/docs/wiki_settings/tests -p "test_*.py" -v
```

`generate.py`는 기본적으로 결과와 Diff만 계산하는 `dry-run`이다. 기존 Wiki의 수동 영역을 반영하려면 `--wiki-dir <Wiki 체크아웃>`을 지정한다. 별도 준비 디렉터리에 결과를 쓸 때만 `--write --output-dir <디렉터리>`를 함께 사용한다. 같은 파라미터의 `MANUAL` 영역은 페이지가 이동해도 그대로 보존되고, 의미 관련 자동 정보가 달라지면 `needs_review`로 계산된다.

`.github/workflows/wiki-settings-check.yaml`은 `carrot-wip` 대상 Pull Request에서 공개 Wiki를 쓰기 자격 증명 없이 읽고 `ci_check.py`를 실행한다. 저장소 권한은 `contents: read`만 사용하며 `summary.json`, `summary.md`, `pages.diff`만 GitHub Actions Artifact로 올린다.

`.github/workflows/wiki-settings-publish.yaml`은 GitHub Actions에서 `cp-set-wiki`로 표시된다. `carrot-wip` 설정 변경, Wiki `gollum`, 수동 실행에서만 동작하며, 신뢰된 기본 브랜치의 생성기와 검사기를 사용해 실제 GitHub Wiki의 자동 영역·전체 설정 목록·한국어 사이드바 설정 트리와 `Settings-Catalog.json`을 한 Wiki 커밋으로 동기화한다. 개발 브랜치나 별도의 생성물 브랜치는 만들지 않는다.

Carrot Web 조회 기준:

```text
https://raw.githubusercontent.com/wiki/ajouatom/openpilot/Settings-Catalog.json
https://raw.githubusercontent.com/wiki/ajouatom/openpilot/{PAGE}.md
```

Carrot Web의 조회 계층은
`openpilot/selfdrive/carrot/web/src/features/settings/published_documentation.js`에 있다.
설정 상세를 열면 30초 단위로 인덱스를 재검증하고 현재 언어의 해당 설정 페이지만
받는다. 인덱스와 페이지의 SHA-256·바이트 크기·마커 메타데이터가 모두 맞아야
GFM 허용 AST로 변환한다. 같은 요청은 병합하며 한 호출자의 취소가 공유 요청을
중단하지 않는다. 네트워크 또는 검증 실패 시 용량이 제한된 마지막 원격 정상본만
사용한다. 정상본도 없으면 설명 탭은 빈 상태를 표시하며 장치 내부 문서 API로
대체하지 않는다. 어떤 경우에도 설정 조회·변경 경로는 차단되지 않는다.
운영 점검 시 `CarrotSettingsRuntime.docs.diagnostics()`에서 로드·요청·응답 바이트,
최근 로드·파싱 시간, 원격 미사용 횟수와 오류율을 읽을 수 있다.

동기화 Workflow는 `contents: write`만 요청한다. 저장소나 조직 정책이 Workflow의 쓰기 토큰을 제한하면 Wiki 푸시가 실패하며, 별도 개인 토큰으로 우회하지 않고 권한 설정 또는 GitHub App을 결정할 때까지 기존 Wiki 정상본을 유지한다.

종료 코드는 성공 `0`, 콘텐츠 오류 `1`, 실행·입력 오류 `2`다. 검사 오류는 파일, 가능한 경우 줄, 구조 위치와 사유를 함께 표시한다.
