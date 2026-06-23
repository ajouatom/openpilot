# Carrot Model Selector

Carrot 파일럿 전용 주행 모델 다운로드/설치/적용 모듈. c3x·c4 하드웨어 공용으로
**carrot web(포트 7000)** 에서 UI를 제공한다.

## 설계 원칙

1. **두 엔진 완전 분리**
   - 기본 모델 (`selfdrive/modeld/models/`) → **upstream `selfdrive/modeld/modeld.py` 가 그대로 담당**
   - 커스텀 모델 (`/data/models/`) → **`carrot.model_selector.carrot_modeld` 가 담당** (3-모델 on/off policy 자체 지원)
   - `modeld_runner` 가 부팅 시 `/data/models` 유효성으로 둘 중 하나를 택일 실행 (monkey-patch 없음)
   - upstream 이 2-모델→3-모델로 바뀌어도 우리 엔진과 무관하게 진행 가능

2. **upstream 파일 최소 침습** (각 1~3줄)
   - `system/manager/process_config.py` — modeld 엔트리를 `carrot.model_selector.modeld_runner` 로 교체
   - `system/manager/manager.py::main()` 초입 — `boot_compile.run()` 호출
   - `selfdrive/carrot/server/app_factory.py` — 라우터 등록
   - `selfdrive/carrot/web/index.html` — 네비 버튼 + `page_models.js` 로드 (`?v=…` 캐시 버스팅)
   - `selfdrive/ui/mici/layouts/home.py` — c4 홈 화면 커밋해시 옆에 `DrivingModelName` 표시
   - `common/params_keys.h` — `DrivingModelName`, `PendingModelName` 파람 등록

3. **독립 패키지**
   - 모델 셀렉터 자체 엔진 포함 전부 `carrot/model_selector/` 에 완결
   - `carrot_parse_model_outputs.py` 는 3-모델 분기가 있는 c3-ms 파서 이식본
   - 원본 파일(`selfdrive/modeld/*`, `common/file_chunker.py` 등)은 읽기/import 전용, 수정 0

4. **파일 세트 호환** (vision + 정책)
   - `driving_vision` 필수 + (`driving_on_policy` 또는 `driving_policy`) 중 하나 필수
   - `driving_off_policy` 는 선택적, 존재 시 3-모델 아키텍처 활성화

## 모듈 구조

| 파일 | 책임 |
|------|------|
| `config.py` | 경로·파일명 상수, 허용 onnx 목록, tinygrad 컴파일 플래그 |
| `keys.py` | Ed25519 공개키 맵 + `MODEL_SELECTOR_VERSION` |
| `manifest.py` | `models.json` fetch (캐시버스터 `?t=unixtime` + no-cache 헤더) + canonical JSON Ed25519 검증. snake_case/camelCase 필드 둘 다 수용 |
| `downloader.py` | ONNX 스트리밍 다운로드 + SHA256/크기 검증. 공백 포함 파일명 URL percent-encoding |
| `validator.py` | `/data/models/` 파일 세트 유효성 검증 + `describe()` 라벨 |
| `installer.py` | tinygrad 컴파일 (compile3) + warp pkl 복사 + atomic swap + 실패 시 backup 복원 |
| `carrot_modeld.py` | **자체 modeld 엔진** — 2/3-모델 공용, `/data/models` 전담 |
| `carrot_parse_model_outputs.py` | 자체 parser (c3-ms 이식, `parse_off_policy_outputs` 포함) |
| `modeld_runner.py` | 부팅 시 커스텀/기본 선택해서 택일 실행 (dispatcher). `/data/model_selector_status` 에 엔진 정보 기록 |
| `boot_compile.py` | `manager.main` 초입 훅, pending 모델 컴파일 트리거 |
| `jobs.py` | 간단한 in-memory job runner (web 진행률 폴링) |
| `web/routes.py` | aiohttp 라우터 (list / status / install / job / apply / reset) |
| `web/frontend/page_models.html` | carrot web 페이지 템플릿 (Material 3 토큰 기반) |
| `web/frontend/page_models.js` | 프런트엔드 로직 (fragment 자동 주입 + Job 폴링 + 확인창 + 재부팅 카운트다운) |

## Params

| 이름 | 타입 | 의미 |
|------|------|------|
| `DrivingModelName` | STRING | 현재 사용 중인 모델 표시용 (c4 홈 화면 커밋해시 옆에도 표시) |
| `PendingModelName` | STRING | 재부팅 후 컴파일 대기 중인 모델 ID |

## 데이터 흐름

### 설치 (Install)
```
[Web UI: "설치" 클릭]
  └─ appConfirm 다이얼로그 ("모델을 설치합니다. 다운로드 후 자동 재부팅됩니다.")
       └─ POST /api/models/install {id}
            └─ manifest 재검증 → jobs.start("install_model")
                 └─ downloader.download_model(entry, progress_cb)   # 파일별 size+sha256 검증
                      → /data/models_tmp/
                 └─ Params.put(PendingModelName=entry.id)
                 └─ job.finish(ok=True, progress=100)

[클라이언트 폴링] GET /api/models/job?id=<job_id>  (320ms)
  └─ snap.done && status=="done"
       └─ POST /api/models/apply  (서버가 5초 후 sudo reboot 예약)
            └─ rebootCountdown(5)  — 0→100 바, "다운로드 완료 — N초 후 재부팅…"

[재부팅 → manager.main() 초입]
  └─ boot_compile.run() → installer.compile_pending()
       ├─ ONNX → tinygrad.pkl + metadata.pkl (QCOM/FLOAT16/NOLOCALS/JIT_BATCH_SIZE=0/IMAGE=1/OPENPILOT_HACKS=1)
       ├─ selected model metadata의 img 입력 크기로 standalone warp pkl 새로 생성 (빌트인 warp 재사용 안 함)
       ├─ atomic swap: /data/models_tmp → /data/models  (실패 시 backup 복원)
       └─ Params: DrivingModelName 기록, PendingModelName 삭제

[process manager → `modeld` 프로세스 (only_onroad)]
  └─ modeld_runner.main()
       ├─ validator.is_valid_model_dir(/data/models) ?
       │    ├─ YES → /data/model_selector_status 기록(engine=carrot_modeld)
       │    │        carrot_modeld.main()        ← 3-모델 엔진
       │    └─ NO  → /data/model_selector_status 기록(engine=upstream_modeld)
       │             upstream modeld.main()       ← 기본 엔진
```

### 기본 모델 복원 (Reset)
```
[Web UI: "기본 모델 복원" 클릭]
  └─ appConfirm 다이얼로그
       └─ POST /api/models/reset
            ├─ installer.reset_to_default()   # /data/models + /data/models_tmp 삭제, 파람 제거
            └─ 5초 후 sudo reboot 예약
       └─ rebootCountdown(5) — "기본 모델 복원 — N초 후 재부팅…"
```

### 런타임 엔진 확인
```
$ cat /data/model_selector_status
engine=carrot_modeld
pid=53610
started=1776677826
describe=/data/models: vision+on_policy+off_policy
```
`ps` 는 `setproctitle` 때문에 Python 모듈 경로가 마스킹되므로 이 파일이 결정적 출처.

## 웹 UI 특징

- **Material 3 카드 레이아웃** — `page_models.html` 에 인라인 CSS, carrot web 의 `--md-*` 디자인 토큰 사용
- **고정 높이 + 내부 스크롤** — `body[data-page="models"] { overflow: hidden }` + `.ms-list { flex:1; overflow-y:auto }` 로 리스트만 스크롤
- **정렬** — `added_at` DESC → `name` 자연 정렬 DESC (v11 > v10)
- **확인창** — `window.appConfirm(message, {title, confirmLabel})` (도구 탭 reboot 팝업과 동일)
- **카운트다운 통일** — 다운로드·재부팅 모두 0→100 방향으로 채워짐
- **네비 active 동기화** — `carrot:pagechange` 이벤트 리스너가 `btnModels.active` 토글
- **한글 UI** — "모델 셀렉터", "현재 모델", "설치중", "남은 공간", "사용 가능한 모델", "기본 모델 복원", 탭 라벨 "모델"
- **자동 bootstrap** — `#pageModels` 가 없으면 `/models/page_models.html` 프래그먼트를 `#swipeContainer` 에 주입 (idempotent)
- **PAGE_ELEMENTS 통합** — `showPage("models")` 로 다른 탭과 동일하게 동작, 탭 이탈 시 정상적으로 숨김 처리
