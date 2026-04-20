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
   - `selfdrive/carrot/web/index.html` — 네비 버튼 + page_models.js 로드

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
| `config.py` | 경로·파일명 상수, 허용 onnx 목록 |
| `keys.py` | Ed25519 공개키 맵 + selector version |
| `manifest.py` | GitHub `models.json` fetch, canonical JSON Ed25519 서명 검증 |
| `downloader.py` | ONNX 스트리밍 다운로드 + SHA256/크기 검증 |
| `validator.py` | `/data/models/` 파일 세트 유효성 검증 |
| `installer.py` | tinygrad 컴파일 (compile3) + warp pkl 복사 + atomic swap |
| `carrot_modeld.py` | **자체 modeld 엔진** — 2/3-모델 공용, `/data/models` 전담 |
| `carrot_parse_model_outputs.py` | 자체 parser (c3-ms 이식, `parse_off_policy_outputs` 포함) |
| `modeld_runner.py` | 부팅 시 커스텀/기본 선택해서 택일 실행 (dispatcher) |
| `boot_compile.py` | `manager.main` 초입 훅, pending 모델 컴파일 트리거 |
| `jobs.py` | 간단한 in-memory job runner (web 진행률 폴링) |
| `web/routes.py` | aiohttp 라우터 (list / status / install / job / apply / reset) |
| `web/frontend/page_models.html` | carrot web 페이지 템플릿 |
| `web/frontend/page_models.js` | 프런트엔드 로직 (fragment 자동 주입 + Job 폴링) |

## Params

| 이름 | 타입 | 의미 |
|------|------|------|
| `DrivingModelName` | STRING | 현재 사용 중인 모델 표시용 |
| `PendingModelName` | STRING | 재부팅 후 컴파일 대기 중인 모델 ID |

## 데이터 흐름

```
[Web UI] → /api/models/* → jobs queue → downloader → /data/models_tmp/
                                         ↓
                                  manifest/서명 검증
                                         ↓
                                  PendingModelName 설정
                                         ↓
                                  5초 후 reboot
                                         ↓
[boot_compile.run()] → installer.compile_pending() → tinygrad + warp 컴파일
                                         ↓
                         atomic swap: /data/models_tmp → /data/models
                                         ↓
[modeld_runner] ──┬── /data/models 유효? ── yes ─→ carrot_modeld.main()  (3-모델)
                  └── 아님                    ─→ upstream modeld.main()  (기본)
```
