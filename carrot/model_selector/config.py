"""Paths, filenames, and constants for the Carrot model selector."""
from __future__ import annotations

from pathlib import Path

# Storage locations
MODELS_DIR = Path("/data/models")
MODELS_TMP_DIR = Path("/data/models_tmp")
MODELS_BACKUP_DIR = Path("/data/models_backup")
COMPILE_STATUS_FILE = Path("/data/model_compile_status")

# 컴파일 환경 스탬프 — /data/models/.compile_env 내용이 현재 태그와 다르면
# boot_compile 이 보존된 onnx 로 자동 재컴파일한다 (구엔진에서 설치한 모델이
# 로드 실패 → 격리되는 대신 부팅 시 재빌드되도록).
COMPILE_ENV_STAMP_NAME = ".compile_env"
# 재컴파일 실패 시 같은 태그로 매 부팅 재시도(부팅 지연)하지 않도록 남기는 마커.
RECOMPILE_FAILED_MARKER_NAME = ".recompile_failed"

# 태그는 git 트리에서 자동 도출한다: 아래 경로에 닿는 커밋이 (자동 체리픽으로
# 들어오더라도) 태그를 저절로 바꿔 재컴파일을 트리거한다 — 사람이 태그를
# 올려줄 필요가 없다. 컴파일 산출물(pkl) 호환성을 결정하는 경로만 나열할 것.
_COMPILE_ENV_PATHS = (
    "tinygrad_repo",
    "openpilot/selfdrive/modeld/compile_modeld.py",
    "openpilot/selfdrive/modeld/helpers.py",
    "openpilot/selfdrive/modeld/constants.py",
    "openpilot/selfdrive/modeld/SConscript",
)
# git 을 못 쓰는 환경(.git 이 없는 배포본 등)에서의 폴백 — 그 환경에서만 수동 관리.
_COMPILE_ENV_TAG_FALLBACK = "2026.07-tg-oob1"
_compile_env_tag_cache: str | None = None


def compile_env_tag() -> str:
    global _compile_env_tag_cache
    if _compile_env_tag_cache is None:
        _compile_env_tag_cache = _derive_compile_env_tag()
    return _compile_env_tag_cache


def _derive_compile_env_tag() -> str:
    import hashlib
    import subprocess
    repo_root = Path(__file__).resolve().parents[2]
    try:
        out = subprocess.run(
            ["git", "-C", str(repo_root), "rev-parse", *[f"HEAD:{p}" for p in _COMPILE_ENV_PATHS]],
            capture_output=True, text=True, timeout=10, check=True,
        )
        digest = hashlib.sha256(out.stdout.encode()).hexdigest()[:16]
        return f"tg-env:{digest}"
    except Exception:
        return _COMPILE_ENV_TAG_FALLBACK

# Default built-in model directory (fallback when no custom model is installed)
# NOTE: carrot-ms 트리는 실제 소스가 openpilot/ 네임스페이스 하위에 있다
# (selfdrive → openpilot/selfdrive).
OPENPILOT_ROOT = Path("/data/openpilot")
DEFAULT_MODEL_DIR = OPENPILOT_ROOT / "openpilot" / "selfdrive" / "modeld" / "models"

# Remote manifest — 이중화:
#  - models_v4.json: 전체 카탈로그 (v4+ 셀렉터용 마스터, supercombo 등 신형 파일명 포함)
#  - models.json:    레거시 파일명 모델만 (v3 이하 구버전용으로 동결)
# 구버전(v3) manifest 파서는 minimum_selector_version 게이트 "이전에" 파일명을
# 검사하고 미지의 파일명이 하나라도 있으면 목록 전체를 실패시키므로, 신형
# 파일명이 든 항목은 models.json 에 절대 실으면 안 된다. openpilot-models
# 저장소의 scripts/update_models.py 가 두 파일을 자동 분리 생성·서명한다.
_MODELS_JSON_BASE = "https://raw.githubusercontent.com/happymaj11r/openpilot-models/main"
MODELS_JSON_URL = f"{_MODELS_JSON_BASE}/models_v4.json"
# models_v4.json fetch 실패 시(파일 부재/일시 오류) 레거시 manifest 로 폴백.
MODELS_JSON_FALLBACK_URL = f"{_MODELS_JSON_BASE}/models.json"
# str.startswith가 튜플을 받으므로 downloader의 검증 로직 변경 없이 동작.
# releases/download 프리픽스는 95MB 초과 파일(GitHub raw 호스팅 불가, 예: Giga의
# driving_vision.onnx 122MB)이 Release 에셋으로 배포되는 경우를 위해 필요.
ALLOWED_URL_PREFIX = (
    "https://raw.githubusercontent.com/happymaj11r/openpilot-models/",
    "https://github.com/happymaj11r/openpilot-models/releases/download/",
)

# Allowed onnx filenames for download (allowlist)
ALLOWED_ONNX_FILES = frozenset({
    "driving_vision.onnx",
    "driving_policy.onnx",
    "driving_on_policy.onnx",
    "driving_off_policy.onnx",
    "driving_supercombo.onnx",
})

# Base names that we compile (.onnx → _tinygrad.pkl + _metadata.pkl)
VISION_BASE = "driving_vision"
ON_POLICY_BASE = "driving_on_policy"
POLICY_BASE = "driving_policy"
OFF_POLICY_BASE = "driving_off_policy"

# New-architecture (lebowski) single-onnx model: compile_modeld.py bundles
# metadata + model JIT + per-resolution warp JITs into one pkl that the
# upstream modeld engine loads directly.
SUPERCOMBO_BASE = "driving_supercombo"
SUPERCOMBO_PKL_NAME = "driving_tinygrad.pkl"

# Env var honored by openpilot/selfdrive/modeld/helpers.py::modeld_pkl_path() to load
# the unified pkl from a custom directory instead of the built-in models dir.
MODELD_MODELS_DIR_ENV = "MODELD_MODELS_DIR"

# Params keys
PARAM_DRIVING_MODEL_NAME = "DrivingModelName"
PARAM_PENDING_MODEL_NAME = "PendingModelName"

# tinygrad compile flags (must match openpilot/selfdrive/modeld/SConscript)
TINYGRAD_COMPILE_ENV_QCOM = {
    "DEV": "QCOM",
    "FLOAT16": "1",
    "NOLOCALS": "1",
    "JIT_BATCH_SIZE": "0",
    "IMAGE": "1",
    "OPENPILOT_HACKS": "1",
}
TINYGRAD_COMPILE_ENV_FALLBACK = {
    "DEV": "CPU:LLVM",
    # THREADS=0 은 SConscript 에 없는 의도적 추가 (PC 폴백 컴파일 안정성용).
    # 커널 구조는 pkl 에 구워지므로 런타임과 충돌하지 않는다.
    "THREADS": "0",
}

# Model ID validation (matches model_manager.cc isValidModelId())
MODEL_ID_REGEX = r"^[A-Za-z0-9_\-\s]{1,64}$"
