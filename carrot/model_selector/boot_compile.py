"""Tiny hook called once from `system/manager/manager.py::main()` so the
upstream patch stays a single line:

    from openpilot.carrot.model_selector.boot_compile import run as _ms_boot
    _ms_boot()

All heavy lifting lives in `installer`.  This wrapper exists so the patch
site doesn't have to import anything heavy (e.g. tinygrad) unless a pending
model is present or the installed model needs a recompile.
"""
from __future__ import annotations

from openpilot.common.swaglog import cloudlog

from .config import (
    COMPILE_ENV_STAMP_NAME,
    MODELS_DIR,
    MODELS_TMP_DIR,
    RECOMPILE_FAILED_MARKER_NAME,
    compile_env_tag,
)


def _tag_file_matches(path) -> bool:
    try:
        return path.read_text().strip() == compile_env_tag()
    except (OSError, ValueError):
        # 부재/손상(비 UTF-8) 파일은 불일치로 취급한다.
        return False


def run() -> None:
    try:
        if MODELS_TMP_DIR.exists():
            from .installer import compile_pending
            compile_pending()
            # 여기서 return 하지 않는다 — 성공이면 새 /data/models 의 스탬프가
            # 일치해 아래 검사가 no-op 이고, 실패 정리로 끝났으면 같은 부팅에서
            # 바로 복구 재컴파일을 시도해야 한다 (안 그러면 그 부팅 동안 stale
            # pkl 이 modeld 에 노출되어 크래시루프 → 격리로 이어진다).

        # 설치된 커스텀 모델이 구 컴파일 환경(다른 tinygrad/pkl 포맷)에서
        # 빌드된 경우 보존된 onnx 로 부팅 시 자동 재컴파일한다.
        # 스탬프 일치(정상) 또는 실패 마커 존재(재시도 무의미) 시에는
        # heavy import 없이 바로 통과한다.
        stamp_ok = _tag_file_matches(MODELS_DIR / COMPILE_ENV_STAMP_NAME)
        already_failed = _tag_file_matches(MODELS_DIR / RECOMPILE_FAILED_MARKER_NAME)
        if MODELS_DIR.is_dir() and not stamp_ok and not already_failed:
            from .installer import recompile_stale_if_needed
            recompile_stale_if_needed()
    except Exception as e:
        # Never crash manager on install errors — installer already restores
        # the backup and wipes the tmp dir on failure.
        cloudlog.error(f"model_selector boot_compile: {e}")
