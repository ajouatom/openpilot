"""Tiny hook called once from `system/manager/manager.py::main()` so the
upstream patch stays a single line:

    from openpilot.carrot.model_selector.boot_compile import run as _ms_boot
    _ms_boot()

All heavy lifting lives in `installer.compile_pending`.  This wrapper exists
so the patch site doesn't have to import anything heavy (e.g. tinygrad) until
a pending model is actually present.
"""
from __future__ import annotations

from openpilot.common.swaglog import cloudlog

from .config import MODELS_TMP_DIR


def run() -> None:
    if not MODELS_TMP_DIR.exists():
        return
    try:
        from .installer import compile_pending
        compile_pending()
    except Exception as e:
        # Never crash manager on install errors — installer already restores
        # the backup and wipes the tmp dir on failure.
        cloudlog.error(f"model_selector boot_compile: {e}")
