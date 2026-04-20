"""Compile a pending model at boot and atomically swap it into /data/models.

Invoked from `boot_compile.run()` in `manager.main()`.  Does the equivalent of
the upstream c3-ms `manager.py::compile_pending_model()` but lives here so the
manager patch stays a one-liner.
"""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import TICI

from .config import (
    DEFAULT_MODEL_DIR,
    MODELS_BACKUP_DIR,
    MODELS_DIR,
    MODELS_TMP_DIR,
    OFF_POLICY_BASE,
    ON_POLICY_BASE,
    OPENPILOT_ROOT,
    PARAM_DRIVING_MODEL_NAME,
    PARAM_PENDING_MODEL_NAME,
    POLICY_BASE,
    TINYGRAD_COMPILE_ENV_FALLBACK,
    TINYGRAD_COMPILE_ENV_QCOM,
    VISION_BASE,
)


METADATA_SCRIPT = OPENPILOT_ROOT / "selfdrive" / "modeld" / "get_model_metadata.py"
COMPILE3_SCRIPT = OPENPILOT_ROOT / "tinygrad_repo" / "examples" / "openpilot" / "compile3.py"
COMPILE_WARP_SCRIPT = OPENPILOT_ROOT / "selfdrive" / "modeld" / "compile_warp.py"
BUILTIN_MODELS_DIR = OPENPILOT_ROOT / "selfdrive" / "modeld" / "models"


class InstallError(Exception):
    pass


def _tinygrad_env() -> dict[str, str]:
    env = os.environ.copy()
    flags = TINYGRAD_COMPILE_ENV_QCOM if TICI else TINYGRAD_COMPILE_ENV_FALLBACK
    env.update(flags)
    return env


def _run(cmd: list[str], env: dict[str, str] | None = None) -> None:
    result = subprocess.run(
        cmd, cwd=str(OPENPILOT_ROOT), env=env, capture_output=True, check=False
    )
    if result.returncode != 0:
        stderr = result.stderr.decode(errors="replace").strip()
        raise InstallError(f"{' '.join(cmd)} failed: {stderr}")


def _bases_to_compile(tmp_dir: Path) -> list[str]:
    bases: list[str] = [VISION_BASE]
    has_on = (tmp_dir / f"{ON_POLICY_BASE}.onnx").exists()
    has_policy = (tmp_dir / f"{POLICY_BASE}.onnx").exists()
    if has_on:
        bases.append(ON_POLICY_BASE)
    elif has_policy:
        bases.append(POLICY_BASE)
    else:
        raise InstallError(
            "pending model has neither driving_on_policy.onnx nor driving_policy.onnx"
        )
    if (tmp_dir / f"{OFF_POLICY_BASE}.onnx").exists():
        bases.append(OFF_POLICY_BASE)
    return bases


def _compile_one(base: str, tmp_dir: Path, env: dict[str, str]) -> None:
    onnx = tmp_dir / f"{base}.onnx"
    pkl = tmp_dir / f"{base}_tinygrad.pkl"
    meta = tmp_dir / f"{base}_metadata.pkl"
    cloudlog.warning(f"model_selector: generating metadata for {base}")
    _run(["python3", str(METADATA_SCRIPT), str(onnx), str(meta)], env=env)
    cloudlog.warning(f"model_selector: compiling {base} with tinygrad")
    _run(["python3", str(COMPILE3_SCRIPT), str(onnx), str(pkl)], env=env)


def _copy_warp_pkls(tmp_dir: Path) -> None:
    # warp depends only on camera geometry; copy the built-in pkls so that a
    # swapped-in /data/models directory is self-contained.
    for warp in BUILTIN_MODELS_DIR.glob("warp_*_tinygrad.pkl"):
        shutil.copy2(warp, tmp_dir / warp.name)


def _atomic_swap(tmp_dir: Path) -> None:
    if MODELS_BACKUP_DIR.exists():
        shutil.rmtree(MODELS_BACKUP_DIR)
    if MODELS_DIR.exists():
        MODELS_DIR.rename(MODELS_BACKUP_DIR)
    tmp_dir.rename(MODELS_DIR)
    # Swap succeeded — backup can go.
    if MODELS_BACKUP_DIR.exists():
        shutil.rmtree(MODELS_BACKUP_DIR, ignore_errors=True)


def _restore_backup_if_needed() -> None:
    if MODELS_BACKUP_DIR.exists() and not MODELS_DIR.exists():
        MODELS_BACKUP_DIR.rename(MODELS_DIR)
        cloudlog.warning("model_selector: restored previous model from backup")


def compile_pending() -> None:
    """Called at boot by `boot_compile.run()`.  No-op when nothing is pending.

    Behaviour mirrors the c3-ms installer:
      * validate ONNX files in /data/models_tmp
      * generate metadata.pkl + tinygrad.pkl for each model
      * compile warp pkls
      * atomic swap tmp → /data/models
      * write DrivingModelName, clear PendingModelName
      * on any failure, wipe /data/models_tmp and restore backup
    """
    if not MODELS_TMP_DIR.exists():
        return

    vision = MODELS_TMP_DIR / f"{VISION_BASE}.onnx"
    on_policy = MODELS_TMP_DIR / f"{ON_POLICY_BASE}.onnx"
    policy = MODELS_TMP_DIR / f"{POLICY_BASE}.onnx"
    if not vision.exists() or not (on_policy.exists() or policy.exists()):
        cloudlog.warning("model_selector: tmp dir incomplete, cleaning up")
        shutil.rmtree(MODELS_TMP_DIR, ignore_errors=True)
        return

    params = Params()
    model_name = params.get(PARAM_PENDING_MODEL_NAME)
    if not model_name:
        cloudlog.warning("model_selector: PendingModelName empty, cleaning up")
        shutil.rmtree(MODELS_TMP_DIR, ignore_errors=True)
        return

    env = _tinygrad_env()

    try:
        bases = _bases_to_compile(MODELS_TMP_DIR)
        cloudlog.warning(f"model_selector: compiling {bases}")
        for base in bases:
            _compile_one(base, MODELS_TMP_DIR, env)

        cloudlog.warning("model_selector: compiling warp")
        _run(["python3", str(COMPILE_WARP_SCRIPT)], env=env)
        _copy_warp_pkls(MODELS_TMP_DIR)

        cloudlog.warning("model_selector: installing")
        _atomic_swap(MODELS_TMP_DIR)

        params.put(PARAM_DRIVING_MODEL_NAME, model_name)
        params.remove(PARAM_PENDING_MODEL_NAME)
        cloudlog.warning(f"model_selector: installed {model_name}")
    except Exception as e:
        cloudlog.error(f"model_selector: install failed — {e}")
        shutil.rmtree(MODELS_TMP_DIR, ignore_errors=True)
        _restore_backup_if_needed()
        params.remove(PARAM_PENDING_MODEL_NAME)


def reset_to_default() -> None:
    """Remove /data/models and clear params so the built-in fallback is used."""
    if MODELS_DIR.exists():
        shutil.rmtree(MODELS_DIR, ignore_errors=True)
    if MODELS_TMP_DIR.exists():
        shutil.rmtree(MODELS_TMP_DIR, ignore_errors=True)
    params = Params()
    params.remove(PARAM_DRIVING_MODEL_NAME)
    params.remove(PARAM_PENDING_MODEL_NAME)


__all__ = [
    "InstallError",
    "compile_pending",
    "reset_to_default",
    "DEFAULT_MODEL_DIR",
]
