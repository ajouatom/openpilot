"""Compile a pending model at boot and atomically swap it into /data/models.

Invoked from `boot_compile.run()` in `manager.main()`.  Does the equivalent of
the upstream c3-ms `manager.py::compile_pending_model()` but lives here so the
manager patch stays a one-liner.
"""
from __future__ import annotations

import os
import pickle
import shutil
import subprocess
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
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
LEGACY_WARP_SCRIPT = OPENPILOT_ROOT / "carrot" / "model_selector" / "compile_legacy_warp.py"
CAMERA_CONFIGS = (
    (_ar_ox_fisheye.width, _ar_ox_fisheye.height),  # tici: 1928x1208
    (_os_fisheye.width, _os_fisheye.height),        # mici: 1344x760
)


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


def _model_input_size(tmp_dir: Path) -> tuple[int, int]:
    metadata_path = tmp_dir / f"{VISION_BASE}_metadata.pkl"
    with open(metadata_path, "rb") as f:
        metadata = pickle.load(f)
    img_shape = metadata["input_shapes"].get("img")
    if img_shape is None or len(img_shape) != 4:
        raise InstallError(f"cannot infer model input size from {metadata_path}: img={img_shape}")
    # img shape is (batch, channels, h/2, w/2) after NV12 packing.
    return int(img_shape[3]) * 2, int(img_shape[2]) * 2


def _compile_legacy_warp_pkls(tmp_dir: Path, env: dict[str, str]) -> None:
    model_w, model_h = _model_input_size(tmp_dir)
    for cam_w, cam_h in CAMERA_CONFIGS:
        output = tmp_dir / f"warp_{cam_w}x{cam_h}_tinygrad.pkl"
        cloudlog.warning(f"model_selector: compiling model-specific legacy warp {cam_w}x{cam_h} -> {model_w}x{model_h}")
        _run([
            "python3",
            str(LEGACY_WARP_SCRIPT),
            "--camera-resolution",
            f"{cam_w}x{cam_h}",
            "--model-size",
            f"{model_w}x{model_h}",
            "--output",
            str(output),
        ], env=env)


def _ensure_warp_pkls(tmp_dir: Path, env: dict[str, str]) -> None:
    # Do not reuse built-in warp pkls here. Warp output shape is tied to the
    # selected model's image input shape, so copying built-in artifacts can break
    # custom models whose input dimensions differ. New upstream modeld embeds
    # warps in a unified pkl; Carrot's split-model runner still needs standalone
    # warp_* pkls, so compile them from the selected model metadata every time.
    _compile_legacy_warp_pkls(tmp_dir, env)


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

        _ensure_warp_pkls(MODELS_TMP_DIR, env)

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
