"""Compile a pending model at boot and atomically swap it into /data/models.

Invoked from `boot_compile.run()` in `manager.main()`.  Does the equivalent of
the upstream c3-ms `manager.py::compile_pending_model()` but lives here so the
manager patch stays a one-liner.
"""
from __future__ import annotations

import json
import os
import pickle
import platform
import shutil
import subprocess
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
from openpilot.system.hardware import TICI

from .config import (
    COMPILE_ENV_STAMP_NAME,
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
    RECOMPILE_FAILED_MARKER_NAME,
    SUPERCOMBO_BASE,
    SUPERCOMBO_PKL_NAME,
    TINYGRAD_COMPILE_ENV_FALLBACK,
    TINYGRAD_COMPILE_ENV_QCOM,
    VISION_BASE,
    compile_env_tag,
)
from .validator import is_valid_model_dir


METADATA_SCRIPT = OPENPILOT_ROOT / "openpilot" / "selfdrive" / "modeld" / "get_model_metadata.py"
COMPILE3_SCRIPT = OPENPILOT_ROOT / "tinygrad_repo" / "examples" / "openpilot" / "compile3.py"
COMPILE_MODELD_SCRIPT = OPENPILOT_ROOT / "openpilot" / "selfdrive" / "modeld" / "compile_modeld.py"
LEGACY_WARP_SCRIPT = OPENPILOT_ROOT / "carrot" / "model_selector" / "compile_legacy_warp.py"
CAMERA_CONFIGS = (
    (_ar_ox_fisheye.width, _ar_ox_fisheye.height),  # tici: 1928x1208
    (_os_fisheye.width, _os_fisheye.height),        # mici: 1344x760
)


class InstallError(Exception):
    pass


def _probe_tg_devices() -> set[str]:
    """Mirror SConscript's device probe (subprocess so device locks release)."""
    try:
        out = subprocess.run(
            ["python3", "-c",
             "from tinygrad import Device\nprint('\\n'.join(Device.get_available_devices()))"],
            capture_output=True, text=True, check=True, timeout=120,
            cwd=str(OPENPILOT_ROOT),
        )
        return set(out.stdout.strip().splitlines())
    except Exception:
        return set()


def _tinygrad_env() -> dict[str, str]:
    env = os.environ.copy()
    if TICI:
        flags = TINYGRAD_COMPILE_ENV_QCOM
    else:
        # Non-device (PC) compile: the runtime reads its tensor devices from the
        # build-time tg_input_devices.json (CUDA when available), so the compile
        # backend must match or the pickled JITs replay on the wrong device.
        available = _probe_tg_devices()
        if "CUDA" in available:
            flags = {"DEV": "CUDA"}
        elif platform.system() == "Darwin":
            flags = {"DEV": "CPU"}
        else:
            flags = TINYGRAD_COMPILE_ENV_FALLBACK
    env.update(flags)
    return env


# 컴파일 서브프로세스 안전 타임아웃 — manager.main() 이 프로세스 기동 전에
# 동기적으로 기다리므로, 서브프로세스가 행업하면 장치가 부팅 불능으로 멈춘다.
# 넉넉하게 잡고, 초과 시 InstallError 로 기존 실패 복구 경로에 태운다.
COMPILE_TIMEOUT_SECONDS = 30 * 60


def _run(cmd: list[str], env: dict[str, str] | None = None) -> None:
    try:
        result = subprocess.run(
            cmd, cwd=str(OPENPILOT_ROOT), env=env, capture_output=True, check=False,
            timeout=COMPILE_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired as e:
        raise InstallError(f"{' '.join(cmd)} timed out after {COMPILE_TIMEOUT_SECONDS}s") from e
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
    cloudlog.warning(f"model_selector: generating metadata for {base}")
    # get_model_metadata.py 는 출력 경로를 스스로 정한다: {onnx stem}_metadata.pkl
    _run(["python3", str(METADATA_SCRIPT), str(onnx)], env=env)
    cloudlog.warning(f"model_selector: compiling {base} with tinygrad")
    _run(["python3", str(COMPILE3_SCRIPT), str(onnx), str(pkl)], env=env)


def _load_metadata(tmp_dir: Path, base: str) -> dict:
    metadata_path = tmp_dir / f"{base}_metadata.pkl"
    with open(metadata_path, "rb") as f:
        return pickle.load(f)


def _model_input_size(tmp_dir: Path, base: str = VISION_BASE) -> tuple[int, int]:
    metadata = _load_metadata(tmp_dir, base)
    img_shape = metadata["input_shapes"].get("img")
    if img_shape is None or len(img_shape) != 4:
        raise InstallError(f"cannot infer model input size from {base} metadata: img={img_shape}")
    # img shape is (batch, channels, h/2, w/2) after NV12 packing.
    return int(img_shape[3]) * 2, int(img_shape[2]) * 2


def _validate_supercombo_metadata(metadata: dict) -> None:
    """Reject models the upstream runtime cannot run BEFORE installing them,
    so an incompatible download fails cleanly instead of crash-looping modeld."""
    from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE
    from openpilot.selfdrive.modeld.constants import ModelConstants

    input_shapes = metadata.get("input_shapes") or {}
    required = {"img", "big_img", "desire_pulse", "traffic_convention", "action_t", "features_buffer"}
    missing = sorted(required - input_shapes.keys())
    if missing:
        raise InstallError(f"supercombo model is missing required inputs: {missing}")

    # modeld.py 가 고정 길이 배열로 채우는 비이미지 입력들 — 길이가 다르면
    # 설치는 통과하지만 매 기동 broadcast 크래시(격리 폴백)하므로 미리 거른다.
    fixed_last_dims = {
        "desire_pulse": ModelConstants.DESIRE_LEN,
        "traffic_convention": ModelConstants.TRAFFIC_CONVENTION_LEN,
        "action_t": 2,  # modeld.py 는 [lat_action_t, long_action_t] 를 넣는다
        "features_buffer": ModelConstants.FEATURE_LEN,
    }
    for name, expected in fixed_last_dims.items():
        shape = input_shapes[name]
        if not shape or int(shape[-1]) != int(expected):
            raise InstallError(f"supercombo {name} shape {shape} incompatible (last dim must be {expected})")

    if tuple(input_shapes["img"]) != tuple(input_shapes["big_img"]):
        raise InstallError(
            f"img/big_img shape mismatch: {input_shapes['img']} vs {input_shapes['big_img']}"
        )

    # The runtime warp matrices (get_warp_matrix) are built from MEDMODEL
    # 512x256 intrinsics; a model with a different input frame would silently
    # get geometrically wrong camera inputs, so fail closed.
    img_shape = input_shapes["img"]
    model_size = (int(img_shape[3]) * 2, int(img_shape[2]) * 2)
    if model_size != tuple(MEDMODEL_INPUT_SIZE):
        raise InstallError(
            f"supercombo input size {model_size} != supported {tuple(MEDMODEL_INPUT_SIZE)}"
        )

    output_slices = metadata.get("output_slices") or {}
    if "hidden_state" not in output_slices:
        raise InstallError("supercombo model has no 'hidden_state' output slice (required by runtime)")


def _cleanup_stale_compile_artifacts(tmp_dir: Path) -> None:
    """compile_modeld.py unchunks the onnx to `{onnx}.unchunked` next to it and
    cleans it via atexit, which never runs if the compile is OOM-killed — drop
    leftovers first so they don't get swapped into /data/models."""
    try:
        for stale in tmp_dir.glob("*.unchunked"):
            stale.unlink(missing_ok=True)
    except Exception:
        pass


def _compile_supercombo(tmp_dir: Path, env: dict[str, str]) -> None:
    """Compile a new-architecture single-onnx model with the upstream
    compile_modeld pipeline.  The resulting pkl bundles metadata, the model
    JIT, and one warp JIT per camera resolution — the upstream modeld engine
    loads it as-is (no separate metadata/warp pkls needed)."""
    from openpilot.selfdrive.modeld.constants import ModelConstants

    onnx = tmp_dir / f"{SUPERCOMBO_BASE}.onnx"
    output = tmp_dir / SUPERCOMBO_PKL_NAME

    # Metadata pkl is only needed here for validation and input-size
    # derivation; the runtime reads metadata from inside the unified pkl.
    cloudlog.warning(f"model_selector: generating metadata for {SUPERCOMBO_BASE}")
    _run(["python3", str(METADATA_SCRIPT), str(onnx)], env=env)
    metadata = _load_metadata(tmp_dir, SUPERCOMBO_BASE)
    _validate_supercombo_metadata(metadata)
    model_w, model_h = _model_input_size(tmp_dir, base=SUPERCOMBO_BASE)
    _cleanup_stale_compile_artifacts(tmp_dir)

    # Must match the runtime: modeld.py derives frame_skip from ModelConstants
    # when building input queues, so the compiled JIT has to use the same value.
    frame_skip = ModelConstants.MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ

    cloudlog.warning(
        f"model_selector: compiling {SUPERCOMBO_BASE} with compile_modeld "
        f"(model {model_w}x{model_h}, frame_skip {frame_skip})"
    )
    cmd = [
        "python3", str(COMPILE_MODELD_SCRIPT),
        "--model-size", f"{model_w}x{model_h}",
        "--camera-resolutions", *[f"{w}x{h}" for w, h in CAMERA_CONFIGS],
        "--onnx", str(onnx),
        "--output", str(output),
        "--frame-skip", str(frame_skip),
    ]
    _run(cmd, env=env)
    if not output.exists() or output.stat().st_size == 0:
        raise InstallError(f"compile_modeld produced no output at {output}")


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
      * new architecture: driving_supercombo.onnx → unified driving_tinygrad.pkl
        (compile_modeld bundles metadata + model JIT + warp JITs)
      * legacy: generate metadata.pkl + tinygrad.pkl per model + warp pkls
      * atomic swap tmp → /data/models
      * write DrivingModelName, clear PendingModelName
      * on any failure, wipe /data/models_tmp and restore backup
    """
    if not MODELS_TMP_DIR.exists():
        return

    params = Params()
    model_name = params.get(PARAM_PENDING_MODEL_NAME)

    supercombo = MODELS_TMP_DIR / f"{SUPERCOMBO_BASE}.onnx"
    vision = MODELS_TMP_DIR / f"{VISION_BASE}.onnx"
    on_policy = MODELS_TMP_DIR / f"{ON_POLICY_BASE}.onnx"
    policy = MODELS_TMP_DIR / f"{POLICY_BASE}.onnx"
    has_legacy_set = vision.exists() and (on_policy.exists() or policy.exists())
    if not supercombo.exists() and not has_legacy_set:
        # A pending name without a complete file set is stale (e.g. a later
        # download attempt failed after wiping tmp) — clear it too, or the UI
        # shows "installing" forever.
        cloudlog.warning("model_selector: tmp dir incomplete, cleaning up")
        shutil.rmtree(MODELS_TMP_DIR, ignore_errors=True)
        params.remove(PARAM_PENDING_MODEL_NAME)
        return

    if not model_name:
        cloudlog.warning("model_selector: PendingModelName empty, cleaning up")
        shutil.rmtree(MODELS_TMP_DIR, ignore_errors=True)
        return

    env = _tinygrad_env()

    try:
        if supercombo.exists():
            _compile_supercombo(MODELS_TMP_DIR, env)
        else:
            bases = _bases_to_compile(MODELS_TMP_DIR)
            cloudlog.warning(f"model_selector: compiling {bases}")
            for base in bases:
                _compile_one(base, MODELS_TMP_DIR, env)

            _ensure_warp_pkls(MODELS_TMP_DIR, env)

            # carrot_modeld 가 기동 시 tinygrad DEV 백엔드를 컴파일 시점과 맞추기
            # 위해 읽는 파일. 온디바이스(QCOM)는 자동선택과 우연히 일치하지만,
            # CPU:LLVM 등 폴백 백엔드로 컴파일한 pkl 은 이 파일이 없으면 잘못된
            # 디바이스로 JIT 재생을 시도한다.
            (MODELS_TMP_DIR / "tg_compiled_flags.json").write_text(json.dumps({"DEV": env["DEV"]}) + "\n")

        # 스왑 전에 스탬프를 넣어 설치와 함께 원자적으로 반영한다.
        (MODELS_TMP_DIR / COMPILE_ENV_STAMP_NAME).write_text(compile_env_tag() + "\n")

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


def _read_text_or_empty(path: Path) -> str:
    try:
        return path.read_text().strip()
    except (OSError, ValueError):
        # ValueError 는 UnicodeDecodeError 포함 — 손상 스탬프/마커는 빈 값
        # (= 불일치) 취급해 재컴파일이 스스로 복구하게 한다.
        return ""


def _write_failed_marker() -> None:
    try:
        (MODELS_DIR / RECOMPILE_FAILED_MARKER_NAME).write_text(compile_env_tag() + "\n")
    except OSError:
        pass


def _has_compilable_onnx_set(model_dir: Path) -> bool:
    supercombo = model_dir / f"{SUPERCOMBO_BASE}.onnx"
    vision = model_dir / f"{VISION_BASE}.onnx"
    on_policy = model_dir / f"{ON_POLICY_BASE}.onnx"
    policy = model_dir / f"{POLICY_BASE}.onnx"
    return supercombo.exists() or (vision.exists() and (on_policy.exists() or policy.exists()))


def recompile_stale_if_needed() -> None:
    """구 컴파일 환경(다른 tinygrad/pkl 포맷)에서 빌드된 /data/models 를 감지해
    보존된 onnx 로 재컴파일한다.  스탬프가 현재 태그와 일치하면 no-op.

    onnx 세트가 없거나 직전 재컴파일이 같은 태그로 이미 실패했으면 손대지
    않는다 — 그 경우 modeld_runner 의 크래시루프 차단기(3회 후 격리)가
    폴백을 책임진다."""
    if MODELS_TMP_DIR.exists() or not is_valid_model_dir(MODELS_DIR):
        return
    tag = compile_env_tag()
    if _read_text_or_empty(MODELS_DIR / COMPILE_ENV_STAMP_NAME) == tag:
        return
    if _read_text_or_empty(MODELS_DIR / RECOMPILE_FAILED_MARKER_NAME) == tag:
        cloudlog.warning("model_selector: stale model dir, but recompile already failed for this tag — skipping")
        return
    if not _has_compilable_onnx_set(MODELS_DIR):
        cloudlog.error("model_selector: compile env changed but no onnx set preserved — leaving as-is")
        # 스스로 해소되지 않는 상태이므로 마커를 남겨 매 부팅 heavy import 를 피한다.
        _write_failed_marker()
        return

    params = Params()
    model_name = params.get(PARAM_DRIVING_MODEL_NAME) or "custom"
    cloudlog.warning(f"model_selector: compile env changed — recompiling {model_name!r} from preserved onnx")
    try:
        MODELS_TMP_DIR.mkdir(parents=True, exist_ok=True)
        for onnx in MODELS_DIR.glob("*.onnx"):
            shutil.copy2(onnx, MODELS_TMP_DIR / onnx.name)
        params.put(PARAM_PENDING_MODEL_NAME, model_name)
    except Exception:
        # 부분 복사된 tmp 를 남기면 다음 부팅이 pending 설치로 오인한다.
        shutil.rmtree(MODELS_TMP_DIR, ignore_errors=True)
        raise
    compile_pending()

    if _read_text_or_empty(MODELS_DIR / COMPILE_ENV_STAMP_NAME) != tag:
        # compile_pending 이 실패 복구(백업 복원/tmp 정리)까지 마친 뒤이므로,
        # 여기서는 매 부팅 재시도로 인한 부팅 지연만 막는다.
        _write_failed_marker()


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
    "recompile_stale_if_needed",
    "reset_to_default",
    "DEFAULT_MODEL_DIR",
]
