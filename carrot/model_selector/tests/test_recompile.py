from __future__ import annotations

import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest


# Keep these state-machine tests runnable without the native/full openpilot
# runtime. The production modules only need these four lightweight interfaces
# for the paths exercised below.
class _Cloudlog:
  def __getattr__(self, _name):
    return lambda *args, **kwargs: None


swaglog_stub = types.ModuleType("openpilot.common.swaglog")
swaglog_stub.cloudlog = _Cloudlog()

params_stub = types.ModuleType("openpilot.common.params")
params_stub.Params = object

camera_stub = types.ModuleType("openpilot.common.transformations.camera")
camera_stub._ar_ox_fisheye = SimpleNamespace(width=1928, height=1208)
camera_stub._os_fisheye = SimpleNamespace(width=1344, height=760)

hardware_stub = types.ModuleType("openpilot.system.hardware")
hardware_stub.TICI = False

_missing = object()
_stubs = {
  "openpilot.common.swaglog": swaglog_stub,
  "openpilot.common.params": params_stub,
  "openpilot.common.transformations.camera": camera_stub,
  "openpilot.system.hardware": hardware_stub,
}
_saved_modules = {name: sys.modules.get(name, _missing) for name in _stubs}
try:
  sys.modules.update(_stubs)
  from carrot.model_selector import config, installer, modeld_runner
finally:
  for name, previous in _saved_modules.items():
    if previous is _missing:
      sys.modules.pop(name, None)
    else:
      sys.modules[name] = previous
  for parent_name, child_name, module_name in (
    ("openpilot.common", "swaglog", "openpilot.common.swaglog"),
    ("openpilot.common", "params", "openpilot.common.params"),
    ("openpilot.common.transformations", "camera", "openpilot.common.transformations.camera"),
    ("openpilot.system", "hardware", "openpilot.system.hardware"),
  ):
    parent = sys.modules.get(parent_name)
    if parent is not None and _saved_modules[module_name] is _missing and getattr(parent, child_name, None) is _stubs[module_name]:
      delattr(parent, child_name)


CURRENT_TAG = "tg-env:test-current"


class FakeParams:
  values: dict[str, str] = {}

  def get(self, key: str):
    return self.values.get(key)

  def put(self, key: str, value: str) -> None:
    self.values[key] = value

  def remove(self, key: str) -> None:
    self.values.pop(key, None)


@pytest.fixture
def model_paths(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
  models = tmp_path / "models"
  pending = tmp_path / "models_tmp"
  backup = tmp_path / "models_backup"

  monkeypatch.setattr(installer, "MODELS_DIR", models)
  monkeypatch.setattr(installer, "MODELS_TMP_DIR", pending)
  monkeypatch.setattr(installer, "MODELS_BACKUP_DIR", backup)
  monkeypatch.setattr(installer, "Params", FakeParams)
  monkeypatch.setattr(installer, "compile_env_tag", lambda: CURRENT_TAG)
  monkeypatch.setattr(installer, "_tinygrad_env", lambda: {"DEV": "QCOM"})

  monkeypatch.setattr(config, "compile_env_tag", lambda: CURRENT_TAG)
  monkeypatch.setattr(modeld_runner, "CUSTOM_MODELS_DIR", models)
  monkeypatch.setattr(modeld_runner, "STATUS_FILE", str(tmp_path / "selector_status"))

  FakeParams.values = {}
  return models, pending, backup


def _write_supercombo(model_dir: Path, *, stamp: str | None, onnx: bool = False, pkl: bytes = b"old-pkl") -> None:
  model_dir.mkdir(parents=True, exist_ok=True)
  (model_dir / config.SUPERCOMBO_PKL_NAME).write_bytes(pkl)
  if onnx:
    (model_dir / f"{config.SUPERCOMBO_BASE}.onnx").write_bytes(b"onnx")
  if stamp is not None:
    (model_dir / config.COMPILE_ENV_STAMP_NAME).write_text(stamp)


def _write_legacy(model_dir: Path, stamp: str, *, onnx: bool = False) -> None:
  model_dir.mkdir(parents=True, exist_ok=True)
  for stem in (config.VISION_BASE, config.ON_POLICY_BASE):
    (model_dir / f"{stem}_tinygrad.pkl").write_bytes(b"pkl")
    (model_dir / f"{stem}_metadata.pkl").write_bytes(b"metadata")
    if onnx:
      (model_dir / f"{stem}.onnx").write_bytes(b"onnx")
  (model_dir / config.COMPILE_ENV_STAMP_NAME).write_text(stamp)


def test_compile_env_sidecar_match_is_fail_closed(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
  monkeypatch.setattr(config, "compile_env_tag", lambda: CURRENT_TAG)
  stamp = tmp_path / ".compile_env"

  assert not config.compile_env_tag_file_matches(stamp)
  stamp.write_text(CURRENT_TAG + "\n")
  assert config.compile_env_tag_file_matches(stamp)
  stamp.write_bytes(b"\xff")
  assert not config.compile_env_tag_file_matches(stamp)


def test_runner_uses_current_supercombo_and_legacy(model_paths):
  models, _, _ = model_paths
  _write_supercombo(models, stamp=CURRENT_TAG)
  assert modeld_runner._select_engine() == "upstream_custom"

  for path in models.iterdir():
    path.unlink()
  _write_legacy(models, CURRENT_TAG)
  assert modeld_runner._select_engine() == "carrot_legacy"


@pytest.mark.parametrize("stamp", [None, "tg-env:old"])
def test_runner_rejects_stale_custom_pickle_before_load(model_paths, stamp):
  models, _, _ = model_paths
  _write_supercombo(models, stamp=stamp)

  assert modeld_runner._select_engine() == "upstream_default"
  assert (models / config.SUPERCOMBO_PKL_NAME).read_bytes() == b"old-pkl"
  assert not (models / modeld_runner.LOAD_ATTEMPTS_NAME).exists()
  assert "stale compile environment" in Path(modeld_runner.STATUS_FILE).read_text()


def test_current_stamp_skips_recompile(model_paths, monkeypatch: pytest.MonkeyPatch):
  models, pending, _ = model_paths
  _write_supercombo(models, stamp=CURRENT_TAG, onnx=True)
  monkeypatch.setattr(installer, "compile_pending", lambda: pytest.fail("compile should not run"))

  installer.recompile_stale_if_needed()

  assert (models / config.SUPERCOMBO_PKL_NAME).read_bytes() == b"old-pkl"
  assert not pending.exists()


def test_stale_supercombo_recompiles_and_atomically_swaps(model_paths, monkeypatch: pytest.MonkeyPatch):
  models, pending, backup = model_paths
  _write_supercombo(models, stamp="tg-env:old", onnx=True)
  FakeParams.values[config.PARAM_DRIVING_MODEL_NAME] = "rdf-v4"
  compile_calls = []

  def compile_supercombo(tmp_dir: Path, env: dict[str, str]) -> None:
    compile_calls.append((tmp_dir, env["DEV"]))
    assert (tmp_dir / f"{config.SUPERCOMBO_BASE}.onnx").read_bytes() == b"onnx"
    (tmp_dir / config.SUPERCOMBO_PKL_NAME).write_bytes(b"new-pkl")

  monkeypatch.setattr(installer, "_compile_supercombo", compile_supercombo)
  installer.recompile_stale_if_needed()

  assert compile_calls == [(pending, "QCOM")]
  assert (models / config.SUPERCOMBO_PKL_NAME).read_bytes() == b"new-pkl"
  assert (models / f"{config.SUPERCOMBO_BASE}.onnx").read_bytes() == b"onnx"
  assert (models / config.COMPILE_ENV_STAMP_NAME).read_text().strip() == CURRENT_TAG
  assert FakeParams.values[config.PARAM_DRIVING_MODEL_NAME] == "rdf-v4"
  assert config.PARAM_PENDING_MODEL_NAME not in FakeParams.values
  assert not pending.exists()
  assert not backup.exists()
  assert modeld_runner._select_engine() == "upstream_custom"


def test_stale_legacy_model_recompiles_with_warps(model_paths, monkeypatch: pytest.MonkeyPatch):
  models, _, _ = model_paths
  _write_legacy(models, "tg-env:old", onnx=True)
  FakeParams.values[config.PARAM_DRIVING_MODEL_NAME] = "legacy-v3"
  compiled = []

  def compile_one(stem: str, tmp_dir: Path, env: dict[str, str]) -> None:
    compiled.append((stem, env["DEV"]))
    (tmp_dir / f"{stem}_tinygrad.pkl").write_bytes(b"new-pkl")
    (tmp_dir / f"{stem}_metadata.pkl").write_bytes(b"new-metadata")

  def compile_warps(tmp_dir: Path, _env: dict[str, str]) -> None:
    for width, height in installer.CAMERA_CONFIGS:
      (tmp_dir / f"warp_{width}x{height}_tinygrad.pkl").write_bytes(b"warp")

  monkeypatch.setattr(installer, "_compile_one", compile_one)
  monkeypatch.setattr(installer, "_ensure_warp_pkls", compile_warps)
  installer.recompile_stale_if_needed()

  assert compiled == [(config.VISION_BASE, "QCOM"), (config.ON_POLICY_BASE, "QCOM")]
  assert (models / config.COMPILE_ENV_STAMP_NAME).read_text().strip() == CURRENT_TAG
  assert (models / "tg_compiled_flags.json").read_text().strip() == '{"DEV": "QCOM"}'
  assert all((models / f"warp_{width}x{height}_tinygrad.pkl").is_file()
             for width, height in installer.CAMERA_CONFIGS)
  assert modeld_runner._select_engine() == "carrot_legacy"


def test_failed_recompile_preserves_old_model_and_falls_back(model_paths, monkeypatch: pytest.MonkeyPatch):
  models, pending, backup = model_paths
  _write_supercombo(models, stamp="tg-env:old", onnx=True)
  FakeParams.values[config.PARAM_DRIVING_MODEL_NAME] = "rdf-v4"

  def fail_compile(_tmp_dir: Path, _env: dict[str, str]) -> None:
    raise RuntimeError("compiler failed")

  monkeypatch.setattr(installer, "_compile_supercombo", fail_compile)
  installer.recompile_stale_if_needed()

  assert (models / config.SUPERCOMBO_PKL_NAME).read_bytes() == b"old-pkl"
  assert (models / config.COMPILE_ENV_STAMP_NAME).read_text() == "tg-env:old"
  assert (models / config.RECOMPILE_FAILED_MARKER_NAME).read_text().strip() == CURRENT_TAG
  assert not pending.exists()
  assert not backup.exists()
  assert modeld_runner._select_engine() == "upstream_default"


def test_missing_onnx_marks_failure_without_loading_stale_pickle(model_paths, monkeypatch: pytest.MonkeyPatch):
  models, _, _ = model_paths
  _write_supercombo(models, stamp="tg-env:old", onnx=False)
  monkeypatch.setattr(installer, "compile_pending", lambda: pytest.fail("compile should not run"))

  installer.recompile_stale_if_needed()

  assert (models / config.SUPERCOMBO_PKL_NAME).read_bytes() == b"old-pkl"
  assert (models / config.RECOMPILE_FAILED_MARKER_NAME).read_text().strip() == CURRENT_TAG
  assert modeld_runner._select_engine() == "upstream_default"


def test_new_compiler_tag_retries_after_previous_failure(model_paths, monkeypatch: pytest.MonkeyPatch):
  models, _, _ = model_paths
  _write_supercombo(models, stamp="tg-env:old", onnx=True)
  (models / config.RECOMPILE_FAILED_MARKER_NAME).write_text("tg-env:failed-version")
  next_tag = "tg-env:next-version"
  monkeypatch.setattr(installer, "compile_env_tag", lambda: next_tag)

  def compile_supercombo(tmp_dir: Path, _env: dict[str, str]) -> None:
    (tmp_dir / config.SUPERCOMBO_PKL_NAME).write_bytes(b"retried-pkl")

  monkeypatch.setattr(installer, "_compile_supercombo", compile_supercombo)
  installer.recompile_stale_if_needed()

  assert (models / config.SUPERCOMBO_PKL_NAME).read_bytes() == b"retried-pkl"
  assert (models / config.COMPILE_ENV_STAMP_NAME).read_text().strip() == next_tag
  assert not (models / config.RECOMPILE_FAILED_MARKER_NAME).exists()
