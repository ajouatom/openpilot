from pathlib import Path

import pytest

from scripts import usbpd_kernel_trial as trial


@pytest.mark.parametrize("model, expected", [
  ("comma mici\x00", "mici"),
  ("comma tici\x00", "tici"),
])
def test_device_type(monkeypatch, tmp_path: Path, model: str, expected: str):
  model_path = tmp_path / "model"
  model_path.write_text(model, encoding="utf-8")
  monkeypatch.setattr(trial, "DEVICE_MODEL_PATH", model_path)

  assert trial.device_type() == expected


def test_require_c4_rejects_non_mici(monkeypatch):
  monkeypatch.setattr(trial, "device_type", lambda: "tici")

  with pytest.raises(trial.TrialError, match="comma four"):
    trial.require_c4()


def test_v2_release_isolated_from_v1():
  assert trial.RELEASE_TAG == "usbpd-test-v2-c4"
  assert trial.RELEASE_TAG in trial.BOOT_URL
  assert trial.DATA_DIR == Path("/data/usbpd-kernel-v2")


def test_read_diagnostic_files_preserves_binary_data(tmp_path: Path):
  diagnostic = tmp_path / "console-ramoops"
  diagnostic.write_bytes(b"usbpd state\xff")

  output = trial.read_diagnostic_files([diagnostic])

  assert str(diagnostic) in output
  assert "usbpd state" in output
