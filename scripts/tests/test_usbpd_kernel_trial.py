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


@pytest.mark.parametrize("device", ["c3", "tici", "tizi", "mici"])
def test_supported_sdm845_devices(monkeypatch, device: str):
  monkeypatch.setattr(trial, "device_type", lambda: device)

  assert trial.require_supported_device() == device


def test_supported_device_rejects_unknown_hardware(monkeypatch):
  monkeypatch.setattr(trial, "device_type", lambda: "unknown")

  with pytest.raises(trial.TrialError, match="C3/C3 clone/C3X/C4"):
    trial.require_supported_device()


@pytest.mark.parametrize("device, expected", [
  ("c3", "boot-c3-c3clone.img"),
  ("tici", "boot-c3-c3clone.img"),
  ("tizi", "boot-c3x-c4.img"),
  ("mici", "boot-c3x-c4.img"),
])
def test_boot_asset_follows_existing_device_split(device: str, expected: str):
  assert trial.boot_asset(device) == expected


def test_v3_release_isolated_from_earlier_trials():
  assert trial.RELEASE_TAG == "usbpd-test-v3-agnos19.6"
  assert trial.RELEASE_TAG in trial.release_paths("mici")[0]
  assert trial.DATA_DIR == Path("/data/usbpd-kernel-v3")


def test_ensure_confirms_successful_trial_boot(monkeypatch, tmp_path: Path):
  state_path = tmp_path / "state.json"
  state_path.write_text("{}", encoding="utf-8")
  expected_hash = "a" * 64
  monkeypatch.setattr(trial, "STATE_PATH", state_path)
  monkeypatch.setattr(trial, "require_root", lambda: None)
  monkeypatch.setattr(trial, "require_supported_device", lambda: "mici")
  monkeypatch.setattr(trial, "check_version", lambda: None)
  monkeypatch.setattr(trial, "current_slot", lambda: "_b")
  monkeypatch.setattr(trial, "load_state", lambda: {
    "phase": "ready",
    "trial_slot": "_b",
    "boot_size": 123,
    "boot_sha256": expected_hash,
  })
  monkeypatch.setattr(trial, "partition_path", lambda _name, _slot: tmp_path / "boot")
  monkeypatch.setattr(trial, "sha256_file", lambda _path, _size=None: expected_hash)
  confirmed = []
  monkeypatch.setattr(trial, "confirm", lambda: confirmed.append(True))

  assert trial.ensure() == 0
  assert confirmed == [True]


def test_boot_check_leaves_successful_trial_pending_for_health_confirmation(monkeypatch, tmp_path: Path):
  state_path = tmp_path / "state.json"
  state_path.write_text("{}", encoding="utf-8")
  expected_hash = "a" * 64
  monkeypatch.setattr(trial, "STATE_PATH", state_path)
  monkeypatch.setattr(trial, "require_root", lambda: None)
  monkeypatch.setattr(trial, "require_supported_device", lambda: "mici")
  monkeypatch.setattr(trial, "check_version", lambda: None)
  monkeypatch.setattr(trial, "current_slot", lambda: "_b")
  monkeypatch.setattr(trial, "load_state", lambda: {
    "phase": "ready",
    "trial_slot": "_b",
    "boot_size": 123,
    "boot_sha256": expected_hash,
  })
  monkeypatch.setattr(trial, "partition_path", lambda _name, _slot: tmp_path / "boot")
  monkeypatch.setattr(trial, "sha256_file", lambda _path, _size=None: expected_hash)
  confirmed = []
  monkeypatch.setattr(trial, "confirm", lambda: confirmed.append(True))

  assert trial.ensure_boot() == trial.ENSURE_CONFIRM_PENDING
  assert confirmed == []


@pytest.mark.parametrize("slot_number, expected", [(0, "_a"), (1, "_b")])
def test_prepared_agnos_installs_matching_boot_before_first_reboot(monkeypatch, slot_number: int, expected: str):
  calls = []
  monkeypatch.setattr(
    trial,
    "install",
    lambda *, boot_install, prepared_target: calls.append((boot_install, prepared_target)),
  )

  trial.install_prepared_agnos(slot_number)

  assert calls == [(True, expected)]


def test_prepared_agnos_path_does_not_require_current_slot_version_or_offroad(monkeypatch, tmp_path: Path):
  class PreparedSlotReached(Exception):
    pass

  monkeypatch.setattr(trial, "STATE_PATH", tmp_path / "missing-state.json")
  monkeypatch.setattr(trial, "require_root", lambda: None)
  monkeypatch.setattr(trial, "require_offroad", lambda: pytest.fail("boot integration must run before manager"))
  monkeypatch.setattr(trial, "require_supported_device", lambda: "mici")
  monkeypatch.setattr(trial, "check_version", lambda: pytest.fail("the active slot can still run the previous AGNOS"))
  monkeypatch.setattr(trial, "current_slot", lambda: "_a")
  monkeypatch.setattr(
    trial,
    "verify_inactive_agnos",
    lambda slot, device: (_ for _ in ()).throw(PreparedSlotReached((slot, device))),
  )

  with pytest.raises(PreparedSlotReached, match="_b.*mici"):
    trial.install(boot_install=True, prepared_target="_b")


def test_ensure_reactivates_unbooted_trial_in_same_boot(monkeypatch, tmp_path: Path):
  state_path = tmp_path / "state.json"
  state_path.write_text("{}", encoding="utf-8")
  monkeypatch.setattr(trial, "STATE_PATH", state_path)
  monkeypatch.setattr(trial, "require_root", lambda: None)
  monkeypatch.setattr(trial, "require_supported_device", lambda: "tici")
  monkeypatch.setattr(trial, "check_version", lambda: None)
  monkeypatch.setattr(trial, "current_slot", lambda: "_a")
  monkeypatch.setattr(trial, "current_boot_id", lambda: "same-boot")
  monkeypatch.setattr(trial, "load_state", lambda: {
    "phase": "ready",
    "previous_slot": "_a",
    "trial_slot": "_b",
    "install_boot_id": "same-boot",
  })
  activated = []
  monkeypatch.setattr(trial, "activate", lambda: activated.append(True))

  assert trial.ensure() == trial.ENSURE_REBOOT_REQUIRED
  assert activated == [True]


def test_read_diagnostic_files_preserves_binary_data(tmp_path: Path):
  diagnostic = tmp_path / "console-ramoops"
  diagnostic.write_bytes(b"usbpd\x00 state\xff")

  output = trial.read_diagnostic_files([diagnostic])

  assert str(diagnostic) in output
  assert "usbpd" in output
  assert "state" in output
  assert "\\x00" in output
  assert "\x00" not in output
