import importlib.util
import os
import sys
from pathlib import Path
from types import SimpleNamespace


MODULE_PATH = Path(__file__).resolve().parents[3] / "system" / "hardware" / "usbgpu.py"
SPEC = importlib.util.spec_from_file_location("usbgpu_test_module", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
usbgpu = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = usbgpu
SPEC.loader.exec_module(usbgpu)


def make_device(root: Path, vendor: str = "3801", speed: str = "5000",
                product: str = "custom ed4e39b7-CLEAN") -> Path:
  device = root / "4-1.4"
  device.mkdir()
  (device / "idVendor").write_text(vendor)
  (device / "idProduct").write_text("0001")
  (device / "speed").write_text(speed)
  (device / "manufacturer").write_text("tiny")
  (device / "product").write_text(product)
  (device / "busnum").write_text("4")
  (device / "devnum").write_text("7")
  return device


def test_usbgpu_status_distinguishes_link_and_runtime_states(tmp_path):
  assert usbgpu.usbgpu_status(False, False, False, False, tmp_path) == "not detected"

  device = make_device(tmp_path, speed="480")
  assert usbgpu.usbgpu_status(True, False, False, False, tmp_path) == "slow USB (480 Mbps)"

  (device / "speed").write_text("5000")
  assert usbgpu.usbgpu_status(False, False, False, False, tmp_path) == "model not compiled"
  assert usbgpu.usbgpu_status(True, True, False, False, tmp_path) == "loading"
  assert usbgpu.usbgpu_status(True, False, True, False, tmp_path) == "active"
  assert usbgpu.usbgpu_status(False, False, True, False, tmp_path, compile_pending=True) == "reboot to compile"
  assert usbgpu.usbgpu_status(True, False, False, True, tmp_path) == "startup failed"
  assert usbgpu.usbgpu_status(True, False, False, False, tmp_path) == "ready"


def test_usbgpu_status_checks_firmware(tmp_path):
  make_device(tmp_path, product="custom old-CLEAN")
  assert usbgpu.usbgpu_status(True, False, False, False, tmp_path) == "firmware mismatch"


def test_usbgpu_badge_state_prioritizes_failures_and_loading():
  assert usbgpu.usbgpu_badge_state(False, False, False, False) == "not_compiled"
  assert usbgpu.usbgpu_badge_state(True, False, False, False) == "ready"
  assert usbgpu.usbgpu_badge_state(True, False, True, False) == "active"
  assert usbgpu.usbgpu_badge_state(False, False, True, False, True) == "compile_pending"
  assert usbgpu.usbgpu_badge_state(True, True, True, False) == "loading"
  assert usbgpu.usbgpu_badge_state(True, True, True, True) == "error"


def test_check_usbgpu_reports_pcie_and_success(monkeypatch, tmp_path):
  make_device(tmp_path)

  failed = SimpleNamespace(returncode=1, stdout="", stderr="PCIe link not up (LTSSM=0x00)")
  monkeypatch.setattr(usbgpu.time, "sleep", lambda _seconds: None)
  monkeypatch.setattr(usbgpu.subprocess, "run", lambda *_args, **_kwargs: failed)
  assert usbgpu.check_usbgpu(tmp_path) == "12V / PCIe not ready"

  passed = SimpleNamespace(returncode=0, stdout="", stderr="")
  monkeypatch.setattr(usbgpu.subprocess, "run", lambda *_args, **_kwargs: passed)
  assert usbgpu.check_usbgpu(tmp_path) is None


def test_check_usbgpu_preserves_openpilot_and_existing_python_paths(monkeypatch, tmp_path):
  make_device(tmp_path)
  existing_paths = ["/data/openpilot/pydeps", "/custom/python"]
  monkeypatch.setenv("PYTHONPATH", os.pathsep.join(existing_paths))
  captured_env = {}
  real_run = usbgpu.subprocess.run

  def run(*_args, **kwargs):
    captured_env.update(kwargs["env"])
    return SimpleNamespace(returncode=0, stdout="", stderr="")

  monkeypatch.setattr(usbgpu.subprocess, "run", run)
  assert usbgpu.check_usbgpu(tmp_path) is None
  assert captured_env["PYTHONPATH"].split(os.pathsep) == [
    str(Path(usbgpu.BASEDIR) / "tinygrad_repo"),
    usbgpu.BASEDIR,
    *existing_paths,
  ]
  probe = real_run(
    [sys.executable, "-c", "from openpilot.common.usbgpu_bus_lock import usbgpu_bus_lock"],
    cwd=Path(usbgpu.BASEDIR) / "openpilot" / "system" / "manager",
    env=captured_env,
    capture_output=True,
    text=True,
    check=False,
  )
  assert probe.returncode == 0, probe.stderr


def test_check_usbgpu_retries_in_fresh_process(monkeypatch, tmp_path):
  make_device(tmp_path)
  results = iter((
    SimpleNamespace(returncode=1, stdout="", stderr="PCIe link not up (LTSSM=0x00)"),
    SimpleNamespace(returncode=0, stdout="", stderr=""),
  ))
  sleeps = []
  monkeypatch.setattr(usbgpu.subprocess, "run", lambda *_args, **_kwargs: next(results))
  monkeypatch.setattr(usbgpu.time, "sleep", lambda seconds: sleeps.append(seconds))

  assert usbgpu.check_usbgpu(tmp_path) is None
  assert sleeps == [usbgpu.USBGPU_CHECK_RETRY_INTERVAL]


def test_check_usbgpu_power_uses_firmware_rail_status(monkeypatch, tmp_path):
  make_device(tmp_path)

  monkeypatch.setattr(usbgpu, "get_usbgpu_power_status",
                      lambda: usbgpu.UsbGpuPowerStatus(12100, 850, False))
  assert usbgpu.check_usbgpu_power(tmp_path) is None

  monkeypatch.setattr(usbgpu, "get_usbgpu_power_status",
                      lambda: usbgpu.UsbGpuPowerStatus(120, 0, False))
  assert usbgpu.check_usbgpu_power(tmp_path) == "12V off (120 mV)"

  monkeypatch.setattr(usbgpu, "get_usbgpu_power_status",
                      lambda: usbgpu.UsbGpuPowerStatus(11900, 0, True))
  assert usbgpu.check_usbgpu_power(tmp_path) == "eGPU power fault (11900 mV, 0 mA)"


def test_check_usbgpu_reports_new_link_errors(monkeypatch):
  before = usbgpu.UsbGpuDevice("4-1.4", 0x3801, 1, 5000, "tiny", "custom ed4e39b7-CLEAN", 4, 7, 3)
  after = usbgpu.UsbGpuDevice("4-1.4", 0x3801, 1, 5000, "tiny", "custom ed4e39b7-CLEAN", 4, 7, 4)
  devices = iter((before, after))
  monkeypatch.setattr(usbgpu, "get_usbgpu_device", lambda _path: next(devices))
  monkeypatch.setattr(usbgpu.subprocess, "run", lambda *_args, **_kwargs: SimpleNamespace(returncode=0, stdout="", stderr=""))

  assert usbgpu.check_usbgpu() == "USB link errors"


def test_check_usbgpu_can_accept_recoverable_link_errors(monkeypatch):
  before = usbgpu.UsbGpuDevice("4-1.4", 0x3801, 1, 5000, "tiny", "custom ed4e39b7-CLEAN", 4, 7, 3)
  after = usbgpu.UsbGpuDevice("4-1.4", 0x3801, 1, 5000, "tiny", "custom ed4e39b7-CLEAN", 4, 7, 4)
  devices = iter((before, after))
  monkeypatch.setattr(usbgpu, "get_usbgpu_device", lambda _path: next(devices))
  monkeypatch.setattr(usbgpu.subprocess, "run", lambda *_args, **_kwargs: SimpleNamespace(returncode=0, stdout="", stderr=""))

  assert usbgpu.check_usbgpu(require_clean_link=False) is None
