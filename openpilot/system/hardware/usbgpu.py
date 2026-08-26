import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from openpilot.common.basedir import BASEDIR


USBGPU_FW_VERSION = "ed4e39b7"
USBGPU_USB_IDS = ((0xADD1, 0x0001), (0x3801, 0x0001))
USB_DEVICES_PATH = Path("/sys/bus/usb/devices")
USBGPU_CHECK_ATTEMPTS = 2
USBGPU_CHECK_RETRY_INTERVAL = 1.0


@dataclass(frozen=True)
class UsbGpuDevice:
  sysfs_name: str
  vendor_id: int
  product_id: int
  speed_mbps: int
  manufacturer: str
  product: str
  busnum: int
  devnum: int
  link_error_count: int


def _read(path: Path) -> str:
  try:
    return path.read_text().strip()
  except OSError:
    return ""


def _read_int(path: Path, base: int = 10) -> int:
  try:
    return int(_read(path), base)
  except ValueError:
    return 0


def is_current_usbgpu_firmware(product: str) -> bool:
  return product == f"custom {USBGPU_FW_VERSION}-CLEAN"


def _controller(device: Path) -> Path | None:
  try:
    return next((parent for parent in device.resolve().parents if parent.name.endswith(".ssusb")), None)
  except OSError:
    return None


def get_usbgpu_devices(devices_path: Path = USB_DEVICES_PATH) -> list[UsbGpuDevice]:
  devices: list[UsbGpuDevice] = []
  try:
    paths = sorted(devices_path.glob("*"), key=lambda path: path.name)
  except OSError:
    return devices

  for path in paths:
    vendor_id = _read_int(path / "idVendor", 16)
    product_id = _read_int(path / "idProduct", 16)
    if (vendor_id, product_id) not in USBGPU_USB_IDS:
      continue
    controller = _controller(path)
    devices.append(UsbGpuDevice(
      sysfs_name=path.name,
      vendor_id=vendor_id,
      product_id=product_id,
      speed_mbps=_read_int(path / "speed"),
      manufacturer=_read(path / "manufacturer"),
      product=_read(path / "product"),
      busnum=_read_int(path / "busnum"),
      devnum=_read_int(path / "devnum"),
      link_error_count=_read_int(controller / "portli", 0) & 0xFFFF if controller is not None else 0,
    ))
  return devices


def get_usbgpu_device(devices_path: Path = USB_DEVICES_PATH) -> UsbGpuDevice | None:
  devices = get_usbgpu_devices(devices_path)
  return devices[0] if len(devices) == 1 else None


def usbgpu_status(compiled: bool, loading: bool, active: bool, startup_failed: bool,
                  devices_path: Path = USB_DEVICES_PATH) -> str:
  devices = get_usbgpu_devices(devices_path)
  if not devices:
    return "not detected"
  if len(devices) != 1:
    return "multiple devices"

  device = devices[0]
  if device.speed_mbps < 5000:
    return f"slow USB ({device.speed_mbps} Mbps)"
  if not is_current_usbgpu_firmware(device.product):
    return "firmware mismatch"
  if startup_failed:
    return "startup failed"
  if loading:
    return "loading"
  if active:
    return "active"
  if not compiled:
    return "model not compiled"
  return "ready"


def check_usbgpu(devices_path: Path = USB_DEVICES_PATH, timeout: float = 15.0) -> str | None:
  """Run an offroad connection check. None means the USB and GPU checks passed."""
  device = get_usbgpu_device(devices_path)
  if device is None:
    return "USB not connected"
  if device.speed_mbps < 5000:
    return f"USB link {device.speed_mbps} Mbps"
  if not is_current_usbgpu_firmware(device.product):
    return "firmware mismatch"
  link_errors = device.link_error_count

  env = {**os.environ, "DEV": "USB+AMD:LLVM", "GMMU": "0", "PYTHONPATH": os.path.join(BASEDIR, "tinygrad_repo")}
  code = "from tinygrad import Tensor; x = Tensor.rand(1 << 20).realize(); [x.numpy() for _ in range(8)]"
  for attempt in range(USBGPU_CHECK_ATTEMPTS):
    try:
      result = subprocess.run([sys.executable, "-c", code], env=env, capture_output=True, text=True,
                              timeout=timeout, check=False)
    except subprocess.TimeoutExpired:
      return "GPU check timed out"

    if result.returncode == 0:
      break

    output = f"{result.stdout}\n{result.stderr}".lower()
    pcie_not_ready = "pcie link not up" in output or "read(0xb450" in output
    if pcie_not_ready and attempt + 1 < USBGPU_CHECK_ATTEMPTS:
      # CustomASM24Controller resets the bridge before reporting a failed
      # link. Give it time to re-enumerate, then verify using a fresh process.
      time.sleep(USBGPU_CHECK_RETRY_INTERVAL)
      continue
    return "12V / PCIe not ready" if pcie_not_ready else "GPU incompatible"

  device_after = get_usbgpu_device(devices_path)
  if device_after is None or device_after.link_error_count > link_errors:
    return "USB link errors"
  return None
