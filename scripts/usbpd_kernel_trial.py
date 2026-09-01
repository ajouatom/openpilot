#!/usr/bin/env python3
"""Safely install, confirm, or roll back the SDM845 USB-PD v3 kernel."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request


EXPECTED_AGNOS_VERSION = "19.6"
SUPPORTED_DEVICE_TYPES = frozenset(("c3", "tici", "tizi", "mici"))
C3_DEVICE_TYPES = frozenset(("c3", "tici"))
RELEASE_TAG = "usbpd-test-v3-agnos19.6"
RELEASE_BASE = f"https://github.com/ajouatom/agnos-builder/releases/download/{RELEASE_TAG}"
ENSURE_REBOOT_REQUIRED = 10
ENSURE_CONFIRM_PENDING = 11
STATE_PATH = Path("/data/usbpd-kernel-trial.json")
DATA_DIR = Path("/data/usbpd-kernel-v3")
AUTO_FAILED_PATH = DATA_DIR / f"auto-failed-{RELEASE_TAG}.json"
INSTALLED_PATH = DATA_DIR / "installed.json"
PARAMS_PATH = Path("/data/params/d")
DEVICE_MODEL_PATH = Path("/sys/firmware/devicetree/base/model")
PSTORE_PATH = Path("/sys/fs/pstore")
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST_PATH = REPO_ROOT / "openpilot/system/hardware/tici/agnos.json"
C3_MANIFEST_PATH = REPO_ROOT / "openpilot/system/hardware/tici/agnos-tici.json"


class TrialError(Exception):
  pass


def run(*args: str, capture: bool = False) -> str:
  if capture:
    result = subprocess.run(args, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
  else:
    result = subprocess.run(args, check=True, text=True)
  return result.stdout.strip() if capture else ""


def require_root() -> None:
  if not hasattr(os, "geteuid") or os.geteuid() != 0:
    raise TrialError("sudo를 붙여서 실행해야 합니다.")


def require_offroad() -> None:
  try:
    if (PARAMS_PATH / "IsOnroad").read_bytes().strip() == b"1":
      raise TrialError("주행 중에는 설치할 수 없습니다. 시동을 끄고 offroad 상태에서 실행하세요.")
  except FileNotFoundError:
    pass


def device_type() -> str:
  try:
    model = DEVICE_MODEL_PATH.read_text(encoding="utf-8").strip("\x00")
  except (FileNotFoundError, OSError) as exc:
    raise TrialError(f"장치 종류를 확인할 수 없습니다: {exc}") from exc
  return model.split("comma ")[-1]


def require_supported_device() -> str:
  actual = device_type()
  if actual not in SUPPORTED_DEVICE_TYPES:
    raise TrialError(f"이 커널은 C3/C3 clone/C3X/C4 전용입니다. 현재 장치: {actual}")
  return actual


def boot_asset(device: str) -> str:
  return "boot-c3-c3clone.img" if device in C3_DEVICE_TYPES else "boot-c3x-c4.img"


def manifest_path(device: str) -> Path:
  return C3_MANIFEST_PATH if device in C3_DEVICE_TYPES else DEFAULT_MANIFEST_PATH


def release_paths(device: str) -> tuple[str, str, Path, Path]:
  asset = boot_asset(device)
  return (
    f"{RELEASE_BASE}/{asset}",
    f"{RELEASE_BASE}/{asset}.sha256",
    DATA_DIR / asset,
    DATA_DIR / f"{asset}.sha256",
  )


def current_slot() -> str:
  slot = run("abctl", "--boot_slot", capture=True)
  if slot not in ("_a", "_b"):
    raise TrialError(f"알 수 없는 부팅 슬롯입니다: {slot!r}")
  return slot


def current_boot_id() -> str:
  try:
    return Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()
  except OSError:
    return ""


def other_slot(slot: str) -> str:
  return "_b" if slot == "_a" else "_a"


def slot_number(slot: str) -> int:
  return 0 if slot == "_a" else 1


def set_active_slot(slot: str) -> None:
  last_output = ""
  for _ in range(5):
    last_output = run("abctl", "--set_active", str(slot_number(slot)), capture=True)
    if "No such file or directory" not in last_output and "lun as boot lun" in last_output:
      return
    time.sleep(1)
  raise TrialError(f"부팅 슬롯 {slot} 활성화 실패: {last_output}")


def partition_path(name: str, slot: str) -> Path:
  return Path(f"/dev/disk/by-partlabel/{name}{slot}")


def sha256_file(path: Path, size: int | None = None) -> str:
  digest = hashlib.sha256()
  remaining = size
  with path.open("rb", buffering=0) as stream:
    while remaining is None or remaining > 0:
      count = 1024 * 1024 if remaining is None else min(1024 * 1024, remaining)
      chunk = stream.read(count)
      if not chunk:
        break
      digest.update(chunk)
      if remaining is not None:
        remaining -= len(chunk)
  if remaining not in (None, 0):
    raise TrialError(f"{path} 크기가 예상보다 작습니다.")
  return digest.hexdigest()


def load_manifest(device: str) -> list[dict]:
  return json.loads(manifest_path(device).read_text(encoding="utf-8"))


def manifest_partition(name: str, device: str) -> dict:
  for partition in load_manifest(device):
    if partition["name"] == name:
      return partition
  raise TrialError(f"AGNOS manifest에서 {name} 파티션을 찾지 못했습니다.")


def verify_partition(partition: dict, slot: str) -> bool:
  path = partition_path(partition["name"], slot)
  if not path.exists():
    raise TrialError(f"파티션이 없습니다: {path}")

  expected = partition["hash_raw"].lower()
  size = int(partition["size"])
  if partition.get("full_check", False):
    return sha256_file(path, size) == expected

  with path.open("rb", buffering=0) as stream:
    stream.seek(size)
    return stream.read(64).decode("ascii", errors="ignore").lower() == expected


def verify_inactive_agnos(slot: str, device: str) -> list[str]:
  mismatches = []
  for partition in load_manifest(device):
    if not verify_partition(partition, slot):
      mismatches.append(partition["name"])
  return mismatches


def check_version() -> None:
  version = Path("/VERSION").read_text(encoding="utf-8").strip()
  if version != EXPECTED_AGNOS_VERSION:
    raise TrialError(f"이 시험 이미지는 AGNOS {EXPECTED_AGNOS_VERSION} 전용입니다. 현재 버전: {version}")


def download_file(url: str, destination: Path) -> None:
  destination.parent.mkdir(parents=True, exist_ok=True)
  with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as tmp:
    temp_path = Path(tmp.name)
    try:
      with urllib.request.urlopen(url, timeout=60) as response:
        shutil.copyfileobj(response, tmp, length=1024 * 1024)
      tmp.flush()
      os.fsync(tmp.fileno())
      temp_path.replace(destination)
    except Exception:
      temp_path.unlink(missing_ok=True)
      raise


def download_and_verify_boot(device: str) -> tuple[Path, str, int]:
  boot_url, checksum_url, boot_image_path, checksum_path = release_paths(device)
  print("USB-PD 시험 커널과 checksum을 내려받습니다.", flush=True)
  download_file(checksum_url, checksum_path)

  checksum_fields = checksum_path.read_text(encoding="utf-8").split()
  if not checksum_fields or len(checksum_fields[0]) != 64:
    raise TrialError("배포 checksum 형식이 올바르지 않습니다.")
  expected_hash = checksum_fields[0].lower()
  if not boot_image_path.exists() or sha256_file(boot_image_path) != expected_hash:
    download_file(boot_url, boot_image_path)

  image_size = boot_image_path.stat().st_size
  if image_size <= 0:
    raise TrialError("시험 boot.img가 비어 있습니다.")

  actual_hash = sha256_file(boot_image_path)
  if actual_hash != expected_hash:
    raise TrialError(f"시험 boot.img checksum 불일치: {actual_hash}")
  return boot_image_path, actual_hash, image_size


def save_state(state: dict) -> None:
  STATE_PATH.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n", encoding="utf-8")
  with STATE_PATH.open("rb") as stream:
    os.fsync(stream.fileno())


def load_state() -> dict:
  if not STATE_PATH.exists():
    raise TrialError("진행 중인 USB-PD 커널 시험 기록이 없습니다.")
  return json.loads(STATE_PATH.read_text(encoding="utf-8"))


def archive_state(state: dict, result: str) -> None:
  state["phase"] = result
  state["finished_at"] = int(time.time())
  DATA_DIR.mkdir(parents=True, exist_ok=True)
  destination = DATA_DIR / f"trial-{result}-{state['finished_at']}.json"
  destination.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n", encoding="utf-8")
  STATE_PATH.unlink(missing_ok=True)


def save_installed(state: dict) -> None:
  installed = {
    "release_tag": state.get("release_tag", RELEASE_TAG),
    "device_type": state.get("device_type", device_type()),
    "boot_asset": state.get("boot_asset", boot_asset(device_type())),
    "boot_sha256": state.get("boot_sha256", ""),
    "boot_size": int(state.get("boot_size", 0)),
    "slot": current_slot(),
  }
  DATA_DIR.mkdir(parents=True, exist_ok=True)
  INSTALLED_PATH.write_text(json.dumps(installed, indent=2, sort_keys=True) + "\n", encoding="utf-8")
  os.sync()


def best_effort_command(*args: str) -> str:
  try:
    result = subprocess.run(args, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, check=False)
    return f"$ {' '.join(args)}\nexit={result.returncode}\n{result.stdout.rstrip()}\n"
  except OSError as exc:
    return f"$ {' '.join(args)}\nerror={exc}\n"


def read_diagnostic_files(paths: list[Path]) -> str:
  output = []
  for path in paths:
    try:
      value = path.read_bytes().decode("utf-8", errors="replace").rstrip()
      value = "".join(
        char if char in "\n\r\t" or ord(char) >= 32 else f"\\x{ord(char):02x}"
        for char in value
      )
    except OSError as exc:
      value = f"<error: {exc}>"
    output.append(f"--- {path} ---\n{value}")
  return "\n".join(output)


def usb_sysfs_snapshot() -> str:
  fields = ("idVendor", "idProduct", "manufacturer", "product", "speed",
            "devpath", "busnum", "devnum", "authorized")
  output = []
  for device in sorted(Path("/sys/bus/usb/devices").glob("*")):
    values = []
    for field in fields:
      try:
        values.append(f"{field}={(device / field).read_text().strip()}")
      except OSError:
        pass
    if values:
      output.append(f"{device.name}: " + " ".join(values))
  return "\n".join(output)


def collect_diagnostics() -> None:
  require_root()
  DATA_DIR.mkdir(parents=True, exist_ok=True)
  destination = DATA_DIR / f"diagnostics-{int(time.time())}.txt"

  sections = [
    ("trial", STATE_PATH.read_text(encoding="utf-8").rstrip() if STATE_PATH.exists() else "no active trial"),
    ("release", RELEASE_TAG),
    ("device", device_type()),
    ("slot", current_slot()),
    ("cmdline", Path("/proc/cmdline").read_text(encoding="utf-8").rstrip()),
    ("uname", best_effort_command("uname", "-a")),
    ("usb tree", best_effort_command("lsusb", "-t")),
    ("usb devices", best_effort_command("lsusb")),
    ("pci devices", best_effort_command("lspci", "-nn")),
    ("usb sysfs", usb_sysfs_snapshot()),
    ("power supply", read_diagnostic_files(sorted(Path("/sys/class/power_supply/usb").glob("*")))),
    ("dmesg", best_effort_command("dmesg", "-T")),
    ("pstore", read_diagnostic_files(sorted(PSTORE_PATH.glob("*")))),
  ]
  contents = "\n\n".join(f"===== {name} =====\n{value}" for name, value in sections)
  destination.write_text(contents + "\n", encoding="utf-8")
  os.sync()
  print(f"진단 자료를 저장했습니다: {destination}")


def prepare_inactive_agnos(interactive: bool = True, boot_install: bool = False) -> None:
  require_root()
  if not boot_install:
    require_offroad()
  device = require_supported_device()
  check_version()
  if STATE_PATH.exists():
    raise TrialError("이미 진행 중인 시험이 있습니다. 먼저 confirm 또는 rollback을 실행하세요.")

  target = other_slot(current_slot())
  print(f"비활성 슬롯 {target}에 순정 AGNOS {EXPECTED_AGNOS_VERSION}을 준비합니다.", flush=True)
  if interactive:
    print("다운로드와 기록에 시간이 오래 걸릴 수 있으며 완료될 때까지 전원을 끄면 안 됩니다.", flush=True)
    answer = input("계속하려면 PREPARE를 입력하세요: ").strip()
    if answer != "PREPARE":
      raise TrialError("취소했습니다.")
  env = os.environ.copy()
  env["PYTHONPATH"] = f"{REPO_ROOT}:{env.get('PYTHONPATH', '')}"
  subprocess.run(
    [sys.executable, str(REPO_ROOT / "openpilot/system/hardware/tici/agnos.py"), str(manifest_path(device))],
    check=True,
    cwd=REPO_ROOT,
    env=env,
  )

  mismatches = verify_inactive_agnos(target, device)
  if mismatches:
    raise TrialError(f"비활성 슬롯 검증 실패: {', '.join(mismatches)}")
  print(f"비활성 슬롯 {target}의 순정 AGNOS 준비가 완료됐습니다. 이제 install을 실행하세요.")


def install(boot_install: bool = False, prepared_target: str | None = None) -> None:
  require_root()
  if not boot_install:
    require_offroad()
  device = require_supported_device()
  if prepared_target is None:
    check_version()
  elif prepared_target not in ("_a", "_b"):
    raise TrialError(f"알 수 없는 준비 슬롯입니다: {prepared_target!r}")
  if STATE_PATH.exists():
    state = load_state()
    if prepared_target is not None and state.get("phase") == "ready" and state.get("trial_slot") == prepared_target:
      archive_state(state, "reinstalled")
    else:
      raise TrialError("이미 진행 중인 시험이 있습니다. 먼저 status를 확인하세요.")

  active = current_slot()
  target = prepared_target or other_slot(active)
  if target != other_slot(active):
    raise TrialError(f"현재 슬롯 {active}의 비활성 슬롯은 {other_slot(active)}이며 요청 슬롯 {target}과 다릅니다.")
  mismatches = verify_inactive_agnos(target, device)
  if mismatches:
    names = ", ".join(mismatches)
    raise TrialError(
      f"비활성 슬롯 {target}이 현재 AGNOS {EXPECTED_AGNOS_VERSION}과 다릅니다: {names}\n"
      "먼저 다음 명령을 실행한 뒤 install을 다시 실행하세요:\n"
      "  sudo ./scripts/usbpd_kernel_trial.py prepare"
    )

  boot_partition = manifest_partition("boot", device)
  image_path, image_hash, image_size = download_and_verify_boot(device)
  target_boot = partition_path("boot", target)
  partition_size = int(run("blockdev", "--getsize64", str(target_boot), capture=True))
  if image_size > partition_size:
    raise TrialError(f"시험 boot.img({image_size})가 boot 파티션({partition_size})보다 큽니다.")
  DATA_DIR.mkdir(parents=True, exist_ok=True)
  created_at = int(time.time())
  backup_path = DATA_DIR / f"boot{target}-before-usbpd-{created_at}.img"
  print(f"비활성 boot 파티션을 {backup_path}에 백업합니다.", flush=True)
  with target_boot.open("rb", buffering=0) as source, backup_path.open("xb", buffering=0) as destination:
    remaining = int(boot_partition["size"])
    while remaining:
      chunk = source.read(min(1024 * 1024, remaining))
      if not chunk:
        raise TrialError("boot 파티션 백업이 예상보다 일찍 끝났습니다.")
      destination.write(chunk)
      remaining -= len(chunk)
    destination.flush()
    os.fsync(destination.fileno())
  if backup_path.stat().st_size != int(boot_partition["size"]):
    raise TrialError("boot 파티션 백업 크기가 올바르지 않습니다.")
  if sha256_file(backup_path) != boot_partition["hash_raw"].lower():
    raise TrialError("백업된 순정 boot 파티션 checksum이 manifest와 다릅니다.")

  state = {
    "phase": "writing",
    "created_at": created_at,
    "install_boot_id": current_boot_id(),
    "previous_slot": active,
    "trial_slot": target,
    "boot_sha256": image_hash,
    "boot_size": image_size,
    "backup_path": str(backup_path),
    "agnos_version": EXPECTED_AGNOS_VERSION,
    "device_type": device,
    "boot_asset": boot_asset(device),
    "release_tag": RELEASE_TAG,
  }
  save_state(state)

  print(f"시험 커널을 비활성 슬롯 {target}에 기록합니다. 전원을 끄지 마세요.", flush=True)
  run("abctl", "--set_unbootable", str(slot_number(target)))
  size = image_size
  with image_path.open("rb", buffering=0) as source, target_boot.open("r+b", buffering=0) as destination:
    remaining = size
    while remaining:
      chunk = source.read(min(1024 * 1024, remaining))
      if not chunk:
        raise TrialError("boot.img가 기록 도중 예상보다 일찍 끝났습니다.")
      destination.write(chunk)
      remaining -= len(chunk)
    destination.flush()
    os.fsync(destination.fileno())

  written_hash = sha256_file(target_boot, size)
  if written_hash != image_hash:
    raise TrialError(f"기록 후 checksum 검증 실패: {written_hash}")

  state["phase"] = "ready"
  save_state(state)
  set_active_slot(target)
  print("\n시험 커널 설치와 검증이 완료됐습니다.")
  print(f"현재 슬롯 {active}는 그대로 보존됐고 다음 부팅 슬롯은 {target}입니다.")
  print("준비되면 'sudo reboot'를 실행하세요.")
  print("시험 성공 후: sudo ./scripts/usbpd_kernel_trial.py confirm")
  print("원복할 때:    sudo ./scripts/usbpd_kernel_trial.py rollback")


def should_defer_success() -> int:
  try:
    state = load_state()
    return 0 if state.get("phase") == "ready" and current_slot() == state.get("trial_slot") else 1
  except Exception:
    return 1


def activate() -> None:
  require_root()
  require_offroad()
  require_supported_device()
  state = load_state()
  if state.get("phase") != "ready":
    raise TrialError(f"활성화할 수 없는 시험 상태입니다: {state.get('phase')}")

  target = state.get("trial_slot")
  if target not in ("_a", "_b"):
    raise TrialError("시험 슬롯 정보가 올바르지 않습니다.")
  size = int(state.get("boot_size", 0))
  expected_hash = state.get("boot_sha256", "")
  if size <= 0 or len(expected_hash) != 64:
    raise TrialError("시험 boot 이미지 검증 정보가 올바르지 않습니다.")
  written_hash = sha256_file(partition_path("boot", target), size)
  if written_hash != expected_hash:
    raise TrialError(f"시험 boot 파티션 checksum 불일치: {written_hash}")

  set_active_slot(target)
  print(f"다음 부팅 슬롯을 시험 슬롯 {target}로 설정했습니다.")
  print("준비되면 'sudo reboot'를 실행하세요.")


def confirm() -> None:
  require_root()
  require_supported_device()
  state = load_state()
  if current_slot() != state.get("trial_slot"):
    raise TrialError("현재 시험 슬롯으로 부팅된 상태가 아니므로 확정할 수 없습니다.")
  run("abctl", "--set_success")
  save_installed(state)
  archive_state(state, "confirmed")
  print("USB-PD 시험 커널 부팅을 성공으로 확정했습니다. 이 슬롯을 계속 사용합니다.")


def rollback() -> None:
  require_root()
  state = load_state()
  previous = state.get("previous_slot")
  if previous not in ("_a", "_b"):
    raise TrialError("원래 슬롯 정보가 올바르지 않습니다.")
  set_active_slot(previous)
  archive_state(state, "rolled-back")
  print(f"다음 부팅 슬롯을 원래 슬롯 {previous}로 되돌렸습니다.")
  print("준비되면 'sudo reboot'를 실행하세요.")


def record_auto_failure(state: dict, reason: str) -> None:
  DATA_DIR.mkdir(parents=True, exist_ok=True)
  failure = {
    "release_tag": RELEASE_TAG,
    "failed_at": int(time.time()),
    "reason": reason,
    "state": state,
  }
  AUTO_FAILED_PATH.write_text(json.dumps(failure, indent=2, sort_keys=True) + "\n", encoding="utf-8")
  os.sync()


def verify_current_trial(state: dict, active: str) -> None:
  if state.get("phase") != "ready" or active != state.get("trial_slot"):
    raise TrialError("현재 부팅 슬롯은 확정할 수 있는 USB-PD 시험 슬롯이 아닙니다.")
  size = int(state.get("boot_size", 0))
  expected_hash = str(state.get("boot_sha256", ""))
  if size <= 0 or len(expected_hash) != 64:
    raise TrialError("자동 확정에 필요한 boot 이미지 정보가 올바르지 않습니다.")
  actual_hash = sha256_file(partition_path("boot", active), size)
  if actual_hash != expected_hash:
    raise TrialError(f"자동 확정 전 boot checksum 불일치: {actual_hash}")


def ensure(confirm_trial: bool = True, boot_install: bool = False) -> int:
  """Install the matching kernel automatically and request a reboot with exit 10."""
  require_root()
  device = require_supported_device()
  check_version()
  active = current_slot()

  if STATE_PATH.exists():
    state = load_state()
    if state.get("phase") == "ready" and active == state.get("trial_slot"):
      verify_current_trial(state, active)
      if not confirm_trial:
        print("USB-PD v3 시험 슬롯이 부팅됐습니다. manager 건강 확인 후 자동 확정합니다.", flush=True)
        return ENSURE_CONFIRM_PENDING
      confirm()
      print(f"{device} USB-PD v3 커널 업데이트를 자동 확정했습니다.", flush=True)
      return 0

    previous = state.get("previous_slot")
    if previous in ("_a", "_b") and active != state.get("trial_slot"):
      if state.get("install_boot_id") and state.get("install_boot_id") == current_boot_id():
        activate()
        print("새 커널 슬롯은 아직 부팅되지 않았습니다. 안전한 시점에 자동 재부팅합니다.", flush=True)
        return ENSURE_REBOOT_REQUIRED
      reason = "시험 슬롯 부팅 실패 또는 기존 슬롯 자동 복귀"
      record_auto_failure(state, reason)
      archive_state(state, "auto-failed")
      print(f"USB-PD v3 커널 자동 적용을 중단했습니다: {reason}", flush=True)
      return 0

    raise TrialError(f"처리할 수 없는 커널 업데이트 상태입니다: {state.get('phase')}")

  if INSTALLED_PATH.exists():
    try:
      installed = json.loads(INSTALLED_PATH.read_text(encoding="utf-8"))
      installed_size = int(installed.get("boot_size", 0))
      installed_hash = str(installed.get("boot_sha256", ""))
      if (installed.get("release_tag") == RELEASE_TAG and installed.get("device_type") == device and
          installed_size > 0 and len(installed_hash) == 64 and
          sha256_file(partition_path("boot", active), installed_size) == installed_hash):
        print(f"{device}에 맞는 USB-PD v3 커널이 이미 적용되어 있습니다.", flush=True)
        return 0
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
      pass

  if not boot_install:
    require_offroad()
  _image_path, image_hash, image_size = download_and_verify_boot(device)
  active_hash = sha256_file(partition_path("boot", active), image_size)
  if active_hash == image_hash:
    AUTO_FAILED_PATH.unlink(missing_ok=True)
    save_installed({
      "release_tag": RELEASE_TAG,
      "device_type": device,
      "boot_asset": boot_asset(device),
      "boot_sha256": image_hash,
      "boot_size": image_size,
    })
    print(f"{device}에 맞는 USB-PD v3 커널이 이미 적용되어 있습니다.", flush=True)
    return 0

  if AUTO_FAILED_PATH.exists():
    print(
      f"이 릴리스의 자동 적용 실패 기록이 있어 재설치를 건너뜁니다: {AUTO_FAILED_PATH}",
      flush=True,
    )
    return 0

  target = other_slot(active)
  mismatches = verify_inactive_agnos(target, device)
  if mismatches:
    message = f"비활성 슬롯 {target}을 AGNOS {EXPECTED_AGNOS_VERSION}으로 자동 준비합니다: {', '.join(mismatches)}"
    print(message, flush=True)
    prepare_inactive_agnos(interactive=False, boot_install=boot_install)

  # install() reuses the verified local image, backs up the stock boot, writes
  # only the inactive boot partition, verifies it, and selects that slot.
  if not boot_install:
    require_offroad()
  install(boot_install=boot_install)
  print("USB-PD v3 커널 적용을 위해 자동 재부팅합니다.", flush=True)
  return ENSURE_REBOOT_REQUIRED


def ensure_boot() -> int:
  """Check and install before manager starts, but leave trial confirmation pending."""
  return ensure(confirm_trial=False, boot_install=True)


def install_prepared_agnos(target_slot_number: int) -> None:
  """Replace stock boot in a freshly prepared AGNOS slot before its first reboot."""
  if target_slot_number not in (0, 1):
    raise TrialError(f"알 수 없는 AGNOS 대상 슬롯 번호입니다: {target_slot_number}")
  target = "_a" if target_slot_number == 0 else "_b"
  install(boot_install=True, prepared_target=target)


def confirm_healthy_boot() -> None:
  """Confirm only an already-booted trial after launcher-observed manager health."""
  require_root()
  device = require_supported_device()
  check_version()
  state = load_state()
  verify_current_trial(state, current_slot())
  confirm()
  print(f"{device} USB-PD v3 커널 업데이트를 건강 확인 후 자동 확정했습니다.", flush=True)


def retry_auto() -> None:
  require_root()
  AUTO_FAILED_PATH.unlink(missing_ok=True)
  print("USB-PD v3 커널 자동 적용 실패 기록을 지웠습니다. 다음 시작 때 다시 시도합니다.")


def status() -> None:
  print(f"현재 슬롯: {current_slot()}")
  print(f"AGNOS 버전: {Path('/VERSION').read_text(encoding='utf-8').strip()}")
  print(f"장치: {device_type()}")
  print(f"시험 릴리스: {RELEASE_TAG}")
  if device_type() in SUPPORTED_DEVICE_TYPES:
    print(f"대상 이미지: {boot_asset(device_type())}")
  if STATE_PATH.exists():
    print(STATE_PATH.read_text(encoding="utf-8").rstrip())
  else:
    print("진행 중인 USB-PD 커널 시험이 없습니다.")


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("command", choices=(
    "status", "prepare", "install", "activate", "confirm", "rollback",
    "collect", "ensure", "ensure-boot", "install-prepared", "confirm-healthy",
    "retry-auto", "should-defer-success",
  ))
  parser.add_argument("--target-slot", type=int, choices=(0, 1))
  args = parser.parse_args()

  try:
    if args.command == "status":
      status()
    elif args.command == "prepare":
      prepare_inactive_agnos()
    elif args.command == "install":
      install()
    elif args.command == "activate":
      activate()
    elif args.command == "confirm":
      confirm()
    elif args.command == "rollback":
      rollback()
    elif args.command == "collect":
      collect_diagnostics()
    elif args.command == "ensure":
      return ensure()
    elif args.command == "ensure-boot":
      return ensure_boot()
    elif args.command == "install-prepared":
      if args.target_slot is None:
        raise TrialError("install-prepared에는 --target-slot이 필요합니다.")
      install_prepared_agnos(args.target_slot)
    elif args.command == "confirm-healthy":
      confirm_healthy_boot()
    elif args.command == "retry-auto":
      retry_auto()
    else:
      return should_defer_success()
  except (TrialError, OSError, subprocess.CalledProcessError, urllib.error.URLError) as error:
    print(f"오류: {error}", file=sys.stderr)
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
