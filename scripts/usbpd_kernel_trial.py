#!/usr/bin/env python3
"""Safely install, confirm, or roll back the experimental C4 USB-PD v3 kernel."""

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
EXPECTED_DEVICE_TYPE = "mici"
RELEASE_TAG = "usbpd-test-v3-c4"
RELEASE_BASE = f"https://github.com/ajouatom/agnos-builder/releases/download/{RELEASE_TAG}"
BOOT_URL = f"{RELEASE_BASE}/boot.img"
CHECKSUM_URL = f"{RELEASE_BASE}/boot.img.sha256"
STATE_PATH = Path("/data/usbpd-kernel-trial.json")
DATA_DIR = Path("/data/usbpd-kernel-v3")
BOOT_IMAGE_PATH = DATA_DIR / "boot.img"
CHECKSUM_PATH = DATA_DIR / "boot.img.sha256"
PARAMS_PATH = Path("/data/params/d")
DEVICE_MODEL_PATH = Path("/sys/firmware/devicetree/base/model")
PSTORE_PATH = Path("/sys/fs/pstore")
REPO_ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = REPO_ROOT / "openpilot/system/hardware/tici/agnos.json"


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


def require_c4() -> None:
  actual = device_type()
  if actual != EXPECTED_DEVICE_TYPE:
    raise TrialError(f"이 시험 커널은 comma four(mici) 전용입니다. 현재 장치: {actual}")


def current_slot() -> str:
  slot = run("abctl", "--boot_slot", capture=True)
  if slot not in ("_a", "_b"):
    raise TrialError(f"알 수 없는 부팅 슬롯입니다: {slot!r}")
  return slot


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


def load_manifest() -> list[dict]:
  return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))


def manifest_partition(name: str) -> dict:
  for partition in load_manifest():
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


def verify_inactive_agnos(slot: str) -> list[str]:
  mismatches = []
  for partition in load_manifest():
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


def download_and_verify_boot() -> tuple[Path, str, int]:
  print("USB-PD 시험 커널과 checksum을 내려받습니다.", flush=True)
  download_file(CHECKSUM_URL, CHECKSUM_PATH)
  download_file(BOOT_URL, BOOT_IMAGE_PATH)

  checksum_fields = CHECKSUM_PATH.read_text(encoding="utf-8").split()
  if not checksum_fields or len(checksum_fields[0]) != 64:
    raise TrialError("배포 checksum 형식이 올바르지 않습니다.")
  expected_hash = checksum_fields[0].lower()
  image_size = BOOT_IMAGE_PATH.stat().st_size
  if image_size <= 0:
    raise TrialError("시험 boot.img가 비어 있습니다.")

  actual_hash = sha256_file(BOOT_IMAGE_PATH)
  if actual_hash != expected_hash:
    raise TrialError(f"시험 boot.img checksum 불일치: {actual_hash}")
  return BOOT_IMAGE_PATH, actual_hash, image_size


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


def prepare_inactive_agnos() -> None:
  require_root()
  require_offroad()
  require_c4()
  check_version()
  if STATE_PATH.exists():
    raise TrialError("이미 진행 중인 시험이 있습니다. 먼저 confirm 또는 rollback을 실행하세요.")

  target = other_slot(current_slot())
  print(f"비활성 슬롯 {target}에 순정 AGNOS {EXPECTED_AGNOS_VERSION}을 준비합니다.", flush=True)
  print("다운로드와 기록에 시간이 오래 걸릴 수 있으며 완료될 때까지 전원을 끄면 안 됩니다.", flush=True)
  answer = input("계속하려면 PREPARE를 입력하세요: ").strip()
  if answer != "PREPARE":
    raise TrialError("취소했습니다.")
  env = os.environ.copy()
  env["PYTHONPATH"] = f"{REPO_ROOT}:{env.get('PYTHONPATH', '')}"
  subprocess.run(
    [sys.executable, str(REPO_ROOT / "openpilot/system/hardware/tici/agnos.py"), str(MANIFEST_PATH)],
    check=True,
    cwd=REPO_ROOT,
    env=env,
  )

  mismatches = verify_inactive_agnos(target)
  if mismatches:
    raise TrialError(f"비활성 슬롯 검증 실패: {', '.join(mismatches)}")
  print(f"비활성 슬롯 {target}의 순정 AGNOS 준비가 완료됐습니다. 이제 install을 실행하세요.")


def install() -> None:
  require_root()
  require_offroad()
  require_c4()
  check_version()
  if STATE_PATH.exists():
    raise TrialError("이미 진행 중인 시험이 있습니다. 먼저 status를 확인하세요.")

  active = current_slot()
  target = other_slot(active)
  mismatches = verify_inactive_agnos(target)
  if mismatches:
    names = ", ".join(mismatches)
    raise TrialError(
      f"비활성 슬롯 {target}이 현재 AGNOS {EXPECTED_AGNOS_VERSION}과 다릅니다: {names}\n"
      "먼저 다음 명령을 실행한 뒤 install을 다시 실행하세요:\n"
      "  sudo ./scripts/usbpd_kernel_trial.py prepare"
    )

  boot_partition = manifest_partition("boot")
  image_path, image_hash, image_size = download_and_verify_boot()
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
    "previous_slot": active,
    "trial_slot": target,
    "boot_sha256": image_hash,
    "boot_size": image_size,
    "backup_path": str(backup_path),
    "agnos_version": EXPECTED_AGNOS_VERSION,
    "device_type": EXPECTED_DEVICE_TYPE,
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
  require_c4()
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
  require_c4()
  state = load_state()
  if current_slot() != state.get("trial_slot"):
    raise TrialError("현재 시험 슬롯으로 부팅된 상태가 아니므로 확정할 수 없습니다.")
  run("abctl", "--set_success")
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


def status() -> None:
  print(f"현재 슬롯: {current_slot()}")
  print(f"AGNOS 버전: {Path('/VERSION').read_text(encoding='utf-8').strip()}")
  print(f"장치: {device_type()}")
  print(f"시험 릴리스: {RELEASE_TAG}")
  if STATE_PATH.exists():
    print(STATE_PATH.read_text(encoding="utf-8").rstrip())
  else:
    print("진행 중인 USB-PD 커널 시험이 없습니다.")


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("command", choices=("status", "prepare", "install", "activate", "confirm", "rollback", "collect", "should-defer-success"))
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
    else:
      return should_defer_success()
  except (TrialError, OSError, subprocess.CalledProcessError, urllib.error.URLError) as error:
    print(f"오류: {error}", file=sys.stderr)
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
