#!/usr/bin/env python3
import os
import selectors
import subprocess
import time
from pathlib import Path

# NOTE: Do NOT import anything here that needs be built (e.g. params)
from openpilot.common.basedir import BASEDIR
from openpilot.common.file_chunker import get_manifest_path
from openpilot.common.spinner import Spinner
from openpilot.common.text_window import TextWindow
from openpilot.common.swaglog import cloudlog, add_file_handler
from openpilot.system.hardware import HARDWARE, AGNOS
from openpilot.system.version import get_build_metadata

MAX_CACHE_SIZE = 4e9 if "CI" in os.environ else 2e9
CACHE_DIR = Path("/data/scons_cache" if AGNOS else "/tmp/scons_cache")

TOTAL_SCONS_NODES = 2705
MAX_BUILD_PROGRESS = 100
USBGPU_BUILD_ATTEMPTS = 6
USBGPU_BUILD_RETRY_INTERVAL = 2.0
USBGPU_READINESS_ATTEMPTS = 6
USBGPU_READINESS_RETRY_INTERVAL = 3.0
USBGPU_READINESS_TIMEOUT = 30.0
USBGPU_TRANSIENT_READINESS_ERRORS = {"12V / PCIe not ready", "USB link errors", "GPU check timed out", "GPU incompatible"}
USBGPU_ENUMERATION_WAIT_SECONDS = 20.0
USBGPU_ENUMERATION_POLL_INTERVAL = 1.0


def build_usbgpu_model(spinner: Spinner) -> bool:
  """Build the optional big model without making the normal build depend on it."""
  from openpilot.selfdrive.modeld.big_model import active_manifest, active_model_path, model_cache_dir
  from openpilot.selfdrive.modeld.big_model_status import write_big_model_status
  from openpilot.selfdrive.modeld.helpers import modeld_pkl_path, usbgpu_pcie_not_ready, usbgpu_present
  from openpilot.system.hardware.usbgpu import check_usbgpu

  model_path = active_model_path()
  manifest = active_manifest()
  if model_path is None or manifest is None:
    return False

  status_values = {
    "model_id": manifest.model_id,
    "sha256": manifest.sha256,
    "downloaded_bytes": manifest.size,
    "total_bytes": manifest.size,
  }
  present = usbgpu_present()
  if not present:
    wait_started = time.monotonic()
    write_big_model_status(model_cache_dir(), "waiting_for_ignition",
                           detail="waiting for eGPU USB re-enumeration", **status_values)
    while time.monotonic() - wait_started < USBGPU_ENUMERATION_WAIT_SECONDS:
      elapsed = int(time.monotonic() - wait_started)
      spinner.update(f"USB eGPU big model\nWaiting for USB · {elapsed:02d}s")
      time.sleep(USBGPU_ENUMERATION_POLL_INTERVAL)
      present = usbgpu_present()
      if present:
        print(f"USB eGPU returned after {time.monotonic() - wait_started:.1f}s; continuing optional model compilation")
        break

  if not present:
    write_big_model_status(model_cache_dir(), "waiting_for_ignition",
                           detail="turn ignition on, then restart to compile", **status_values)
    return False

  pkl_path = Path(modeld_pkl_path(usbgpu=True))
  manifest_path = Path(get_manifest_path(pkl_path))
  if manifest_path.is_file():
    write_big_model_status(model_cache_dir(), "compiled", **status_values)
    return True

  readiness_error = None
  for readiness_attempt in range(1, USBGPU_READINESS_ATTEMPTS + 1):
    check_message = (f"USB eGPU big model\nChecking GPU · " +
                     f"{readiness_attempt}/{USBGPU_READINESS_ATTEMPTS} · up to {USBGPU_READINESS_TIMEOUT:.0f}s")
    spinner.update(check_message)
    write_big_model_status(model_cache_dir(), "checking", detail=check_message.replace("\n", " "), **status_values)
    # A successful GPU workload proves the compile path is usable. The xHCI
    # LEC can still record recoverable packet retries, which the interactive
    # diagnostic reports strictly but must not block an optional compilation.
    readiness_error = check_usbgpu(timeout=USBGPU_READINESS_TIMEOUT, require_clean_link=False)
    if readiness_error is None:
      break
    if readiness_error not in USBGPU_TRANSIENT_READINESS_ERRORS or readiness_attempt >= USBGPU_READINESS_ATTEMPTS:
      break
    message = (f"USB eGPU transient readiness error: {readiness_error}; retrying in "
               f"{USBGPU_READINESS_RETRY_INTERVAL:.0f}s ({readiness_attempt}/{USBGPU_READINESS_ATTEMPTS})")
    print(message)
    spinner.update(message)
    write_big_model_status(model_cache_dir(), "waiting_for_ignition", detail=message, **status_values)
    time.sleep(USBGPU_READINESS_RETRY_INTERVAL)
  if readiness_error is not None:
    message = f"USB eGPU not ready for optional model compilation: {readiness_error}"
    print(message)
    spinner.update(message)
    write_big_model_status(model_cache_dir(), "waiting_for_ignition", detail=readiness_error, **status_values)
    return False

  env = os.environ.copy()
  env['BUILD_USB_GPU_MODEL'] = '1'
  env['PYTHONUNBUFFERED'] = '1'
  target = os.path.relpath(manifest_path, BASEDIR)
  all_output: list[bytes] = []
  compile_started_at = time.time()
  for attempt in range(1, USBGPU_BUILD_ATTEMPTS + 1):
    # Never delete generated PKL artifacts here. A compiled model must survive
    # transient eGPU/hub failures and subsequent boot attempts. Model versions
    # use SHA-derived names, and compile_modeld overwrites its raw working PKL
    # when an incomplete build is retried.
    spinner.update("USB eGPU big model\nCompiling · 00:00 · keep ignition on")
    write_big_model_status(model_cache_dir(), "compiling", started_at=compile_started_at,
                           detail=f"compile attempt {attempt}/{USBGPU_BUILD_ATTEMPTS}", **status_values)
    process = subprocess.Popen(["scons", "-j1", "--cache-populate", target], cwd=BASEDIR, env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    assert process.stdout is not None
    output: list[bytes] = []
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    last_heartbeat = 0.0
    last_status_heartbeat = 0.0
    while process.poll() is None:
      for _key, _events in selector.select(timeout=1.0):
        line = process.stdout.readline().rstrip()
        if line:
          output.append(line)
          print(line.decode('utf8', 'replace'))
      now = time.time()
      if now - last_heartbeat >= 1.0:
        elapsed = max(0, int(now - compile_started_at))
        elapsed_text = f"{elapsed // 60:02d}:{elapsed % 60:02d}"
        spinner.update(f"USB eGPU big model\nCompiling · {elapsed_text} · keep ignition on")
        last_heartbeat = now
      if now - last_status_heartbeat >= 10.0:
        write_big_model_status(model_cache_dir(), "compiling", started_at=compile_started_at,
                               detail=f"compile attempt {attempt}/{USBGPU_BUILD_ATTEMPTS}", **status_values)
        last_status_heartbeat = now
    process.wait()
    selector.close()
    output.extend(line.rstrip() for line in process.stdout.readlines() if line.rstrip())
    all_output.extend(output)

    if process.returncode == 0 and manifest_path.is_file():
      write_big_model_status(model_cache_dir(), "compiled", **status_values)
      return True

    error_text = b"\n".join(output).decode('utf8', 'replace')
    if attempt < USBGPU_BUILD_ATTEMPTS and usbgpu_pcie_not_ready(error_text):
      message = (f"USB eGPU PCIe link not ready; retrying in {USBGPU_BUILD_RETRY_INTERVAL:.0f}s "
                 f"({attempt}/{USBGPU_BUILD_ATTEMPTS})")
      print(message)
      spinner.update(message)
      time.sleep(USBGPU_BUILD_RETRY_INTERVAL)
      continue
    break

  add_file_handler(cloudlog)
  cloudlog.error("optional USB eGPU model build failed\n" + b"\n".join(all_output).decode('utf8', 'replace'))
  spinner.update("USB eGPU model unavailable; using the internal model")
  write_big_model_status(model_cache_dir(), "error", detail="compile failed; using internal model", **status_values)
  return False

def build(spinner: Spinner, dirty: bool = False, minimal: bool = False) -> None:
  env = os.environ.copy()
  env['SCONS_PROGRESS'] = "1"
  env['PYTHONUNBUFFERED'] = "1"
  nproc = os.cpu_count()
  if nproc is None:
    nproc = 2

  extra_args = ["--minimal"] if minimal else []

  if AGNOS:
    HARDWARE.set_power_save(False)
    os.sched_setaffinity(0, range(8))  # ensure we can use the isolcpus cores

  # building with all cores can result in using too
  # much memory, so retry with less parallelism
  compile_output: list[bytes] = []
  for n in (nproc, nproc/2, 1):
    compile_output.clear()
    scons: subprocess.Popen = subprocess.Popen(["scons", f"-j{int(n)}", "--cache-populate", *extra_args], cwd=BASEDIR, env=env,
                                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    assert scons.stdout is not None

    # Read progress and build output from scons and update spinner.
    while scons.poll() is None:
      try:
        line = scons.stdout.readline()
        if line is None:
          continue
        line = line.rstrip()

        prefix = b'progress: '
        if line.startswith(prefix):
          i = int(line[len(prefix):])
          spinner.update_progress(MAX_BUILD_PROGRESS * min(1., i / TOTAL_SCONS_NODES), 100.)
        elif len(line):
          compile_output.append(line)
          line_text = line.decode('utf8', 'replace')
          print(line_text)
          spinner.update(line_text)
      except Exception:
        pass

    if scons.returncode == 0:
      break

  if scons.returncode != 0:
    # Read remaining output
    if scons.stdout is not None:
      compile_output += scons.stdout.read().split(b'\n')

    # Build failed log errors
    error_s = b"\n".join(compile_output).decode('utf8', 'replace')
    add_file_handler(cloudlog)
    cloudlog.error("scons build failed\n" + error_s)

    # Show TextWindow
    spinner.close()
    if not os.getenv("CI"):
      with TextWindow("openpilot failed to build\n \n" + error_s) as t:
        t.wait_for_exit()
    exit(1)

  # enforce max cache size
  cache_files = [f for f in CACHE_DIR.rglob('*') if f.is_file()]
  cache_files.sort(key=lambda f: f.stat().st_mtime)
  cache_size = sum(f.stat().st_size for f in cache_files)
  for f in cache_files:
    if cache_size < MAX_CACHE_SIZE:
      break
    cache_size -= f.stat().st_size
    f.unlink()


if __name__ == "__main__":
  spinner = Spinner()
  spinner.update_progress(0, 100)
  build_metadata = get_build_metadata()
  build(spinner, build_metadata.openpilot.is_dirty, minimal = AGNOS)
  build_usbgpu_model(spinner)
