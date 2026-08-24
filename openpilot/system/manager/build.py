#!/usr/bin/env python3
import os
import subprocess
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


def build_usbgpu_model(spinner: Spinner) -> bool:
  """Build the optional big model without making the normal build depend on it."""
  from openpilot.selfdrive.modeld.big_model import active_model_path
  from openpilot.selfdrive.modeld.helpers import modeld_pkl_path, usbgpu_present

  if not usbgpu_present() or active_model_path() is None:
    return False

  pkl_path = Path(modeld_pkl_path(usbgpu=True))
  manifest_path = Path(get_manifest_path(pkl_path))
  if manifest_path.is_file():
    return True

  # Remove only an incomplete artifact for the currently selected model. Older
  # verified versions remain available as a runtime fallback.
  for path in pkl_path.parent.glob(pkl_path.name + "*"):
    if path.is_file():
      path.unlink()

  env = os.environ.copy()
  env['BUILD_USB_GPU_MODEL'] = '1'
  env['PYTHONUNBUFFERED'] = '1'
  target = os.path.relpath(manifest_path, BASEDIR)
  spinner.update("Compiling optional USB eGPU big model")
  process = subprocess.Popen(["scons", "-j1", "--cache-populate", target], cwd=BASEDIR, env=env,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
  assert process.stdout is not None
  output: list[bytes] = []
  for line in iter(process.stdout.readline, b''):
    line = line.rstrip()
    if line:
      output.append(line)
      line_text = line.decode('utf8', 'replace')
      print(line_text)
      spinner.update(line_text)
  process.wait()

  if process.returncode == 0 and manifest_path.is_file():
    return True

  for path in pkl_path.parent.glob(pkl_path.name + "*"):
    if path.is_file():
      path.unlink()
  add_file_handler(cloudlog)
  cloudlog.error("optional USB eGPU model build failed\n" + b"\n".join(output).decode('utf8', 'replace'))
  spinner.update("USB eGPU model unavailable; using the internal model")
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
