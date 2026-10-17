"""Run GC-global lifecycle checks outside the pytest process."""
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[4]


def test_failed_jpeg_sessions_release_frozen_traceback_cycles():
  code = r'''
import gc
import sys
import weakref
from pathlib import Path
sys.path.insert(0, str(Path.cwd() / "openpilot/selfdrive/carrot/cluster"))
from cluster_profile import ProfileReporter, freeze_gc_after_init, restore_gc_after_run
from cluster_usb_pipeline import AsyncJpegUsbPipeline

class DisconnectedDisplay:
  def clear_profile_samples(self): pass
  def encode_jpeg(self, rgba, width, height): return b"jpeg"
  def send_jpeg(self, jpeg): raise ConnectionError("S3 display is closed")
  def profile_samples(self): return ()

original = gc.get_threshold()
refs = []
def session():
  pipeline = AsyncJpegUsbPipeline(DisconnectedDisplay())
  pipeline.start()
  previous = freeze_gc_after_init(ProfileReporter(False, 1))
  try:
    pipeline.submit_rgba(bytes(1920 * 462 * 4), 1920, 462)
    pipeline._thread.join(2)
    assert not pipeline._thread.is_alive()
    try:
      pipeline.check_error()
    except RuntimeError as exc:
      assert isinstance(exc.__cause__, ConnectionError)
    else:
      raise AssertionError("Expected disconnected JPEG failure")
  finally:
    restore_gc_after_run(previous)
    pipeline.close()
  return weakref.ref(pipeline)

for _ in range(12):
  refs.append(session())
  gc.collect()
  assert not any(ref() is not None for ref in refs), "Failed HUD session retained"
  assert gc.get_threshold() == original, (gc.get_threshold(), original)
restore_gc_after_run(None)
assert gc.get_threshold() == original
'''
  result = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True, timeout=30)
  assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize('failed', [True, False])
@pytest.mark.parametrize('discovery_misses', [0, 2])
def test_autorun_reclaims_finished_session_before_retry(failed, discovery_misses):
  # Execute the real retry loop in isolation; native/device setup is replaced,
  # but the actual JPEG worker, error traceback and GC policy are exercised.
  code = r'''
import ast, gc, sys, types, weakref
from pathlib import Path
from unittest.mock import Mock
sys.path.insert(0, str(Path.cwd() / 'openpilot/selfdrive/carrot/cluster'))
from cluster_profile import ProfileReporter, freeze_gc_after_init, restore_gc_after_run
from cluster_usb_pipeline import AsyncJpegUsbPipeline
failed = FAILED
discovery_misses = DISCOVERY_MISSES
discovery_attempts = 0
def discover(config):
  global discovery_attempts
  discovery_attempts += 1
  if discovery_attempts <= discovery_misses:
    raise TimeoutError('Registered S3 display not found on this Wi-Fi')
  return '192.0.2.1'
for name, attrs in {
  'openpilot.common.realtime': {'drop_realtime': lambda: None},
  'openpilot.common.display_scheduling': {'DisplayScheduler': lambda *a, **k: Mock()},
  'cluster_graphics_cleanup': {'ensure_clean_autorun_process': lambda: None},
  'cluster_wifi_protocol': {'load_config': lambda: {'configured': True}, 'discover': discover},
}.items():
  module = types.ModuleType(name)
  module.__dict__.update(attrs)
  sys.modules[name] = module
refs = []
class Display:
  def clear_profile_samples(self): pass
  def encode_jpeg(self, *args): return b'jpeg'
  def send_jpeg(self, data):
    if failed: raise ConnectionError('S3 display is closed')
  def profile_samples(self): return ()
def run(*args, **kwargs):
  pipeline = AsyncJpegUsbPipeline(Display())
  # A real failed frame retains the caller through the raised traceback.
  assets = bytearray(8 * 1024 * 1024)
  if not failed: pipeline.session_cycle = pipeline
  refs.append(weakref.ref(pipeline))
  pipeline.start()
  previous = freeze_gc_after_init(ProfileReporter(False, 1))
  try:
    pipeline.submit_rgba(bytes(1920 * 462 * 4), 1920, 462)
    pipeline.close()
    pipeline.check_error()
  finally:
    restore_gc_after_run(previous)
    pipeline.close()
def retry_sleep(seconds):
  if discovery_attempts <= discovery_misses:
    assert not refs, 'missing display still initialized renderer/pipeline'
    return
  assert refs and all(r() is None for r in refs), 'completed HUD remains alive during retry wait'
mode_reads_after_run = 0
def hud_mode(params):
  global mode_reads_after_run
  if not refs: return 1
  mode_reads_after_run += 1
  return 1 if not failed and mode_reads_after_run == 1 else 0
source = Path('openpilot/selfdrive/carrot/cluster_autorun.py')
tree = ast.parse(source.read_text(encoding='utf8'))
main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
scope = {
 'gc': gc, 'TICI': False, 'Params': lambda: Mock(), 'HUD_PARAM': 'ClusterHud',
 'USBGPU_ACTIVE_PARAM': 'UsbGpuActive', 'RETRY_INTERVAL_S': 5,
 'time': types.SimpleNamespace(sleep=retry_sleep),
 '_configure_autorun_locale': lambda: None, '_ensure_cluster_paths': lambda: None,
 '_read_hud_mode': hud_mode,
 '_read_encoder_mode': lambda p: 2, '_read_orientation': lambda p: None,
 '_hud_output_allowed': lambda p: True, '_wait_for_usbgpu_startup': lambda p: None,
 '_run_cluster_once': run, '_report_cluster_failure': lambda exc: None,
}
gc.disable()
exec(compile(ast.Module(body=[main], type_ignores=[]), str(source), 'exec'), scope)
scope['main']()
assert refs and all(r() is None for r in refs)
assert discovery_attempts == discovery_misses + 1
'''.replace('FAILED', repr(failed)).replace('DISCOVERY_MISSES', repr(discovery_misses))
  result = subprocess.run([sys.executable, '-c', code], cwd=ROOT, capture_output=True, text=True, timeout=30)
  assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize('platform, has_trim', [('linux', True), ('linux', False), ('win32', True)])
def test_finished_session_collection_precedes_optional_native_heap_release(platform, has_trim):
  code = r'''
import ctypes, gc, sys, types, weakref
from pathlib import Path
sys.path.insert(0, str(Path.cwd() / 'openpilot/selfdrive/carrot/cluster'))
from cluster_profile import release_finished_session_memory
class Cycle: pass
gc.disable()
value = Cycle()
value.self = value
ref = weakref.ref(value)
del value
calls = []
class Trim:
  def __call__(self, padding):
    assert ref() is None, 'native trimming ran before dead session collection'
    calls.append(padding)
    return 1
lib = types.SimpleNamespace(malloc_trim=Trim()) if HAS_TRIM else object()
ctypes.CDLL = lambda name: lib
sys.platform = PLATFORM
release_finished_session_memory()
assert ref() is None
assert calls == ([0] if PLATFORM == 'linux' and HAS_TRIM else [])
'''.replace('HAS_TRIM', repr(has_trim)).replace('PLATFORM', repr(platform))
  result = subprocess.run([sys.executable, '-c', code], cwd=ROOT, capture_output=True, text=True, timeout=30)
  assert result.returncode == 0, result.stdout + result.stderr
