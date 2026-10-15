"""Run GC-global lifecycle checks outside the pytest process."""
from pathlib import Path
import subprocess
import sys


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
