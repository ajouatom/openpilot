import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from boot_display import lines, render
import boot_status


def test_offline_diagnosis_without_comma_model_or_xorg(tmp_path):
  title, rows, detail, error = lines({'stage': {'stage': 'root-readonly-check', 'error': 'RuntimeError'},
                                    'units': {'carrot-protected-storage': {'ActiveState': 'failed'}}})
  assert error and title[1] == 'BOOT NEEDS ATTENTION'
  assert rows[3][2] == 'Waiting for network'
  assert 'carrot-protected-storage' in detail
  # The frame is CPU-only and fully renderable without raylib/TensorRT/Xorg.
  frame = render({'stage': {'stage': 'root-readonly-check', 'error': 'RuntimeError'},
                  'uptime': 25})
  assert frame.size == (1920, 462)
  frame.save(tmp_path / 'boot-error.png')


def test_active_service_does_not_claim_model_ready():
  title, rows, _, _ = lines({'units': {'carrot-jetlink': {'ActiveState': 'active'}}})
  assert 'WAITING' in title[1]
  assert rows[-1] == ('콤마 USB 서비스', 'COMMA USB SERVICE', 'active')


@pytest.mark.skipif(sys.platform != 'linux', reason='Offline Linux service installation')
def test_pinned_diagnostic_install_is_independent_of_model_and_xorg(tmp_path):
  from install_boot_display import configure
  configure(tmp_path, Path(__file__).parent)
  pinned = tmp_path / 'usr/local/lib/carrot-jetlink/boot-display'
  service = (tmp_path / 'etc/systemd/system/carrot-jetlink-boot-display.service').read_text()
  assert 'Requires=' not in service and 'After=local-fs.target systemd-udev-trigger.service' in service
  assert '/opt/carrot-jetlink/current/tools/jetlink/boot_display.py' not in service
  code = '''import sys
sys.path.insert(0, sys.argv[1] + '/tools/jetlink')
import boot_display
import cluster_usb_display
assert not any(name in sys.modules for name in ('pyray', 'tensorrt', 'capnp'))
assert boot_display.render({'uptime':1}).size == (1920,462)
'''
  subprocess.run([sys.executable, '-I', '-c', code, str(pinned)], check=True)


def test_boot_record_does_not_persist_arbitrary_input(tmp_path, monkeypatch):
  original = Path.read_text
  monkeypatch.setattr(Path, 'read_text', lambda self, *a, **k:
                      'test-boot-id' if self.as_posix() == '/proc/sys/kernel/random/boot_id' else original(self, *a, **k))
  boot_status.persist_result(tmp_path, {'stage': 'protected', 'password': 'must-not-save',
                                        'journal': 'private log', 'error': ''})
  value = json.loads((tmp_path / 'BOOT-STATUS.json').read_text())
  assert value == {'format': 1, 'boot_id': 'test-boot-id', 'stage': 'protected', 'error': ''}


@pytest.mark.skipif(sys.platform != 'linux', reason='Real flock/process identity test')
def test_panel_handoff_and_crashed_hud_release(tmp_path):
  from display_owner import hud_requested, panel_owner
  code = '''
import sys,time
from pathlib import Path
from display_owner import panel_owner
with panel_owner(hud=True, directory=Path(sys.argv[1])) as acquired:
 print('HUD_OWNS' if acquired else 'ERROR', flush=True)
 time.sleep(30)
'''
  child = None
  try:
    with panel_owner(directory=tmp_path) as acquired:
      assert acquired
      child = subprocess.Popen([sys.executable, '-c', code, str(tmp_path)],
                               env=dict(os.environ, PYTHONPATH=str(Path(__file__).parent)),
                               stdout=subprocess.PIPE, text=True)
      deadline = time.monotonic() + 5
      while not hud_requested(tmp_path) and time.monotonic() < deadline:
        time.sleep(.02)
      assert hud_requested(tmp_path)
      assert child.poll() is None
    assert child.stdout.readline().strip() == 'HUD_OWNS'
    with panel_owner(directory=tmp_path) as acquired:
      assert not acquired
    child.kill()
    child.wait(timeout=5)
    with panel_owner(directory=tmp_path) as acquired:
      assert acquired
    (tmp_path / 'hud-request').write_text(json.dumps({'pid': os.getpid(), 'identity': ['old-boot', '0']}))
    assert not hud_requested(tmp_path)
  finally:
    if child and child.poll() is None:
      child.kill()
      child.wait(timeout=5)
