"""Fresh interpreters expose package collisions hidden by cached test imports."""
import os
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[5]


@pytest.mark.parametrize('paths', [('root',), ('root', 'legacy'), ('legacy', 'root')])
@pytest.mark.parametrize('preload_tools', [False, True])
def test_shared_indicator_imports_with_cluster_and_vehicle_paths(tmp_path, paths, preload_tools):
  directories = {'root': ROOT, 'legacy': ROOT / 'openpilot'}
  code = """
import importlib
import sys
from pathlib import Path

root = Path(sys.argv[1])
if sys.argv[2] == 'True':
  importlib.import_module('tools')
from openpilot.selfdrive.carrot.bluetooth.b1_monitor import Monitor
from tools.jetlink.blink_owner import Owner
from tools.jetlink import blink_protocol
from tools.jetlink import usb_release, usb_release_client, model_library

monitor = Monitor()
assert type(monitor.output_owner) is Owner
assert blink_protocol.Owner is Owner
assert Path(blink_protocol.__file__).resolve() == root / 'tools/jetlink/blink_protocol.py'
assert (Path(blink_protocol.__file__).parent / 'blink_source.py').is_file()
assert usb_release_client.validate is usb_release.validate
assert model_library.Client
import openpilot.tools
assert Path(openpilot.tools.__file__).resolve() == root / 'openpilot/tools/__init__.py'
if 'legacy' in sys.argv[3:]:
  import tools.lib
  assert Path(tools.lib.__file__).resolve() == root / 'openpilot/tools/lib/__init__.py'
"""
  result = subprocess.run(
    [sys.executable, '-c', code, str(ROOT), str(preload_tools), *paths],
    cwd=tmp_path, env={**os.environ, 'PYTHONPATH': os.pathsep.join(str(directories[path]) for path in paths)},
    text=True, capture_output=True, timeout=20,
  )
  assert result.returncode == 0, result.stdout + result.stderr
