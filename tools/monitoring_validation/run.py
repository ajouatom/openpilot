"""Offline monitoring checks with real policies/schema; native IPC and device access replaced."""

import sys, types
from pathlib import Path

sys.path.insert(0, str(Path.cwd()))


# Native IPC is unavailable on Windows; tests replace all sockets. Keep the
# actual Python messaging classes, frequency checks, and Cap'n Proto payloads.
def unavailable(*a, **k):
  raise RuntimeError('Native IPC must be mocked in these tests')


msgq = types.ModuleType('msgq')
for name in ('Context', 'Poller', 'SubSocket', 'PubSocket', 'SocketEventHandle'):
  setattr(msgq, name, type(name, (), {'__init__': unavailable}))
for name in ('fake_event_handle', 'drain_sock_raw', 'toggle_fake_events', 'set_fake_prefix', 'get_fake_prefix', 'delete_fake_prefix', 'wait_for_one_event'):
  setattr(msgq, name, unavailable)
msgq.MultiplePublishersError = type('MultiplePublishersError', (Exception,), {})
msgq.IpcError = type('IpcError', (Exception,), {})
sys.modules['msgq'] = msgq
params = types.ModuleType('openpilot.common.params')
params.Params = type('Params', (), {'get_bool': lambda self, k: False, 'get_int': lambda self, k: 0})
sys.modules[params.__name__] = params
hardware = types.ModuleType('openpilot.system.hardware')
hardware.PC = True
hardware.TICI = False
hardware.MICI = False
sys.modules[hardware.__name__] = hardware
import pytest

raise SystemExit(pytest.main(['--noconftest', '-o', 'addopts=', '-q', *sys.argv[1:]]))
