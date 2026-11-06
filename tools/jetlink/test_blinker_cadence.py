"""Exercise the production bridge loop against the receiver's 500 ms health lease."""
import ast
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'third_party/jetlink'))


def bridge(monkeypatch, clock, **extra):
  # Load the real loop without platform-specific USB startup/import side effects.
  path = ROOT/'openpilot/selfdrive/modeld/jetlink/daemon.py'
  function = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == '_serve_local')
  for module, value in {
    'openpilot.common.params':NS(Params=lambda:NS()),
    'openpilot.common.runtime_diagnostics':NS(RuntimeDiagnostics=lambda *a, **kw:NS()),
    'openpilot.common.swaglog':NS(cloudlog=NS(event=lambda *a, **kw:None)),
    'tools.jetlink.model_library':NS(Client=lambda _:NS(idle=lambda *a:None)),
  }.items():
    monkeypatch.setitem(sys.modules, module, value)
  scope = {'time':NS(monotonic=lambda:clock[0], thread_time=lambda:clock[0]),
           'host_attached':lambda:clock[0]<12, 'update_affinity':lambda:None,
           'publish':lambda *a, **kw:None, 'check_loaded':lambda *a:None,
           'HUD_USB_CAPABILITY':'hud', 'np':np, 'json':__import__('json'), **extra}
  exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), scope)
  return scope['_serve_local']


def test_idle_bridge_refreshes_health_before_lease_expires(monkeypatch):
  clock, received, ages = [10.], [10.], []
  class Listener:
    def accept(self):
      clock[0] += .05
      raise TimeoutError
  def state():
    received[0] = clock[0]
    return {}
  client=NS(spec=None, last_state={}, state=state)
  blinker=NS(send=lambda _:ages.append(clock[0]-received[0]))
  bridge(monkeypatch, clock)(Listener(), client, {'protocol':2}, None, None, blinker=blinker)
  assert len(ages)>30
  assert max(ages)<.3  # Former two-second poll repeatedly revoked ownership.


@pytest.mark.parametrize('enabled', [False, True])
def test_inference_frame_one_requests_fresh_health_only_for_indicator_channel(monkeypatch, enabled):
  clock, requested, timeout = [10.], [], []
  packet=struct.Struct('<III')
  data=packet.pack(1,0,0)+bytes(8)
  class ReachedInference(BaseException):pass
  class Connection:
    def __enter__(self):return self
    def __exit__(self,*args):return False
    def settimeout(self,value):timeout.append(value)
  spec=NS(warped_nbytes=4, packed_nbytes=4, warped_shape=(4,), packed_nelem=1, to_dict=lambda:{})
  def infer(*args, **kwargs):
    requested.append(kwargs['want_state'])
    raise ReachedInference
  client=NS(spec=spec,last_state={},last_timings=[],output_projection_available=False,infer_begin=infer)
  run=bridge(monkeypatch, clock, REQUEST=packet, send=lambda *a:None,
             PacketReader=lambda _:NS(receive=lambda _:data))
  with pytest.raises(ReachedInference):
    run(NS(accept=lambda:(Connection(),None)),client,{'protocol':2},None,None,blinker=NS() if enabled else None)
  assert requested == [enabled]
  assert timeout == [.1 if enabled else .5]
