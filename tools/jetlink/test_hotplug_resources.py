import sys
import subprocess
import textwrap
import threading
from concurrent.futures import Future
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.modeld.jetlink import link, model


@pytest.mark.parametrize('cancel', [False, True])
def test_handshake_wait_does_not_block_caller_and_cancel_closes_socket(monkeypatch, cancel):
  entered, release, closed = threading.Event(), threading.Event(), threading.Event()

  def connect():
    entered.set()
    assert release.wait(2)
    return NS(close=closed.set)

  monkeypatch.setattr(link, 'Client', connect)
  connection = link.ClientConnection()
  try:
    assert entered.wait(1)
    assert not connection.future.done()
    if cancel:
      connection.close()
    release.set()
    if cancel:
      assert closed.wait(1)
    else:
      assert connection.future.result(timeout=1).close == closed.set
      connection.close()
      assert closed.is_set()
  finally:
    release.set()


def joining_model(monkeypatch):
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', NS(cloudlog=NS(warning=lambda *a: None, exception=lambda *a: None)))
  m = object.__new__(model.JoiningModel)
  m.client = m.connection = None
  m.preparation = None
  m.warp_size = (1344, 760)
  m.small_runs = 3
  m.ready = m.join_allowed = True
  m.next_join = m.next_status = 0
  m.active = False
  m.error = ''
  m.small = NS(run=lambda *a: 'local')
  m.packed = np.zeros(link.SPEC.packed_nelem, np.float32)
  m.prev_desire = np.zeros(8, np.float32)
  m.views = {name: a.reshape(shape) for (name, shape), a in zip(
    link.SPEC.packed_shapes.items(), np.split(m.packed, np.cumsum(link.SPEC.packed_sizes[:-1])), strict=True)}
  m.frame = m.source_sof = 0
  m.last_slow_log = 0
  m.warp = lambda *a: None
  m.parser = NS(parse_outputs=lambda _: 'external')
  return m


def test_pending_connection_keeps_local_frames_and_rechecks_join_permission(monkeypatch):
  m = joining_model(monkeypatch)
  future = Future()
  closed = []
  attempt = NS(future=future, close=lambda: closed.append(True))
  monkeypatch.setattr(model, 'ClientConnection', lambda: attempt)
  for _ in range(5):
    assert m.run({}, {}, {}, False) == 'local'
  assert m.connection is attempt and m.small_runs == 8
  future.set_result(NS())
  m.join_allowed = False
  assert m.run({}, {}, {}, False) == 'local'
  assert closed == [True] and m.connection is None and m.client is None


def test_completed_handshake_still_resets_external_history_and_recovers_on_failure(monkeypatch, tmp_path):
  m = joining_model(monkeypatch)
  calls = []
  output = np.ones(link.SPEC.output_nelem, np.float32)

  def infer(images, packed, frame, reset, source_sof):
    calls.append((frame, reset, packed.copy()))
    if len(calls) == 2:
      raise ConnectionError('unplugged')
    return output

  client = NS(infer=infer, close=lambda: calls.append('closed'))
  future = Future()
  future.set_result(client)
  m.connection = NS(future=future)
  m.small.reset = lambda: calls.append('reset')
  fault = tmp_path / 'fault'
  monkeypatch.setattr(model, 'FAULT', fault)
  inputs = {'desire_pulse': np.zeros(8), 'traffic_convention': [1, 0], 'action_t': [.2, .3]}
  assert m.run({}, {}, inputs, False) == 'external'
  assert calls[0][0:2] == (1, True)
  assert not calls[0][2][12:].any()
  assert m.active and m.client is client
  assert m.run({}, {}, inputs, False) == 'local'
  assert calls[-2:] == ['closed', 'reset'] and fault.exists()
  assert m.client is None and not m.active and m.next_join > 0


def test_failed_handshake_keeps_local_model_and_retry_delay(monkeypatch):
  m = joining_model(monkeypatch)
  future = Future()
  future.set_exception(ValueError('wrong model'))
  monkeypatch.setattr(model, 'ClientConnection', lambda: NS(future=future))
  assert m.run({}, {}, {}, False) == 'local'
  assert m.error == 'wrong model' and m.next_join > 0 and m.client is None


def test_native_fallback_queues_are_reset_in_place(monkeypatch):
  m = joining_model(monkeypatch)
  copied = []
  queue = NS(numel=lambda: 4, dtype=NS(itemsize=4), _buffer=lambda: NS(copyin=lambda b: copied.append(bytes(b))))
  values = np.ones(8, np.float32)
  m.small = NS(prev_desire=values, npy={'prev_feat': values.copy()}, input_queues={'feat_q': queue, 'tfm': object()})
  m._reset_small()
  assert m.small.prev_desire is values and not values.any()
  assert not m.small.npy['prev_feat'].any() and copied == [bytes(16)]


def test_preview_worker_follows_display_lifetime(monkeypatch):
  import hud_protocol
  import hud_camera
  from openpilot.common import jetlink_status
  monkeypatch.setattr(jetlink_status, '_fresh', lambda *a: {})
  settings = {'ClusterHud': 0, 'IsOnroad': True, 'ClusterHudDebug': 0}
  created, closed = [], []

  def camera():
    created.append(True)
    return NS(latest={'road': 'frame'}, close=lambda: closed.append(True))

  monkeypatch.setattr(hud_camera, 'CameraPublisher', camera)
  builder = object.__new__(hud_protocol.SnapshotBuilder)
  builder.params = NS(get_int=lambda k: settings[k], get_bool=lambda k: settings[k])
  builder.camera = None
  assert builder.camera_previews() == {} and not created
  settings['ClusterHud'] = 1
  for _ in range(3):
    assert builder.camera_previews() == {'road': 'frame'}
  assert len(created) == 1
  settings['IsOnroad'] = False
  assert builder.camera_previews() == {} and len(closed) == 1
  settings['ClusterHudDebug'] = 1
  assert builder.camera_previews() == {'road': 'frame'} and len(created) == 2
  settings['ClusterHud'] = 0
  assert builder.camera_previews() == {} and len(closed) == 2


def test_jetson_panel_heartbeat_enables_preview_with_vehicle_switch_off(monkeypatch):
  import hud_protocol
  import hud_camera
  from openpilot.common import jetlink_status
  created, closed = [], []
  record = {'updated': 20., 'telemetry_updated': 20., 'peer': {'carrot_host': 'jetson'},
            'telemetry': {'carrot_hud_connected': True}}
  monkeypatch.setattr(jetlink_status, '_fresh', lambda *a: record)
  monkeypatch.setattr(hud_protocol.time, 'monotonic', lambda: 20.)
  def camera():
    created.append(True)
    return NS(latest={'road': 'frame'}, close=lambda: closed.append(True))
  monkeypatch.setattr(hud_camera, 'CameraPublisher', camera)
  builder = object.__new__(hud_protocol.SnapshotBuilder)
  builder.camera = None
  settings = {'ClusterHud': 0, 'IsOnroad': True, 'ClusterHudDebug': 0}
  builder.params = NS(get_int=lambda k: settings[k], get_bool=lambda k: settings[k])
  assert builder.camera_previews() == {'road': 'frame'} and len(created) == 1
  record['telemetry_updated'] = 17.
  assert builder.camera_previews() == {} and len(closed) == 1
  record['telemetry_updated'] = 20.
  record['telemetry']['carrot_hud_connected'] = False
  assert builder.camera_previews() == {} and len(created) == 1
  record['telemetry']['carrot_hud_connected'] = True
  settings['IsOnroad'] = False
  assert builder.camera_previews() == {} and len(created) == 1
  settings['ClusterHudDebug'] = 1
  assert builder.camera_previews() == {'road': 'frame'} and len(created) == 2


def test_daemon_freezes_startup_graph_but_new_session_cycles_are_collected():
  # Isolate the real GC change from pytest's own object graph. Stop main before
  # it touches scheduling, Params, USB, sockets, or vehicle paths.
  script = textwrap.dedent('''
    import gc, sys, weakref
    from types import SimpleNamespace
    if sys.platform == 'win32':
      sys.modules['fcntl'] = SimpleNamespace()
    from openpilot.selfdrive.modeld.jetlink import daemon
    class Cycle:
      def __init__(self): self.cycle = self
    class Stop(Exception): pass
    def stop(*args): raise Stop
    old = Cycle()
    old_ref = weakref.ref(old)
    daemon.os.sched_setscheduler = stop
    daemon.os.SCHED_OTHER = 0
    daemon.os.sched_param = lambda value: value
    try: daemon.main()
    except Stop: pass
    assert not gc.isenabled() and gc.get_freeze_count() > 0
    new = Cycle()
    new_ref = weakref.ref(new)
    del new, old
    gc.collect()
    assert old_ref() is not None and new_ref() is None
  ''')
  subprocess.run([sys.executable, '-c', script], check=True, timeout=15, capture_output=True)
