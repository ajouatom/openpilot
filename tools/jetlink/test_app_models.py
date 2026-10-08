"""App-selected models, offroad approval, dynamic IPC and Carrot parsing."""
from dataclasses import replace
import json
from pathlib import Path
import socket
import sys
import threading
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'third_party/jetlink'))

from openpilot.selfdrive.modeld.jetlink import contracts, daemon, link, mac, model, selection
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from jetlink.spec import ModelSpec

PEERS = [
  ('usb', {'protocol': 3, 'backend': 'ort', 'device': 'ane-Apple_M4'}),
  ('auto', {'protocol': 3, 'backend': 'ort', 'device': 'ane-Apple_M4'}),
  ('auto', {'protocol': 3, 'backend': 'litert', 'device': 'gpu-Tensor_G4'}),
  ('ios', {'protocol': 3, 'backend': 'ort', 'device': 'ane-Apple_A17_Pro'}),
  ('android', {'protocol': 3, 'backend': 'litert', 'device': 'gpu-Tensor_G4'}),
]


def queued_spec():
  return replace(link.SPEC, sha256='b' * 64, checkpoint='app-queued-model')


def stateful_spec():
  spec = queued_spec()
  # ONNX-derived Cinque Terre V3 layout in upstream test_model_state.py.
  inputs = {'new_img': (2, 6, 128, 256), 'desire': (8,), 'traffic_convention': (1, 2), 'action_t': (1, 2),
            'state_img_q': (2, 5, 6, 128, 256), 'state_desire_q': (132, 1, 8), 'state_feat_q': (128, 1, 16384)}
  outputs = dict(outputs=(1, 18452), **{'next_' + name: shape for name, shape in inputs.items() if name.startswith('state_')})
  return replace(spec, sha256='c' * 64, input_shapes=inputs, output_shapes=outputs, checkpoint='app-stateful-model')


def compact_spec():
  spec = stateful_spec()
  outputs = dict(spec.output_shapes, outputs=(1, 2068))
  slices = {name: at for name, at in spec.output_slices.items() if name not in ('hidden_state', 'pad')}
  slices['pad'] = slice(2066, 2068)
  return replace(spec, sha256='d' * 64, output_shapes=outputs, output_slices=slices, checkpoint='app-compact-model')


@pytest.mark.parametrize('factory', [queued_spec, stateful_spec, compact_spec])
def test_model_spec_round_trip_and_parser_shapes(factory):
  spec = factory()
  contracts.validate_model_spec(ModelSpec.from_dict(spec.to_dict()))
  parsed = contracts.parse_outputs(Parser(), spec, np.zeros(spec.output_nelem, np.float32))
  assert parsed['plan'].shape == (1, 33, 15)
  assert parsed['plan_stds'].shape == (1, 33, 15)
  assert parsed['lead'].shape == (1, 3, 6, 4)
  assert parsed['pose'].shape == (1, 6)
  assert parsed['lane_lines'].shape == (1, 4, 33, 2)
  assert spec.packed_nelem == (12 if spec.stateful else 16396)


def test_older_multi_hypothesis_plan_and_leads_are_parsed():
  spec = queued_spec()
  slices, offset = {}, 0
  for name, at in spec.output_slices.items():
    width = {'plan': 4955, 'lead': 102}.get(name, at.stop - at.start)
    slices[name] = slice(offset, offset + width)
    offset += width
  spec = replace(spec, output_shapes={'outputs': (1, offset)}, output_slices=slices)
  contracts.validate_model_spec(spec)
  output = np.zeros(offset, np.float32)
  raw_plan = output[slices['plan']].reshape(5, -1)
  raw_plan[3, :495] = 7
  raw_plan[3, -1] = 10
  parsed = contracts.parse_outputs(Parser(), spec, output)
  assert np.all(parsed['plan'] == 7)
  assert parsed['plan_hypotheses'].shape == (1, 5, 33, 15)
  assert parsed['lead'].shape == (1, 3, 6, 4)


def test_open_and_negative_slice_bounds_use_python_semantics():
  spec = queued_spec()
  slices = dict(spec.output_slices, lane_lines=slice(None, 528), hidden_state=slice(-16386, -2), pad=slice(-2, None))
  contracts.validate_model_spec(replace(spec, output_slices=slices))


@pytest.mark.parametrize('change', [
  {'sha256': '../../model'}, {'nbytes': True}, {'frame_skip': True}, {'frame_skip': 3},
  {'input_shapes': dict(queued_spec().input_shapes, img=(1, 12, 256, 512))},
  {'input_shapes': dict(queued_spec().input_shapes, features_buffer=(1, 32, 512))},
  {'input_shapes': dict(queued_spec().input_shapes, traffic_convention=(1, 3))},
  {'output_slices': dict(queued_spec().output_slices, plan=slice(917, 1906))},
  {'output_slices': dict(queued_spec().output_slices, pose=slice(0, 12))},
  {'output_slices': dict(queued_spec().output_slices, action=slice(2062, 2070))},
  {'output_slices': dict(queued_spec().output_slices, pad=slice(-20000, None))},
])
def test_incompatible_models_are_rejected_before_ipc(change):
  with pytest.raises(ValueError):
    contracts.validate_model_spec(replace(queued_spec(), **change))


def test_stateful_model_requires_matching_state_outputs():
  spec = stateful_spec()
  with pytest.raises(ValueError, match='state contract'):
    contracts.validate_model_spec(replace(spec, output_shapes={'outputs': (1, 2068)}))
  with pytest.raises(ValueError, match='stateful'):
    contracts.validate_model_spec(replace(spec, input_shapes=dict(spec.input_shapes, desire=(1, 9))))


@pytest.mark.parametrize(('mode', 'description'), PEERS)
@pytest.mark.parametrize('factory', [queued_spec, stateful_spec, compact_spec])
def test_apps_adopt_prepared_model_and_reuse_exact_approval_onroad(tmp_path, mode, description, factory):
  spec = factory()
  calls = []
  original = object()
  def ensure(sha, nbytes, **kwargs):
    calls.append((sha, nbytes, kwargs['frame_skip']))
    return spec
  client = SimpleNamespace(t=original, ensure_engine=ensure, state=lambda: {'loaded': spec.sha256})
  peer = dict(description, loaded=spec.sha256)
  mac.prepare(client, peer, lambda: True, lambda: True, lambda *a: None, tmp_path, mode=mode)
  assert calls == [(spec.sha256, 0, 4)]
  assert selection.approved_spec(tmp_path).to_dict() == spec.to_dict()
  mac.prepare(client, peer, lambda: False, lambda: True, lambda *a: None, tmp_path, mode=mode)
  assert len(calls) == 2 and client.t is original


@pytest.mark.parametrize(('mode', 'description'), PEERS)
def test_new_app_pick_requires_offroad_and_never_requests_old_model(tmp_path, mode, description):
  client = SimpleNamespace(ensure_engine=lambda *a, **kw: pytest.fail('must not request any model'))
  with pytest.raises(mac.PreparationDeferred, match='ignition off'):
    mac.prepare(client, dict(description, loaded=queued_spec().sha256), lambda: False, lambda: True,
                lambda *a: None, tmp_path, mode=mode)
  selection.remember_spec(tmp_path, queued_spec())
  with pytest.raises(mac.PreparationDeferred, match='ignition off'):
    mac.prepare(client, dict(description, loaded=stateful_spec().sha256), lambda: False, lambda: True,
                lambda *a: None, tmp_path, mode=mode)
  with pytest.raises(mac.PreparationDeferred, match='select and prepare'):
    mac.prepare(client, description, lambda: True, lambda: True, lambda *a: None, tmp_path, mode=mode)


@pytest.mark.parametrize('return_to_default', [False, True])
def test_ignition_transition_before_approval_cancels_model_adoption(tmp_path, return_to_default):
  allowed = [True]
  spec = link.SPEC if return_to_default else queued_spec()
  if return_to_default:
    selection.remember_spec(tmp_path, queued_spec())
  def ensure(*a, **kw):
    allowed[0] = False
    return spec
  client = SimpleNamespace(t=object(), ensure_engine=ensure, state=lambda: {'loaded': spec.sha256})
  with pytest.raises(mac.PreparationDeferred, match='ignition off'):
    mac.prepare(client, dict(PEERS[0][1], loaded=spec.sha256), lambda: allowed[0], lambda: True,
                lambda *a: None, tmp_path)
  if return_to_default:
    assert selection.approved_spec(tmp_path).sha256 == queued_spec().sha256
  else:
    assert not (tmp_path / 'selected.json').exists()


def test_approved_contract_changes_and_corrupt_approval_are_not_silenced(tmp_path):
  spec = queued_spec()
  selection.remember_spec(tmp_path, spec)
  client = SimpleNamespace(t=object(), ensure_engine=lambda *a, **kw: replace(spec, checkpoint='different'),
                           state=lambda: {'loaded': spec.sha256})
  with pytest.raises(ValueError, match='contract changed'):
    mac.prepare(client, dict(PEERS[0][1], loaded=spec.sha256), lambda: True, lambda: True, lambda *a: None, tmp_path)
  (tmp_path / 'selected.json').write_text('not JSON')
  with pytest.raises(ValueError):
    selection.approved_spec(tmp_path)


@pytest.mark.parametrize('loaded', [None, 'd' * 64])
def test_live_model_change_is_a_failure_not_a_silent_contract_swap(loaded):
  client = SimpleNamespace(spec=queued_spec())
  with pytest.raises(selection.ModelChanged):
    selection.check_loaded(client, {'loaded': loaded})
  selection.check_loaded(client, {'loaded': client.spec.sha256})
  selection.check_loaded(client, {'temperature': 50})


def test_stale_hello_pick_is_not_requested_or_restored(tmp_path):
  spec = queued_spec()
  client = SimpleNamespace(t=object(), state=lambda: {'loaded': stateful_spec().sha256},
                           ensure_engine=lambda *a, **kw: pytest.fail('must not restore the stale model'))
  with pytest.raises(selection.ModelChanged, match='before approval'):
    mac.prepare(client, dict(PEERS[0][1], loaded=spec.sha256), lambda: True, lambda: True, lambda *a: None, tmp_path)
  assert not (tmp_path / 'selected.json').exists()


@pytest.mark.parametrize('factory', [queued_spec, stateful_spec, compact_spec])
def test_dynamic_ipc_handshake_and_reply_sizes(tmp_path, factory):
  spec = factory()
  listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
  path = str(tmp_path / 'ipc')
  listener.bind(path)
  listener.listen(1)
  errors = []
  def serve():
    try:
      with listener.accept()[0] as conn:
        conn.settimeout(2)
        link.send(conn, json.dumps({'spec': spec.to_dict()}).encode())
        reader = link.PacketReader(link.REQUEST.size + spec.warped_nbytes + spec.packed_nbytes)
        for frame in (1, 2):
          request = reader.receive(conn)
          assert len(request) == len(reader.payload)
          assert link.REQUEST.unpack_from(request) == (frame, int(frame == 1), 0)
          link.send_parts(conn, link.REPLY.pack(frame, 1, 2, 3), np.full(spec.output_nelem, frame, np.float32))
    except BaseException as exc:
      errors.append(exc)
  worker = threading.Thread(target=serve)
  worker.start()
  client = link.Client(path, timeout=1)
  try:
    assert client.spec.to_dict() == spec.to_dict()
    for frame in (1, 2):
      result = client.infer(np.zeros(spec.warped_shape, np.uint8), np.zeros(spec.packed_nelem, np.float32), frame, reset=frame == 1)
      assert result.shape == (spec.output_nelem,) and np.all(result == frame)
  finally:
    client.close()
    worker.join(3)
    listener.close()
  assert not worker.is_alive() and not errors


def test_waiting_for_app_keeps_same_cable_and_sends_no_engine_request(monkeypatch):
  monkeypatch.setattr(daemon, 'TRANSPORT_MODE', 'android')
  monkeypatch.setattr(daemon, 'host_attached', lambda: True)
  monkeypatch.setattr(daemon.time, 'sleep', lambda _: None)
  answers = iter([{'loaded': None}, {'loaded': stateful_spec().sha256}])
  client = SimpleNamespace(state=lambda: next(answers))
  progress = []
  peer = daemon.wait_for_selection(client, PEERS[2][1], lambda *a: progress.append(a))
  assert peer['loaded'] == stateful_spec().sha256 and len(progress) == 2


@pytest.mark.parametrize('offroad', [True, False])
def test_idle_session_releases_old_request_offroad_and_detects_app_switch(monkeypatch, offroad):
  spec = queued_spec()
  monkeypatch.setattr(daemon, 'TRANSPORT_MODE', 'usb')
  monkeypatch.setattr(daemon, 'host_attached', lambda: True)
  monkeypatch.setattr(daemon, 'update_affinity', lambda: None)
  monkeypatch.setattr(daemon, 'publish', lambda *a, **kw: None)
  clock = [0.]
  def monotonic():
    clock[0] += .6
    return clock[0]
  monkeypatch.setattr(daemon.time, 'monotonic', monotonic)
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', SimpleNamespace(
    Params=lambda: SimpleNamespace(get_bool=lambda key: offroad if key == 'IsOffroad' else not offroad)))
  monkeypatch.setitem(sys.modules, 'openpilot.common.runtime_diagnostics', SimpleNamespace(RuntimeDiagnostics=lambda *a: None))
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', SimpleNamespace(cloudlog=SimpleNamespace(event=lambda *a: None)))
  calls = []
  def hello():
    calls.append('hello')
    return {'loaded': spec.sha256}
  def accept():
    raise TimeoutError
  remote = SimpleNamespace(spec=spec, hello=hello, state=lambda: {'loaded': stateful_spec().sha256}, last_state=None)
  with pytest.raises(selection.ModelChanged):
    daemon._serve_local(SimpleNamespace(accept=accept), remote, PEERS[0][1], None, None)
  assert calls == (['hello'] if offroad else [])


@pytest.mark.parametrize('factory', [queued_spec, stateful_spec, compact_spec])
def test_joining_model_uses_selected_buffers_parser_and_recurrence(tmp_path, monkeypatch, factory):
  spec = factory()
  monkeypatch.setitem(sys.modules, 'openpilot.system.hardware',
                      SimpleNamespace(HARDWARE=SimpleNamespace(get_device_type=lambda: 'tici')))
  logs = SimpleNamespace(warning=lambda *a: None, exception=lambda *a: None)
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', SimpleNamespace(cloudlog=logs))
  class Warp:
    timings = (0., 0., 0.)
    def __init__(self, *a, **kw): pass
    def __call__(self, *a): return np.zeros(spec.warped_shape, np.uint8)
  monkeypatch.setattr(model, 'Warp', Warp)
  # The current destination prepares tiny NPY inputs before the model joins.
  # This test substitutes camera/GPU work; keep that preparation substituted too.
  monkeypatch.setattr(model, 'make_warp_inputs', lambda: ({}, {}))
  monkeypatch.setattr(model, 'WarpPreparation', lambda *a: SimpleNamespace(
    future=SimpleNamespace(done=lambda: True, result=lambda: (b'validated', .01)),
    close=lambda: None))
  closed = []
  class Remote:
    timings = (0, 0, 0)
    def infer(self, image, packed, frame, reset, **kw):
      assert packed.size == spec.packed_nelem
      assert bool(np.any(packed[12:])) == (not reset and not spec.stateful)
      output = np.zeros(spec.output_nelem, np.float32)
      if not spec.stateful:
        output[spec.output_slices['hidden_state']] = 7
      return output
    def close(self): closed.append(True)
  remote = Remote()
  remote.spec = spec
  connection = SimpleNamespace(future=SimpleNamespace(done=lambda: True, result=lambda: remote))
  monkeypatch.setattr(model, 'ClientConnection', lambda: connection)
  resets = []
  small = SimpleNamespace(run=lambda *a: {'fallback': True}, reset=lambda: resets.append(True))
  joining = model.JoiningModel(small, 1928, 1208)
  joining.small_runs, joining.ready, joining.join_allowed = 3, True, True
  inputs = {'desire_pulse': np.zeros(8, np.float32), 'traffic_convention': np.array([1, 0], np.float32),
            'action_t': np.array([.2, .3], np.float32)}
  for _ in range(2):
    output = joining.run({}, {}, inputs, False)
    assert output['plan'].shape == (1, 33, 15)
  assert joining.active and joining.spec.sha256 == spec.sha256 and not closed
  def failed(*a, **kw):
    raise ConnectionError('App replaced its engine')
  remote.infer = failed
  fault = tmp_path / 'fault'
  monkeypatch.setattr(model, 'FAULT', fault)
  assert joining.run({}, {}, inputs, False) == {'fallback': True}
  assert not joining.active and joining.client is None
  assert closed == [True] and resets == [True] and float(fault.read_text()) > 0
