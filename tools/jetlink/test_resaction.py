"""Selected stateful host history, wire contract and byte-queue parity."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import onnx
from onnx import TensorProto as T, helper as H, numpy_helper
from onnx.reference import ReferenceEvaluator
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'third_party/jetlink'), str(Path(__file__).parent)]
from openpilot.common import jetson_model
from openpilot.selfdrive.modeld.jetlink import compat, contracts, mac, startup
from jetlink import protocol as P
from jetlink.onnx_patch import needs_patch, patch_uint8_inputs
from jetlink.queues import StatefulQueues, make_queues
from jetlink.server.session import EngineHost, Session, _check_shapes
from test_v3 import MemoryStream, packet

SPEC = contracts.JETSON_SPEC


def inputs():
  return {name: np.zeros(shape, np.float16 if name in ('new_img', 'state_img_q') else np.float32)
          for name, shape in SPEC.input_shapes.items()}


def advanced(dest, value=1):
  # State intentionally precedes the driving output in dictionary order.
  return {**{'next_' + name: np.full_like(array, value) for name, array in dest.items() if name.startswith('state_')},
          'outputs': np.full(SPEC.output_shapes['outputs'], value, np.float32)}


def test_exact_export_identity_and_unchanged_driving_parser_contract():
  assert SPEC.sha256 == '01effc679eb46d6036ce0302673a87fe2dcfcc95a679daeeb929679e53c3647a'
  assert SPEC.nbytes == 814591107 and SPEC.frame_skip == 4 and SPEC.packed_nbytes == (12 + 65536 + 47) * 4
  for name, at in contracts.DEFAULT_SPEC.output_slices.items():
    if name != 'pad':
      assert SPEC.output_slices[name] == at
  assert SPEC.warped_shape == contracts.DEFAULT_SPEC.warped_shape
  assert contracts.validate_model_spec(SPEC) == SPEC
  with pytest.raises(ValueError, match='contract mismatch'):
    contracts.validate_model_spec(replace(SPEC, checkpoint='different-export'))


def test_history_feedback_is_retained_and_reset_without_client_feature_upload():
  dest = inputs()
  queues = make_queues(SPEC, dest)
  assert isinstance(queues, StatefulQueues)
  warped = np.arange(SPEC.warped_nbytes, dtype=np.uint8).reshape(SPEC.warped_shape)
  packed = np.arange(SPEC.packed_nelem, dtype=np.float32)
  for value in (1, 2, 3):
    queues.step_into(warped, packed, dest)
    np.testing.assert_array_equal(dest['new_img'], warped)
    np.testing.assert_array_equal(dest['desire'], packed[:8])
    np.testing.assert_array_equal(dest['traffic_convention'], packed[8:10].reshape(1, 2))
    np.testing.assert_array_equal(dest['action_t'], packed[10:12].reshape(1, 2))
    np.testing.assert_array_equal(dest['nav_map'], packed[12:12 + 65536].reshape(1, 1, 256, 256))
    np.testing.assert_array_equal(dest['navigation'], packed[-47:].reshape(1, 47))
    assert all(np.all(array == value - 1) for array in queues.state.values())
    result = advanced(dest, value)
    queues.feedback(result)
    result['next_state_feat_q'].fill(99)  # engine's reusable outputs cannot alias retained state
    assert np.all(dest['state_feat_q'] == value)
  queues.reset()
  assert all(not np.any(array) for array in queues.state.values())


@pytest.mark.parametrize('failure', ['nan_state', 'nan_driving', 'shape', 'missing', 'dtype', 'scalar'])
def test_invalid_frame_discards_all_recurrent_state(failure):
  dest = inputs()
  queues = make_queues(SPEC, dest)
  queues.feedback(advanced(dest, 2))
  values = advanced(dest, 3)
  error = ValueError
  if failure in ('nan_state', 'nan_driving'):
    values['next_state_feat_q' if failure == 'nan_state' else 'outputs'].flat[0] = np.nan
    error = FloatingPointError
  elif failure == 'shape':
    values['next_state_desire_q'] = np.zeros(1, np.float32)
  elif failure == 'missing':
    del values['next_state_feat_q']
  elif failure == 'dtype':
    values['next_state_img_q'] = values['next_state_img_q'].astype(np.uint8)
  with pytest.raises(error):
    if failure == 'scalar':
      queues.step_into(np.zeros(SPEC.warped_shape, np.uint8), np.full(SPEC.packed_nelem, np.inf, np.float32), dest)
    else:
      queues.feedback(values)
  assert all(not np.any(array) for array in queues.state.values())


def test_session_uses_named_driving_output_and_reset_and_rejects_nonfinite_state():
  dest = inputs()
  bad = [False]
  def run():
    result = advanced(dest, float(dest['state_feat_q'].flat[0]) + 1)
    if bad[0]:
      result['next_state_feat_q'].flat[0] = np.nan
    return result
  engine = NS(inputs={name: NS(shape=shape) for name, shape in SPEC.input_shapes.items()},
              outputs={name: NS(shape=shape) for name, shape in reversed(list(SPEC.output_shapes.items()))},
              host_input=dest.__getitem__, warm=lambda: 'warm', run=run, last_gpu_us=1)
  loaded = EngineHost._warm(None, engine, SPEC)
  sent = []
  transport = NS(send=lambda kind, seq, parts, flags: sent.append(b''.join(memoryview(part).cast('B') for part in parts)))
  host = NS(telemetry=NS(read=dict), frame_stats=NS(record=lambda *a: None))
  session = Session(transport, host)
  for seq, reset in enumerate((True, False, True), 1):
    request = (P.pack_infer_req(seq, P.Flag.RESET_QUEUES if reset else 0) +
               bytes(SPEC.warped_nbytes + SPEC.packed_nbytes))
    session._infer(loaded, NS(seq=seq, payload=memoryview(request)))
    response = memoryview(sent[-1])
    assert P.unpack_infer_resp(response)[:2] == (seq, P.Status.OK)
    assert len(response) == P.INFER_RESP_SIZE + SPEC.output_nbytes
    assert np.all(np.frombuffer(response, np.float32, offset=P.INFER_RESP_SIZE) == (1 if reset else 2))
  bad[0] = True
  session._infer(loaded, NS(seq=4, payload=memoryview(P.pack_infer_req(4, 0) + bytes(SPEC.warped_nbytes + SPEC.packed_nbytes))))
  assert P.unpack_infer_resp(memoryview(sent[-1]))[1] == P.Status.NOT_FINITE
  assert all(not np.any(array) for array in loaded.queues.state.values())
  engine.outputs['next_state_feat_q'] = NS(shape=(1,))
  with pytest.raises(ValueError, match='next_state_feat_q'):
    _check_shapes(engine, SPEC)


@pytest.mark.parametrize('protocol', [2, 3])
def test_jetson_selects_exact_resaction_and_never_uses_mac_downloader(protocol, monkeypatch):
  monkeypatch.setattr(mac, 'model_file', lambda *a: pytest.fail('Mac download on Jetson'))
  calls = []
  def ensure(sha, size, **kwargs):
    calls.append((sha, size, kwargs['frame_skip']))
    return SPEC
  client = NS(ensure_engine=ensure)
  peer = {'protocol': protocol, 'backend': 'trt', 'device': 'Orin-sm87', 'loaded': SPEC.sha256}
  assert mac.prepare(client, peer, lambda: False, lambda: True, lambda *a: None) == SPEC
  assert calls == [(SPEC.sha256, SPEC.nbytes, 4)]
  peer['loaded'] = contracts.DEFAULT_SPEC.sha256
  with pytest.raises(mac.PreparationDeferred, match='ignition off'):
    mac.prepare(client, peer, lambda: False, lambda: True, lambda *a: None)
  assert len(calls) == 1
  client.ensure_engine = lambda *a, **kw: contracts.DEFAULT_SPEC
  with pytest.raises(ValueError, match='contract mismatch'):
    mac.prepare(client, peer, lambda: True, lambda: True, lambda *a: None)


def test_v3_adapter_sends_only_resaction_scalars_and_receives_full_driving_output():
  expected = np.arange(SPEC.output_nelem, dtype=np.float32)
  reply = P.pack_infer_resp(17, P.Status.OK, 1, 2, 3) + expected.tobytes()
  stream = MemoryStream(3, incoming=packet(3, 9, 1, reply))
  client = compat.V3Client(stream, deadline=2)
  client.spec = SPEC
  result = client.infer(np.zeros(SPEC.warped_shape, np.uint8), np.arange(SPEC.packed_nelem, dtype=np.float32), 17, reset=True)
  np.testing.assert_array_equal(result, expected)
  assert len(stream.out) == 32 + P.INFER_REQ_SIZE + SPEC.warped_nbytes + SPEC.packed_nbytes


def test_old_release_is_not_sent_by_any_bootstrap_path(tmp_path, monkeypatch):
  old_release = tmp_path / 'host_release.json'
  release = {'model': {'sha256': contracts.DEFAULT_SPEC.sha256, 'size': contracts.DEFAULT_SPEC.nbytes}}
  old_release.write_text(json.dumps(release))
  monkeypatch.setattr(startup, 'RELEASE', old_release)
  assert jetson_model.matching_release(old_release) is None
  with pytest.raises(ValueError, match='matching signed'):
    startup.wait_for_boot_update(NS(), {'carrot_host': 'jetson', startup.CAPABILITY: True}, None, lambda: True, None)
  release['model'].update(sha256=SPEC.sha256, size=SPEC.nbytes)
  path = tmp_path / 'release.json'
  path.write_text(json.dumps(release))
  assert jetson_model.matching_release(path) == release  # compatibility check; not signature approval


def test_resaction_requires_explicit_download_source_before_network_access(tmp_path, monkeypatch):
  import download_model
  monkeypatch.setattr(download_model.urllib.request, 'urlopen', lambda *a, **k: pytest.fail('network access'))
  with pytest.raises(ValueError, match='--manifest-url'):
    download_model.provision(tmp_path / 'model.onnx')


def test_host_bundle_pin_is_explicit_and_old_bundles_remain_readable(tmp_path):
  import build_host_bundle
  directory = tmp_path / jetson_model.MODEL_DIR
  directory.mkdir(parents=True)
  (directory / 'cinque_v2.json').write_text(json.dumps(contracts.DEFAULT_SPEC.to_dict()))
  assert jetson_model.read_spec(tmp_path) == contracts.DEFAULT_SPEC.to_dict()
  (directory / 'jetson_model.json').write_text(json.dumps(SPEC.to_dict()))
  assert jetson_model.read_spec(tmp_path) == SPEC.to_dict()
  assert (jetson_model.MODEL_DIR / 'jetson_model.json').as_posix() in build_host_bundle.PATHS


def queue_graph():
  # Same image queue operators/order as the exact 2fb4ac4 ONNX, with small tensors.
  init = [numpy_helper.from_array(np.array(value, np.int64), name) for name, value in
          [('start', [1]), ('end', [5]), ('axis', [1]), ('step', [1]), ('zero', 0), ('flat', [5, 256])]]
  nodes = [H.make_node('Slice', ['state_img_q', 'start', 'end', 'axis', 'step'], ['old']),
           H.make_node('Unsqueeze', ['new_img', 'axis'], ['new']),
           H.make_node('Concat', ['old', 'new'], ['next_state_img_q'], axis=1),
           H.make_node('Gather', ['next_state_img_q', 'zero'], ['one_camera'], axis=0),
           H.make_node('Reshape', ['one_camera', 'flat'], ['pixels']),
           H.make_node('Cast', ['pixels'], ['outputs'], to=T.FLOAT16)]
  graph = H.make_graph(nodes, 'image_queue',
                       [H.make_tensor_value_info('new_img', T.UINT8, [2, 256]),
                        H.make_tensor_value_info('state_img_q', T.UINT8, [2, 5, 256])],
                       [H.make_tensor_value_info('outputs', T.FLOAT16, [5, 256]),
                        H.make_tensor_value_info('next_state_img_q', T.UINT8, [2, 5, 256])], init)
  return H.make_model(graph, opset_imports=[H.make_opsetid('', 18)])


def test_image_queue_fp16_patch_is_exact_across_frames_and_resets():
  original = queue_graph()
  assert needs_patch(original)
  patched = patch_uint8_inputs(deepcopy(original))
  assert not needs_patch(patched)
  onnx.checker.check_model(patched, full_check=True)
  before, after = ReferenceEvaluator(original), ReferenceEvaluator(patched)
  state = np.zeros((2, 5, 256), np.uint8)
  for frame in range(8):
    pixels = np.roll(np.tile(np.arange(256, dtype=np.uint8), (2, 1)), frame, axis=1)
    if frame == 4:
      state.fill(0)
    expected = before.run(None, {'new_img': pixels, 'state_img_q': state})
    actual = after.run(None, {'new_img': pixels.astype(np.float16), 'state_img_q': state.astype(np.float16)})
    for a, b in zip(actual, expected, strict=True):
      np.testing.assert_array_equal(a, b)
    state = expected[1]


def test_image_patch_refuses_unproven_integer_arithmetic_without_mutation():
  model = queue_graph()
  model.graph.node.insert(0, H.make_node('Add', ['new_img', 'new_img'], ['unexpected']))
  serialized = model.SerializeToString()
  with pytest.raises(ValueError, match='Unsupported image queue operation'):
    patch_uint8_inputs(model)
  assert model.SerializeToString() == serialized


def test_half_finite_check_matches_every_binary16_pattern():
  q = StatefulQueues(SPEC, inputs())
  # Exercise the NumPy fallback's scratch mask explicitly. The native path
  # returns only a boolean and has its own exhaustive per-pattern test.
  q._native_finite = None
  all_bits = np.arange(65536, dtype=np.uint16).view(np.float16)
  assert not q._all_finite(all_bits)
  np.testing.assert_array_equal(q._finite_bad[:65536], ~np.isfinite(all_bits))
  finite = all_bits[np.isfinite(all_bits)]
  assert q._all_finite(finite)
  # Reused scratch, both infinities and non-canonical positive/negative NaNs.
  for bits in (0x7c00, 0xfc00, 0x7c01, 0x7fff, 0xfc01, 0xffff):
    value = np.array([0, bits, 0x8000], np.uint16).view(np.float16)
    assert not q._all_finite(value)
    assert q._all_finite(np.array([0., -0., 65504., -65504.], np.float16))


@pytest.mark.parametrize('bits', [0x7c00, 0xfc00, 0x7c01, 0xffff])
def test_half_nonfinite_feedback_still_resets_every_state(bits):
  dest = inputs()
  q = StatefulQueues(SPEC, dest)
  q.feedback(advanced(dest, 2))
  result = advanced(dest, 3)
  result['next_state_img_q'].view(np.uint16).flat[-1] = bits
  with pytest.raises(FloatingPointError):
    q.feedback(result)
  assert all(not np.any(v) for v in q.state.values())
