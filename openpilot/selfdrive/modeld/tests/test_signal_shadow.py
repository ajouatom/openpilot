import ast
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.modeld.signal_shadow import BASE_ROWS, BASE_SIZE, SHADOW_SIZE, ShadowLogger, selected_artifact, split_output, validate_metadata


def metadata():
  return {'version': 1, 'mode': 'comparison_only', 'base_size': BASE_SIZE, 'shadow_size': SHADOW_SIZE,
          'base_sha256': 'a' * 64, 'candidate_sha256': 'b' * 64}


def test_never_replaces_or_mutates_control_output():
  output = np.arange(BASE_SIZE + SHADOW_SIZE, dtype=np.float32)
  before = output.copy()
  control, payload = split_output(output, metadata())
  np.testing.assert_array_equal(control, before[:BASE_SIZE])
  np.testing.assert_array_equal(payload[1], before[BASE_ROWS])
  payload[1][:] = -99
  payload[2][:] = -88
  np.testing.assert_array_equal(output, before)


def test_no_active_control_metadata():
  bad = metadata()
  bad['mode'] = 'control'
  with pytest.raises(ValueError):
    validate_metadata(bad)


def test_bad_shape():
  with pytest.raises(ValueError):
    split_output(np.zeros(BASE_SIZE), metadata())


def test_disabled_does_not_touch_models(tmp_path):
  assert selected_artifact('/missing', tmp_path) is None
  (tmp_path / 'enabled').write_text('0')
  assert selected_artifact('/missing', tmp_path) is None


def test_enabled_artifact_identity(tmp_path):
  base = tmp_path / 'base.onnx'
  base.write_bytes(b'base')
  artifact = tmp_path / 'shadow_tinygrad.pkl'
  artifact.write_bytes(b'compiled')
  meta = metadata()
  meta['base_sha256'] = hashlib.sha256(b'base').hexdigest()
  manifest = {'signal_shadow': meta, 'compiled_sha256': hashlib.sha256(b'compiled').hexdigest()}
  (tmp_path / 'installed.json').write_text(json.dumps(manifest))
  (tmp_path / 'enabled').write_text('1')
  assert selected_artifact(base, tmp_path) == (artifact, meta)
  artifact.write_bytes(b'corrupt')
  with pytest.raises(ValueError):
    selected_artifact(base, tmp_path)
  artifact.write_bytes(b'compiled')
  base.write_bytes(b'new-model')
  with pytest.raises(ValueError):
    selected_artifact(base, tmp_path)


def test_logging_sample_and_frame_identity():
  events = []
  logger = ShadowLogger(lambda name, **data: events.append((name, data)))
  payload = (metadata(), np.arange(99, dtype=float), np.arange(99, dtype=float) + 1)
  for frame in range(100, 109):
    logger.record(payload, frame, frame + 2, frame * 50000000)
  assert [e[1]['frame_id'] for e in events] == [100, 104, 108]
  assert events[0][1]['frame_id_extra'] == 102
  assert events[0][1]['baseline_v'][0] == 1
  assert events[0][1]['candidate_v'][0] == 2
  assert events[0][1]['mode'] == 'comparison_only'


def test_nonfinite_candidate_disables_logging_only():
  events = []
  logger = ShadowLogger(lambda name, **data: events.append(name))
  original = np.arange(BASE_SIZE + SHADOW_SIZE, dtype=float)
  original[-1] = np.nan
  control, payload = split_output(original, metadata())
  logger.record(payload, 1, 1, 1)
  assert np.isfinite(control).all()
  assert logger.failed
  assert events == ['signalModelShadowDisabled']


def test_logging_failure_is_contained():
  def broken(*args, **kwargs):
    raise OSError('logger unavailable')
  logger = ShadowLogger(broken)
  logger.record((metadata(), np.zeros(99), np.zeros(99)), 1, 1, 1)
  assert logger.failed


@pytest.mark.parametrize('selection', ['off', 'corrupt', 'load_error', 'good'])
def test_internal_loader_falls_back_to_original(selection):
  tree = ast.parse((Path(__file__).parents[1] / 'modeld.py').read_text())
  node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'load_internal_model')
  calls = []
  def select(_):
    if selection == 'corrupt':
      raise ValueError('bad hash')
    return None if selection == 'off' else ('shadow.pkl', metadata())
  def factory(*args):
    calls.append(args)
    if len(args) == 4 and selection == 'load_error':
      raise ValueError('bad model')
    return type('FakeModel', (), {'signal_shadow': metadata() if len(args) == 4 else None})()
  logger = type('Logger', (), {'exception': lambda *args: None, 'event': lambda *args, **kwargs: None})()
  scope = {'selected_artifact': select, 'modeld_pkl_path': lambda _: Path('models/base.pkl'), 'ModelState': factory, 'cloudlog': logger}
  exec(compile(ast.Module(body=[node], type_ignores=[]), '<internal loader>', 'exec'), scope)
  model = scope['load_internal_model'](1344, 760)
  assert calls[-1] == ((1344, 760, False, 'shadow.pkl') if selection == 'good' else (1344, 760, False))
  assert (model.signal_shadow is not None) == (selection == 'good')
