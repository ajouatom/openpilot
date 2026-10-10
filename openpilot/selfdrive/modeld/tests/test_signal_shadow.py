import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.modeld.signal_shadow import (BASE_ROWS, ShadowClient, ShadowHistory,
                                                      selected_artifact, sha256_file, validate_metadata)


def metadata():
  return {'version': 2, 'mode': 'comparison_only', 'base_size': 2576, 'shadow_size': 99,
              'base_sha256': 'a' * 64, 'candidate_sha256': 'b' * 64}


def test_artifact_defaults_off_and_checks_identity(tmp_path):
  base = tmp_path / 'base.onnx'
  base.write_bytes(b'original')
  candidate = tmp_path / 'policy_shadow.onnx'
  candidate.write_bytes(b'observer')
  assert selected_artifact(base, tmp_path) is None
  meta = metadata()
  meta['base_sha256'] = sha256_file(base)
  manifest = {'signal_shadow': meta, 'policy_sha256': sha256_file(candidate)}
  (tmp_path / 'installed.json').write_text(json.dumps(manifest))
  (tmp_path / 'enabled').write_text('1')
  assert selected_artifact(base, tmp_path) == (candidate, meta)
  candidate.write_bytes(b'damaged')
  with pytest.raises(ValueError, match='checksum'):
    selected_artifact(base, tmp_path)


def test_rejects_old_combined_graph():
  meta = metadata()
  meta['version'] = 1
  with pytest.raises(ValueError):
    validate_metadata(meta)


def test_history_phase_desire_pooling_and_no_mutation():
  history = ShadowHistory()
  output = np.arange(2576, dtype=np.float32)
  before = output.copy()
  for frame in range(1, 105):
    inputs = {'prev_feat': np.full((1, 512), frame - 1, np.float32),
                  'desire': np.eye(8, dtype=np.float32)[frame % 8],
                  'traffic_convention': np.array([[1, 0]], np.float32), 'action_t': np.array([[.4, .3]], np.float32)}
    snapshot = {k: v.copy() for k, v in inputs.items()}
    sample = history.capture(output, inputs)
    assert (sample is not None) == (frame % 4 == 0)
    for key in inputs:
      np.testing.assert_array_equal(inputs[key], snapshot[key])
  feeds = sample['feeds']
  np.testing.assert_array_equal(feeds['features_buffer'][0, :, 0], np.arange(8, 104, 4))
  # At frame 104 the oldest retained pulse is frame 5; pool frames 5..8.
  np.testing.assert_array_equal(feeds['desire_pulse'][0, 0], [1, 0, 0, 0, 0, 1, 1, 1])
  np.testing.assert_array_equal(feeds['current_hidden_flat'][0], output[1064:1576])
  np.testing.assert_array_equal(sample['baseline'], output[BASE_ROWS])
  feeds['features_buffer'][:] = -1
  sample['baseline'][:] = -1
  np.testing.assert_array_equal(output, before)
  assert history.features.min() == 8


@pytest.mark.parametrize('failure', [BlockingIOError(), OSError('worker closed')])
def test_queue_failure_never_reaches_control(failure):
  class BrokenSocket:
    def send(self, data):
      raise failure

    def close(self):
      pass
  client = object.__new__(ShadowClient)
  client.failed = False
  client.dropped = 0
  client.process = SimpleNamespace(poll=lambda: None)
  client.sock = BrokenSocket()
  client.submit({}, 1, 2, 3)
  assert client.dropped == int(isinstance(failure, BlockingIOError))
  assert client.failed == (not isinstance(failure, BlockingIOError))


def test_dead_worker_disables_observation():
  client = object.__new__(ShadowClient)
  client.failed = False
  client.process = SimpleNamespace(poll=lambda: 1)
  client.sock = SimpleNamespace(close=lambda: None)
  client.submit({}, 1, 2, 3)
  assert client.failed


def test_modeld_keeps_original_artifact_and_output():
  source = (Path(__file__).parents[1] / 'modeld.py').read_text()
  assert "load_oob(open_file_chunked(pkl_path or modeld_pkl_path(usbgpu)))" in source
  assert 'split_output' not in source
  assert 'shadow_tinygrad' not in source
  assert "self.npy['prev_feat'][:] = model_output[self.output_slices['hidden_state']]" in source
  assert source.index('self.signal_shadow.capture(model_output, self.npy)') < source.index("self.npy['prev_feat'][:] = model_output")
