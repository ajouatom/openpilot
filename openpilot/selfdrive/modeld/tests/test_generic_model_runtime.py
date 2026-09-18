import copy
import ast
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

from openpilot.selfdrive.modeld.generic_model_runtime import GenericModelRuntime, model_metadata
from openpilot.selfdrive.modeld.parse_model_outputs import Parser


@pytest.fixture
def artifact():
  # Metadata extracted without executing the official PR #38932 pickle.
  return json.loads((Path(__file__).parent / 'fixtures/cinque_v3_metadata.json').read_text())


def test_official_v3_output_contract_and_internal_history(artifact):
  checkpoint, slices, pairs, count = model_metadata(artifact)
  assert '/f78ed37d-afad-4dbc-8050-40ea885eedde/12864' in checkpoint
  assert pairs == {name: f'next_{name}' for name in ('state_img_q', 'state_desire_q', 'state_feat_q')}
  assert count == 18452
  raw = np.zeros((1, count), np.float32)
  outputs = Parser().parse_outputs({name: raw[:, section].copy() for name, section in slices.items()})
  assert outputs['plan'].shape == (1, 33, 15)
  assert np.isfinite(outputs['action']).all()


@pytest.mark.parametrize('change', ['state_shape', 'state_dtype', 'image_dtype', 'unknown_input', 'device'])
def test_reject_incompatible_generic_artifact(artifact, change):
  if change == 'state_shape':
    artifact['output_specs']['next_state_feat_q'][0][0] += 1
  elif change == 'state_dtype':
    artifact['output_specs']['next_state_feat_q'][1] = 'float16'
  elif change == 'image_dtype':
    artifact['input_specs']['new_img'][1] = 'float32'
  elif change == 'unknown_input':
    artifact['input_specs']['unknown'] = copy.deepcopy(artifact['input_specs']['desire'])
  else:
    artifact['input_specs']['new_img'][2] = 'QCOM'
  with pytest.raises(ValueError):
    model_metadata(artifact)


def test_generic_dispatch_uploads_warps_and_feeds_back_state(monkeypatch):
  from openpilot.selfdrive.modeld import generic_model_runtime
  wall_clock = iter([0, .01, .03, .06, .10, .10, .12, .13, .17, .23])
  cpu_clock = iter([0, .001, .003, .006, .010, .010, .012, .013, .017, .023])
  monkeypatch.setattr(generic_model_runtime, 'time', SimpleNamespace(
    monotonic=lambda: next(wall_clock), thread_time=lambda: next(cpu_clock)))
  runtime = object.__new__(GenericModelRuntime)
  calls = []
  runtime.host, runtime.frames, runtime.transforms = object(), object(), object()
  runtime.device_buffer = SimpleNamespace(copy_from=lambda host: calls.append(('upload', host)))
  state = np.zeros(1)
  runtime.queues = {'state': state}
  result = np.array([[1, 2]], np.float32)
  runtime.outputs = {'next_state': state, 'outputs': SimpleNamespace(numpy=lambda: result)}
  def warp(**inputs):
    assert inputs == {'input_frame': runtime.frames, 'M_inv': runtime.transforms}
    calls.append(('warp',))
    return 'warped image'
  def run(output_buffers, **inputs):
    assert inputs['new_img'] == 'warped image'
    assert output_buffers['next_state'] is inputs['state']
    state[:] += 1
    calls.append(('model',))
  runtime.run_warp, runtime.run_model = warp, run
  np.testing.assert_array_equal(runtime.run(), [1, 2])
  assert runtime.last_timings['input_upload_ms'] == pytest.approx(10)
  assert runtime.last_timings['output_read_ms'] == pytest.approx(40)
  runtime.run()
  assert state[0] == 2
  assert [call[0] for call in calls] == ['upload', 'warp', 'model'] * 2
  assert runtime.last_timings == pytest.approx({
    'input_upload_ms': 20, 'input_upload_cpu_ms': 2,
    'warp_call_ms': 10, 'warp_call_cpu_ms': 1,
    'model_call_ms': 40, 'model_call_cpu_ms': 4,
    'output_read_ms': 60, 'output_read_cpu_ms': 6,
  })


def test_precompiled_only_boot_failure_skips_local_compilation(monkeypatch, tmp_path):
  from openpilot.selfdrive.modeld import big_model, big_model_status, precompiled_model
  source = Path(__file__).parents[3] / 'system/manager/build.py'
  tree = ast.parse(source.read_text(encoding='utf8'))
  function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'build_usbgpu_model')
  namespace = {'Spinner': object}
  exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
  monkeypatch.setattr(big_model, 'active_manifest', big_model.fetch_manifest)
  monkeypatch.setattr(big_model, 'active_model_path', lambda: tmp_path / 'model.pkl')
  monkeypatch.setattr(big_model, 'model_cache_dir', lambda: tmp_path)
  statuses = []
  monkeypatch.setattr(big_model_status, 'write_big_model_status', lambda *args, **kw: statuses.append((args, kw)))
  def unavailable(*args, **kwargs):
    raise OSError('test server offline')
  monkeypatch.setattr(precompiled_model, 'ensure_precompiled', unavailable)
  monkeypatch.setitem(sys.modules, 'openpilot.system.hardware.usbgpu', SimpleNamespace(check_usbgpu=lambda: None))
  assert namespace['build_usbgpu_model'](SimpleNamespace(update=lambda *args: None)) is False
  assert statuses[-1][0][1] == 'error'
  assert 'using internal model' in statuses[-1][1]['detail']
