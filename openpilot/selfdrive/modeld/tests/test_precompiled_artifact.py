import copy
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

from openpilot.selfdrive.modeld.precompiled_artifact import load_artifact, prepare_jit
from openpilot.selfdrive.modeld.generic_model_runtime import model_metadata
from openpilot.selfdrive.modeld.precompiled_model import validate_catalog
from openpilot.selfdrive.modeld.parse_model_outputs import Parser


@pytest.mark.parametrize('previous_fixture', ['cinque_v3_metadata.json', 'mdm_v1_metadata.json'])
def test_mdm_contract_matches_parser_and_preserves_history(previous_fixture):
  fixtures = Path(__file__).parent / 'fixtures'
  mdm = json.loads((fixtures / 'mdm2_metadata.json').read_text())
  previous = json.loads((fixtures / previous_fixture).read_text())
  checkpoint, slices, pairs, count = model_metadata(mdm)
  assert '/870a4823-d2ec-4b9a-8015-b947e9901fc7/12864' in checkpoint
  def normalize(specs):
    return {name: (shape, np.dtype(dtype), device) for name, (shape, dtype, device) in specs.items()}
  for kind in ('input_specs', 'output_specs'):
    assert normalize(mdm[kind]) == normalize(previous[kind])
  assert slices == model_metadata(previous)[1]
  assert set(pairs) == {'state_img_q', 'state_desire_q', 'state_feat_q'}
  raw = np.zeros((1, count), np.float32)
  parsed = Parser().parse_outputs({name: raw[:, section].copy() for name, section in slices.items()})
  assert parsed['plan'].shape == (1, 33, 15)
  assert np.isfinite(parsed['action']).all()


def test_persistent_loader_uses_upstream_arena_loader(monkeypatch, tmp_path):
  artifact, calls = object(), []
  def load(path, *, out_of_band):
    calls.append((path, out_of_band))
    return artifact
  monkeypatch.setitem(sys.modules, 'examples.openpilot.helpers', SimpleNamespace(load_pickle=load))
  path = tmp_path / 'model.pkl'
  assert load_artifact(path, 'persistent-buffer-v1') is artifact
  assert calls == [(path, True)]
  with pytest.raises(ValueError, match='serialization'):
    load_artifact(path, 'unknown')


def test_legacy_loader_and_trailing_data(monkeypatch, tmp_path):
  from openpilot.selfdrive.modeld.helpers import dump_oob
  path = tmp_path / 'model.pkl'
  with path.open('wb') as stream:
    dump_oob({'metadata': 'old format'}, stream)
  assert load_artifact(path, 'oob-v1') == {'metadata': 'old format'}
  with path.open('ab') as stream:
    stream.write(b'unexpected')
  with pytest.raises(ValueError, match='trailing'):
    load_artifact(path, 'oob-v1')


def test_retargetable_dispatch_is_compiled_before_use(monkeypatch):
  original, compiled = object(), object()
  calls = []
  def lower(linear):
    calls.append(linear)
    return compiled
  monkeypatch.setitem(sys.modules, 'tinygrad.engine.realize', SimpleNamespace(lower_and_compile=lower))
  jit = SimpleNamespace(captured=SimpleNamespace(_linear=original))
  assert prepare_jit(jit) is jit
  assert jit.captured._linear is compiled and calls == [original]
  legacy = SimpleNamespace()
  assert prepare_jit(legacy) is legacy
  assert calls == [original]


@pytest.mark.parametrize('serialization,artifact_format,valid', [
  ('persistent-buffer-v1', 'comma-generic-onnx', True),
  ('oob-v1', 'comma-generic-onnx', True),
  ('persistent-buffer-v1', 'comma-run-model', False),
  ('unknown', 'comma-generic-onnx', False),
])
def test_catalog_serialization_is_explicit(serialization, artifact_format, valid):
  artifact = {'sha256': 'a' * 64, 'size': 1, 'url': 'artifact'}
  value = {'protocol': 1, 'format': artifact_format, 'serialization': serialization, 'gpu_arch': 'gfx1200',
           'frame_skip': 4, 'camera_resolutions': [[1928, 1208], [1344, 760]],
           'model_sha256': 'a' * 64, 'onnx_sha256': 'a' * 64,
           'pickle': copy.deepcopy(artifact), 'runtime': copy.deepcopy(artifact)}
  if valid:
    assert validate_catalog(value, 'a' * 64, 'https://nas.example/precompiled.json') is value
  else:
    with pytest.raises(ValueError, match='serialization'):
      validate_catalog(value, 'a' * 64, 'https://nas.example/precompiled.json')
