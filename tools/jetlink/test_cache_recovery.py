"""복구 기록 손상은 보존·무시하고 정상 모델 명세는 그대로 사용한다."""
import json
import sys
import threading
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'third_party/jetlink'))
from jetlink.server.cache import EngineCache
from jetlink.server.session import EngineHost
from jetlink.spec import ModelSpec

BACKEND = NS(name='fixture', suffix='.plan', tag=lambda: 'fixture')


def model_spec(sha='a' * 64):
  return ModelSpec(sha256=sha, nbytes=32, frame_skip=4,
                   input_shapes={'new_img': (1, 1, 1, 2), 'state_history': (1, 2)},
                   output_shapes={'outputs': (1, 3), 'next_state_history': (1, 2)},
                   output_slices={'plan': slice(0, 3)}, checkpoint=None)


@pytest.fixture
def configured(tmp_path):
  spec = model_spec()
  cache = EngineCache(tmp_path, BACKEND)
  cache.remember_loaded(spec.sha256, spec.frame_skip)
  entry = cache.entry(spec.sha256)
  entry.path.write_bytes(b'fixture-plan-not-executed')
  entry.meta_path.write_text(json.dumps({'spec': spec.to_dict()}))
  host = EngineHost.__new__(EngineHost)
  host.cache, host.loaded, host.job, host.lock = cache, None, None, threading.RLock()
  starts = []
  host._start = lambda *args: starts.append(args)
  return spec, cache, entry, host, starts


@pytest.mark.parametrize('bad', [[], None, {'spec': [1]}, {'spec': {'sha256': 'a' * 64}},
                                {'spec': {**model_spec().to_dict(), 'sha256': 42}}])
def test_malformed_sidecar_is_preserved_and_ignored(configured, bad):
  _, _, entry, host, starts = configured
  entry.meta_path.write_text(json.dumps(bad))
  previous = entry.meta_path.read_bytes()
  host.preload()
  assert starts == []
  assert entry.meta_path.read_bytes() == previous


@pytest.mark.parametrize('bad', [None, [], {'sha256': 42, 'frame_skip': 4},
                                {'sha256': None, 'frame_skip': 4}, {'sha256': 'a' * 64, 'frame_skip': True},
                                {'sha256': 'a' * 64, 'frame_skip': 0}, {'sha256': 'a' * 64, 'frame_skip': '4'}])
def test_malformed_last_loaded_is_preserved_and_ignored(configured, bad):
  _, cache, _, host, starts = configured
  path = cache.root / 'last-loaded.json'
  path.write_text(json.dumps(bad))
  previous = path.read_bytes()
  host.preload()
  assert starts == []
  assert path.read_bytes() == previous


@pytest.mark.parametrize('sha', ['a' * 64, 'b' * 64])
def test_valid_model_contract_is_not_target_specific(tmp_path, sha):
  spec = model_spec(sha)
  cache = EngineCache(tmp_path, BACKEND)
  cache.remember_loaded(sha, 4)
  entry = cache.entry(sha)
  entry.path.write_bytes(b'fixture-plan')
  entry.write_meta({'spec': spec.to_dict()})
  host = EngineHost.__new__(EngineHost)
  host.cache, host.loaded, host.job, host.lock = cache, None, None, threading.RLock()
  starts = []
  host._start = lambda *args: starts.append(args)
  host.preload()
  assert len(starts) == 1
  assert starts[0][-1].to_dict() == spec.to_dict()


def test_truncated_json_keeps_original_skip_behavior(configured):
  _, _, entry, host, starts = configured
  entry.meta_path.write_text('{')
  host.preload()
  assert starts == []
  assert entry.meta_path.read_text() == '{'


def test_bad_spec_can_use_original_model_revalidation(configured, monkeypatch):
  spec, cache, entry, host, starts = configured
  model = cache.model_path(spec.sha256)
  model.write_bytes(b'x' * spec.nbytes)
  entry.meta_path.write_text(json.dumps({'spec': {'sha256': spec.sha256}}))
  calls = []

  def derive(path, frame_skip):
    assert path == model and frame_skip == spec.frame_skip
    calls.append(True)
    return spec

  monkeypatch.setattr(host, '_derive_spec', derive)
  request = NS(sha256=spec.sha256, nbytes=spec.nbytes, frame_skip=spec.frame_skip)
  assert host._spec_on_disk(entry, model, request).to_dict() == spec.to_dict()
  assert calls == [True]
  assert starts == []


def test_bad_spec_without_model_stays_unavailable(configured):
  spec, cache, entry, host, starts = configured
  entry.meta_path.write_text(json.dumps({'spec': [1]}))
  request = NS(sha256=spec.sha256, nbytes=spec.nbytes, frame_skip=spec.frame_skip)
  assert host._spec_on_disk(entry, cache.model_path(spec.sha256), request) is None
  assert starts == []


def test_cached_size_mismatch_stays_unavailable_without_model(configured):
  spec, cache, entry, host, starts = configured
  request = NS(sha256=spec.sha256, nbytes=spec.nbytes + 1, frame_skip=spec.frame_skip)
  assert host._spec_on_disk(entry, cache.model_path(spec.sha256), request) is None
  assert starts == []
