"""캐시 저장 IO 회귀. TensorRT 계산은 작은 직렬화 바이트를 반환하는 대역이다."""
from __future__ import annotations

import ast
import errno
import json
import logging
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'third_party/jetlink'))
from jetlink.server import cache

OLD_PLAN = b'previous-valid-fixture-plan'
NEW_PLAN = b'new-serialized-fixture-plan'


def build_function():
  # 현재 함수 본문을 그대로 실행하고 GPU/ONNX 계산 경계만 대체한다.
  path = ROOT / 'third_party/jetlink/jetlink/server/backends/trt/build.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  node = next(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == 'build_engine')

  class Logger:
    WARNING = 1

    def __init__(self, *args):
      pass

  class Builder:
    def __init__(self, *args):
      pass

    def create_network(self, *args):
      return NS(num_layers=1)

    def create_builder_config(self):
      return NS(set_memory_pool_limit=lambda *args: None)

    def build_serialized_network(self, *args):
      return NEW_PLAN

  fake = NS(Logger=Logger, Builder=Builder, __version__='fixture-no-gpu',
            OnnxParser=lambda *args: NS(parse_from_file=lambda path: True), MemoryPoolType=NS(WORKSPACE=0))
  names = {'Path': Path, 'json': json, 'os': os, 'shutil': shutil, 'tempfile': tempfile, 'time': time,
           'log': logging.getLogger('jetlink-storage-test'), 'trt': fake,
           'workspace_bytes': lambda: 256 << 20, 'available_bytes': lambda: 1 << 30,
           'network_flags': lambda: 0, 'configure_precision': lambda *args: 'fixture',
           'device_tag': lambda: 'fixture', 'strongly_typed_only': lambda: False,
           '_Monitor': lambda *args: NS(), '_load_timing_cache': lambda *args: None,
           '_save_timing_cache': lambda *args: None}
  future = ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)
  module = ast.fix_missing_locations(ast.Module(body=[future, node], type_ignores=[]))
  exec(compile(module, str(path), 'exec'), names)
  return names['build_engine']


def entry_at(tmp_path):
  entry = cache.CacheEntry(tmp_path / 'model.plan', tmp_path / 'model.json')
  entry.path.write_bytes(OLD_PLAN)
  entry.meta_path.write_text(json.dumps({'generation': 'previous'}))
  return entry


def run_build(entry, monkeypatch):
  model = entry.path.with_suffix('.onnx')
  model.write_bytes(b'fixture-not-an-onnx-model')
  monkeypatch.setitem(sys.modules, 'jetlink.onnx_patch',
                      NS(patch_file=lambda source, dest: Path(dest).write_bytes(Path(source).read_bytes())))
  return build_function()(model, entry.path, meta_extra={'fixture': True})


def test_sidecar_partial_write_preserves_previous_json(tmp_path, monkeypatch):
  entry = entry_at(tmp_path)
  original = entry.meta_path.read_bytes()
  write_text = Path.write_text

  def partial(path, text, *args, **kwargs):
    if path == entry.meta_path:
      with path.open('w') as output:
        output.write(text[:4])
      raise OSError(errno.ENOSPC, 'fixture-full')
    return write_text(path, text, *args, **kwargs)

  with monkeypatch.context() as change:
    change.setattr(Path, 'write_text', partial)
    change.setattr(os, 'fsync', lambda *_: (_ for _ in ()).throw(OSError(errno.ENOSPC, 'fixture-full')))
    with pytest.raises(OSError):
      entry.write_meta({'generation': 'next'})
  assert entry.meta_path.read_bytes() == original
  assert not list(tmp_path.glob('.model.json-*'))


def test_sidecar_replace_failure_preserves_previous_json(tmp_path, monkeypatch):
  entry = entry_at(tmp_path)
  original = entry.meta_path.read_bytes()
  with monkeypatch.context() as change:
    change.setattr(cache.os, 'replace', lambda *_: (_ for _ in ()).throw(OSError(errno.EROFS, 'fixture-readonly')))
    with pytest.raises(OSError):
      entry.write_meta({'generation': 'next'})
  assert entry.meta_path.read_bytes() == original
  assert not list(tmp_path.glob('.model.json-*'))


@pytest.mark.skipif(os.name != 'posix', reason='POSIX file mode semantics')
def test_sidecar_success_preserves_existing_mode(tmp_path):
  entry = entry_at(tmp_path)
  entry.meta_path.chmod(0o640)
  entry.write_meta({'generation': 'next'})
  assert entry.meta() == {'generation': 'next'}
  assert entry.meta_path.stat().st_mode & 0o777 == 0o640


def test_plan_publish_failure_does_not_copy_over_previous_plan(tmp_path, monkeypatch):
  entry = entry_at(tmp_path)
  rename, replace, copyfile = os.rename, os.replace, shutil.copyfile
  copies = []

  def fail_publish(operation):
    def publish(source, target, *args, **kwargs):
      if Path(source).name == 'engine.plan':
        raise OSError(errno.ENOSPC, 'fixture-full')
      return operation(source, target, *args, **kwargs)
    return publish

  def partial_copy(source, target, *args, **kwargs):
    if Path(source).name == 'engine.plan':
      copies.append(True)
      Path(target).write_bytes(NEW_PLAN[:4])
      raise OSError(errno.ENOSPC, 'fixture-full')
    return copyfile(source, target, *args, **kwargs)

  monkeypatch.setattr(os, 'rename', fail_publish(rename))
  monkeypatch.setattr(os, 'replace', fail_publish(replace))
  monkeypatch.setattr(shutil, 'copyfile', partial_copy)
  with pytest.raises(OSError):
    run_build(entry, monkeypatch)
  assert entry.path.read_bytes() == OLD_PLAN
  assert copies == []


def test_build_success_keeps_metadata_contract(tmp_path, monkeypatch):
  entry = entry_at(tmp_path)
  assert run_build(entry, monkeypatch) == entry.path
  assert entry.path.read_bytes() == NEW_PLAN
  assert entry.meta()['fixture'] is True
  assert not list(tmp_path.glob('tmp*'))


def test_metadata_failure_is_reported_after_plan_publication(tmp_path, monkeypatch):
  entry = entry_at(tmp_path)
  before = entry.meta_path.read_bytes()
  # 이 검사는 두 파일 transaction을 보장하지 않는다. 남는 한계를 명시한다.
  monkeypatch.setattr(cache.CacheEntry, 'write_meta',
                      lambda *_: (_ for _ in ()).throw(OSError(errno.ENOSPC, 'fixture-full')))
  with pytest.raises(OSError):
    run_build(entry, monkeypatch)
  assert entry.path.read_bytes() == NEW_PLAN
  assert entry.meta_path.read_bytes() == before
