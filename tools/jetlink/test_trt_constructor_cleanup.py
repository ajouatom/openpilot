"""실제 TRT 생성자와 작은 가짜 allocator로 실패 회수를 검사한다. GPU 시험은 아니다."""
import ctypes
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import numpy as np
import pytest

ENGINE = Path(__file__).resolve().parents[2] / 'third_party/jetlink/jetlink/server/backends/trt/engine.py'


class AllocationFailure(RuntimeError):
  pass


class Allocator:
  def __init__(self, failure=None, cleanup_error=False):
    self.failure, self.cleanup_error = failure, cleanup_error
    self.calls = {}
    self.device, self.host, self.stream = set(), {}, set()
    self.next_id = 100
    self.releases, self.invalid_releases = [], []

  def check(self, kind):
    self.calls[kind] = self.calls.get(kind, 0) + 1
    if self.failure == (kind, self.calls[kind]):
      raise AllocationFailure('synthetic_allocation_failure')

  def stream_create(self):
    self.check('stream')
    self.next_id += 1
    self.stream.add(self.next_id)
    return self.next_id

  def malloc(self, size):
    self.check('device')
    self.next_id += 1
    self.device.add(self.next_id)
    return self.next_id

  def host_alloc(self, size):
    self.check('host')
    backing = ctypes.create_string_buffer(size)
    pointer = ctypes.addressof(backing)
    self.host[pointer] = backing
    return pointer

  def release(self, kind, pointer, active):
    self.releases.append((kind, pointer))
    if pointer not in active:
      self.invalid_releases.append((kind, pointer))
      raise RuntimeError('synthetic_double_free')
    if kind == 'device' and self.cleanup_error:
      raise RuntimeError('synthetic_release_failure')
    if isinstance(active, dict):
      del active[pointer]
    else:
      active.remove(pointer)

  def free(self, pointer):
    self.release('device', pointer, self.device)

  def host_free(self, pointer):
    self.release('host', pointer, self.host)

  def stream_destroy(self, stream):
    self.release('stream', stream, self.stream)

  def counts(self):
    return len(self.device), len(self.host), len(self.stream)


def load_engine(monkeypatch, allocator, address_failure=None):
  class Context:
    def __init__(self):
      self.count = 0

    def set_tensor_address(self, name, pointer):
      self.count += 1
      if self.count == address_failure:
        raise AllocationFailure('synthetic_address_failure')

  class Plan:
    num_io_tensors = 2
    def get_tensor_name(self, index): return ('input', 'output')[index]
    def get_tensor_mode(self, name): return 0 if name == 'input' else 1
    def get_tensor_shape(self, name): return (2,)
    def get_tensor_dtype(self, name): return 0
    def create_execution_context(self): return Context()

  class Logger:
    WARNING = 0
    def __init__(self, severity): pass

  trt = ModuleType('tensorrt')
  trt.Logger = Logger
  trt.DataType = NS(FLOAT=0, HALF=1, INT8=2, INT32=3, INT64=4, BOOL=5, UINT8=6)
  trt.TensorIOMode = NS(INPUT=0)
  trt.Runtime = lambda logger: NS(deserialize_cuda_engine=lambda raw: Plan())
  package = ModuleType('jetlink.server.backends.trt')
  package.cudart = allocator
  name = 'trt_constructor_cleanup_test'
  spec = importlib.util.spec_from_file_location(name, ENGINE)
  module = importlib.util.module_from_spec(spec)
  # 시험별 import만 격리한다. 실제 CUDA/TensorRT 라이브러리는 로드하지 않는다.
  monkeypatch.setitem(sys.modules, 'tensorrt', trt)
  monkeypatch.setitem(sys.modules, 'jetlink.server.backends.trt', package)
  monkeypatch.setitem(sys.modules, name, module)
  spec.loader.exec_module(module)
  return module


@pytest.fixture
def plan(tmp_path):
  path = tmp_path / 'fixture.plan'
  path.write_bytes(b'synthetic plan for allocation fixture')
  return str(path)


@pytest.mark.parametrize('failure', [('stream', 1), ('device', 1), ('host', 1), ('device', 2), ('host', 2)])
def test_constructor_failure_releases_already_owned_resources(monkeypatch, plan, failure):
  allocator = Allocator(failure)
  module = load_engine(monkeypatch, allocator)
  with pytest.raises(AllocationFailure, match='^synthetic_allocation_failure$'):
    module.TrtEngine(plan)
  assert allocator.counts() == (0, 0, 0)
  assert not allocator.invalid_releases


@pytest.mark.parametrize('stage', ['view', 'address'])
def test_binding_creation_failure_releases_pending_allocations(monkeypatch, plan, stage):
  allocator = Allocator()
  module = load_engine(monkeypatch, allocator, address_failure=2 if stage == 'address' else None)
  original, calls = module._np_from_ptr, []
  def view(*args):
    calls.append(1)
    if len(calls) == 2:
      raise AllocationFailure('synthetic_view_failure')
    return original(*args)
  if stage == 'view':
    monkeypatch.setattr(module, '_np_from_ptr', view)
  with pytest.raises(AllocationFailure):
    module.TrtEngine(plan)
  assert allocator.counts() == (0, 0, 0)
  assert not allocator.invalid_releases


def test_failed_retries_do_not_accumulate_and_later_success_closes_once(monkeypatch, plan):
  allocator = Allocator(('host', 2))
  module = load_engine(monkeypatch, allocator)
  with pytest.raises(AllocationFailure):
    module.TrtEngine(plan)
  assert allocator.counts() == (0, 0, 0)
  allocator.failure = ('device', allocator.calls['device'] + 1)
  with pytest.raises(AllocationFailure):
    module.TrtEngine(plan)
  assert allocator.counts() == (0, 0, 0)
  allocator.failure = None
  engine = module.TrtEngine(plan)
  assert allocator.counts() == (2, 2, 1)
  engine.close()
  assert allocator.counts() == (0, 0, 0)
  # 정상 allocator는 해제한 주소를 다시 쓸 수 있으므로 과거 주소의 재사용 자체는 거절하지 않는다.
  assert not allocator.invalid_releases


def test_failed_second_engine_preserves_first_engine(monkeypatch, plan):
  allocator = Allocator()
  module = load_engine(monkeypatch, allocator)
  healthy = module.TrtEngine(plan)
  healthy.host_input('input')[:] = (3.5, 4.5)
  allocator.failure = ('host', allocator.calls['host'] + 1)
  try:
    with pytest.raises(AllocationFailure):
      module.TrtEngine(plan)
    assert allocator.counts() == (2, 2, 1)
    np.testing.assert_array_equal(healthy.host_input('input'), [3.5, 4.5])
  finally:
    healthy.close()
  assert allocator.counts() == (0, 0, 0)
  assert not allocator.invalid_releases


def test_cleanup_error_preserves_original_error_and_continues_other_releases(monkeypatch, plan, caplog):
  allocator = Allocator(('host', 2), cleanup_error=True)
  module = load_engine(monkeypatch, allocator)
  with caplog.at_level('WARNING', logger='jetlink.trt'), pytest.raises(AllocationFailure, match='^synthetic_allocation_failure$'):
    module.TrtEngine(plan)
  # 드라이버의 free 실패까지 회수 성공으로 표시하지 않는다.
  assert allocator.counts() == (2, 0, 0)
  assert len([row for row in allocator.releases if row[0] == 'device']) == 2
  assert not allocator.invalid_releases


def test_successful_constructor_keeps_live_buffers_until_close(monkeypatch, plan):
  allocator = Allocator()
  module = load_engine(monkeypatch, allocator)
  engine = module.TrtEngine(plan)
  assert allocator.counts() == (2, 2, 1)
  assert not allocator.releases
  inputs = engine.host_input('input')
  inputs[:] = (1.25, -2.5)
  np.testing.assert_array_equal(engine.host_input('input'), [1.25, -2.5])
  assert engine.input_shapes == {'input': (2,)}
  assert engine.output_shapes == {'output': (2,)}
  engine.close()
  assert allocator.counts() == (0, 0, 0)
  assert not allocator.invalid_releases
