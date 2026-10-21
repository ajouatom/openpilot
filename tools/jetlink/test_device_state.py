"""Deferred CUDA work checks recurrence/reset ordering; not real GPU validation."""
import ctypes
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'third_party/jetlink'))
from jetlink.queues import make_queues

TRT = Path(__file__).resolve().parents[2] / 'third_party/jetlink/jetlink/server/backends/trt'
PREFIX = 'jetlink.server.backends.trt'


class DeferredCuda:
  def __init__(self):
    self.memory, self.pending, self.capturing = {}, [], None
    self.calls = []

  def malloc(self, size):
    buf = ctypes.create_string_buffer(size)
    ptr = ctypes.addressof(buf)
    self.memory[ptr] = buf
    return ptr

  host_alloc = malloc

  def free(self, ptr):
    assert not self.pending, 'freed memory before trailing GPU work'
    del self.memory[ptr]

  host_free = free

  def enqueue(self, label, action):
    (self.pending if self.capturing is None else self.capturing).append((label, action))

  def copy(self, kind, dst, src, size, stream):
    self.calls.append((kind, dst, src, size))
    self.enqueue(kind, lambda: ctypes.memmove(dst, src, size))

  def memcpy_h2d_async(self, *args): self.copy('h2d', *args)
  def memcpy_d2h_async(self, *args): self.copy('d2h', *args)
  def memcpy_d2d_async(self, *args): self.copy('d2d', *args)

  def memset_async(self, ptr, size, stream):
    self.enqueue('zero', lambda: ctypes.memset(ptr, 0, size))

  def event_create(self): return object()

  def event_record(self, event, stream, *, external=False):
    assert external == (self.capturing is not None)
    self.enqueue(event, lambda: None)

  def event_sync(self, event):
    while self.pending:
      label, action = self.pending.pop(0)
      action()
      if label is event:
        return
    raise AssertionError('no new reply event')

  def stream_sync(self, stream):
    while self.pending:
      _, action = self.pending.pop(0)
      action()

  def stream_begin_capture(self, stream):
    assert self.capturing is None
    self.capturing = []

  def stream_end_capture(self, stream):
    graph, self.capturing = self.capturing, None
    return graph

  def graph_instantiate(self, graph): return list(graph)
  def graph_launch(self, graph, stream): self.pending.extend(graph)
  def graph_destroy(self, graph): pass
  def graph_exec_destroy(self, graph): pass
  def event_destroy(self, event): pass
  def stream_destroy(self, stream): pass


@pytest.fixture
def runtime(monkeypatch):
  cuda = DeferredCuda()
  pkg = ModuleType(PREFIX)
  pkg.__path__ = [str(TRT)]
  pkg.cudart = cuda
  monkeypatch.setitem(sys.modules, PREFIX, pkg)
  monkeypatch.setitem(sys.modules, PREFIX + '.cudart', cuda)
  trt = ModuleType('tensorrt')
  trt.Logger = NS(WARNING=0)
  trt.DataType = NS(**{n: n for n in ('FLOAT', 'HALF', 'INT8', 'INT32', 'INT64', 'BOOL', 'UINT8')})
  monkeypatch.setitem(sys.modules, 'tensorrt', trt)

  def load(name):
    spec = importlib.util.spec_from_file_location(PREFIX + '.' + name, TRT / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module

  device = load('device_state')
  engine = load('engine')
  real_state = device.DeviceState

  class Kernel:
    def launch(self, binding, bad, stream):
      def scan():
        values = np.frombuffer(cuda.memory[binding.device_ptr], dtype=binding.dtype)
        if not np.isfinite(values).all(): ctypes.c_uint32.from_address(bad).value |= 1
      cuda.enqueue('finite', scan)

    def close(self):
      assert not cuda.pending

  monkeypatch.setattr(device, 'DeviceState', lambda *a: real_state(*a, kernel_factory=Kernel))
  return NS(cuda=cuda, engine=engine, device=device, state=real_state, kernel=Kernel)


def make_engine(runtime, enabled, monkeypatch, dtype=np.float16):
  monkeypatch.setenv('CARROT_TRT_DEVICE_STATE', '1' if enabled else '0')
  e = runtime.engine.TrtEngine.__new__(runtime.engine.TrtEngine)
  e.stream, e.graph_exec, e.device_state, e._state_host_sources = 1, None, None, {}
  e.bindings = {}
  pairs = {'state_a': 'next_state_a', 'state_b': 'next_state_b'}
  names = {'new_img': ((2, 2), dtype), 'desire': ((1, 2), np.float32),
           'state_a': ((1, 3), dtype), 'state_b': ((1, 2), np.float32),
           'next_state_a': ((1, 3), dtype), 'next_state_b': ((1, 2), np.float32),
           'outputs': ((1, 2), np.float32)}
  for name, (shape, typ) in names.items():
    typ = np.dtype(typ)
    size = int(np.prod(shape)) * typ.itemsize
    dev, host = runtime.cuda.malloc(size), runtime.cuda.host_alloc(size)
    view = runtime.engine._np_from_ptr(host, shape, typ)
    e.bindings[name] = runtime.engine.Binding(name, shape, typ, size, dev, host, view,
                                            not name.startswith('next_') and name != 'outputs')
  e.inputs = {n: b for n, b in e.bindings.items() if b.is_input}
  e.outputs = {n: b for n, b in e.bindings.items() if not b.is_input}
  e.bad_name = None

  def view(name):
    b = e.bindings[name]
    return np.frombuffer(runtime.cuda.memory[b.device_ptr], dtype=b.dtype).reshape(b.shape)

  def execute(stream):
    def compute():
      delta = view('new_img').mean() + view('desire')[0, 0]
      for name, out in pairs.items(): view(out)[:] = view(name) + delta
      view('outputs')[:] = [view('next_state_a').sum(), view('next_state_b').sum()]
      if e.bad_name: view(e.bad_name).flat[-1] = np.nan
    runtime.cuda.enqueue('execute', compute)
    return True

  e.context = NS(execute_async_v3=execute)
  e.configure_state_feedback(pairs)
  spec = NS(stateful=True, state_pairs=pairs, input_shapes=e.input_shapes, output_shapes=e.output_shapes,
            warped_shape=(2, 2), packed_shapes={'desire': (1, 2)}, packed_sizes=[2], packed_nelem=2)
  host_inputs = {n: e.host_input(n) for n in e.inputs}
  queues = make_queues(spec, host_inputs, e.device_state)
  return e, queues, host_inputs


@pytest.mark.parametrize('graph', [False, True])
@pytest.mark.parametrize('dtype', [np.float16, np.float32])
def test_recurrence_matches_host_path_and_reset_discards_trailing_feedback(runtime, monkeypatch, graph, dtype):
  results = []
  for enabled in (False, True):
    e, q, host = make_engine(runtime, enabled, monkeypatch, dtype)
    q.step_into(np.zeros((2, 2), np.uint8), np.zeros(2, np.float32), host)
    if graph:
      e.warm()
      q.reset()
    values = []
    for i in range(8):
      if i == 5: q.reset()  # Previous reply left D2D queued; reset must follow it.
      q.step_into(np.full((2, 2), i + 1, np.uint8), np.array([i / 2, 0], np.float32), host)
      out = e.run()
      q.feedback(out)
      values.append(out['outputs'].copy())
      if enabled:
        assert set(out) == {'outputs'}
        assert [name for name, _ in runtime.cuda.pending] == ['d2d', 'd2d']
    e.close()
    assert not runtime.cuda.memory and not runtime.cuda.pending
    results.append(values)
  np.testing.assert_array_equal(results[0], results[1])


@pytest.mark.parametrize('bad_name', ['outputs', 'next_state_a', 'next_state_b'])
def test_nonfinite_output_rejected_and_next_reset_frame_is_fresh(runtime, monkeypatch, bad_name):
  e, q, host = make_engine(runtime, True, monkeypatch)
  q.step_into(np.ones((2, 2), np.uint8), np.zeros(2, np.float32), host)
  e.bad_name = bad_name
  with pytest.raises(FloatingPointError): q.feedback(e.run())
  assert e.device_state.needs_reset
  e.bad_name = None
  q.feedback(e.run())
  np.testing.assert_array_equal(e.outputs['outputs'].host, [[3, 2]])
  e.close()


def test_state_is_not_transferred_to_host_and_explicit_upload_is_rejected(runtime, monkeypatch):
  e, q, host = make_engine(runtime, True, monkeypatch)
  q.step_into(np.ones((2, 2), np.uint8), np.zeros(2, np.float32), host)
  q.feedback(e.run())
  calls = runtime.cuda.calls
  state_in = {e.inputs[n].device_ptr for n in q.spec.state_pairs}
  state_out = {e.outputs[n].device_ptr for n in q.spec.state_pairs.values()}
  assert not any(kind == 'h2d' and dst in state_in for kind, dst, src, size in calls)
  assert not any(kind == 'd2h' and src in state_out for kind, dst, src, size in calls)
  assert sum(size for kind, dst, src, size in calls if kind == 'd2h') == 12  # driving output + status
  with pytest.raises(RuntimeError, match='queue-managed'): e.load_inputs(host)
  e.close()


@pytest.mark.parametrize('enabled', [False, True])
def test_capture_failure_ends_capture_and_refuses_device_mode(runtime, monkeypatch, enabled):
  e, q, host = make_engine(runtime, enabled, monkeypatch)
  e.context.execute_async_v3 = lambda stream: False
  if enabled:
    with pytest.raises(RuntimeError, match='execute_async'): e.capture_graph()
  else:
    assert not e.capture_graph()
  assert runtime.cuda.capturing is None and e.graph_exec is None
  e.close()


def test_enabled_by_default_explicit_disable_and_bad_contract_refused(runtime, monkeypatch):
  e, q, host = make_engine(runtime, False, monkeypatch)
  assert e.device_state is None
  monkeypatch.delenv('CARROT_TRT_DEVICE_STATE')
  e.configure_state_feedback(q.spec.state_pairs)
  assert e.device_state is not None
  with pytest.raises(ValueError, match='nonempty'):
    runtime.state(e.inputs, e.outputs, {}, e.stream, kernel_factory=runtime.kernel)
  e.close()
