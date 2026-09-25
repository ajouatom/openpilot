"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

ONNX -> a captured tinygrad JIT, pickled with its weights.

The same job openpilot's compile_modeld.py does for the comma's GPU, minus the
warp and the queues, which live on the comma and in jetlink.queues. Three calls
make a JIT: the first runs eagerly and compiles every kernel, the second
captures the linear program, the third replays it. The replay has to reproduce
the capture bit for bit on the same inputs, and that is asserted before
anything is written, as compile_modeld does.

Inputs are staged on tinygrad's NPY device: a tensor that *is* a numpy array,
so the server writes the model's inputs straight into it and the host-to-device
copy is captured inside the JIT. That is the same "write into the staging
buffer, then run" shape TensorRT gets from pinned memory.

The pickle is written out of band the way the fork's helpers do it: the
opcodes first, then every buffer in turn, so peak memory is one buffer rather
than the whole model twice.
"""
from __future__ import annotations

import io
import json
import logging
import os
import pickle
import shutil
import struct
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from jetlink.server.backends.base import ProgressFn, write_sidecar

log = logging.getLogger('jetlink.tinygrad')

# Bumped when what the pickle contains changes shape; an older pickle is
# ArtifactInvalid and rebuilt.
FORMAT = 1

# Everything the JIT is fed with during the capture calls, so the pruned graph
# and the replay check see values a road could produce rather than zeros that
# fold whole layers away.
SEED = 42


@dataclass(frozen=True)
class Staged:
  """One model input as the engine stages it: the numpy dtype the host writes
  and the dtype the graph declared, which differ for the uint8 images."""
  name: str
  shape: tuple[int, ...]
  host_dtype: str      # numpy name
  model_dtype: str     # numpy name


def tinygrad_identity() -> str:
  """What version of tinygrad this is, precisely enough to key a pickle on.

  The package version first, then the git commit when there is one: openpilot
  pins tinygrad as a submodule at a sha, and its metadata version can lag the
  code by a release. A pickle from another commit may still load; one that
  does not is ArtifactInvalid and rebuilds.

  Where the commit comes from matters. `git rev-parse` in the source directory
  walks *up* until it finds a repository, so an installed tinygrad inside an
  app bundle or a venv under a checkout reported that checkout's HEAD: the
  identity, and with it the cache key, changed on every commit to this repo
  and every prepared engine came back stale. pip records the real commit in
  direct_url.json for a git install, so ask that first, and only run git when
  the source directory is itself a repository.
  """
  import tinygrad
  version = 'unknown'
  try:
    from importlib.metadata import version as pkg_version
    version = pkg_version('tinygrad')
  except Exception:
    pass
  commit = _direct_url_commit()
  if commit:
    return f"{version}+{commit}"
  src = Path(tinygrad.__file__).resolve().parent.parent
  if not (src / '.git').exists():
    return version
  try:
    sha = subprocess.run(['git', '-C', str(src), 'rev-parse', '--short=12', 'HEAD'],
                         capture_output=True, text=True, timeout=5).stdout.strip()
    if sha:
      return f"{version}+{sha}"
  except (OSError, subprocess.SubprocessError):
    pass
  return version


def _direct_url_commit() -> str | None:
  """The commit pip installed tinygrad from, per PEP 610, or None."""
  try:
    from importlib.metadata import distribution
    raw = distribution('tinygrad').read_text('direct_url.json')
    if not raw:
      return None
    commit = json.loads(raw).get('vcs_info', {}).get('commit_id')
    return str(commit)[:12] if commit else None
  except Exception:
    return None


def host_dtype_for(model_dtype: np.dtype) -> np.dtype:
  """The queues gather into float16 at memcpy speed and 0..255 is exact in
  it, so a uint8 graph input is staged as fp16 and cast on the device; that
  is one small kernel and nothing in the queues."""
  return np.dtype(np.float16) if np.dtype(model_dtype) == np.uint8 else np.dtype(model_dtype)


def plan_inputs(runner) -> list[Staged]:
  from tinygrad.dtype import _to_np_dtype
  out = []
  for name, spec in runner.graph_inputs.items():
    shape = tuple(int(d) for d in spec.shape)
    if any(d <= 0 for d in shape):
      raise ValueError(f"input {name} has a dynamic shape {spec.shape}; jetlink builds fixed-shape engines")
    model_dtype = np.dtype(_to_np_dtype(spec.dtype))
    out.append(Staged(name, shape, host_dtype_for(model_dtype).name, model_dtype.name))
  return out


def make_staging(plan: list[Staged]) -> tuple[dict[str, np.ndarray], dict]:
  """Numpy arrays the host writes into, and the NPY tensors that view them."""
  from tinygrad import Tensor
  arrays = {s.name: np.zeros(s.shape, np.dtype(s.host_dtype)) for s in plan}
  tensors = {name: Tensor(arr, device='NPY').realize() for name, arr in arrays.items()}
  return arrays, tensors


def make_fn(runner, plan: list[Staged], device: str):
  from tinygrad.dtype import _from_np_dtype
  from tinygrad.engine.jit import TinyJit

  casts = {s.name: _from_np_dtype(np.dtype(s.model_dtype)) for s in plan if s.model_dtype != s.host_dtype}

  def fn(**staged):
    inputs = {}
    for s in plan:
      t = staged[s.name].to(device)
      if s.name in casts:
        t = t.cast(casts[s.name])
      inputs[s.name] = t
    out = next(iter(runner(inputs).values()))
    # float32 out, as openpilot's JIT returns it and the protocol carries it
    return out.cast('float32').contiguous().realize()

  return TinyJit(fn, prune=True)


def fill_random(arrays: dict[str, np.ndarray], plan: list[Staged], seed: int) -> None:
  rng = np.random.default_rng(seed)
  for s in plan:
    a = arrays[s.name]
    if np.dtype(s.model_dtype) == np.uint8:
      a[...] = rng.integers(0, 256, s.shape)
    else:
      a[...] = (rng.standard_normal(s.shape) * 0.1).astype(a.dtype)


def _stop_compile_workers() -> None:
  """tinygrad compiles kernels on a pool of worker processes it keeps for the
  life of the process. The server compiles once and serves for hours, so the
  pool is idle memory afterwards; and a process that exits with the pool up
  beside another runtime (onnxruntime, in the test suite) was seen to abort in
  a C++ mutex during teardown. Older tinygrads have no pool to stop."""
  try:
    from tinygrad.engine.worker import terminate_worker_pool
  except ImportError:
    return
  try:
    terminate_worker_pool()
  except Exception as e:
    log.info("could not stop tinygrad's compile workers: %s", e)


def dump_oob(obj, f) -> None:
  """Protocol 5 pickle with the buffers appended after the opcodes."""
  with tempfile.TemporaryFile(dir=os.path.dirname(f.name) or '.') as bufs:
    def buffer_callback(pb: pickle.PickleBuffer):
      m = pb.raw()
      bufs.write(struct.pack('<q', m.nbytes))
      bufs.write(m)
      pb.release()   # keep peak RAM at one buffer
    ops = io.BytesIO()
    pickle.Pickler(ops, protocol=5, buffer_callback=buffer_callback).dump(obj)
    opcodes = ops.getvalue()
    f.write(struct.pack('<q', len(opcodes)))
    f.write(opcodes)
    bufs.seek(0)
    shutil.copyfileobj(bufs, f)


def load_oob(f):
  head = f.read(8)
  if len(head) != 8:
    raise EOFError('not a jetlink tinygrad artifact')
  opcodes = f.read(struct.unpack('<q', head)[0])

  def buffers():
    while (h := f.read(8)):
      pb = pickle.PickleBuffer(bytearray(struct.unpack('<q', h)[0]))
      f.readinto(pb)
      yield pb
  return pickle.load(io.BytesIO(opcodes), buffers=buffers())


def build_jit(onnx_path: str | Path, out_path: str | Path, device: str,
              report: ProgressFn | None = None, meta_extra: dict | None = None) -> Path:
  from tinygrad import Device
  from tinygrad.nn.onnx import OnnxRunner

  onnx_path, out_path = Path(onnx_path), Path(out_path)
  report = report or (lambda *_: None)
  t0 = time.time()
  identity = tinygrad_identity()
  log.info("tinygrad %s on %s: compiling %s", identity, device, onnx_path.name)

  report('parse', 0.0, 'reading the model into tinygrad')
  runner = OnnxRunner(onnx_path)
  plan = plan_inputs(runner)
  arrays, tensors = make_staging(plan)
  jit = make_fn(runner, plan, device)
  report('parse', 1.0, f'{len(plan)} inputs, {len(runner.graph_outputs)} outputs')

  # Compile, capture, replay. Each call is timed for the log because there is
  # no progress inside tinygrad's compile, only its end.
  phases = ('compiling kernels', 'capturing the jit', 'first replay')
  for i, what in enumerate(phases):
    fill_random(arrays, plan, SEED + i)
    report('build', i / len(phases), what)
    t = time.perf_counter()
    out = jit(**tensors)
    Device[device].synchronize()
    log.info("%s: %.1f s", what, time.perf_counter() - t)
  out_shape = tuple(int(d) for d in out.shape)

  # The replay must reproduce the capture: same inputs, same bits. compile_modeld
  # checks the same thing, and a JIT that fails it is not a model.
  fill_random(arrays, plan, SEED)
  baseline = jit(**tensors).numpy().copy()
  fill_random(arrays, plan, SEED + 1)
  other = jit(**tensors).numpy().copy()
  fill_random(arrays, plan, SEED)
  again = jit(**tensors).numpy()
  if not np.array_equal(baseline, again):
    raise RuntimeError('the captured jit does not reproduce its own output on the same inputs')
  if np.array_equal(baseline, other):
    raise RuntimeError('the captured jit ignores its inputs')
  if not np.all(np.isfinite(baseline)):
    raise RuntimeError('the captured jit produced non-finite output on synthetic inputs')
  report('build', 1.0, 'jit verified')

  # The runner holds every intermediate of the last eager call; the pickle
  # wants the captured program and its weights only.
  del runner
  _stop_compile_workers()
  payload = {
    'format': FORMAT,
    'tinygrad': identity,
    'device': device,
    'inputs': [(s.name, list(s.shape), s.host_dtype, s.model_dtype) for s in plan],
    'outputs': [('outputs', list(out_shape), 'float32')],
    'jit': jit,
  }
  report('save', 0.0, 'writing the captured jit')
  out_path.parent.mkdir(parents=True, exist_ok=True)
  with tempfile.TemporaryDirectory(dir=str(out_path.parent)) as tmp:
    staged = Path(tmp) / 'engine.pkl'
    with open(staged, 'wb') as f:
      dump_oob(payload, f)
    shutil.move(str(staged), str(out_path))

  meta = {
    'backend': 'tinygrad',
    'tinygrad': identity,
    'device': device,
    'build_seconds': round(time.time() - t0, 1),
    'onnx': onnx_path.name,
    'built_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
    **(meta_extra or {}),
  }
  write_sidecar(out_path, meta)
  report('save', 1.0, f"done in {meta['build_seconds']}s")
  return out_path
