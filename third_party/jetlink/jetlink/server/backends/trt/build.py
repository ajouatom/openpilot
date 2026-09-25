"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

ONNX -> TensorRT engine, with progress.

A plan is specific to the TensorRT version, the GPU architecture and the build
flags, so the cache key encodes all three: a new JetPack or a different Jetson
rebuilds instead of loading something that cannot run.

The build takes ~160 s for the big model, so progress is streamed back to the
comma, where it shows up the way a model download does.

Two TensorRT generations are served from this one file. JetPack 6 ships 10.3
and is what the car was validated on; nothing here changes what that builds.
PyPI ships 11.x for desktop GPUs, which removed weakly typed networks and the
per-precision builder flags with them: precision follows the ONNX, which is
fp16 end to end, so the engine is the same and only the calls differ.
"""
from __future__ import annotations

import json
import logging
import shutil
import tempfile
import time
from pathlib import Path

import tensorrt as trt

from jetlink.server.backends.base import ProgressFn, sanitize
from jetlink.server.platform import available_bytes

log = logging.getLogger('jetlink.builder')

# A ceiling, not an allocation: TensorRT picks tactics that fit inside it. A
# flat 4 GB on an 8 GB Orin met the OOM killer on a 1.76 GB model, so size it
# from what is free.
MAX_WORKSPACE_BYTES = 4 << 30
MIN_WORKSPACE_BYTES = 256 << 20
WORKSPACE_FRACTION = 0.4

# Kernel timings mostly do not depend on the model: a warm cache cut a Lebowski
# build from 254 s to 173 s. Keyed like the plans, because a timing from another
# version or chip is not one. Advisory, so every path here fails open.
TIMING_CACHE = 'timing'


def workspace_bytes() -> int:
  free = available_bytes()
  if free <= 0:
    return MAX_WORKSPACE_BYTES
  return max(MIN_WORKSPACE_BYTES, min(MAX_WORKSPACE_BYTES, int(free * WORKSPACE_FRACTION)))


def device_tag(device: int = 0) -> str:
  """Identifies the hardware a plan is valid for.

  The compute capability is the part that matters; the name makes the cache
  filename readable. From CUDA, not /proc/device-tree, which the container has
  no mount for.
  """
  try:
    from jetlink.server.backends.trt import cudart
    name, cc_major, cc_minor = cudart.device_name(device)
    return sanitize(f"{name}-sm{cc_major}{cc_minor}")
  except Exception:
    return 'unknown'


def version_tag(device: int = 0) -> str:
  """'trt10.3.0.Orin-sm87': the part of every cache key that is TensorRT's."""
  return f"trt{sanitize(trt.__version__)}.{device_tag(device)}"


def timing_cache_path(engines_dir: str | Path, device: int = 0) -> Path:
  return Path(engines_dir) / f"{TIMING_CACHE}.{version_tag(device)}.cache"


def strongly_typed_only() -> bool:
  """TensorRT 11 dropped weak typing and BuilderFlag.FP16 with it."""
  return not hasattr(trt.BuilderFlag, 'FP16')


def network_flags() -> int:
  """The flags for create_network.

  0 on TensorRT 10: a weakly typed network with the FP16 flag set on the
  config, which is the build the car was validated against. On 11 the network
  is strongly typed whether asked or not; asking makes the intent visible and
  keeps working should a later release grow a second mode again.
  """
  if not strongly_typed_only():
    return 0
  flag = getattr(getattr(trt, 'NetworkDefinitionCreationFlag', None), 'STRONGLY_TYPED', None)
  return 0 if flag is None else 1 << int(flag)


def configure_precision(config, fp16: bool) -> str:
  """Ask for fp16 the way this TensorRT allows. Returns what was done, for the log."""
  if strongly_typed_only():
    return 'strongly typed network; precision follows the ONNX'
  if fp16:
    config.set_flag(trt.BuilderFlag.FP16)
    return 'fp16 enabled'
  return 'fp32'


class _Monitor(trt.IProgressMonitor):
  """Turns TensorRT's build phases into a single 0..1 fraction."""

  def __init__(self, report: ProgressFn):
    super().__init__()
    self.report = report
    self.phases: dict[str, tuple[int, int]] = {}
    self.root: str | None = None

  def _emit(self) -> None:
    if self.root and self.root in self.phases:
      step, total = self.phases[self.root]
      frac = (step / total) if total else 0.0
      self.report('build', min(max(frac, 0.0), 1.0), self.root)

  def phase_start(self, phase_name, parent_phase, num_steps):
    if parent_phase is None:
      self.root = phase_name
    self.phases[phase_name] = (0, num_steps)
    self._emit()

  def step_complete(self, phase_name, step):
    total = self.phases.get(phase_name, (0, 0))[1]
    self.phases[phase_name] = (step, total)
    self._emit()
    return True  # False would abort the build

  def phase_finish(self, phase_name):
    self.phases.pop(phase_name, None)
    if phase_name == self.root:
      self.root = None


def _load_timing_cache(config, path: str | Path | None):
  """Seed the builder's tactic timings from a previous build.

  Fails open: another TensorRT version's cache is rejected and a killed build's
  is truncated, and either way the build runs, just cold.
  """
  if path is None:
    return None
  blob = b''
  try:
    blob = Path(path).read_bytes()
  except OSError:
    pass
  try:
    cache = config.create_timing_cache(blob)
    if cache is not None:
      config.set_timing_cache(cache, ignore_mismatch=False)
    return cache
  except Exception as e:
    log.warning("timing cache unusable (%s), building cold", e)
    return None


def _save_timing_cache(cache, path: str | Path | None) -> None:
  if cache is None or path is None:
    return
  try:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    # Atomic like the plan: a build killed mid-write would leave a truncated
    # cache for the next one to read.
    tmp = p.with_suffix(p.suffix + '.tmp')
    tmp.write_bytes(memoryview(cache.serialize()))
    tmp.replace(p)
  except (OSError, AttributeError) as e:
    log.warning("could not write the timing cache: %s", e)


def build_engine(onnx_path: str | Path, out_path: str | Path,
                 report: ProgressFn | None = None,
                 fp16: bool = True, optimization_level: int = 3,
                 workspace: int | None = None,
                 meta_extra: dict | None = None,
                 timing_cache: str | Path | None = None) -> Path:
  """Patch, parse and build. Writes the plan atomically.

  `meta_extra` lands in the sidecar json next to the plan; the server keeps
  the model spec there so a later load needs neither the ONNX nor a parser.
  """
  onnx_path, out_path = Path(onnx_path), Path(out_path)
  report = report or (lambda *_: None)
  workspace = workspace_bytes() if workspace is None else workspace
  t0 = time.time()
  log.info("building with a %d MB workspace (%d MB available)",
           workspace >> 20, available_bytes() >> 20)

  logger = trt.Logger(trt.Logger.WARNING)
  init_plugins = getattr(trt, 'init_libnvinfer_plugins', None)
  if init_plugins is not None:
    init_plugins(logger, '')

  # Not at module scope: pulls in the onnx package, which a comma running the
  # tests does not have.
  from jetlink.onnx_patch import patch_file

  with tempfile.TemporaryDirectory(dir=str(out_path.parent)) as tmp:
    report('patch', 0.0, 'retyping uint8 image inputs to fp16')
    patched = Path(tmp) / 'patched.onnx'
    patch_file(str(onnx_path), str(patched))
    report('patch', 1.0, 'patched')

    builder = trt.Builder(logger)
    network = builder.create_network(network_flags())
    parser = trt.OnnxParser(network, logger)
    report('parse', 0.0, 'parsing onnx')
    if not parser.parse_from_file(str(patched)):
      errs = [str(parser.get_error(i)) for i in range(parser.num_errors)]
      raise RuntimeError("onnx parse failed:\n" + "\n".join(errs))
    report('parse', 1.0, f'{network.num_layers} layers')

    config = builder.create_builder_config()
    precision = configure_precision(config, fp16)
    log.info("tensorrt %s: %s", trt.__version__, precision)
    config.builder_optimization_level = optimization_level
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace)
    config.progress_monitor = _Monitor(report)
    cache = _load_timing_cache(config, timing_cache)

    report('build', 0.0, 'building engine')
    plan = builder.build_serialized_network(network, config)
    if plan is None:
      raise RuntimeError("TensorRT returned no engine; see the build log")
    _save_timing_cache(cache, timing_cache)

    staged = Path(tmp) / 'engine.plan'
    with open(staged, 'wb') as f:
      f.write(plan)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(staged), str(out_path))

  meta = {
    'backend': 'trt',
    'trt_version': trt.__version__,
    'device': device_tag(),
    'fp16': fp16,
    'strongly_typed': strongly_typed_only(),
    'optimization_level': optimization_level,
    'build_seconds': round(time.time() - t0, 1),
    'onnx': onnx_path.name,
    'built_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
    **(meta_extra or {}),
  }
  out_path.with_suffix('.json').write_text(json.dumps(meta, indent=2))
  report('build', 1.0, f"done in {meta['build_seconds']}s")
  return out_path
