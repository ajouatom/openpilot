"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

onnxruntime: CoreML on a Mac, CUDA or plain CPU anywhere else.

On Apple silicon the model runs in one CoreML session on the GPU
(`--device coreml`, the default there): 43 ms round trip at 20 Hz on an
M1 Pro, p99 44, parity gate passed. `--device ane` allows every unit, the
Neural Engine included, and carries the two rewrites that make the Neural
Engine correct (jetlink.onnx_patch, measured 2026-09-08): a Gather with a
negative index gathers garbage there, so `normalize_gather_indices` writes
every such index from the front; and its fp16 LayerNormalization overflows
on this model's residual stream, so `layernorm_in_fp32` runs the policy's
LayerNormalizations in fp32, which CoreML places off the Neural Engine. With
both, the gate passes with the GPU path's precision and the round trip is
33 ms back to back. It is not the default because at 20 Hz, a frame every
50 ms with the units idle in between, the same session measured 45 ms with
a p99 of 59 on the M1 Pro against the GPU's 43 and 44: every CoreML unit
pays a cost on the first request after an idle gap, and the Neural Engine
pays more of it. A faster Mac may not; measure it with
scripts/bench_link.py --rate 20 before choosing (docs/platforms.md).
Every session costs minutes of compile that onnxruntime's cache directory
did not shorten.

Elsewhere it is one session with the plain graph. The ONNX gets the same
surgery TensorRT's build does: the `org.tinygrad` layout op stripped,
because onnxruntime rejects a domain it does not know, and the uint8 images
retyped to fp16, which is what the queues already produce, so no per-frame
cast is needed. CUDA through onnxruntime is a fallback for a laptop without
TensorRT, not a peer of it; CPU is for a bench with a small model.

The session runs in a worker process (worker.py): onnxruntime holds the GIL
while it creates a session, and a ten-minute compile with the GIL held would
stop the server answering anything. The frame's inputs and outputs sit in
shared memory, so the cost is a message each way. The server process never
imports onnxruntime itself (see `quiet`).

The artifact is a directory: the prepared ONNX, a manifest naming the
session(s), and onnxruntime's cache. A load needs nothing else on disk.
"""
from __future__ import annotations

import json
import logging
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from jetlink.server.backends.base import (
  ArtifactInvalid,
  ProgressFn,
  sanitize,
  write_sidecar,
)
from jetlink.server.platform import gpu_name

log = logging.getLogger('jetlink.ort')

MANIFEST = 'sessions.json'

PROVIDERS = {
  'coreml': 'CoreMLExecutionProvider',
  'ane': 'CoreMLExecutionProvider',
  'cuda': 'CUDAExecutionProvider',
  'cpu': 'CPUExecutionProvider',
}

# CoreML compute units per device name. `ane` is correct only with the two
# rewrites in the module docstring, which the build applies for it.
COREML_UNITS = {'coreml': 'CPUAndGPU', 'ane': 'ALL'}

# The last resort for the compile stage's fraction: only a first build of a
# model, whose sidecar records nothing yet, and only if the compile writes
# nothing the walk below can see. The big model's compile measured 3.9 s on an
# M1 Pro on 2026-09-10, once the weights stopped going through the MIL as
# text; ten leaves room for a larger model on a slower Mac.
EXPECTED_COREML_SECONDS = 10.0

# The two subtrees onnxruntime fills in the cache directory, in the order it
# fills them: it writes the MLProgram out first, then coremlc compiles that
# into compiled_model.mlmodelc. Which one is growing is the stage.
CONVERTED_DIR = 'Data'
COMPILED_DIR = 'compiled_model.mlmodelc'


def quiet(ort) -> None:
  """Turn onnxruntime's telemetry off in a process that has imported it.

  The macOS wheel carries Microsoft's events SDK, which uploads over HTTP
  from a worker thread of its own. A response that lands as the process
  exits is dispatched through a mutex static destruction has already torn
  down, and the process aborts: `recursive_mutex lock failed` from
  `Microsoft::Applications::Events::HttpClientManager::onHttpResponse` in
  the crash report, one test run in three. Disabling the events at import
  was not enough, an event logged by the import itself still uploads, so
  the server process never imports onnxruntime at all: the version comes
  from the package metadata and the providers from a probe in a child
  (`available_providers`). The worker calls this, and a car has no business
  making the request in the first place.
  """
  disable = getattr(ort, 'disable_telemetry_events', None)
  if disable is not None:
    disable()


def _probe_providers(conn) -> None:
  # runs in a child: the one import of onnxruntime the server never makes
  try:
    import onnxruntime as ort
    quiet(ort)
    conn.send(list(ort.get_available_providers()))
  except Exception as e:
    conn.send(e)
  finally:
    conn.close()


def available_providers() -> list[str]:
  """onnxruntime's providers on this machine, asked in a spawned child."""
  import multiprocessing as mp
  ctx = mp.get_context('spawn')
  parent, child = ctx.Pipe()
  proc = ctx.Process(target=_probe_providers, args=(child,), name='jetlink-ort-probe', daemon=True)
  proc.start()
  child.close()
  try:
    if not parent.poll(60):
      raise RuntimeError('onnxruntime did not answer the provider probe in 60 s')
    answer = parent.recv()
  finally:
    proc.join(5)
    parent.close()
  if isinstance(answer, Exception):
    raise RuntimeError(f"onnxruntime could not be imported: {answer}") from answer
  return answer


def runtime_version() -> str:
  from importlib.metadata import version
  return version('onnxruntime')


def _pick_device(providers: list[str], device: str) -> str:
  have = set(providers)
  if device in ('auto', '', None):
    # never `ane` on auto: it is the measured opt-in, see the module docstring
    order = ('coreml', 'cuda', 'cpu') if sys.platform == 'darwin' else ('cuda', 'cpu')
    for d in order:
      if PROVIDERS[d] in have:
        return d
    raise RuntimeError(f"onnxruntime has none of {list(PROVIDERS.values())}; has {sorted(have)}")
  d = device.lower()
  if d not in PROVIDERS:
    raise ValueError(f"onnxruntime device must be one of {list(PROVIDERS)}, not {device!r}")
  if PROVIDERS[d] not in have:
    raise RuntimeError(f"onnxruntime here has no {PROVIDERS[d]} (has {sorted(have)})")
  return d


def _cache_key(out_path: Path, part: str) -> str:
  # onnxruntime wants the key alphanumeric and under 64 characters
  return re.sub(r'[^A-Za-z0-9]', '', out_path.stem + part)[:63]


def _prepared_model(onnx_path: Path, for_ane: bool, for_coreml: bool = False):
  """The ONNX as onnxruntime will see it, in memory."""
  import onnx

  from jetlink.onnx_patch import (
    gemm_with_transposed_weight,
    layernorm_in_fp32,
    needs_patch,
    normalize_gather_indices,
    patch_uint8_inputs,
    strip_tinygrad_ops,
    vision_nodes,
  )
  model = onnx.load(str(onnx_path))
  stripped = strip_tinygrad_ops(model)
  patched = needs_patch(model)
  if patched:
    patch_uint8_inputs(model)
  gathers = normalize_gather_indices(model)
  norms = 0
  if for_ane:
    policy = {n.name for n in model.graph.node} - vision_nodes(model)
    norms = layernorm_in_fp32(model, only=policy)
  # Only for CoreML: it is what puts the weights in the weight file instead of
  # the MIL text. The CUDA and CPU providers are happy with the transB=0 Gemm
  # onnxruntime's own fusion makes, and gain nothing from the rewrite.
  gemms = gemm_with_transposed_weight(model) if for_coreml else 0
  log.info("prepared %s: stripped %d tinygrad op(s), %s, %d negative Gather index(es) normalized, "
           "%d LayerNormalization(s) in fp32, %d MatMul+Add rewritten as Gemm(transB=1)",
           onnx_path.name, stripped,
           'images retyped to fp16' if patched else 'inputs left as declared', gathers, norms, gemms)
  return model


# The metadata_props key onnxruntime reads (coreml_provider_factory.h). Under
# any other name it keys the compiled-model cache on a hash of the model's
# path instead, so a compile done under the build's temp dir was never found
# from the artifact's final path: the first load compiled the whole model a
# second time, 8.5 min and 4.8 GB on an M1 Pro, and the disk kept both.
COREML_CACHE_KEY = 'COREML_CACHE_KEY'


def _with_cache_key(model, key: str):
  # The compiled-model cache is looked up by this rather than by the path, so
  # the same key finds the same compile after the move out of the temp dir.
  for prop in [p for p in model.metadata_props if p.key in (COREML_CACHE_KEY, 'CACHE_KEY')]:
    model.metadata_props.remove(prop)
  entry = model.metadata_props.add()
  entry.key, entry.value = COREML_CACHE_KEY, key
  return model


def _has_cache_key(model, key: str) -> bool:
  return any(p.key == COREML_CACHE_KEY and p.value == key for p in model.metadata_props)


def repair_coreml_cache(artifact: Path, model_name: str, cache: Path, key: str) -> None:
  """Bring an artifact built under the wrong metadata key up to date, in place.

  onnxruntime keyed those caches on the model path: one compile sits under a
  hash of the build's temp path, never to be found again, and after the first
  load another sits under a hash of the final path. The second is the same
  compiled program the key would name, so it is renamed rather than rebuilt;
  everything else in the cache directory is stale and goes. The model gets the
  key written in so the next load hits the renamed directory.
  """
  import onnx

  model_path = artifact / model_name
  model = onnx.load(str(model_path), load_external_data=False)
  if not _has_cache_key(model, key):
    log.info("writing the CoreML cache key into %s", model_path.name)
    onnx.save(_with_cache_key(model, key), str(model_path))
  for entry in sorted(cache.iterdir()):
    if not entry.is_dir() or entry.name == key:
      continue
    recorded = ''
    try:
      recorded = (entry / 'model.txt').read_text().strip()
    except OSError:
      pass
    compiled_here = False
    try:
      compiled_here = bool(recorded) and Path(recorded).resolve() == model_path.resolve()
    except OSError:
      pass
    if compiled_here and not (cache / key).exists():
      log.info("keeping the CoreML compile for this artifact as %s (was %s)", key, entry.name)
      entry.rename(cache / key)
    else:
      log.info("removing a stale CoreML compile %s (compiled for %s)", entry.name, recorded or 'unknown')
      shutil.rmtree(entry, ignore_errors=True)


def tree_bytes(root: Path, split: str | None = None) -> tuple[int, int]:
  """Bytes under `root`, as (outside `split`, inside it). Missing is 0.

  os.scandir, not a stat of every path: the compile writes several GB and the
  walk runs every couple of seconds beside it.
  """
  outside = inside = 0
  stack = [(str(root), False)]
  while stack:
    path, within = stack.pop()
    try:
      with os.scandir(path) as entries:
        for entry in entries:
          try:
            if entry.is_dir(follow_symlinks=False):
              stack.append((entry.path, within or entry.name == split))
            elif within:
              inside += entry.stat(follow_symlinks=False).st_size
            else:
              outside += entry.stat(follow_symlinks=False).st_size
          except OSError:
            pass          # a file the compile removed between scandir and stat
    except OSError:
      pass                # the directory does not exist yet, which is 0 bytes
  return outside, inside


def worker_rss(pid: int) -> int:
  """Resident bytes of the worker process, 0 where it cannot be read.

  The session is created in a child (worker.py), so the parent's own resident
  size says nothing about how far a load has got. Nothing on Windows, which
  has no jetlink server anyway.
  """
  if not pid:
    return 0
  try:
    if sys.platform == 'darwin':
      out = subprocess.run(['ps', '-o', 'rss=', '-p', str(pid)],
                           capture_output=True, text=True, timeout=5)
      return int(out.stdout.strip() or 0) * 1024
    if sys.platform.startswith('linux'):
      for line in Path(f'/proc/{pid}/status').read_text().splitlines():
        if line.startswith('VmRSS:'):
          return int(line.split()[1]) * 1024
  except (OSError, ValueError, IndexError, subprocess.SubprocessError):
    return 0
  return 0


def _size(n: float) -> str:
  return f"{n / 1e9:.1f} GB" if n >= 1e9 else f"{n / 1e6:.0f} MB"


def _secs(n: float) -> str:
  return f"{n / 60:.0f} min" if n >= 90 else f"{n:.0f} s"


class CoreMLProgress:
  """What a CoreML session creation is doing, read off the cache directory.

  CoreML reports nothing at all until it is done, so the elapsed time was all
  the bar had and a nine-minute load looked like a hung server. The work is
  visible on disk instead: onnxruntime writes the converted MLProgram under
  the partition's `Data` directory, then coremlc compiles that into
  `compiled_model.mlmodelc`. Which of the two is growing is the stage, and
  the bytes written against the bytes the last build of this model needed is
  the fraction.

  `expect` is the previous build's sidecar. Without one, the first build of a
  model falls back to elapsed time against EXPECTED_COREML_SECONDS and says
  so rather than inventing a total. A load from a warm cache writes nothing,
  so it is measured by the worker's resident size against the last load's.
  """

  def __init__(self, caches, weights_bytes: int = 0, expect: dict | None = None,
               loading: bool = False):
    self.caches = [Path(c) for c in caches]
    self.weights = int(weights_bytes or 0)
    self.expect = expect or {}
    self.loading = loading
    self.peak_rss = 0     # the largest the worker got, for the next load to aim at

  def measure(self) -> tuple[int, int]:
    converted = compiled = 0
    for cache in self.caches:
      out, inside = tree_bytes(cache, split=COMPILED_DIR)
      converted += out
      compiled += inside
    return converted, compiled

  def tick(self, elapsed: float, pid: int = 0) -> tuple[str, float, str]:
    if self.loading:
      return self._loading(elapsed, pid)
    converted, compiled = self.measure()
    if not compiled:
      total = self.weights or self.expect.get('convert_bytes') or 0
      frac = min(0.95, converted / total) if total else 0.0
      of = f" of {_size(total)}" if total else ''
      return 'convert', frac, f"converting for CoreML, {_size(converted)}{of} written"
    total = self.expect.get('compile_bytes') or 0
    if total:
      return 'compile', min(0.95, compiled / total), \
             f"compiling for CoreML, {_size(compiled)} of {_size(total)} written"
    # Nothing recorded for this model yet, so the clock is all there is.
    took = self.expect.get('compile_seconds') or EXPECTED_COREML_SECONDS
    return 'compile', min(0.95, elapsed / took), \
           f"compiling for CoreML, {_size(compiled)} written, {_secs(elapsed)} elapsed"

  def _loading(self, elapsed: float, pid: int) -> tuple[str, float, str]:
    want = self.expect.get('load_rss_bytes') or 0
    rss = worker_rss(pid)
    self.peak_rss = max(self.peak_rss, rss)
    if want and rss:
      return 'load', min(0.95, rss / want), \
             f"loading the CoreML model, {_size(rss)} of {_size(want)} resident"
    took = self.expect.get('load_seconds') or 0
    if took:
      return 'load', min(0.95, elapsed / took), \
             f"loading the CoreML model, {elapsed:.0f} s of about {took:.0f} s"
    return 'load', 0.0, f"loading the CoreML model, {elapsed:.0f} s elapsed"


def coreml_ticker(report, progress: CoreMLProgress, initial: str | None = None):
  """One tick: the progress a client draws, and a line for the log.

  Progress goes out on every tick, because the comma and the app would
  otherwise sit on one stage at frac 0 for as long as CoreML takes, which
  reads as a hung server. The log does not: a tick every couple of seconds is
  hundreds of identical lines per build and the Logs view has nothing else in
  it, so it is info on the first tick of a stage and once a minute after
  that, debug for the rest. A stage that ends gets its 100 % line with the
  time it took, so the CLI transcript reads as a list of finished phases.

  `initial` is the stage the caller has already reported at 0. Without it a
  convert that finishes inside the first tick has no previous stage for that
  tick to close, and goes from 0 straight to whatever compile reports.
  """
  said = [-1]
  state: dict = {'stage': initial, 'since': 0.0}

  def tick(elapsed: float, pid: int = 0) -> None:
    stage, frac, msg = progress.tick(elapsed, pid)
    if state['stage'] is not None and stage != state['stage']:
      done = elapsed - state['since']
      log.info("%s finished in %.0f s", state['stage'], done)
      if report is not None:
        report(state['stage'], 1.0, f"{state['stage']} done in {done:.0f} s")
      state['since'] = elapsed
      said[0] = -1
    minute = int(elapsed // 60)
    log.log(logging.INFO if minute != said[0] or stage != state['stage'] else logging.DEBUG,
            "%s: %s", stage, msg)
    said[0] = minute
    state['stage'] = stage
    if report is not None:
      report(stage, frac, msg)

  tick.state = state
  return tick


class OrtBackend:
  name = 'ort'
  suffix = '.ortcache'

  def __init__(self, device: str = 'auto', providers: list[str] | None = None):
    self._version = runtime_version()
    self._weights_bytes = 0     # set by _stage, once the prepared model is in hand
    self.device = _pick_device(available_providers() if providers is None else providers, device)
    if self.device == 'cpu':
      log.warning("onnxruntime on the CPU will not make the frame budget; fine for a bench, not a car")

  @property
  def runtime_version(self) -> str:
    return self._version

  def device_tag(self) -> str:
    return sanitize(f"{self.device}-{gpu_name()}")

  def tag(self) -> str:
    return f"ort{sanitize(self.runtime_version)}.{self.device_tag()}"

  def describe(self) -> dict:
    return {'backend': self.name, 'runtime_version': self.runtime_version, 'device': self.device_tag()}

  # -- sessions ---------------------------------------------------------------

  @property
  def _on_coreml(self) -> bool:
    return self.device in COREML_UNITS

  def _providers(self, units: str | None, compiled_dir: Path | None) -> list:
    if self._on_coreml:
      opts = {'ModelFormat': 'MLProgram', 'MLComputeUnits': units or COREML_UNITS[self.device]}
      if compiled_dir is not None:
        opts['ModelCacheDirectory'] = str(compiled_dir)
      return [(PROVIDERS['coreml'], opts), PROVIDERS['cpu']]
    if self.device == 'cuda':
      return [(PROVIDERS['cuda'], {'device_id': 0}), PROVIDERS['cpu']]
    return [PROVIDERS['cpu']]

  def _plan(self, artifact: Path, manifest: list[dict]) -> list[tuple[Path, list]]:
    """[(model path, providers)] for the worker, from a manifest entry per session."""
    plan = []
    for entry in manifest:
      compiled = artifact / entry['cache'] if entry.get('cache') else None
      plan.append((artifact / entry['model'], self._providers(entry.get('units'), compiled)))
    return plan

  def _engine(self, artifact: Path, manifest: list[dict], on_tick=None):
    from jetlink.server.backends.ort.engine import OrtEngine
    # log severity 3: errors only; the CoreML partitioner is chatty
    return OrtEngine(self._plan(artifact, manifest), self.device, log_severity=3, on_tick=on_tick)

  # -- build ------------------------------------------------------------------

  def _stage(self, onnx_path: Path, staged: Path, out_path: Path) -> list[dict]:
    """Write the prepared model into `staged` and return the manifest.

    The manifest is a list because the worker runs sessions as a chain and
    a split graph was measured through it; one session is what ships.
    """
    import onnx

    coreml = self._on_coreml
    model = _prepared_model(onnx_path, for_ane=self.device == 'ane', for_coreml=coreml)
    # What the convert stage is working towards: onnxruntime writes the
    # initializers out as the MLProgram's weight file, so their size is the
    # total the bytes on disk can honestly be reported against.
    self._weights_bytes = sum(len(t.raw_data) for t in model.graph.initializer)
    onnx.save(_with_cache_key(model, _cache_key(out_path, 'model')), str(staged / 'model.onnx'))
    manifest = [{'model': 'model.onnx', 'units': COREML_UNITS[self.device] if coreml else None,
                 'cache': 'coreml' if coreml else None}]
    for entry in manifest:
      if entry['cache']:
        (staged / entry['cache']).mkdir()
    (staged / MANIFEST).write_text(json.dumps(manifest, indent=2))
    return manifest

  def _caches(self, artifact: Path, manifest: list[dict]) -> list[Path]:
    return [artifact / e['cache'] for e in manifest if e.get('cache')]

  @staticmethod
  def _sidecar(out_path: Path) -> dict:
    """The last build's measurements, for the stages to report against."""
    try:
      return json.loads(out_path.with_suffix('.json').read_text())
    except (OSError, ValueError):
      return {}

  def build(self, onnx_path: Path, out_path: Path, report: ProgressFn | None = None,
            meta_extra: dict | None = None) -> Path:
    onnx_path, out_path = Path(onnx_path), Path(out_path)
    report = report or (lambda *_: None)
    t0 = time.time()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=str(out_path.parent)) as tmp:
      staged = Path(tmp) / 'artifact'
      staged.mkdir()
      report('patch', 0.0, 'preparing the model for onnxruntime')
      manifest = self._stage(onnx_path, staged, out_path)
      report('patch', 1.0, 'prepared')

      stages = {}
      if self._on_coreml:
        progress = CoreMLProgress(self._caches(staged, manifest), self._weights_bytes,
                                  expect=self._sidecar(out_path))
        report('convert', 0.0, 'converting for CoreML')
        tick = coreml_ticker(report, progress, initial='convert')
      else:
        report('build', 0.0, f'creating the onnxruntime session ({self.device})')
        tick = lambda elapsed, pid=0: report(  # noqa: E731
          'build', 0.5, f'creating the onnxruntime session ({self.device})')
      t_engine = time.time()
      engine = self._engine(staged, manifest, on_tick=tick)
      try:
        if self._on_coreml:
          converted, compiled = progress.measure()
          stages = {'convert_bytes': converted, 'compile_bytes': compiled,
                    'compile_seconds': round(time.time() - t_engine, 1)}
          # A build short enough that no tick saw the compile start still owes
          # convert its 100 % line: every stage ends with one.
          if tick.state['stage'] != 'compile':
            report('convert', 1.0, f'converted {_size(converted)}')
          report('compile', 1.0, f'compiled in {_secs(time.time() - t_engine)}')
        else:
          report('build', 1.0, f'sessions created in {time.time() - t0:.0f} s')
        # Prove it runs before calling it built; a partition that fell back to
        # the CPU in full would still "work", so the log line says what it used.
        engine.run()
        log.info("onnxruntime providers in use: %s", engine.providers)
        providers = engine.providers
      finally:
        engine.close()

      if out_path.exists():
        shutil.rmtree(out_path)
      shutil.move(str(staged), str(out_path))

    meta = {
      'backend': self.name,
      'onnxruntime': self.runtime_version,
      'device': self.device_tag(),
      'sessions': manifest,
      'providers': providers,
      'build_seconds': round(time.time() - t0, 1),
      'onnx': onnx_path.name,
      'built_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
      **stages,
      **(meta_extra or {}),
    }
    write_sidecar(out_path, meta)
    report('build', 1.0, f"done in {meta['build_seconds']}s")
    return out_path

  # -- load -------------------------------------------------------------------

  def load(self, artifact: Path, report: ProgressFn | None = None):
    artifact = Path(artifact)
    try:
      manifest = json.loads((artifact / MANIFEST).read_text())
    except (OSError, ValueError) as e:
      raise ArtifactInvalid(f"{artifact}: no readable {MANIFEST} inside ({e})") from e
    if not isinstance(manifest, list) or not manifest:
      raise ArtifactInvalid(f"{artifact}: {MANIFEST} names no sessions")
    sidecar = self._sidecar(artifact)
    if self._on_coreml and 'compile_bytes' not in sidecar:
      # Built before the Gemm weights were handed over transposed: its MIL
      # carries the weights as text and a load parses gigabytes of it, 465 s
      # on an M1 Pro against 1.8 s for the same model built since. The host
      # replaces an invalid artifact from the ONNX, and that build is 5 s.
      raise ArtifactInvalid(f"{artifact}: built before the weight rewrite; loads took minutes, a rebuild takes seconds")
    for entry in manifest:
      if not (artifact / entry['model']).is_file():
        raise ArtifactInvalid(f"{artifact}: no {entry['model']} inside")
      cache = artifact / entry['cache'] if entry.get('cache') else None
      if cache is not None and cache.is_dir():
        key = _cache_key(artifact, Path(entry['model']).stem)
        if not (cache / key).is_dir():
          if report is not None:
            report('load', 0.0, 'bringing the compiled model cache up to date')
          repair_coreml_cache(artifact, entry['model'], cache, key)
      # An empty cache would make onnxruntime recompile for minutes under a
      # "loading engine" that never moves. Rebuild instead, which reports
      # progress and ends with a cache.
      if cache is not None and (not cache.is_dir() or not any(cache.iterdir())):
        raise ArtifactInvalid(f"{artifact}: the CoreML cache for {entry['model']} is empty")
    t0 = time.time()
    progress = None
    if self._on_coreml:
      progress = CoreMLProgress(self._caches(artifact, manifest), expect=sidecar, loading=True)
      if report is not None:
        report('load', 0.0, 'loading the CoreML model')
      tick = coreml_ticker(report, progress, initial='load')
    else:
      tick = None if report is None else (lambda elapsed, pid=0: report(
        'load', 0.0, f'creating the onnxruntime session ({self.device})'))
    engine = self._engine(artifact, manifest, on_tick=tick)
    took = time.time() - t0
    log.info("onnxruntime sessions on %s in %.1f s, providers %s", self.device, took,
             engine.providers)
    if report is not None:
      report('load', 1.0, f'loaded in {_secs(took)}')
    # What the next load reports against. A load that came back inside one
    # tick never sampled the worker, and the seconds alone are still worth
    # keeping.
    self._remember_load(artifact, sidecar, took, progress.peak_rss if progress else 0)
    return engine

  def _remember_load(self, artifact: Path, sidecar: dict, took: float, rss: int) -> None:
    """Fold this load's time and peak resident size into the sidecar."""
    if not sidecar:
      return
    sidecar['load_seconds'] = round(took, 1)
    if rss:
      sidecar['load_rss_bytes'] = rss
    try:
      write_sidecar(artifact, sidecar)
    except OSError as e:
      log.debug("could not record the load in the sidecar: %s", e)
