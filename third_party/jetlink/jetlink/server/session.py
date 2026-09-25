"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

The request loop, and the engine it serves.

One client at a time, one message at a time, with builds on a worker thread so
progress keeps flowing through a 160 s build. The inference path neither
allocates nor logs: numpy views are laid over the received buffer in place.

The engine outlives the connection. The comma reconnects at every handover,
inside modeld's 60 s budget and after a re-enumeration seen at 70 s, and
reloading a 770 MB plan costs 13 to 25 s of that. So EngineHost owns the loaded
engine and the build in flight for the life of the process.
"""
from __future__ import annotations

import json
import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from jetlink import protocol as P
from jetlink.server.backends.base import ArtifactInvalid
from jetlink.server.cache import CacheEntry, EngineCache
from jetlink.server.telemetry import CachedTelemetry, NoTelemetry
from jetlink.spec import (
  CHUNK,
  DEFAULT_FRAME_SKIP,
  ModelSpec,
  sha256_file,
  spec_from_onnx,
)
from jetlink.transport.base import LinkError, LinkTimeout, Message, Transport

log = logging.getLogger('jetlink.server')

# Server-side turnaround worth a log line. The comma's own line fires at
# 80 ms end to end; anything past this here is most of that budget.
SLOW_FRAME_US = 60_000

PROGRESS_MIN_INTERVAL = 0.25  # s; the comma only needs a progress bar, not every step


@dataclass
class Loaded:
  """An engine resident on the GPU, with the state that goes with it."""
  sha256: str
  spec: ModelSpec
  engine: object
  queues: object
  host_inputs: dict


@dataclass
class Job:
  """One build or load in flight, or its outcome."""
  sha256: str
  load_only: bool
  state: str = 'building'   # building|ready|failed
  detail: str = ''


class FrameStats:
  """A rolling window of served frames, for the control channel's stats event.

  Appended to by the request loop once the reply is on the wire, read by the
  control server's ticker. Neither side takes a lock: a deque append and a
  deque copy are each one uninterrupted C call, and the hot path may not wait
  on anything. perf_counter rather than monotonic because _infer already reads
  it, and the session tests replace the module's clock with one that has it.
  """
  __slots__ = ('samples',)

  def __init__(self, maxlen: int = 2000):
    self.samples = deque(maxlen=maxlen)

  def record(self, total_us: int, gpu_us: int) -> None:
    self.samples.append((time.perf_counter(), total_us, gpu_us))

  def window(self, seconds: float) -> list:
    """The samples newer than `seconds` ago, oldest first."""
    cutoff = time.perf_counter() - seconds
    return [s for s in self.samples.copy() if s[0] >= cutoff]

  def summary(self, seconds: float = 1.0, frames_total: int = 0) -> dict | None:
    """The `stats` event payload, or None when no frame landed in the window."""
    rows = self.window(seconds)
    if not rows:
      return None
    n = len(rows)
    totals = sorted(r[1] for r in rows)
    return {
      'frames': frames_total,
      'fps': round(n / seconds, 2),
      'total_ms': {'mean': round(sum(totals) / n / 1e3, 2),
                   'p99': round(totals[int(0.99 * (n - 1))] / 1e3, 2),
                   'max': round(totals[-1] / 1e3, 2)},
      'gpu_ms': {'mean': round(sum(r[2] for r in rows) / n / 1e3, 2)},
      'slow': sum(1 for t in totals if t > SLOW_FRAME_US),
      'window_s': round(seconds, 1),
    }


@dataclass(frozen=True)
class Request:
  """What a client asked for: enough to identify the model without the file."""
  sha256: str
  nbytes: int
  frame_skip: int

  def __post_init__(self):
    EngineCache._validate_sha256(self.sha256)
    if self.nbytes < 0 or self.frame_skip <= 0:
      raise ValueError('invalid model size or frame skip')


class EngineHost:
  """Process-wide owner of the loaded engine and of the build in flight.

  Everything that touches `loaded` holds `lock`: the job thread swaps engines
  while the request loop runs frames, and freeing pinned memory a frame's views
  are laid over segfaults rather than raising.
  """

  def __init__(self, cache: EngineCache, telemetry=None, sleep_after: float = 0.0):
    self.cache = cache
    # What --sleep-after was set to, for the hello: the comma decides from it
    # whether letting go of the gadget when parked buys anything. 0 = never.
    self.sleep_after = float(sleep_after)
    self.telemetry = CachedTelemetry(telemetry if telemetry is not None else NoTelemetry())
    self.lock = threading.Lock()
    self.loaded: Loaded | None = None
    self.job: Job | None = None
    self.session: Session | None = None   # who hears about progress and completion
    self._last_progress = 0.0
    self._listeners: list = []            # the control channel, when there is one
    self._last_stage: tuple[str | None, float, str] = (None, 0.0, '')
    self._last_engine: dict | None = None
    self.frame_stats = FrameStats()

  # -- listeners ------------------------------------------------------------

  def subscribe(self, fn) -> None:
    """Hear about progress, engine changes and the link. See control.py.

    Nothing on the frame path calls this, and nothing here may block: the job
    thread is what emits.
    """
    self._listeners.append(fn)

  def emit(self, kind: str, payload: dict) -> None:
    """Fan an event out, never raising into the caller.

    Called from the job thread, the request loop and the main thread, so a
    listener has to be thread-safe. One that throws is logged and dropped
    rather than allowed to take a build down with it.

    An engine payload identical to the last one is dropped: a job finishing
    emits from the progress call and again from the run's finally, and the two
    are the same event.
    """
    if kind == 'engine':
      if payload == self._last_engine:
        return
      self._last_engine = dict(payload)
    for fn in tuple(self._listeners):
      try:
        fn(kind, payload)
      except Exception:
        log.exception("a host listener failed; dropping it")
        try:
          self._listeners.remove(fn)
        except ValueError:
          pass

  def snapshot(self) -> dict:
    """The `engine` control event: what is loaded, or what is being prepared."""
    with self.lock:
      loaded, job = self.loaded, self.job
      stage, frac, msg = self._last_stage
      if loaded is not None:
        return {'state': 'ready', 'sha256': loaded.sha256, 'detail': '', 'stage': None,
                'frac': 1.0, 'msg': msg, 'load_only': bool(job.load_only) if job is not None else False}
      if job is not None and job.state == 'building':
        return {'state': 'loading' if job.load_only else 'building', 'sha256': job.sha256,
                'detail': job.detail, 'stage': stage, 'frac': frac, 'msg': msg,
                'load_only': job.load_only}
      if job is not None and job.state == 'failed':
        return {'state': 'failed', 'sha256': job.sha256, 'detail': job.detail, 'stage': 'failed',
                'frac': frac, 'msg': msg, 'load_only': job.load_only}
      return {'state': 'none', 'sha256': None, 'detail': '', 'stage': None, 'frac': 0.0,
              'msg': '', 'load_only': False}

  # -- what a client sees ---------------------------------------------------

  def status(self, sha256: str | None, frame_skip: int | None = None) -> dict:
    """The engine state for one model, in the shape ENGINE_RESP carries.

    frame_skip is part of the identity: the spec handed back is stamped with the
    value asked for, so matching on the sha alone would answer ready and serve
    another client's spec.
    """
    with self.lock:
      return self._status(sha256, frame_skip)

  def _status(self, sha256: str | None, frame_skip: int | None = None) -> dict:
    """Snapshot while holding lock, including callers already inside request."""
    loaded, job = self.loaded, self.job
    if sha256 is None:
      return {'state': 'none', 'detail': '', 'sha256': None, 'chunk': CHUNK}
    if (loaded is not None and loaded.sha256 == sha256
        and (frame_skip is None or loaded.spec.frame_skip == frame_skip)):
      return {'state': 'ready', 'detail': '', 'sha256': sha256, 'chunk': CHUNK,
              'spec': loaded.spec.to_dict()}
    if job is not None and job.sha256 == sha256 and job.state != 'ready':
      return {'state': job.state, 'detail': job.detail, 'sha256': sha256, 'chunk': CHUNK}
    if job is not None and job.state == 'building':
      return {'state': 'building', 'sha256': sha256, 'chunk': CHUNK,
              'detail': f'another build is in progress ({job.sha256[:16]})'}
    if self._cached_spec(self.cache.entry(sha256)) is not None:
      # Built already, just not loaded. Only `request` consults the cache, so
      # every other caller answered need_upload for a plan sitting on disk, and
      # modeld, which never carries the ONNX, reads that as an engine that is
      # gone. Say what a request for it will do instead.
      return {'state': 'building', 'sha256': sha256, 'chunk': CHUNK,
              'detail': 'engine cached, not loaded yet'}
    return {'state': 'need_upload', 'sha256': sha256, 'chunk': CHUNK,
            'detail': f'have {self._model_bytes(sha256)} of the model'}

  def loaded_sha(self) -> str | None:
    with self.lock:
      return self.loaded.sha256 if self.loaded else None

  @property
  def backend(self):
    return self.cache.backend

  # -- requests -------------------------------------------------------------

  def request(self, req: Request, session: Session) -> dict:
    """Make `req` the model being served, starting whatever that takes."""
    with self.lock:
      if session is not None:
        # A control-channel prepare passes none, and must not detach the
        # comma's session from the progress it is waiting on.
        self.session = session
      if (self.loaded is not None and self.loaded.sha256 == req.sha256
          and self.loaded.spec.frame_skip == req.frame_skip):
        return self._ready(self.loaded)
      if self.job is not None and self.job.state == 'building':
        # Either this model's build, which the client attaches to, or another
        # build that owns the GPU right now.
        return self._status(req.sha256, req.frame_skip)
    entry = self.cache.entry(req.sha256)
    model_path = self.cache.model_path(req.sha256)
    spec = self._spec_on_disk(entry, model_path, req)
    if entry.exists and spec is not None:
      self._start(Job(req.sha256, load_only=True), req, entry, model_path, spec)
    elif _model_complete(model_path, req.nbytes):
      self._start(Job(req.sha256, load_only=False), req, entry, model_path, spec)
    else:
      with self.lock:
        if self.job is not None and self.job.sha256 == req.sha256 and self.job.state == 'failed':
          # Whatever failed has left the disk (an artifact discarded as
          # invalid, with no model to rebuild from). The client was told about
          # the failure when it happened; what it needs now is `need_upload`.
          self.job = None
    return self.status(req.sha256, req.frame_skip)

  def _ready(self, loaded: Loaded) -> dict:
    return {'state': 'ready', 'detail': '', 'sha256': loaded.sha256, 'chunk': CHUNK,
            'spec': loaded.spec.to_dict()}

  def _cached_spec(self, entry: CacheEntry) -> dict | None:
    """The spec a cached artifact's sidecar carries, if it has one.

    A plan on its own cannot be served: without a spec there is nothing to lay
    the engine's IO over, and deriving one means parsing the ONNX.
    """
    if not entry.exists:
      return None
    try:
      return entry.meta().get('spec') or None
    except (OSError, ValueError, KeyError):
      log.warning("unreadable sidecar for %s", entry.path.name)
      return None

  def _model_bytes(self, sha256: str) -> int:
    path = self.cache.model_path(sha256)
    return path.stat().st_size if path.exists() else 0

  def preload(self) -> None:
    """Start loading whatever was loaded last, before a client asks for it.

    A fresh process would otherwise deserialize the plan (6 s for 766 MB) on the
    first ensure_engine, which at a cold ignition lands on modeld's join. From
    the gadget-less poll loop it overlaps the Jetson's boot instead. Guessing
    wrong costs one unload; `request` swaps as it does for any other change.

    Only a plan whose sidecar carries a spec: deriving one means parsing the
    ONNX, too much work to do on a guess.
    """
    remembered = self.cache.last_loaded()
    if remembered is None:
      return
    sha256, frame_skip = remembered
    with self.lock:
      if self.loaded is not None or self.job is not None:
        return
    d = self._cached_spec(self.cache.entry(sha256))
    if d is None:
      return
    entry = self.cache.entry(sha256)
    spec = ModelSpec.from_dict({**d, 'frame_skip': frame_skip})
    log.info("preloading the engine loaded last: %s", entry.path.name)
    self._start(Job(sha256, load_only=True), Request(sha256, 0, frame_skip),
                entry, self.cache.model_path(sha256), spec)

  def _spec_on_disk(self, entry: CacheEntry, model_path: Path, req: Request) -> ModelSpec | None:
    """The spec for a cached plan, from its sidecar or failing that the ONNX.

    A plan whose sidecar predates specs still loads if the model file is there
    to derive one from; otherwise the client uploads and the existing plan is
    reused, not rebuilt.

    Only a whole model file is parsed. One shorter than the size the client
    declared is an upload still arriving or cut short, and the comma asks again
    every few seconds while it has no engine: parsing it fails every time and
    logs a traceback for what `need_upload` already says.
    """
    d = self._cached_spec(entry)
    if d is not None:
      return ModelSpec.from_dict({**d, 'frame_skip': req.frame_skip})
    if _model_complete(model_path, req.nbytes):
      try:
        return self._derive_spec(model_path, req.frame_skip)
      except Exception:
        log.exception("could not derive a spec from %s", model_path.name)
    return None

  # -- the worker -----------------------------------------------------------

  def _start(self, job: Job, req: Request, entry: CacheEntry, model_path: Path,
             spec: ModelSpec | None) -> None:
    job.detail = 'loading engine' if job.load_only else 'building engine'
    with self.lock:
      self.job = job
      # The stage belongs to the job, not to the host: without this the first
      # event of a build carried the last one's ("load", 1.0, "ready") and a
      # client drew a full progress bar over a build that had not started.
      self._last_stage = (None, 0.0, '')
    threading.Thread(target=self._run, args=(job, req, entry, model_path, spec),
                     daemon=True, name='jetlink-build').start()
    self.emit('engine', self.snapshot())

  def _run(self, job: Job, req: Request, entry: CacheEntry, model_path: Path,
           spec: ModelSpec | None) -> None:
    engine = None
    try:
      # One engine resident at a time: a build needs the memory, and two 1.7 GB
      # engines do not fit in 8 GB even for a moment.
      self._unload()
      if not job.load_only:
        spec = self._build_job(req, entry, model_path, spec)
      assert spec is not None
      self._write_spec(entry, spec)

      self._progress('load', 0.0, 'deserializing engine', force=True)
      try:
        engine = self._load_engine(entry.path)
      except ArtifactInvalid as e:
        # Wrong on disk, not wrong here: a pickle from another tinygrad, a
        # compiled-model cache another runtime left. Replace it from the ONNX
        # when there is one, else let the client upload again.
        log.warning("discarding %s: %s", entry.path.name, e)
        entry.remove()
        # A preload names no size (it is a guess from last-loaded.json); a
        # client's request does, and the model has to match it.
        have = model_path.is_file() and (req.nbytes == 0 or model_path.stat().st_size == req.nbytes)
        if job.load_only and not have:
          raise RuntimeError(f"artifact invalid and the model is not on disk: {e}") from e
        spec = self._build_job(req, entry, model_path, spec)
        self._write_spec(entry, spec)
        self._progress('load', 0.0, 'deserializing engine', force=True)
        engine = self._load_engine(entry.path)
      loaded = self._warm(engine, spec)
      engine = None   # owned by `loaded` from here
      with self.lock:
        self.loaded = loaded
        job.state, job.detail = 'ready', ''
      self.cache.remember_loaded(req.sha256, req.frame_skip)
      self._progress('load', 1.0, 'ready', force=True)
      log.info("engine ready: %s", entry.path)
    except Exception as e:
      log.exception("engine preparation failed")
      if engine is not None:
        engine.close()
      with self.lock:
        job.state, job.detail = 'failed', f'{type(e).__name__}: {e}'
      self._progress('failed', 1.0, job.detail, force=True)
    finally:
      session = self.session
      if session is not None:
        self._serve_pending(job, session)
        session.engine_update()
      self.emit('engine', self.snapshot())

  def _serve_pending(self, done: Job, session: Session) -> None:
    """Start what the client is still waiting for, now the GPU is free.

    The job that just finished is not necessarily the one anybody asked for: a
    preload that guessed the wrong sha holds the GPU while the client waits,
    and when it landed nothing started the model actually wanted, so the client
    sat in _await_ready until its build_timeout ran out. A job for the client's
    own model is never retried here, whatever became of it: its outcome is the
    answer.
    """
    req = session.request
    if req is None or done.sha256 == req.sha256:
      return
    self.request(req, session)

  def _build_job(self, req: Request, entry: CacheEntry, model_path: Path,
                 spec: ModelSpec | None) -> ModelSpec:
    if spec is None:
      self._progress('parse', 0.0, 'reading model metadata', force=True)
      spec = self._derive_spec(model_path, req.frame_skip)
    self._build(model_path, entry.path, {'spec': spec.to_dict()})
    # Offroad the clock can be behind every plan on disk, so the one just
    # written has to be protected from the sweep.
    self.cache.prune(protect=entry.path)
    self.cache.sweep_temp()
    return spec

  @staticmethod
  def _write_spec(entry: CacheEntry, spec: ModelSpec) -> None:
    try:
      meta = entry.meta()
    except (OSError, ValueError):
      # A sidecar that predates specs, or one that went missing. Rewrite it
      # rather than fail a build that already ran.
      meta = {}
    if 'spec' not in meta:
      entry.write_meta({**meta, 'spec': spec.to_dict()})

  def _warm(self, engine, spec: ModelSpec) -> Loaded:
    from jetlink.queues import PolicyQueues
    queues = PolicyQueues(spec)
    _check_shapes(engine, spec)
    # Warm runs on zeros, so the first real frame pays for nothing lazy: CUDA
    # state and the graph capture for TensorRT, a first replay for the others.
    host_inputs = {n: engine.host_input(n) for n in engine.inputs}
    warped = np.zeros(spec.warped_shape, np.uint8)
    packed = np.zeros(spec.packed_nelem, np.float32)
    queues.step_into(warped, packed, host_inputs)
    log.info("%s", engine.warm())
    queues.reset()
    return Loaded(spec.sha256, spec, engine, queues, host_inputs)

  def unload(self) -> None:
    """Release the engine on a client's say-so. The job thread uses _unload."""
    self._unload()

  def _unload(self) -> None:
    with self.lock:
      loaded, self.loaded = self.loaded, None
    if loaded is not None:
      loaded.engine.close()
      log.info("engine %s unloaded", loaded.sha256[:16])
      self.emit('engine', self.snapshot())

  def close(self) -> None:
    self.telemetry.close()
    self._unload()

  # The backend seam: everything below touches a runtime or a real ONNX.

  def _load_engine(self, artifact: Path):
    return self.backend.load(artifact, report=self._progress)

  def _derive_spec(self, model_path: Path, frame_skip: int) -> ModelSpec:
    return spec_from_onnx(str(model_path), frame_skip=frame_skip)

  def _build(self, model_path: Path, artifact: Path, meta_extra: dict) -> None:
    self.backend.build(model_path, artifact, report=self._progress, meta_extra=meta_extra)

  # -- talking back ---------------------------------------------------------

  def _progress(self, stage: str, frac: float, msg: str = '', force: bool = False) -> None:
    now = time.monotonic()
    if not force and frac < 1.0 and now - self._last_progress < PROGRESS_MIN_INTERVAL:
      return
    self._last_progress = now
    self._last_stage = (stage, frac, msg)
    self.emit('progress', {'stage': stage, 'frac': round(frac, 4), 'msg': msg})
    session = self.session
    if session is not None:
      session.progress(stage, frac, msg)


def _model_complete(model_path: Path, nbytes: int) -> bool:
  """Uploads land in place chunk by chunk, so the model file is only the model
  once it is the size the client declared."""
  return model_path.is_file() and model_path.stat().st_size == nbytes


def _check_shapes(engine, spec: ModelSpec) -> None:
  """The engine is what will execute and the spec came from the file, so
  disagreement means the artifact on disk is not this model's."""
  for name, io in engine.inputs.items():
    want = spec.input_shapes.get(name)
    if want is None:
      raise ValueError(f"engine input {name!r} is not in the model spec")
    if int(np.prod(io.shape)) != int(np.prod(want)):
      raise ValueError(f"input {name}: engine {tuple(io.shape)} vs spec {want}")
  missing = set(spec.input_shapes) - set(engine.inputs)
  if missing:
    raise ValueError(f"engine has no input(s) {sorted(missing)} the model spec declares")
  out = next(iter(engine.outputs.values())).shape
  if int(np.prod(out)) != spec.output_nelem:
    raise ValueError(f"output: engine {tuple(out)} vs spec {spec.output_nelem}")


class Session:
  def __init__(self, transport: Transport, host: EngineHost):
    self.t = transport
    self.host = host
    self.telemetry = host.telemetry
    self.send_lock = threading.Lock()
    self.client = ''   # who said hello; see _greet
    self._reset(0)
    self.telemetry.read()  # request the first sample without blocking connection setup

  def _reset(self, seq: int) -> None:
    """Everything counted per connection, back where a fresh client expects it:
    the seq it counts replays from, the model it asked for, the frames it has
    been served. The engine is not per connection and stays; see EngineHost."""
    self.last_seq = seq
    self.request: Request | None = None
    self.frames = 0

  # -- plumbing -------------------------------------------------------------

  def _send(self, msg_type: int, seq: int, parts=(), flags: int = 0) -> None:
    with self.send_lock:
      self.t.send(msg_type, seq, parts, flags)

  def _send_json(self, msg_type: int, seq: int, obj, flags: int = 0) -> None:
    self._send(msg_type, seq, (json.dumps(obj).encode(),), flags)

  def _error(self, seq: int, error: str, detail: str = '') -> None:
    log.error("%s: %s", error, detail)
    self._send_json(P.Msg.ERROR, seq, {'error': error, 'detail': detail})

  def progress(self, stage: str, frac: float, msg: str) -> None:
    try:
      self._send_json(P.Msg.PROGRESS, 0, {'stage': stage, 'frac': round(frac, 4), 'msg': msg})
    except LinkError:
      pass  # the comma may have given up and fallen back; the build continues

  def engine_update(self) -> None:
    """The worker finished. Tell the client that is here now, whoever it is."""
    try:
      self._respond_engine(0)
    except LinkError:
      pass

  def close(self) -> None:
    """The connection is gone. The engine stays: see EngineHost."""
    with self.host.lock:
      if self.host.session is self:
        self.host.session = None

  def serve_forever(self) -> None:
    while True:
      try:
        msg = self.t.recv()
      except LinkTimeout:
        continue
      except LinkError as e:
        log.info("link closed: %s", e)
        return
      try:
        self.handle(msg)
      except LinkError:
        raise
      except Exception as e:  # a bad request must not take the server down
        log.exception("handler failed")
        self._error(msg.seq, type(e).__name__, str(e))

  def handle(self, msg: Message) -> None:
    mt = msg.msg_type
    if mt == P.Msg.HELLO_REQ:
      # A hello is the one message that means "a new client process", and it
      # is answered whatever the seq says. A client always starts at seq 1, so
      # a session that outlived its client - the Jetson's hub driver keeping
      # the usb_device across a comma rebind, which is what a handover looks
      # like from here - would drop the new modeld's hello as a replay and
      # answer nothing at all. That was a whole drive on the small model.
      self._greet(msg)
      self.on_hello(msg)
      return
    # dwc3 occasionally sends a request twice (see FfsTransport.write_chunk).
    # Seqs never repeat on a connection, so anything at or below the last one is
    # a replay; running it would push the same image into the queues twice.
    if msg.seq <= self.last_seq:
      log.warning("dropping replayed message type=%d seq=%d (last %d) from %s",
                  msg.msg_type, msg.seq, self.last_seq, self.client or 'an unnamed client')
      return
    self.last_seq = msg.seq
    if mt == P.Msg.INFER_REQ:
      self.on_infer(msg)
    elif mt == P.Msg.PING:
      self._send(P.Msg.PONG, msg.seq)
    elif mt == P.Msg.ENGINE_REQ:
      self.on_engine_req(msg)
    elif mt == P.Msg.UPLOAD_CHUNK:
      self.on_upload_chunk(msg)
    elif mt == P.Msg.UPLOAD_DONE:
      self.on_upload_done(msg)
    elif mt == P.Msg.STATE_REQ:
      self.on_state(msg)
    elif mt == P.Msg.SHUTDOWN_REQ:
      self.on_shutdown(msg)
    else:
      self._error(msg.seq, 'unknown_message', f'type {mt}')

  def _wanted(self) -> tuple[str | None, int | None]:
    """What this session asked for. The frame_skip goes with the sha because
    the engine's identity includes it; see EngineHost.status."""
    return (self.request.sha256, self.request.frame_skip) if self.request else (None, None)

  def _greet(self, msg: Message) -> None:
    """Start the session over for whoever just said hello."""
    who = ''
    try:
      d = json.loads(bytes(msg.payload) or b'{}').get('client') or {}
      who = f"{d.get('name') or 'client'}/{d.get('nonce') or '?'}"
    except (ValueError, AttributeError):
      pass
    if self.client and who != self.client:
      log.info("session handed from %s to %s", self.client, who or 'an unnamed client')
    self.client = who
    self._reset(msg.seq)
    log.info("hello from %s (seq %d)", who or 'an unnamed client', msg.seq)

  def on_hello(self, msg: Message) -> None:
    info = self.host.backend.describe()
    resp = {
      'protocol': P.VERSION,
      **info,   # backend, runtime_version, device
      'engine_state': self.host.status(*self._wanted())['state'],
      'loaded': self.host.loaded_sha(),
      'frames_served': self.frames,
      'cached_models': self.host.cache.inventory(),
      'telemetry': self.telemetry.read(),
      # 0 when this server never suspends. The comma holds the gadget for the
      # whole parked period rather than letting go of it for a box that was
      # never going to sleep; see jetlinkd.go_dormant.
      'sleep_after': self.host.sleep_after,
    }
    if info.get('backend') == 'trt':
      # What the comma logged before there were backends. Kept for one
      # release, and only where it is true.
      resp['trt_version'] = info['runtime_version']
    self._send_json(P.Msg.HELLO_RESP, msg.seq, resp)

  def on_engine_req(self, msg: Message) -> None:
    d = json.loads(bytes(msg.payload))
    self.request = Request(str(d['sha256']), int(d['nbytes']),
                           int(d.get('frame_skip', DEFAULT_FRAME_SKIP)))
    self._send_json(P.Msg.ENGINE_RESP, msg.seq, self.host.request(self.request, self))

  def _respond_engine(self, seq: int) -> None:
    self._send_json(P.Msg.ENGINE_RESP, seq, self.host.status(*self._wanted()))

  def on_upload_chunk(self, msg: Message) -> None:
    req = self.request
    if req is None:
      return self._error(msg.seq, 'no_model', 'send ENGINE_REQ first')
    if msg.payload.nbytes < 8:
      return self._error(msg.seq, 'bad_upload', 'missing chunk offset')
    offset = int.from_bytes(bytes(msg.payload[:8]), 'little')
    data = msg.payload[8:]
    if offset + data.nbytes > req.nbytes:
      return self._error(msg.seq, 'bad_upload', 'chunk exceeds declared model size')
    path = self.host.cache.model_path(req.sha256)
    mode = 'r+b' if path.exists() and offset else 'wb'
    with open(path, mode) as f:
      f.seek(offset)
      f.write(data)
    # Silent on purpose: the client streams chunks without reading between
    # them, so a progress write here stalls for the whole transfer timeout.

  def on_upload_done(self, msg: Message) -> None:
    req = self.request
    if req is None:
      return self._error(msg.seq, 'no_model', 'send ENGINE_REQ first')
    path = self.host.cache.model_path(req.sha256)
    if sha256_file(str(path))[0] != req.sha256:
      path.unlink(missing_ok=True)
      self._send_json(P.Msg.ENGINE_RESP, msg.seq, {
        'state': 'failed', 'detail': 'sha256 mismatch after upload',
        'sha256': req.sha256, 'chunk': CHUNK})
      return
    self.progress('upload', 1.0, 'verified')
    self._send_json(P.Msg.ENGINE_RESP, msg.seq, self.host.request(req, self))

  # -- the hot path ---------------------------------------------------------

  def on_infer(self, msg: Message) -> None:
    host = self.host
    wanted_sha, wanted_skip = self._wanted()
    with host.lock:
      loaded = host.loaded
      if (loaded is None or loaded.sha256 != wanted_sha
          or loaded.spec.frame_skip != wanted_skip):
        self._send(P.Msg.INFER_RESP, msg.seq,
                   (P.pack_infer_resp(0, P.Status.NOT_READY, 0, 0, 0),))
        return
      self._infer(loaded, msg)

  def _infer(self, loaded: Loaded, msg: Message) -> None:
    t0 = time.perf_counter()
    spec = loaded.spec
    if msg.payload.nbytes != spec.infer_req_nbytes:
      # The offsets below come from the spec, not the wire: a client on another
      # model would have its scalars read out of the image, and the result would
      # look perfectly finite.
      self._send(P.Msg.INFER_RESP, msg.seq,
                 (P.pack_infer_resp(0, P.Status.BAD_SHAPE, 0, 0, 0),))
      return
    frame_id, flags = P.unpack_infer_req(msg.payload)
    if flags & P.Flag.RESET_QUEUES:
      loaded.queues.reset()

    off = P.INFER_REQ_SIZE
    warped = np.frombuffer(msg.payload, np.uint8, spec.warped_nbytes, off).reshape(spec.warped_shape)
    off += spec.warped_nbytes
    packed = np.frombuffer(msg.payload, np.float32, spec.packed_nelem, off)

    # Gathers land straight in TensorRT's pinned input buffers: between the
    # wire and the GPU there is exactly one copy.
    loaded.queues.step_into(warped, packed, loaded.host_inputs)
    queue_us = int((time.perf_counter() - t0) * 1e6)

    outputs = loaded.engine.run()
    out = next(iter(outputs.values())).reshape(-1)

    # asarray, not astype: a no-op when the engine already outputs float32.
    # isfinite is ~7x faster on float32 and the non-finites map across exactly.
    # openpilot drops to the small model on a non-finite output either way, so
    # checking here saves the comma rescanning 18452 floats to find out.
    out32 = np.asarray(out, dtype=np.float32)
    status = P.Status.OK if np.all(np.isfinite(out32)) else P.Status.NOT_FINITE

    total_us = int((time.perf_counter() - t0) * 1e6)
    parts = [P.pack_infer_resp(frame_id, status, loaded.engine.last_gpu_us, queue_us, total_us),
             out32]
    if flags & P.Flag.WANT_STATE:
      parts.append(json.dumps(self.telemetry.read()).encode())
    send_started = time.perf_counter()
    self._send(P.Msg.INFER_RESP, msg.seq, parts)
    send_us = int((time.perf_counter() - send_started) * 1e6)
    self.frames += 1
    if total_us > SLOW_FRAME_US or send_us > 10_000:
      # After the reply, so the log never delays it. The comma logs the same
      # frame by its own stages; together they place a slow frame.
      log.warning("slow frame %d: gpu %.1f queue %.1f total %.1f send %.1f ms", frame_id,
                  loaded.engine.last_gpu_us / 1e3, queue_us / 1e3, total_us / 1e3, send_us / 1e3)
    self.host.frame_stats.record(total_us, loaded.engine.last_gpu_us)

  def on_shutdown(self, msg: Message) -> None:
    from jetlink.server.power import request_poweroff
    d = json.loads(bytes(msg.payload) or b'{}')
    reason = str(d.get('reason', ''))
    log.warning("shutdown requested by the client: %s", reason or 'no reason given')
    # Reply first: the host acts on the flag within milliseconds and stops the
    # container on its way down.
    self._send_json(P.Msg.SHUTDOWN_RESP, msg.seq, {'ok': True, 'detail': 'powering off'})
    request_poweroff(self.host.cache.root, reason)

  def on_state(self, msg: Message) -> None:
    st = self.host.status(*self._wanted())
    self._send_json(P.Msg.STATE_RESP, msg.seq, {
      **self.telemetry.read(),
      'engine_state': st['state'],
      'detail': st.get('detail', ''),
      'loaded': self.host.loaded_sha(),
      'frames_served': self.frames,
    })
