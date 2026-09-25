"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

The control channel: what the server is doing, and what to do next.

A local stream socket speaking JSON lines, one object per line. A management
client (the Mac app, or socat) connects and is told everything it needs for a
first screen without asking: hello, server, link, engine, inventory, catalog,
then whatever downloads are in flight. After that it sends commands and hears
events; every event goes to every client.

Nothing here runs on the frame path. The one thing it reads from a served
frame is EngineHost.frame_stats, a lock-free deque the request loop appends to
after the reply is on the wire. Everything slow - a catalog fetch, a download,
a file hash - runs on a single-worker executor so a command reply is never the
thing waiting on the network, and so two downloads can never fight over the
disk. Builds and loads are not started here either: `prepare` is the comma's
ENGINE_REQ path with no comma on the other end, so there is exactly one piece
of code that puts an engine on the GPU.
"""
from __future__ import annotations

import _thread
import errno
import json
import logging
import math
import os
import queue
import signal
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

from jetlink.registry import CATALOG_URL, DEFAULT_BIG_MODEL_REF
from jetlink.server.cache import LAST_LOADED, _SHA256, EngineCache
from jetlink.server.session import EngineHost, Request
from jetlink.spec import DEFAULT_FRAME_SKIP

log = logging.getLogger('jetlink.control')

PROTOCOL = 1

# A client that is not reading is a client that is gone; 1000 lines is minutes
# of events, and holding more of them costs the server memory it needs.
QUEUE_MAX = 1000
MAX_LINE = 1 << 20
BACKLOG = 4

STATS_INTERVAL = 1.0
INVENTORY_TTL = 2.0        # a build finishing publishes twice; the app may ask as well
PROGRESS_INTERVAL = 0.25   # s between download events, matching the engine's progress rate
RATE_INTERVAL = 0.5        # s of bytes behind rate_bps
CATALOG_MAX_AGE = 3600.0
JOIN_TIMEOUT = 1.0

# A handler that has already replied itself.
SILENT = object()


class ControlError(Exception):
  """A command that could not be carried out. The text becomes the reply's error."""


@dataclass
class _Download:
  """One download, queued or running, and the last event it published."""
  sha256: str
  ref: str | None = None
  total: int = 0
  source: str | None = None
  cancel: bool = False
  future: object = None
  frac: float = 0.0
  last: dict = field(default_factory=dict)
  last_event: float = 0.0
  rate: float = 0.0
  rate_bytes: int = 0
  rate_at: float = 0.0


class _Client:
  """One connection: a queue of outgoing lines, a writer and a reader."""

  def __init__(self, sock: socket.socket):
    self.sock = sock
    self.out: queue.Queue = queue.Queue(maxsize=QUEUE_MAX)

  def offer(self, line: bytes | None) -> bool:
    try:
      self.out.put_nowait(line)
      return True
    except queue.Full:
      return False


class ControlServer:
  """Listens, broadcasts events, and runs commands. See docs above."""

  def __init__(self, address: str, host: EngineHost, cache: EngineCache,
               registry=None, info: dict | None = None):
    if registry is None:
      # Built here rather than by main(): this module is the only caller, and
      # it is only imported at all when somebody opened a control channel.
      from jetlink.registry import Registry
      registry = Registry(cache.root)
    self.address = address
    self.host = host
    self.cache = cache
    self.registry = registry
    self.info = dict(info or {})
    self._lock = threading.RLock()
    self._clients: list[_Client] = []
    self._threads: list[threading.Thread] = []
    self._sock: socket.socket | None = None
    self._path: Path | None = None
    self._closing = threading.Event()
    self._link = {'state': 'waiting', 'detail': '', 'peer': None}
    self._active: dict[str, _Download] = {}
    self._catalog_kicked = False
    self._inventory_at = 0.0
    self._inventory_payload: dict | None = None
    self._net = None
    self._downloads = None
    self._commands = {
      'status': self._cmd_status,
      'catalog': self._cmd_catalog,
      'download': self._cmd_download,
      'cancel_download': self._cmd_cancel_download,
      'import': self._cmd_import,
      'prepare': self._cmd_prepare,
      'unload': self._cmd_unload,
      'forget': self._cmd_forget,
      'inventory': self._cmd_inventory,
      'shutdown': self._cmd_shutdown,
    }

  # -- lifecycle ------------------------------------------------------------

  def start(self) -> None:
    from concurrent.futures import ThreadPoolExecutor
    self._sock = self._listen()
    self._net = ThreadPoolExecutor(max_workers=1, thread_name_prefix='jetlink-registry')
    self._downloads = ThreadPoolExecutor(max_workers=1, thread_name_prefix='jetlink-download')
    self.host.subscribe(self._on_host)
    self._spawn('jetlink-control', self._accept_loop)
    self._spawn('jetlink-stats', self._stats_loop)
    log.info("control channel listening on %s", self.address)

  def _listen(self) -> socket.socket:
    if self.address.startswith('tcp://'):
      hostname, _, port = self.address[len('tcp://'):].partition(':')
      if hostname not in ('127.0.0.1', 'localhost'):
        raise ValueError('the control channel listens on 127.0.0.1 only')
      sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      sock.bind(('127.0.0.1', int(port or 0)))
      sock.listen(BACKLOG)
      # Port 0 means "pick one", and the caller has to be able to find out.
      self.address = f"tcp://127.0.0.1:{sock.getsockname()[1]}"
      return sock

    path = Path(self.address)
    if path.exists():
      # A socket file outlives the process that made it. Refused means nothing
      # is listening and it is ours to replace; answered means another server
      # is up and this one must not take its place.
      probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
      try:
        probe.connect(str(path))
      except OSError as e:
        if e.errno not in (errno.ECONNREFUSED, errno.ENOENT):
          raise
        path.unlink(missing_ok=True)
      else:
        raise OSError(f"a jetlink server is already listening on {path}")
      finally:
        probe.close()
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(str(path))
    os.chmod(path, 0o600)
    sock.listen(BACKLOG)
    self._path = path
    return sock

  def _spawn(self, name: str, target, *args) -> None:
    t = threading.Thread(target=target, args=args, daemon=True, name=name)
    self._threads.append(t)
    t.start()

  def close(self) -> None:
    if self._closing.is_set():
      return
    self._closing.set()
    sock, self._sock = self._sock, None
    if sock is not None:
      try:
        sock.close()   # unblocks accept
      except OSError:
        pass
    with self._lock:
      clients, self._clients = list(self._clients), []
      for state in self._active.values():
        state.cancel = True
    for client in clients:
      # The sentinel goes behind whatever is queued, so a reply already written
      # (a shutdown command's) still reaches the client that asked for it.
      client.offer(None)
    for t in self._threads:
      if t is not threading.current_thread():
        t.join(JOIN_TIMEOUT)
    for client in clients:
      _close(client.sock)
    for pool in (self._net, self._downloads):
      if pool is not None:
        pool.shutdown(wait=False, cancel_futures=True)
    if self._path is not None:
      self._path.unlink(missing_ok=True)
    log.info("control channel closed")

  # -- connections ----------------------------------------------------------

  def _accept_loop(self) -> None:
    while not self._closing.is_set():
      try:
        conn, _ = self._sock.accept()
      except OSError:
        return   # closed, or the listening socket is gone
      client = _Client(conn)
      with self._lock:
        self._clients.append(client)
      self._spawn('jetlink-control-write', self._writer, client)
      self._spawn('jetlink-control-read', self._reader, client)

  def _writer(self, client: _Client) -> None:
    try:
      while True:
        line = client.out.get()
        if line is None:
          return
        client.sock.sendall(line)
    except OSError:
      pass
    finally:
      self._drop(client)

  def _reader(self, client: _Client) -> None:
    try:
      self._on_connect(client)
      with client.sock.makefile('rb') as f:
        while not self._closing.is_set():
          raw = f.readline(MAX_LINE + 1)
          if not raw:
            return
          if len(raw) > MAX_LINE:
            self._reply(client, None, False, 'that line is longer than the server accepts')
            return
          self._handle(client, raw)
    except OSError:
      pass
    finally:
      self._drop(client)

  def _drop(self, client: _Client) -> None:
    with self._lock:
      if client in self._clients:
        self._clients.remove(client)
    client.offer(None)
    _close(client.sock)

  def _on_connect(self, client: _Client) -> None:
    """Everything a client needs for its first screen, before it asks."""
    self._to(client, 'hello', {'protocol': PROTOCOL, 'pid': os.getpid(), **self.info})
    self._to(client, 'server', self._server_payload('serving'))
    self._to(client, 'link', dict(self._link))
    self._to(client, 'engine', self.host.snapshot())
    self._to(client, 'inventory', self._inventory())
    catalog = self._catalog_payload()
    self._to(client, 'catalog', catalog)
    with self._lock:
      pending = [dict(d.last) for d in self._active.values() if d.last]
    for payload in pending:
      self._to(client, 'download', payload)
    if catalog.get('fetched_at') is None and not self._catalog_kicked:
      # Nothing cached to show. Go and get it off the burst, once for the
      # process however many clients connect, and publish it when it lands.
      self._catalog_kicked = True
      self._net.submit(self._refresh_catalog, False)

  # -- publishing -----------------------------------------------------------

  def publish(self, event: str, payload: dict) -> None:
    """Send one event to every client."""
    line = _line(event, payload)
    with self._lock:
      clients = list(self._clients)
    for client in clients:
      self._write(client, line)

  def _to(self, client: _Client, event: str, payload: dict) -> None:
    self._write(client, _line(event, payload))

  def _write(self, client: _Client, line: bytes) -> None:
    if not client.offer(line):
      log.warning("a control client stopped reading; dropping it")
      self._drop(client)

  def _reply(self, client: _Client, cid, ok: bool, error: str | None = None, extra: dict | None = None) -> None:
    payload = {'id': cid, 'ok': ok, 'error': error}
    if extra:
      payload.update(extra)
    self._to(client, 'reply', payload)

  def _server_payload(self, state: str, detail: str = '') -> dict:
    try:
      info = self.host.backend.describe()
    except Exception as e:
      info, detail = {}, detail or f'{type(e).__name__}: {e}'
    return {'state': state, 'detail': detail, 'backend': info.get('backend'),
            'runtime_version': info.get('runtime_version'), 'device': info.get('device')}

  def _catalog_payload(self, error: str | None = None) -> dict:
    """What is on disk, with no network at all. Fetching is the `catalog` command.

    registry.catalog() fetches whenever it has nothing cached, whatever max_age
    says, so on a first run with no network a connecting client would wait out
    the http timeout for its sixth event, and again on every reconnect. An
    empty list now and a `catalog` event when the fetch lands is what a client
    can actually render.
    """
    try:
      if not self.registry.catalog_path.exists():
        return {'fetched_at': None, 'url': CATALOG_URL, 'default_ref': DEFAULT_BIG_MODEL_REF,
                'error': error, 'models': []}
      payload = self.registry.catalog(refresh=False, max_age=math.inf)
    except Exception as e:
      log.warning("no catalog to serve: %s", e)
      payload = {'fetched_at': None, 'url': '', 'default_ref': '', 'models': [],
                 'error': f'{type(e).__name__}: {e}'}
    return {**payload, 'error': error} if error is not None else payload

  def _inventory(self, fresh: bool = False) -> dict:
    now = time.monotonic()
    with self._lock:
      if not fresh and self._inventory_payload is not None and now - self._inventory_at < INVENTORY_TTL:
        return self._inventory_payload
    payload = self.registry.inventory(self.cache)
    payload['loaded'] = self.host.loaded_sha()
    with self._lock:
      self._inventory_payload, self._inventory_at = payload, now
    return payload

  def _publish_inventory(self) -> None:
    self.publish('inventory', self._inventory(fresh=True))

  # -- events from the host -------------------------------------------------

  def _on_host(self, kind: str, payload: dict) -> None:
    if self._closing.is_set():
      return
    if kind == 'progress':
      # The snapshot already carries the stage this progress set.
      self.publish('engine', self.host.snapshot())
    elif kind == 'engine':
      snapshot = self.host.snapshot()
      self.publish('engine', snapshot)
      if snapshot['state'] in ('ready', 'failed'):
        self._publish_inventory()   # a build or a load changed the disk
    elif kind == 'link':
      self._link = dict(payload)
      self.publish('link', self._link)

  def _stats_loop(self) -> None:
    """A frame summary a second, while a comma is connected and sending.

    The window is the time since the last tick, not a flat second: waking is
    never exactly on the second, and a fixed window leaves a sliver between
    ticks that a frame can land in and never be counted.
    """
    last = time.perf_counter()
    while not self._closing.wait(STATS_INTERVAL):
      try:
        now = time.perf_counter()
        window, last = now - last, now
        if self._link.get('state') != 'connected':
          continue
        session = self.host.session
        payload = self.host.frame_stats.summary(
          window, frames_total=session.frames if session is not None else 0)
        if payload is not None:
          self.publish('stats', payload)
      except Exception:
        log.exception("the stats ticker failed")

  # -- commands -------------------------------------------------------------

  def _handle(self, client: _Client, raw: bytes) -> None:
    line = raw.strip()
    if not line:
      return
    try:
      msg = json.loads(line)
      if not isinstance(msg, dict):
        raise ValueError('a command must be a json object')
      cid = msg.get('id')
      if not isinstance(cid, int) or isinstance(cid, bool):
        raise ValueError('a command needs an integer id')
    except (ValueError, UnicodeDecodeError) as e:
      self._reply(client, None, False, str(e))
      return
    name = msg.get('cmd')
    fn = self._commands.get(name)
    if fn is None:
      self._reply(client, cid, False, f'there is no command called {name!r}')
      return
    try:
      extra = fn(client, msg)
    except ControlError as e:
      self._reply(client, cid, False, str(e))
    except Exception as e:
      log.exception("the %s command failed", name)
      self._reply(client, cid, False, f'{type(e).__name__}: {e}')
    else:
      if extra is not SILENT:
        self._reply(client, cid, True, None, extra)

  def _cmd_status(self, client: _Client, msg: dict) -> dict:
    self.publish('server', self._server_payload('serving'))
    self.publish('link', dict(self._link))
    self.publish('engine', self.host.snapshot())
    self.publish('inventory', self._inventory())
    self.publish('catalog', self._catalog_payload())
    return {}

  def _cmd_catalog(self, client: _Client, msg: dict) -> dict:
    self._net.submit(self._refresh_catalog, bool(msg.get('refresh', False)))
    return {'queued': True}

  def _refresh_catalog(self, refresh: bool) -> None:
    try:
      payload = self.registry.catalog(refresh=refresh, max_age=CATALOG_MAX_AGE)
      missing = [m['ref'] for m in payload.get('models', []) if not m.get('sha256')]
      if missing:
        self.registry.resolve_missing(missing)
        # Re-read rather than patch: the registry owns what a pointer means.
        payload = self.registry.catalog(refresh=False, max_age=math.inf)
    except Exception as e:
      log.warning("the catalog refresh failed: %s", e)
      self.publish('catalog', self._catalog_payload(error=f'{type(e).__name__}: {e}'))
      return
    self.publish('catalog', payload)

  def _cmd_download(self, client: _Client, msg: dict) -> dict:
    ref, sha256 = msg.get('ref'), msg.get('sha256')
    if bool(ref) == bool(sha256):
      raise ControlError('a download needs exactly one of ref and sha256')
    if sha256:
      # The bytes come from an LFS object, and an LFS object is an oid plus a
      # size: a sha on its own is not enough to ask for one. Its ref is.
      sha256 = str(sha256)
      if _SHA256.fullmatch(sha256) is None:
        raise ControlError('sha256 must be a lowercase SHA-256 digest')
      ref = self._ref_for(sha256)
      if ref is None:
        raise ControlError(f'model {sha256[:16]} is not in the catalog')
    # 134 bytes when it is not already cached, and the reply has to carry the
    # sha, so this one resolve waits.
    try:
      pointer = self.registry.resolve(str(ref))
    except Exception as e:
      raise ControlError(f'could not resolve {ref}: {e}') from e
    sha256, total = pointer.oid, int(pointer.size)
    path = self.registry.model_path(sha256)
    if path.exists() and (not total or path.stat().st_size == total):
      raise ControlError(f'model {sha256[:16]} is already downloaded')
    with self._lock:
      if sha256 in self._active:
        raise ControlError(f'model {sha256[:16]} is already downloading')
      state = _Download(sha256=sha256, ref=str(ref) if ref else None, total=total)
      self._active[sha256] = state
      state.future = self._downloads.submit(self._run_download, state)
    return {'sha256': sha256}

  def _ref_for(self, sha256: str) -> str | None:
    """The catalog ref whose model has this sha, from the pointers or the list."""
    try:
      _, ref = self.registry.name_for(sha256)
      if ref:
        return ref
      for model in self.registry.catalog(refresh=False, max_age=math.inf).get('models', []):
        if model.get('sha256') == sha256:
          return model.get('ref')
    except Exception as e:
      log.warning("could not look up %s in the catalog: %s", sha256[:16], e)
    return None

  def _run_download(self, state: _Download) -> None:
    if state.cancel:
      self._download_event(state, 'cancelled')
      self._forget_download(state)
      return
    self._download_event(state, 'started')
    try:
      self.registry.fetch(state.ref or state.sha256,
                          progress=lambda frac: self._download_progress(state, frac),
                          should_stop=lambda: state.cancel)
    except Exception as e:
      if state.cancel:
        self._download_event(state, 'cancelled')
      else:
        log.warning("downloading %s failed: %s", state.sha256[:16], e)
        self._download_event(state, 'failed', detail=f'{type(e).__name__}: {e}')
      self._forget_download(state)
      return
    if state.cancel:
      self._download_event(state, 'cancelled')
      self._forget_download(state)
      return
    self._download_event(state, 'done', frac=1.0)
    self._forget_download(state)
    self._publish_inventory()

  def _download_progress(self, state: _Download, frac: float) -> None:
    if frac < 1.0 and time.monotonic() - state.last_event < PROGRESS_INTERVAL:
      return
    self._download_event(state, 'progress', frac=frac)

  def _download_event(self, state: _Download, kind: str, frac: float | None = None, detail: str = '') -> None:
    now = time.monotonic()
    if frac is not None:
      state.frac = max(0.0, min(1.0, float(frac)))
    done = int(state.frac * state.total)
    elapsed = now - state.rate_at
    if state.rate_at and elapsed >= RATE_INTERVAL:
      state.rate = (done - state.rate_bytes) / elapsed
    if not state.rate_at or elapsed >= RATE_INTERVAL:
      state.rate_bytes, state.rate_at = done, now
    state.last_event = now
    state.last = {'sha256': state.sha256, 'ref': state.ref, 'state': kind,
                  'frac': round(state.frac, 4), 'bytes': done, 'total': state.total,
                  'rate_bps': round(state.rate, 1), 'detail': detail, 'source': state.source}
    self.publish('download', state.last)

  def _forget_download(self, state: _Download) -> None:
    with self._lock:
      self._active.pop(state.sha256, None)

  def _cmd_cancel_download(self, client: _Client, msg: dict) -> dict:
    sha256 = str(msg.get('sha256') or '')
    with self._lock:
      state = self._active.get(sha256)
    if state is None:
      raise ControlError(f'no download of {sha256[:16] or "that model"} is running')
    state.cancel = True
    if state.future is not None and state.future.cancel():
      # Never started, so nothing will publish for it but us.
      self._download_event(state, 'cancelled')
      self._forget_download(state)
    return {}

  def _cmd_import(self, client: _Client, msg: dict) -> dict:
    raw = str(msg.get('path') or '')
    path = Path(raw).expanduser()
    if not path.is_file():
      raise ControlError(f'{raw} is not a file')
    self._net.submit(self._run_import, path, msg.get('name'))
    return {'queued': True}

  def _run_import(self, path: Path, name) -> None:
    seen = [-1.0]

    def progress(frac: float) -> None:
      # The registry hashes over the first half of the fraction and copies over
      # the second. The sha is only known when it returns, so the events before
      # that carry none rather than a second read of a 766 MB file to find it.
      if frac - seen[0] >= 0.05 or frac >= 1.0:
        seen[0] = frac
        self._import_event(path, 'hashing' if frac < 0.5 else 'copying', frac=frac)

    self._import_event(path, 'hashing')
    try:
      local = self.registry.import_model(path, name=None if name is None else str(name), progress=progress)
    except Exception as e:
      log.warning("importing %s failed: %s", path, e)
      self._import_event(path, 'failed', frac=1.0, detail=f'{type(e).__name__}: {e}')
      return
    self._import_event(path, 'done', frac=1.0, sha256=local.sha256)
    self._publish_inventory()

  def _import_event(self, path: Path, state: str, frac: float = 0.0, sha256: str | None = None,
                    detail: str = '') -> None:
    self.publish('import', {'path': str(path), 'state': state, 'frac': round(frac, 4),
                            'sha256': sha256, 'detail': detail})

  def _cmd_prepare(self, client: _Client, msg: dict) -> dict:
    sha256 = str(msg.get('sha256') or '')
    if _SHA256.fullmatch(sha256) is None:
      raise ControlError('sha256 must be a lowercase SHA-256 digest')
    frame_skip = int(msg.get('frame_skip') or DEFAULT_FRAME_SKIP)
    entry = self.cache.entry(sha256)
    model_path = self.cache.model_path(sha256)
    if not entry.exists and not model_path.is_file():
      raise ControlError(f'model {sha256[:16]} is not downloaded')
    # The comma's own path, with no comma: one piece of code loads an engine.
    self.host.request(Request(sha256, self._nbytes(sha256, entry, model_path), frame_skip), None)
    return {'state': self.host.snapshot()['state']}

  def _nbytes(self, sha256: str, entry, model_path: Path) -> int:
    """What the model weighs, for a Request that has no client behind it."""
    if model_path.is_file():
      return model_path.stat().st_size
    try:
      nbytes = int((entry.meta().get('spec') or {}).get('nbytes', 0))
      if nbytes > 0:
        return nbytes
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
      pass
    try:
      _, ref = self.registry.name_for(sha256)
      if ref:
        return int(self.registry.resolve(ref).size)
    except Exception:
      pass
    return 0

  def _cmd_unload(self, client: _Client, msg: dict) -> dict:
    self.host.unload()
    return {}

  def _cmd_forget(self, client: _Client, msg: dict) -> dict:
    sha256 = str(msg.get('sha256') or '')
    if _SHA256.fullmatch(sha256) is None:
      raise ControlError('sha256 must be a lowercase SHA-256 digest')
    job = self.host.job
    if job is not None and job.sha256 == sha256 and job.state == 'building':
      raise ControlError('a build for this model is running')
    if self.host.loaded_sha() == sha256:
      self.host.unload()
    self.registry.remove(sha256, bool(msg.get('artifacts', False)), bool(msg.get('model', False)))
    remembered = self.cache.last_loaded()
    if remembered is not None and remembered[0] == sha256 and not self.cache.entry(sha256).exists:
      # Preloading a plan that is no longer there costs a start-up failure for
      # nothing; there is no artifact left to load.
      (self.cache.root / LAST_LOADED).unlink(missing_ok=True)
    self._publish_inventory()
    return {}

  def _cmd_inventory(self, client: _Client, msg: dict) -> dict:
    self.publish('inventory', self._inventory())
    return {}

  def _cmd_shutdown(self, client: _Client, msg: dict) -> object:
    # Reply first, then say why the events stop; the writer thread keeps the
    # order, and close() lets it drain before the socket goes.
    self._reply(client, msg.get('id'), True, None)
    self.publish('server', self._server_payload('stopping'))
    interrupt_main()
    return SILENT


def interrupt_main() -> None:
  """Stop the server the way Ctrl-C does, from a thread that is not the main one.

  A process-directed SIGINT rather than _thread.interrupt_main(): the main
  thread sits in a blocking accept() or a libusb call, and a pending Python
  exception is only raised at the next bytecode, so interrupt_main() leaves a
  tcp server hanging until the next client connects. The signal gives the
  syscall EINTR and the handler already in main() takes it from there.
  """
  try:
    if sys.platform == 'win32':
      raise OSError('no process-directed SIGINT on Windows')
    os.kill(os.getpid(), signal.SIGINT)
  except (AttributeError, OSError, ValueError):
    _thread.interrupt_main()


def _line(event: str, payload: dict) -> bytes:
  return (json.dumps({'event': event, 't': time.time(), **payload}, separators=(',', ':')) + '\n').encode()


def _close(sock: socket.socket) -> None:
  try:
    sock.shutdown(socket.SHUT_RDWR)
  except OSError:
    pass
  try:
    sock.close()
  except OSError:
    pass
