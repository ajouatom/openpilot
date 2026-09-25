#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Inference server entrypoint: a Jetson in the car, or any machine with a GPU.

    # as the USB host, which is how the Jetson runs in the car
    python3 -m jetlink.server.main --transport usb

    # over ethernet, for development and benchmarking
    python3 -m jetlink.server.main --transport tcp --port 5599

    # on a Mac: CoreML through onnxruntime by default, tinygrad on Metal by name;
    # scripts/run-mac.sh wraps this
    python3 -m jetlink.server.main --transport tcp
    python3 -m jetlink.server.main --backend tinygrad --device METAL --transport tcp

    # build an engine ahead of time, no client needed
    python3 -m jetlink.server.main --build /path/to/big_driving_supercombo.onnx

--backend auto picks TensorRT where it imports, CoreML on a Mac, then
tinygrad, then onnxruntime on whatever it has; docs/platforms.md has the
measured frame times and start-up costs behind that order.
"""
from __future__ import annotations

import _thread
import argparse
import logging
import os
import signal
import sys
import threading
import time
from pathlib import Path
from platform import python_version

from jetlink.server import platform
from jetlink.server.backends import NAMES, available, select
from jetlink.server.cache import EngineCache
from jetlink.server.session import EngineHost, Session
from jetlink.server.sleep import SLEEP_AFTER, Sleeper
from jetlink.server.telemetry import pick_source
from jetlink.transport.base import LinkError

log = logging.getLogger('jetlink.server')

# Longer than the client's FRAME_TIMEOUT, so it is the one that gives up.
DRAIN_TIMEOUT = 5.0


def _serve(cache: EngineCache, open_transport, sleeper: Sleeper | None = None,
           host: EngineHost | None = None, control=None) -> None:
  """Serve one client at a time forever.

  `open_transport()` returns a transport, or None to wait and retry; the three
  transports differ only in how they open. The engine host is shared across
  sessions: the comma reconnects at every handover and the engine must not
  reload. With a `sleeper`, a long run of None suspends the box; see sleep.py.

  `host` comes from main() when there is a control channel, which subscribes to
  it before this loop starts; on its own this loop makes and closes its own.
  Link transitions are emitted through the host, once per change rather than
  once per poll: a control client renders "waiting for a comma" from them.
  """
  owns_host = host is None
  if host is None:
    host = EngineHost(cache, pick_source(cache.backend.name),
                      sleep_after=sleeper.after if sleeper is not None else 0.0)
  # before accepting anything, so two callers cannot race to start GPU loads
  host.preload()
  waiting_detail = getattr(open_transport, 'waiting_detail', 'waiting for a client')
  link = ''
  try:
    while True:
      transport = open_transport()
      if transport is None:
        if link != 'waiting':
          link = 'waiting'
          host.emit('link', {'state': 'waiting', 'detail': waiting_detail, 'peer': None})
        if sleeper is not None and sleeper.idle():
          continue  # just woke up; look for the gadget right away
        time.sleep(2.0)
        continue
      if sleeper is not None:
        sleeper.touch()
      link = 'connected'
      host.emit('link', {'state': 'connected', 'detail': '', 'peer': getattr(transport, 'peer', None)})
      session = Session(transport, host)
      detail = ''
      try:
        session.serve_forever()
      except LinkError as e:
        detail = str(e)
        log.info("session ended: %s", e)
      finally:
        session.close()
        if getattr(transport, '_desynced', False):
          # The client is still mid-message. Let it finish and time out rather
          # than reopening under it; see StreamTransport.drain.
          transport.drain(DRAIN_TIMEOUT)
        transport.close()
        if sleeper is not None:
          sleeper.touch()
        link = 'disconnected'
        host.emit('link', {'state': 'disconnected', 'detail': detail, 'peer': None})
        log.info("client disconnected")
  finally:
    if owns_host:
      # Ctrl-C or a stop: release the engine on the thread that owns it rather
      # than leaving it to interpreter teardown, which some runtimes survive
      # less well than others (backends/tinygrad/owner.py). With a host from
      # main(), main() is what closes it, after the control channel.
      host.close()


def _watch_parent(pid: int, interval: float = 1.0) -> None:
  """Stop when the process that launched us is gone.

  The Mac app owns this process; orphaned, it would keep the GPU, the engine
  and the gadget for as long as the machine was up. getppid changing is the
  portable signal, and a SIGINT puts the shutdown where every other stop
  already lands, so there is one way down and it is the tested one.
  """
  while True:
    time.sleep(interval)
    if os.getppid() != pid:
      log.warning("parent process gone, shutting down")
      _interrupt_main()
      return


def _interrupt_main() -> None:
  """Ctrl-C from a thread that is not the main one; see control.interrupt_main."""
  try:
    if sys.platform == 'win32':
      raise OSError('no process-directed SIGINT on Windows')
    os.kill(os.getpid(), signal.SIGINT)
  except (AttributeError, OSError, ValueError):
    _thread.interrupt_main()


def _package_version() -> str:
  try:
    import importlib.metadata
    return importlib.metadata.version('jetlink')
  except Exception:
    return '0.0.0'


class _WaitLog:
  """Say why there is no client once, then keep quiet about it.

  The USB and functionfs openers poll every 2 s, so a box parked offroad
  overnight wrote thousands of identical lines: the log file rolls over and the
  app's Logs view has nothing else in it. The first line is the one that means
  something; the rest go to debug. A different reason gets its own first line,
  and connecting resets the whole thing, so the next disconnect says so again.
  """

  def __init__(self):
    self.last: str | None = None

  def __call__(self, msg: str, *args) -> None:
    log.log(logging.DEBUG if msg == self.last else logging.WARNING, msg, *args)
    self.last = msg

  def reset(self) -> None:
    self.last = None


def _tcp_opener(args):
  from jetlink.transport.tcp import TcpTransport
  srv = TcpTransport.listen(args.host, args.port)
  log.info("listening on %s:%d", args.host, args.port)

  def open_transport():
    transport, addr = TcpTransport.accept(srv)
    log.info("client connected from %s", addr)
    # Who is on the other end, for the control channel's link event.
    transport.peer = f"{addr[0]}:{addr[1]}"
    return transport
  open_transport.waiting_detail = f"listening on {args.host}:{args.port}"
  return open_transport


def _usb_opener(args, sleeper: Sleeper | None = None):
  """This end is the USB host. Needs no kernel driver: libusb uses usbfs."""
  from jetlink.transport.usbbulk import UsbBulkTransport
  waiting = _WaitLog()

  def open_transport():
    if not UsbBulkTransport.present(args.vid, args.pid):
      waiting("waiting for a jetlink gadget at %04x:%04x", args.vid, args.pid)
      return None
    if sleeper is not None:
      # Present but not yet openable still means the comma is there; sleeping
      # now needs it to bounce the gadget before anything wakes the box.
      sleeper.touch()
    try:
      transport = UsbBulkTransport.open(args.vid, args.pid, timeout_ms=args.usb_timeout_ms)
      log.info("client connected over usb")
      waiting.reset()
      transport.peer = 'usb'
      return transport
    except Exception as e:
      # Broad on purpose: this loop is the only supervisor, and anything that
      # escapes it turns a retry into a container crash loop.
      waiting("could not open the gadget: %s", e)
      return None
  open_transport.waiting_detail = f"waiting for a jetlink gadget at {args.vid:04x}:{args.pid:04x}"
  return open_transport


def _ffs_opener(args):
  """This end is the USB gadget."""
  from jetlink.transport.ffs import FfsTransport
  mount = Path(args.ffs_mount)
  waiting = _WaitLog()

  def open_transport():
    if not (mount / 'ep0').exists():
      waiting("waiting for functionfs at %s (run scripts/setup_gadget.sh)", mount)
      return None
    try:
      # This writes the descriptors and binds the UDC; either can fail
      # transiently, and returning None just retries.
      transport = FfsTransport(str(mount), gadget=args.gadget, udc=args.udc)
      waiting.reset()
      transport.peer = 'usb'
      return transport
    except Exception as e:
      waiting("could not open the gadget: %s", e)
      return None
  open_transport.waiting_detail = f"waiting for functionfs at {mount}"
  return open_transport


OPENERS = {'tcp': _tcp_opener, 'usb': _usb_opener, 'ffs': _ffs_opener}


def main(argv=None) -> int:
  p = argparse.ArgumentParser(description='jetlink inference server')
  p.add_argument('--backend', choices=('auto', *NAMES), default='auto',
                 help='what runs the model: trt (TensorRT), ort (onnxruntime: CoreML, CUDA or '
                      'CPU), tinygrad. auto takes the first that comes up, in that order')
  p.add_argument('--device', default='auto',
                 help='backend-specific: a CUDA device index for trt; METAL, CUDA, NV, AMD or '
                      'CPU for tinygrad; coreml (the GPU), ane (every unit, the Neural Engine '
                      'included; measure it first, docs/platforms.md), cuda or cpu for ort')
  p.add_argument('--list-backends', action='store_true',
                 help='print the backends whose runtime is installed here, and exit')
  p.add_argument('--transport', choices=('tcp', 'usb', 'ffs'), default='tcp',
                 help='usb = this end is the USB host (the usual case for a Jetson); '
                      'ffs = this end is the USB gadget')
  p.add_argument('--host', default='0.0.0.0')
  p.add_argument('--port', type=int, default=5599)
  p.add_argument('--ffs-mount', default='/dev/ffs-jetlink')
  p.add_argument('--gadget', default='/sys/kernel/config/usb_gadget/jetlink',
                 help='configfs gadget to bind once descriptors are written')
  p.add_argument('--udc', default=None, help='UDC name (default: the first one)')
  p.add_argument('--vid', type=lambda x: int(x, 0), default=0x1209)
  p.add_argument('--pid', type=lambda x: int(x, 0), default=0x0001)
  p.add_argument('--usb-timeout-ms', type=int, default=2000)
  p.add_argument('--sleep-after', type=float, default=0.0, metavar='SECONDS',
                 help='suspend the box (deep, USB wakes it) after this long with no '
                      f'gadget; 0 = never. In the car use {SLEEP_AFTER:.0f}. '
                      'Needs /sys/power writable in the container.')
  p.add_argument('--cache', default=str(platform.default_cache_dir()),
                 help='engines and uploaded models; JETLINK_CACHE sets the default')
  p.add_argument('--control-socket', default=None, metavar='ADDR',
                 help='open a local control channel: a filesystem path (a unix socket) or '
                      'tcp://127.0.0.1:PORT. The Mac app drives the server through it')
  p.add_argument('--parent-pid', type=int, default=None, metavar='PID',
                 help='exit cleanly once this process is no longer our parent, and lead a '
                      'process group of our own so that parent can reap the workers')
  p.add_argument('--build', metavar='ONNX', help='build an engine and exit')
  p.add_argument('--dump-spec', metavar='ONNX',
                 help='write this model\'s spec as json to stdout and exit')
  p.add_argument('--log-level', default='INFO')
  args = p.parse_args(argv)

  logging.basicConfig(
    level=getattr(logging, args.log_level.upper(), logging.INFO),
    format='%(asctime)s %(levelname)-7s %(name)s: %(message)s')

  try:
    # A stop from launchd, docker or the app arrives as SIGTERM; the clean
    # shutdown already written is the one Ctrl-C takes. SIGINT is set as well
    # rather than inherited: a shell that starts this in the background with &
    # hands the child SIG_IGN, and then nothing (the control channel's shutdown
    # included) can stop it short of SIGTERM.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
  except (ValueError, OSError, AttributeError):
    pass  # not the main thread, or a platform without these signals

  if args.parent_pid:
    # Lead a process group of our own, so the app can take the ORT worker down
    # with one killpg when we die by a signal instead of leaving it orphaned
    # with the engine half built. Not done for a terminal: the shell's job
    # control (Ctrl-C reaching the group) is what keeps a bare run stoppable.
    if hasattr(os, 'setpgrp'):
      try:
        os.setpgrp()
      except OSError:
        pass
    threading.Thread(target=_watch_parent, args=(args.parent_pid,), daemon=True,
                     name='jetlink-parent-watch').start()

  if args.dump_spec:
    import json

    from jetlink.spec import spec_from_onnx
    print(json.dumps(spec_from_onnx(args.dump_spec).to_dict()))
    return 0

  if args.list_backends:
    found = available()
    print('\n'.join(found) if found else 'none: pip install jetlink[trt], jetlink[ort] or jetlink[tinygrad]')
    return 0

  backend = select(args.backend, args.device)
  info = backend.describe()
  log.info("backend %s %s on %s, cache %s", info['backend'], info['runtime_version'],
           info['device'], args.cache)
  cache = EngineCache(Path(args.cache), backend)

  if args.build:
    from jetlink.spec import sha256_file, spec_from_onnx
    sha, nbytes = sha256_file(args.build)
    entry = cache.entry(sha)
    log.info("model %s (%d MB) -> %s", sha[:16], nbytes >> 20, entry.path)
    if entry.exists:
      log.info("already built: %s", entry.meta())
      return 0

    # Per stage, not per build: the CoreML stages each run 0 to 1, and a
    # fraction carried over from the last one would swallow every line of the
    # next until it passed the mark the last one finished at.
    seen = {'stage': None, 'frac': 0.0}

    def report(stage, frac, msg):
      if stage != seen['stage']:
        seen['stage'], seen['frac'] = stage, 0.0
      if frac - seen['frac'] >= 0.02 or frac >= 1.0 or frac == 0.0:
        seen['frac'] = frac
        log.info("%-8s %5.1f%%  %s", stage, frac * 100, msg)

    # Carry the spec into the sidecar like a served build does, so the first
    # client to connect loads the artifact instead of reparsing the ONNX for it.
    spec = spec_from_onnx(args.build)
    backend.build(Path(args.build), entry.path, report=report,
                  meta_extra={'spec': spec.to_dict()})
    log.info("built: %s", entry.meta())
    # A served model is what gets preloaded next time; with none on record, the
    # one just built is the best guess. On a Mac that is the difference between
    # a server that starts its nine-minute CoreML load when launched and one
    # that waits for the comma to ask.
    if cache.last_loaded() is None:
      cache.remember_loaded(sha, spec.frame_skip)
    return 0

  from jetlink.server.power import clear_stale_flag
  clear_stale_flag(cache.root)

  sleeper = None
  if args.sleep_after > 0:
    if args.transport != 'usb':
      # tcp blocks in accept and never sees an absent client; ffs is the
      # Jetson-as-gadget inversion, where the host end is the one that sleeps.
      p.error('--sleep-after only makes sense with --transport usb')
    if not platform.can_suspend():
      # macOS and Windows own their own sleep, and a laptop lid is not a USB
      # edge that wakes anything; the flag is the Jetson's.
      p.error('--sleep-after needs /sys/power, which this host has not got')
    sleeper = Sleeper(args.sleep_after)
    log.info("will suspend after %.0f s without a gadget", args.sleep_after)
  opener = _usb_opener(args, sleeper) if args.transport == 'usb' else OPENERS[args.transport](args)

  # sleep_after goes out in the hello: the comma tells a box that suspends when
  # parked from one that is simply gone.
  host = EngineHost(cache, pick_source(backend.name),
                    sleep_after=sleeper.after if sleeper is not None else 0.0)
  control = None
  if args.control_socket:
    from jetlink.server.control import ControlServer
    # The registry is control.py's to build: it is the only caller, and this
    # module must not pull the network code in when nothing asked for it.
    control = ControlServer(args.control_socket, host, cache, info={
      'version': _package_version(), 'python': python_version(), 'platform': sys.platform,
      'cache': str(cache.root), 'transport': args.transport,
      'port': args.port if args.transport == 'tcp' else None})
    control.start()
  try:
    _serve(cache, opener, sleeper, host, control)
  except KeyboardInterrupt:
    log.info("stopped")
  finally:
    host.close()
    if control is not None:
      control.close()
  return 0


if __name__ == '__main__':
  sys.exit(main())
