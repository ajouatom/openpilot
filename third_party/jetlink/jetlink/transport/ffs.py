"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

USB gadget side of the link: a FunctionFS vendor-specific bulk function.

On a comma this is the comma end. The roles look backwards and are not: AGNOS
has CONFIG_USB_F_FS built in, while a host needs no kernel driver at all, which
an L4T rootfs stripped of the gadget modules does not have. See
docs/transport.md.

Bring-up (scripts/setup_gadget.sh does all of this):
    configfs gadget -> functions/ffs.jetlink -> mount -t functionfs
    write descriptors + strings to ep0 -> ep1 (OUT) and ep2 (IN) appear
    echo <udc> > UDC
"""
from __future__ import annotations

import errno
import logging
import os
import signal
import struct
import threading
import time
from collections import deque

from jetlink import protocol as P
from jetlink.transport.base import LinkError, LinkTimeout, StreamTransport, take
from jetlink.transport.priority import background_thread, widen_affinity
from jetlink.transport.watchdog import WriteWatchdog

# --- FunctionFS ABI -------------------------------------------------------

FUNCTIONFS_DESCRIPTORS_MAGIC_V2 = 3
FUNCTIONFS_STRINGS_MAGIC = 2

FLAG_HAS_FS = 1 << 0
FLAG_HAS_HS = 1 << 1
FLAG_HAS_SS = 1 << 2

USB_DT_INTERFACE = 0x04
USB_DT_ENDPOINT = 0x05
USB_DT_SS_ENDPOINT_COMP = 0x30

USB_ENDPOINT_XFER_BULK = 0x02
EP_OUT = 0x01  # host -> device
EP_IN = 0x82   # device -> host

SS_MAX_PACKET = 1024
# FunctionFS kmallocs a contiguous buffer per read: order-5 and order-6 failures
# in ffs_epfile_read_iter stalled replies 100-300 ms, and order-2 does not fail.
# Do not go smaller to dodge the rare slow success: one page per read is ~19
# syscalls for a 74 KB reply and cost ~6 ms a frame.
READ_CHUNK = 16 * SS_MAX_PACKET
# How much the reader may queue before it stops. Only bounds memory if the
# consumer stalls: inference never needs more than one response.
MAX_QUEUED = 8 << 20
# Read buffers kept for reuse. Steady state has one or two in flight, so this is
# only a ceiling for a transient backlog. 16 * 16 KB = 256 KB.
FREE_BUFS = 16
# Just below modeld's frame loop (54), so the loop wins a contended core and the
# reader still preempts everything else. See _raise_reader_priority.
READER_RT_PRIORITY = 51

log = logging.getLogger('jetlink')

# Enumerated is not ready: the host still has to open the device and claim the
# interface, and until then the endpoints return EIO. Its sysfs poll makes that
# a second or two, so wait rather than fail the first frame after connect.
EP_READY_TIMEOUT = 10.0
# How long opening the endpoint files waits for a host to configure the gadget;
# EP_READY_TIMEOUT covers the claim that follows.
EP_OPEN_TIMEOUT = 10.0

# FunctionFS writes cannot time out: the request sits queued until the host
# drains it, measured at 90 s while a Jetson booted and three minutes on a
# drive. Unbinding the UDC dequeues it, the writev returns ESHUTDOWN, and the
# caller retries. A real write is ~3.6 ms.
WRITE_TIMEOUT = 15.0
UDC_SYSFS = '/sys/class/udc'
# How long close() waits for the reader thread after unbinding, which is what
# wakes it. A read the kernel will not complete is left to die with the process.
READER_JOIN_TIMEOUT = 1.0
# How long a rebind leaves the gadget off the bus. Long enough that the host
# sees a disconnect and not a glitch; a dwc3 pull-down is immediate.
REBIND_SETTLE = 0.5
_NOT_READY = (errno.EIO, errno.ESHUTDOWN, errno.ENODEV)

# A signal inside a FunctionFS transfer is not a retry: ffs_epfile_io returns
# EINTR mid-transfer, os.writev re-issues the whole call, and the host sees the
# start of the message twice. modeld's msgq raises SIGUSR2 ~160 times a second,
# which cost one link failure per 30 s of driving. Nothing here waits on one.
_IO_SIGNALS = signal.valid_signals()


def _interface_desc(n_endpoints: int = 2, i_interface: int = 1) -> bytes:
  return struct.pack('<BBBBBBBBB', 9, USB_DT_INTERFACE, 0, 0, n_endpoints,
                     0xFF, 0xFF, 0xFF, i_interface)


def _endpoint_desc(addr: int, max_packet: int) -> bytes:
  return struct.pack('<BBBBHB', 7, USB_DT_ENDPOINT, addr, USB_ENDPOINT_XFER_BULK, max_packet, 0)


def _ss_companion(max_burst: int = 15) -> bytes:
  """SuperSpeed endpoint companion.

  bMaxBurst counts *additional* packets, so 15 is a 16 KB burst. Without
  bursting a 459 KB request takes 167 ms, over the whole frame budget.
  """
  return struct.pack('<BBBBH', 6, USB_DT_SS_ENDPOINT_COMP, max_burst, 0, 0)


def build_descriptors() -> bytes:
  fs = _interface_desc() + _endpoint_desc(EP_OUT, 64) + _endpoint_desc(EP_IN, 64)
  hs = _interface_desc() + _endpoint_desc(EP_OUT, 512) + _endpoint_desc(EP_IN, 512)
  ss = (_interface_desc()
        + _endpoint_desc(EP_OUT, SS_MAX_PACKET) + _ss_companion()
        + _endpoint_desc(EP_IN, SS_MAX_PACKET) + _ss_companion())

  # struct usb_functionfs_descs_head_v2: magic, length, flags, then ALL the
  # per-speed counts together (one __le32 per flag set, in flag order), and only
  # then the descriptor blocks. Interleaving count/block per speed gets EINVAL.
  body = struct.pack('<III', 3, 3, 5) + fs + hs + ss
  flags = FLAG_HAS_FS | FLAG_HAS_HS | FLAG_HAS_SS
  return struct.pack('<III', FUNCTIONFS_DESCRIPTORS_MAGIC_V2, 12 + len(body), flags) + body


def build_strings(name: str = 'jetlink') -> bytes:
  s = name.encode() + b'\0'
  return (struct.pack('<IIII', FUNCTIONFS_STRINGS_MAGIC, 16 + 2 + len(s), 1, 1)
          + struct.pack('<H', 0x0409) + s)


class FfsTransport(StreamTransport):
  """The gadget transport. Reads run on their own thread.

  FunctionFS ignores O_NONBLOCK once the host has enabled the endpoint, so a
  read on the caller's thread cannot honour a deadline: a 0.5 s timeout was
  measured returning after 14 s. The reader hands whole chunks over a condition
  variable the caller can wait on. Writes stay on the caller's thread; a host
  that is not reading is a dead link either way.
  """
  read_chunk = READ_CHUNK
  # dwc3 resends a TRB about once in 400 frames. A message that fits one writev
  # is replayed whole and the receiver drops it by seq; split across two writes
  # the replay lands mid-stream and the link is lost. The largest inference
  # request is 475 KB padded, and only uploads, which sha256 covers, are bigger.
  write_chunk = 512 * SS_MAX_PACKET
  tx_align = P.GADGET_TX_ALIGN

  def __init__(self, mount: str = '/dev/ffs-jetlink', gadget: str | None = None,
               udc: str | None = None):
    self._prepare(mount, gadget)
    try:
      self.ep0 = os.open(os.path.join(mount, 'ep0'), os.O_RDWR)
      os.write(self.ep0, build_descriptors())
      os.write(self.ep0, build_strings())
      if gadget is not None:
        # Bind last: a FunctionFS gadget cannot attach until its descriptors
        # are written, which is why setup_gadget.sh leaves UDC empty.
        self.bind(udc)
      # ep1/ep2 exist now, but opening one before a host enables it costs the
      # gadget until the next reboot; see _ensure_epfiles.
    except BaseException:
      self.close()   # otherwise a failed bring-up leaks the descriptors it did open
      raise

  @classmethod
  def borrowed(cls, mount: str, udc: str, bounce=None, owner_gadget: str | None = None) -> FfsTransport:
    """A transport over a gadget another process owns.

    That process holds ep0 and the UDC bind for its whole life, so the gadget
    does not leave the bus when the link changes hands: this end only opens the
    endpoint files and moves bytes, and nothing here writes descriptors, binds
    or unbinds. The endpoint files take a second open happily; ep0 does not,
    which is why the owner keeps it.

    The ep0 rule is unchanged and `udc` is what keeps it: the endpoints are
    still opened only once that controller reads configured. `bounce` is how a
    write with no reader is freed, since the unbind that frees it belongs to
    the owner. `owner_gadget` is the last resort for when the owner does not
    answer at all; see _abort_write.
    """
    if not udc:
      # _configured() would answer "no controller of ours to ask, assume
      # ready" and _ensure_epfiles would open an endpoint no host has enabled,
      # which costs the gadget until the comma is rebooted.
      raise LinkError('a borrowed gadget must name its controller')
    t = cls.__new__(cls)
    t._prepare(mount, gadget=None)
    t.bound_udc = udc
    t._bounce = bounce
    t._owner_gadget = owner_gadget
    return t

  def _prepare(self, mount: str, gadget: str | None) -> None:
    # This end only receives replies: an INFER_RESP is ~74 KB with telemetry and
    # padding, and the upload goes the other way. 256 KB is a 3x margin the
    # memory-tight comma can spare, and RxBuffer grows past it on demand.
    super().__init__(rx_size=256 << 10)
    self.mount = mount
    self.gadget = gadget
    self.bound_udc: str | None = None
    self._bounce = None
    self._owner_gadget: str | None = None
    self.ep0 = self.ep_out = self.ep_in = -1
    self._state_fd = -1   # held-open UDC 'state' fd; see _udc_state
    self._ready_deadline: float | None = None
    self._read_size = READ_CHUNK
    self._cv = threading.Condition()
    self._chunks: deque[tuple[memoryview, float, float, float]] = deque()
    self._free: deque[bytearray] = deque()   # read buffers the consumer handed back; see _read_loop
    self.last_receive = {'prepare': 0.0, 'read_wait': 0.0, 'handoff': 0.0}
    self._queued = 0
    self._reader_error: str | None = None
    self._closing = False
    self._had_host = False
    self._open_lock = threading.Lock()
    self._write_aborted = False
    self._reader: threading.Thread | None = None
    self._write_guard = WriteWatchdog(self._abort_write)

  def _udc_state(self) -> str | None:
    """The controller's gadget state, off a held-open fd.

    The reader checks this before every readv and cannot skip it (see
    _read_loop). Opening the sysfs file each time allocates, and under memory
    pressure that reclaim stalled a frame 25 ms; a pread at offset 0 still
    re-runs the attribute's show(), so the value is current. pread, not lseek
    and read: the reader and the frame thread both ask during the claim
    window, and two lseek+read pairs interleaved on one fd hand one of them
    an empty read, which _host_gone would take for a host that left. The fd
    is dropped on unbind and reopened lazily in case the controller differs.
    """
    if self.bound_udc is None:
      return None
    if self._state_fd < 0:
      try:
        self._state_fd = os.open(os.path.join(UDC_SYSFS, self.bound_udc, 'state'), os.O_RDONLY)
      except OSError:
        return None
    try:
      state = os.pread(self._state_fd, 64, 0).decode().strip()
    except OSError:
      _close_quietly(self._state_fd)
      self._state_fd = -1
      return None
    return state or None

  def _configured(self) -> bool:
    """Has a host set our configuration? Only then are the endpoints enabled."""
    if self.bound_udc is None:
      return self.gadget is None   # no controller of ours to ask; assume ready
    return self._udc_state() == 'configured'

  def _udc_note(self) -> str:
    """The controller's state, for a failure message.

    The same ENODEV comes from a cable falling out ("not attached"), from a
    host resetting or re-enumerating us ("default", "addressed") and from a
    bus the host suspended ("suspended"). A drive's worth of failures read
    the same without it; with it the log says which layer let go.
    """
    state = self._udc_state()
    return f" (udc: {state})" if state else ''

  def _ensure_epfiles(self) -> None:
    """Open ep1/ep2 and start the reader, once a host has enabled them.

    Never touch an endpoint file before that. ffs_epfile_io sleeps in
    wait_event_interruptible until epfile->ep is set, and unbind does not wake
    it; the sleeping syscall holds the struct file, so ffs_epfile_release never
    runs and every ep0 open after it answers EBUSY, to this process and the next
    one, until the comma is rebooted. The UDC reading configured is the same
    edge that sets epfile->ep.
    """
    if self.ep_out >= 0:
      return
    with self._open_lock:
      if self.ep_out >= 0:
        return
      deadline = time.monotonic() + EP_OPEN_TIMEOUT
      while not self._configured():
        if self._closing:
          raise LinkError("gadget closing")
        if time.monotonic() >= deadline:
          raise LinkTimeout(f"no host configured the gadget within {EP_OPEN_TIMEOUT:.0f}s")
        time.sleep(0.02)
      self.ep_out = os.open(os.path.join(self.mount, 'ep1'), os.O_RDWR)
      self.ep_in = os.open(os.path.join(self.mount, 'ep2'), os.O_RDWR)
      self._had_host = True
      self._reader = threading.Thread(target=self._read_loop, name='jetlink-ffs-read', daemon=True)
      self._reader.start()

  def release_endpoints(self) -> bool:
    """Put the endpoint files down, keeping ep0 and the descriptors.

    An exchange leaves a read queued on the endpoint, and FunctionFS keeps it
    queued until something completes it: nobody else may read that endpoint
    until it does, and the only thing that dequeues it is the unbind. So the
    owner gives the endpoints up like this and binds straight back - one
    re-enumeration - without ever letting go of ep0, which is the whole point,
    since the gadget exists only while somebody holds it.

    False when there is nothing to give up. Afterwards this transport is bare
    and bound, and using it again reopens the endpoints as any first use does.
    """
    if self.gadget is None or self._closing or self.ep_out < 0:
      return False
    udc, self._had_host = self.bound_udc, False
    self.unbind()          # completes the queued read with ESHUTDOWN
    reader, self._reader = self._reader, None
    if reader is not None and reader is not threading.current_thread():
      reader.join(READER_JOIN_TIMEOUT)
    self._close_fds(('ep_in', 'ep_out'), reader)
    # Whatever the last exchange left behind goes with the endpoints: half a
    # message would frame the next one's first reply as garbage.
    with self._cv:
      self._chunks.clear()
      self._free.clear()
      self._queued = 0
      self._reader_error = None
      self._cv.notify_all()
    self.rx.start = self.rx.end = 0
    self._ready_deadline = None
    self._write_aborted = False
    time.sleep(REBIND_SETTLE)
    self.bind(udc)
    return True

  def bind(self, udc: str | None = None) -> None:
    if self.gadget is None:
      raise LinkError("no gadget path given")
    if udc is None:
      udcs = sorted(os.listdir(UDC_SYSFS))
      if not udcs:
        raise LinkError("no USB device controller found")
      udc = udcs[0]
    with open(os.path.join(self.gadget, 'UDC'), 'w') as f:
      f.write(udc + '\n')
    self.bound_udc = udc

  @property
  def lendable(self) -> bool:
    """Bound to a controller, with no endpoint file open on this end.

    The state another process can take the endpoints over from. FunctionFS
    keeps a queued read queued until something completes it, so a reader here
    would sit in front of the borrower and take its reply; and without a bound
    controller there is nothing to take over.
    """
    return bool(self.gadget is not None and self.bound_udc and self.ep_out < 0
                and not self._closing)

  def rebind(self) -> bool:
    """One unplug and replug, as the host sees it.

    For a host that answered our bind with a bus reset and then stopped: the
    UDC sits in `default` or `addressed` and only another edge moves it. A
    Jetson whose hubs are not armed for remote wakeup does exactly that.

    Refused while this end has the endpoint files open. Unbinding then
    completes the reader's request with ESHUTDOWN and the transport is done;
    the caller wants a working link, not a freshly bound dead one.
    """
    if self._closing or self.ep_out >= 0:
      return False
    if self._bounce is not None:
      return bool(self._bounce())   # a borrowed gadget: the bind is the owner's
    if not self.lendable:
      return False
    udc = self.bound_udc
    self.unbind()
    time.sleep(REBIND_SETTLE)
    self.bind(udc)
    return True

  def unbind(self, gadget: str | None = None) -> None:
    """Take the gadget off the bus. `gadget` overrides our own path, which is
    how a borrower reaches an owner that has stopped answering."""
    gadget = gadget or self.gadget
    if gadget is None or self.bound_udc is None:
      return
    try:
      with open(os.path.join(gadget, 'UDC'), 'w') as f:
        f.write('\n')
    except OSError:
      pass
    self.bound_udc = None
    if self._state_fd >= 0:   # reopens lazily against whatever udc we bind next
      _close_quietly(self._state_fd)
      self._state_fd = -1

  def _shrink(self, attr: str) -> bool:
    """Halve a transfer size after the kernel refused to allocate for one.

    ENOMEM here is contiguous memory, so it tracks fragmentation: a size that
    worked at boot can fail an hour in, which is what a 1.7 GB model does to a
    comma. Slow beats broken, and both directions need it.
    """
    floor = (self.packet_size or SS_MAX_PACKET) * 16
    current = getattr(self, attr)
    if current <= floor:
      return False
    setattr(self, attr, max(floor, current // 2))
    if attr == '_read_size':
      self._free.clear()   # pooled buffers are the old, larger size now
    log.warning("jetlink: gadget could not allocate for %s, dropping to %d KB",
                attr, getattr(self, attr) >> 10)
    return True

  def _shrink_write(self) -> bool:
    return self._shrink('write_chunk')

  def _abort_write(self) -> None:
    """Take the link down so a write nobody is reading returns. See WRITE_TIMEOUT.

    On the watchdog thread: the one stuck in writev cannot act.
    """
    self._write_aborted = True
    log.warning("jetlink: no reader for %.3f s, dropping the gadget to free the write",
                getattr(self, '_write_budget', WRITE_TIMEOUT))
    if self._bounce is None:
      return self.unbind()
    # A borrowed gadget: the unbind that dequeues this belongs to whoever owns
    # ep0, so ask. If nobody answers - the owner died, or is wedged itself -
    # take the gadget down from here anyway. This runs while a frame thread is
    # inside a writev the kernel will never return from on its own, and a
    # re-enumeration costs one rejoin where a wedged modeld costs the whole
    # drive, the small model included.
    if self._bounce():
      return
    log.error("jetlink: the gadget's owner did not answer; dropping the link from here")
    self.unbind(self._owner_gadget)

  def send(self, *args, **kwargs) -> None:
    # Reset the quantum at each message: a latched shrink would split every
    # later request across two writes and re-arm the dwc3 replay (see
    # write_chunk). The pressure that forced it is usually gone by the next
    # frame.
    self.write_chunk = type(self).write_chunk
    super().send(*args, **kwargs)

  def _write(self, bufs: list[memoryview]) -> int:
    self._ensure_epfiles()
    self._write_budget = self._write_timeout(WRITE_TIMEOUT)
    # See _IO_SIGNALS: a signal here duplicates data on the wire.
    was = signal.pthread_sigmask(signal.SIG_BLOCK, _IO_SIGNALS)
    try:
      if not self._write_guard.arm(self._write_budget):
        raise LinkError('gadget write watchdog already expired or closed')
      while True:
        try:
          n = os.writev(self.ep_in, bufs)
          self._had_host = True
          return n
        except OSError as e:
          if self._write_aborted:
            # Say whose doing it was: ENODEV alone reads like a cable falling
            # out of a socket.
            raise LinkError(f"gadget write had no reader for {self._write_budget:.3f}s") from e
          # FunctionFS submits a write as one request, so a failed writev put
          # nothing on the wire and is safe to retry.
          if e.errno in _NOT_READY and self._wait_for_host_ready():
            continue
          if e.errno == errno.ENOMEM and self._shrink_write():
            # A short write is fine: send() loops until the message is out.
            bufs = take(bufs, self.write_chunk)
            continue
          raise LinkError(f"gadget write failed: {e}{self._udc_note()}") from e
    finally:
      completed = self._write_guard.disarm()
      signal.pthread_sigmask(signal.SIG_SETMASK, was)
      if not completed:
        raise LinkError('gadget write had no reader before deadline; link abandoned')

  # -- the reader thread ---------------------------------------------------

  def _widen_affinity(self) -> None:
    """Move the reader off the core modeld pinned its frame loop to.

    A thread created under config_realtime_process(7, 54) inherits SCHED_FIFO 54
    and the single-core pin, and sharing that core serialises the reader with
    the frame loop. Usually a no-op: the creator is normally a background thread
    already. See priority.widen_affinity.
    """
    widen_affinity()

  def _raise_reader_priority(self) -> None:
    """Realtime priority, so a completed read leaves the endpoint at once.

    The creator has dropped to SCHED_OTHER, and under recording plus onroad load
    the wait to be scheduled after a transfer was 17 ms mean, 23 ms max - over
    budget on read_wait alone. The reader copies a chunk and blocks again, so it
    never holds a core. Best effort, and open-coded because jetlink must not
    import openpilot.
    """
    try:
      os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(READER_RT_PRIORITY))
    except (OSError, AttributeError, ValueError):
      pass

  def _read_loop(self) -> None:
    # As in _write: an interrupted read drops the packets it already took. This
    # thread handles no signals, so mask them for its whole life.
    signal.pthread_sigmask(signal.SIG_BLOCK, _IO_SIGNALS)
    self._widen_affinity()
    self._raise_reader_priority()
    # Fill the pool up front: recycling alone leaves a gap while a reply's
    # chunks arrive and the consumer is a scheduling beat behind, and an
    # allocation there reclaims under memory pressure (24 ms, over budget).
    self._free.extend(bytearray(self._read_size) for _ in range(FREE_BUFS))
    while not self._closing:
      with self._cv:
        while self._queued >= MAX_QUEUED and not self._closing:
          self._cv.wait(0.1)
      if self._closing:
        return
      prepare_started = time.monotonic()
      if not self._configured():
        # Same trap as _ensure_epfiles: the host can drop the configuration
        # between two reads, and the readv after that sleeps forever. Treat it
        # as the endpoint error it would have been.
        if self._wait_for_host_ready():
          continue
        self._fail(f"host dropped the gadget configuration{self._udc_note()}")
        return
      # Reuse a buffer the consumer handed back: a fresh bytearray here was
      # measured stalling 20+ ms mid-frame under memory pressure, and the read
      # cannot start until it returns. Only this thread pops the free list.
      buf = self._free.popleft() if self._free else bytearray(self._read_size)
      read_started = time.monotonic()
      try:
        # Multiples of the packet size only: the OUT endpoint rejects anything
        # else, and _read_size is only ever halved from one.
        got = os.readv(self.ep_out, [buf])
      except OSError as e:
        if self._closing:
          return
        if e.errno in _NOT_READY and self._wait_for_host_ready():
          continue
        if e.errno == errno.ENOMEM and self._shrink('_read_size'):
          continue
        self._fail(f"gadget read failed: {e}{self._udc_note()}")
        return
      if got == 0:
        self._fail(f"gadget read returned EOF (host disconnected){self._udc_note()}")
        return
      read_finished = time.monotonic()
      self._ready_deadline = None
      self._had_host = True
      with self._cv:
        self._chunks.append((memoryview(buf)[:got], read_finished,
                             read_started - prepare_started, read_finished - read_started))
        self._queued += got
        self._cv.notify_all()

  def _fail(self, why: str) -> None:
    with self._cv:
      self._reader_error = why
      self._cv.notify_all()

  def recv(self, timeout: float | None = None):
    # Per-message maxima, in seconds. read_wait covers the peer and kernel IO,
    # handoff the consumer's wake; neither is a pure scheduler measurement.
    self.last_receive = {'prepare': 0.0, 'read_wait': 0.0, 'handoff': 0.0}
    return super().recv(timeout)

  def _read_into(self, dest: memoryview, timeout: float | None) -> int:
    self._ensure_epfiles()
    with self._cv:
      if not self._chunks and self._reader_error is None:
        self._cv.wait(timeout)
      if self._chunks:
        chunk, arrived, prepare, read_wait = self._chunks[0]
        self.last_receive['prepare'] = max(self.last_receive['prepare'], prepare)
        self.last_receive['read_wait'] = max(self.last_receive['read_wait'], read_wait)
        self.last_receive['handoff'] = max(self.last_receive['handoff'], time.monotonic() - arrived)
        n = min(chunk.nbytes, dest.nbytes)
        dest[:n] = chunk[:n]
        if n < chunk.nbytes:
          self._chunks[0] = (chunk[n:], arrived, prepare, read_wait)
        else:
          self._chunks.popleft()
          # Copied out, so return the buffer to the pool. chunk.obj survives a
          # reslice; skip anything not from this pool at the current size, such
          # as a shrunk buffer or a test's bytes.
          buf = chunk.obj
          if type(buf) is bytearray and len(buf) == self._read_size and len(self._free) < FREE_BUFS:
            self._free.append(buf)
        self._queued -= n
        self._cv.notify_all()
        return n
      if self._reader_error is not None:
        raise LinkError(self._reader_error)
      return 0   # timed out with nothing new; _fill owns the deadline

  def _wait_for_host_ready(self) -> bool:
    """True while still inside the grace period for the host to claim the link.

    The endpoints return EIO until the host claims the interface. The clock
    starts on the first such error, not at open, so it covers a re-enumeration.
    """
    if self._host_gone():
      return False
    now = time.monotonic()
    if self._ready_deadline is None:
      self._ready_deadline = now + EP_READY_TIMEOUT
    if now < self._ready_deadline:
      time.sleep(0.005)
      return True
    return False

  def _host_gone(self) -> bool:
    """A host had the gadget configured and no longer does.

    The endpoints answer ENODEV both before a host configures the gadget and
    after it disconnects, and only the first deserves the grace period; giving
    it to the second cost 10 s and 201 frames on a mid-drive disconnect. The UDC
    state separates them, and it is set in the interrupt that disabled the
    endpoint, so it is current when the failed call returns.
    """
    if not self._had_host or self.bound_udc is None:
      return False
    state = self._udc_state()
    return state is not None and state != 'configured'

  def close(self) -> None:
    self._closing = True
    self._write_guard.close()
    self._write_guard.thread.join(READER_JOIN_TIMEOUT)
    if self._write_guard.thread.is_alive():
      # It is stuck inside its own abort, which is already unbinding the
      # gadget. This used to raise, to keep a late abort from unbinding a
      # gadget the next owner had bound - but the raise came before a single
      # fd was closed, so ep0 stayed open for the life of the process and
      # every descriptor write after it answered ESRCH. unbind() is
      # idempotent, so a late abort finds nothing left to let go of; the leak
      # was the worse half by far.
      log.warning("jetlink: gadget watchdog teardown did not finish in %.0f s, "
                  "closing the endpoints anyway", READER_JOIN_TIMEOUT)
    # Unbinding disables the endpoints, which completes the reader's pending
    # request with ESHUTDOWN and lets the thread exit before its fd goes away.
    self.unbind()
    reader, self._reader = self._reader, None
    if reader is not None and reader is not threading.current_thread():
      reader.join(READER_JOIN_TIMEOUT)
    with self._cv:
      self._cv.notify_all()
    self._close_fds(('_state_fd', 'ep_in', 'ep_out', 'ep0'), reader)

  def _close_fds(self, names, reader: threading.Thread | None) -> None:
    """Close these descriptors, clearing each as it goes.

    Clearing matters because a second close() would otherwise shut whatever
    those descriptor numbers had been recycled into. ep_out is closed on its
    own thread while a read is still outstanding: the read never came back
    (nothing to unbind, or a kernel that will not complete it), and close()
    returns regardless on Linux but the caller must not be risked on that.
    """
    for name in names:
      fd = getattr(self, name, -1)
      setattr(self, name, -1)
      if fd is None or fd < 0:
        continue
      if name == 'ep_out' and reader is not None and reader.is_alive():
        threading.Thread(target=_close_quietly, args=(fd,), daemon=True).start()
        continue
      _close_quietly(fd)


def _close_quietly(fd: int) -> None:
  # Runs on its own thread when a read never came back; that thread inherits
  # the creator's scheduling, which in modeld is the frame loop's.
  background_thread()
  try:
    os.close(fd)
  except OSError:
    pass
