"""CANable 2.0 SLCAN transport using the verified Carrot firmware."""
import os
from pathlib import Path
import select
import subprocess
import sys
import time

PRODUCTS = (('16d0', '117e'), ('ad50', '60c4'))
REMOTE = b'github.com/normaldotcom/canable2.git'
FIRMWARE = b'carrot-b1-slcan-2 '


def identity(port):
  path = Path(port).resolve(strict=True)
  if path.parent != Path('/dev') or not path.name.startswith('ttyACM') or not path.name[6:].isdigit():
    raise ValueError('Expected a CANable USB serial port')
  device = (Path('/sys/class/tty') / path.name / 'device').resolve(strict=True)
  if (device/'driver').resolve().name != 'cdc_acm':
    raise ValueError('Expected the CANable CDC driver')
  for parent in device.parents:
    if (parent/'idVendor').is_file():
      product = tuple((parent/name).read_text().strip() for name in ('idVendor', 'idProduct'))
      if product not in PRODUCTS:
        break
      return str(device), (parent/'serial').read_text().strip(), path.stat().st_rdev
  raise ValueError('Expected a supported CANable 2.0 USB identity')


def candidates():
  found = []
  for path in Path('/dev/serial/by-id').glob('*'):
    try:
      identity(str(path))
      found.append(str(path))
    except (OSError, ValueError):
      continue
  return found


def frame(line):
  # Ignore unrelated standard, extended and CAN-FD traffic. Only the two
  # classic eight-byte messages may update the controller's CAN guard.
  if not line.startswith(b't') or len(line) < 5:
    return None
  address = int(line[1:4], 16)
  if address not in (0x3c1, 0x413):
    return None
  if line[4:5] != b'8' or len(line) != 21:
    raise ValueError('Unexpected B1 SLCAN frame length')
  return address, bytes.fromhex(line[5:].decode('ascii'))


WATCHDOG = '''import os,select,sys,termios
fd=int(sys.argv[1])
while select.select([0],[],[],.75)[0]:
 if not os.read(0,256):break
try:
 termios.tcflush(fd,termios.TCOFLUSH)
 os.write(fd,b'C\\r')
except OSError:pass
os.close(fd)
'''


class Bus:
  def __init__(self, port):
    import fcntl
    import tty
    from blink_protocol import CONFIG
    self.iface, self.index = port, identity(port)
    self.latest, self.buffer = {}, bytearray()
    self.sock, self.watch, self.transmit = None, None, False
    self.version, self.version_count = b'', 0
    self.report, self.report_count = None, 0
    self.next_lease = 0.
    self.lock = (CONFIG.parent/'blinker.lock').open('a')
    try:
      fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
      self.sock = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
      fcntl.flock(self.sock, fcntl.LOCK_EX | fcntl.LOCK_NB)
      tty.setraw(self.sock)
      self.watch = subprocess.Popen([sys.executable, '-c', WATCHDOG, str(self.sock)], pass_fds=(self.sock,),
                                    stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
      self._command(b'C\rV\r')
      if self.version != FIRMWARE+REMOTE:
        raise ValueError('CANable requires carrot-b1-slcan-2 firmware with verified mode and arbitration/error reporting')
      self.mode(False)
    except BaseException:
      self.close()
      raise

  def _write(self, data):
    if os.write(self.sock, data) != len(data):
      raise OSError('Incomplete CANable USB write')

  def _command(self, data, report=False, timeout=.2):
    previous = self.report_count if report else self.version_count
    self._write(data)
    until = time.monotonic()+timeout
    # This firmware acknowledges no individual commands. Its V reply is a
    # processing barrier, not a CAN transmission acknowledgement.
    while (self.report_count if report else self.version_count) == previous:
      remaining = until-time.monotonic()
      if remaining <= 0:
        raise TimeoutError('CANable command processing timeout')
      select.select([self.sock], [], [], min(.002, remaining))
      self.receive()

  def mode(self, transmit):
    if identity(self.iface) != self.index:
      raise ValueError('CANable adapter changed')
    # S6=500k, M1=silent, A0=disable automatic retransmission. The mode
    # commands are issued while closed as required by the CANable firmware.
    self.transmit = False
    self._command(b'C\rS6\rA0\r'+(b'M0\r' if transmit else b'M1\r')+b'O\rQ\r', report=True)
    if self.report[0] != (b'T' if transmit else b'L'):
      raise ValueError('CANable hardware mode did not match the bounded transmission contract')
    self.transmit = transmit

  def receive(self):
    for _ in range(16):
      try:
        chunk = os.read(self.sock, 4096)
      except BlockingIOError:
        break
      if not chunk:
        raise OSError('CANable disconnected')
      self.buffer.extend(chunk)
      if len(self.buffer) > 131072:
        raise ValueError('CANable receive overflow')
      while b'\r' in self.buffer:
        line, _, tail = self.buffer.partition(b'\r')
        self.buffer = bytearray(tail)
        if b'\x07' in line:
          raise ValueError('CANable rejected a command')
        if line.startswith(b'Q'):
          if len(line) != 31 or line[1:2] not in (b'C', b'L', b'T', b'E') or line[2:3] not in (b'0', b'1', b'2', b'3', b'4'):
            raise ValueError('Invalid CANable hardware status')
          self.report = (bytes(line[1:2]), int(line[2:3]), int(line[3:7], 16))
          self.diagnostics = tuple(int(line[i:i+8], 16) for i in (7, 15, 23))
          self.report_count += 1
        elif line.endswith(REMOTE):
          self.version, self.version_count = bytes(line), self.version_count+1
        else:
          parsed = frame(line)
          if parsed:
            self.latest[parsed[0]] = (time.monotonic(), parsed[1])
    now = time.monotonic()
    if now >= self.next_lease:
      self.watch.stdin.write(b'1')
      self.watch.stdin.flush()
      self.next_lease = now+.1

  def neutral(self, now):
    from blink_can import Bus as SocketBus
    return SocketBus.neutral(self, now)

  def send(self, data):
    if not self.transmit or len(data) != 8:
      raise ValueError('CANable transmission is not armed')
    deadline = time.monotonic()+.015
    # FDCAN DAR cancels a request after lost arbitration too. Re-submit only
    # a confirmed unsuccessful request, never an unknown/acknowledged one.
    # All attempts share one 15ms deadline, below the smallest schedule gap.
    for attempt in range(8):
      expected = (self.report[2]+1) & 0xffff
      remaining = deadline-time.monotonic()
      if remaining <= 0:
        raise TimeoutError('CANable CAN transmission completion timeout')
      self._command(b't3C18'+data.hex().upper().encode('ascii')+b'\rQ\r', report=True, timeout=remaining)
      while True:
        mode, state, sequence = self.report
        if mode != b'T' or sequence != expected:
          raise OSError('CANable unexpected TX status: %r expected=%d' % (self.report, expected))
        if state == 2:
          return
        if state == 3:
          raise OSError('CANable CAN protocol error: %r registers=%r' % (self.report, getattr(self, 'diagnostics', None)))
        if state == 4:
          # Only an explicit cancellation with no CAN protocol error may
          # retry. A missing result, USB timeout or bus error never does.
          time.sleep(min(max(0., deadline-time.monotonic()), .0006+(time.monotonic_ns()%901)/1e6))
          break
        remaining = deadline-time.monotonic()
        if remaining <= 0:
          raise TimeoutError('CANable CAN transmission completion timeout')
        self._command(b'Q\r', report=True, timeout=remaining)
    raise OSError('CANable arbitration budget exhausted: %r' % (self.report,))

  def close(self):
    if self.watch is not None:
      self.watch.stdin.close()
      self.watch.wait(timeout=3)
      self.watch = None
    if self.sock is not None:
      os.close(self.sock)
      self.sock = None
    self.lock.close()
