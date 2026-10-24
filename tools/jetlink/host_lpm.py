"""Optional USB3 port power lease; kernel writes never run in inference.

The root child owns a pipe, a bounded activity lease and a recovery journal.
Only the exact claimed 1209:0001 device's port can be changed. Model bytes,
runtime autosuspend, other ports and global USB quirks are untouched.
"""
import json
import logging
import os
from pathlib import Path
import select
import signal
import stat
import subprocess
import sys
import time

log = logging.getLogger('carrot.jetlink.lpm')
PERMITS = {'0', 'u1', 'u2', 'u1_u2'}
QUIET_SECONDS = 30.


def find_device(bus, address, root=Path('/sys')):
  if not (type(bus) is int and type(address) is int and 1 <= bus <= 999 and 1 <= address <= 127):
    raise ValueError('invalid USB identity')
  matches = []
  for device in (root / 'bus/usb/devices').glob('*'):
    try:
      if ((device / 'idVendor').read_text().strip(), (device / 'idProduct').read_text().strip()) != ('1209', '0001'):
        continue
      if (int((device / 'busnum').read_text()), int((device / 'devnum').read_text())) != (bus, address):
        continue
      if float((device / 'speed').read_text()) < 5000:
        raise ValueError('not a USB3 link')
      permit = (device / 'port').resolve(strict=True) / 'usb3_lpm_permit'
      validate_port(permit, root)
      states = [(device / 'power' / ('usb3_hardware_lpm_' + n)).read_text().strip() for n in ('u1', 'u2')]
      if any(s not in ('enabled', 'disabled') for s in states):
        raise ValueError('unknown USB3 power state')
      matches.append((device.resolve(), permit))
    except (FileNotFoundError, NotADirectoryError):
      continue
  if len(matches) != 1:
    raise ValueError('exact claimed USB device unavailable')
  return matches[0]


def validate_port(permit, root):
  resolved = permit.resolve(strict=True)
  if resolved != permit or permit.name != 'usb3_lpm_permit' or not permit.is_relative_to((root / 'devices').resolve()):
    raise ValueError('invalid USB port path')
  if permit.read_text().strip() not in PERMITS:
    raise ValueError('unknown USB3 permit')


class PortLease:
  def __init__(self, journal, boot, *, root=Path('/sys'), write=None):
    self.journal, self.boot, self.root = journal, boot, root
    self.write = write or (lambda path, value: path.write_text(value + '\n'))
    self.owned = None

  def recover(self):
    if not self.journal.exists():
      return
    data = json.loads(self.journal.read_text())
    if (set(data) != {'boot', 'permit', 'original'} or data['original'] not in PERMITS
        or not isinstance(data['boot'], str) or not isinstance(data['permit'], str)):
      raise ValueError('invalid LPM recovery journal')
    if data['boot'] == self.boot:
      path = Path(data['permit'])
      if path.exists():
        validate_port(path, self.root)
        self.owned = data
        self.restore()
        return
    self.journal.unlink()

  def activate(self, device, permit):
    if self.owned is not None:
      return
    validate_port(permit, self.root)
    original = permit.read_text().strip()
    # Preserve an existing owner's setting. Never assume 'stock' is u1_u2.
    if original == '0':
      return
    self.owned = {'boot': self.boot, 'permit': str(permit), 'original': original}
    temporary = self.journal.with_suffix('.tmp')
    temporary.write_text(json.dumps(self.owned))
    temporary.replace(self.journal)
    try:
      self.write(permit, '0')
      states = [(device / 'power' / ('usb3_hardware_lpm_' + n)).read_text().strip() for n in ('u1', 'u2')]
      if permit.read_text().strip() != '0' or states != ['disabled', 'disabled']:
        raise RuntimeError('USB3 LPM disable readback failed')
    except BaseException:
      self.restore()
      raise

  def restore(self):
    if self.owned is None:
      return
    path = Path(self.owned['permit'])
    if path.exists():
      validate_port(path, self.root)
      if path.read_text().strip() == '0':
        self.write(path, self.owned['original'])
        if path.read_text().strip() != self.owned['original']:
          raise RuntimeError('USB3 LPM restore readback failed')
      # An externally changed nonzero permit is no longer ours to overwrite.
    self.journal.unlink(missing_ok=True)
    self.owned = None


def serve(lease, locate, owner_fd, *, clock=time.monotonic, wait=select.select, read=os.read, stopped=lambda: False):
  device, permit = locate()
  identity = device.stat().st_ino
  last_heard = None
  try:
    while not stopped():
      ready, _, _ = wait([owner_fd], [], [], 1.)
      if ready:
        data = read(owner_fd, 4096)
        if not data:
          break
        if data.strip(b'1'):
          raise ValueError('invalid LPM activity token')
        last_heard = clock()
      if locate() != (device, permit) or device.stat().st_ino != identity:
        raise RuntimeError('claimed USB device changed')
      if last_heard is not None and clock() - last_heard < QUIET_SECONDS:
        lease.activate(device, permit)
      else:
        lease.restore()
  finally:
    lease.restore()


def run(bus, address, owner_fd=0):
  import fcntl
  if os.geteuid() != 0 or not Path('/etc/nv_tegra_release').is_file():
    raise RuntimeError('Jetson root helper required')
  if not stat.S_ISFIFO(os.fstat(owner_fd).st_mode):
    raise ValueError('owner-lifetime pipe required')
  os.umask(0o077)
  os.nice(10)
  folder = Path('/run/carrot-jetlink-lpm')
  folder.mkdir(mode=0o700, exist_ok=True)
  info = folder.lstat()
  if not stat.S_ISDIR(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o077:
    raise ValueError('unsafe LPM journal directory')
  with (folder / 'lock').open('w') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    lease = PortLease(folder / 'lease.json', Path('/proc/sys/kernel/random/boot_id').read_text().strip())
    stopping = False

    def stop(*_):
      nonlocal stopping
      stopping = True

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
      signal.signal(sig, stop)
    lease.recover()
    serve(lease, lambda: find_device(bus, address), owner_fd, stopped=lambda: stopping)


class SessionLease:
  def __init__(self, transport):
    self.process = None
    self.next_touch = 0.
    self.failed = False
    if (sys.platform != 'linux' or os.environ.get('CARROT_USB_LPM_OFF', '1') != '1'
        or not Path('/etc/nv_tegra_release').is_file()):
      return
    from jetlink.transport.usbbulk import UsbBulkTransport
    if not isinstance(transport, UsbBulkTransport):
      return
    try:
      device = transport.handle.getDevice()
      command = [sys.executable, str(Path(__file__).resolve()), str(device.getBusNumber()), str(device.getDeviceAddress())]
      if os.geteuid() != 0:
        command = ['sudo', '-n', *command]
      self.process = subprocess.Popen(command, stdin=subprocess.PIPE, bufsize=0, start_new_session=True)
      os.set_blocking(self.process.stdin.fileno(), False)
    except Exception:
      self.close()
      log.exception('USB3 power lease unavailable; existing power policy retained')

  def touch(self):
    if self.process is None or self.failed:
      return
    now = time.monotonic()
    if now < self.next_touch:
      return
    self.next_touch = now + 1.
    try:
      os.write(self.process.stdin.fileno(), b'1')
    except BlockingIOError:
      pass
    except OSError:
      self.failed = True
      self.process.stdin.close()
      log.warning('USB3 power lease ended; existing power policy retained')

  def close(self):
    if self.process is not None:
      process, self.process = self.process, None
      process.stdin.close()
      try:
        if process.wait(timeout=3) != 0:
          log.warning('USB3 power lease helper failed; inspect recovery journal')
      except subprocess.TimeoutExpired:
        # Never kill the restorer while a kernel control transfer owns it.
        log.warning('USB3 power restoration still pending; helper retains ownership')


if __name__ == '__main__':
  try:
    run(int(sys.argv[1]), int(sys.argv[2]))
  except Exception as exc:
    print(f'Jetlink USB3 power lease: {exc}', file=sys.stderr, flush=True)
    sys.exit(1)
