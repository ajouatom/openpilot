"""명시적으로 허용한 Konik 기본 ADB gadget만 정차 상태에서 인계한다."""
import argparse
import fcntl
import os
from pathlib import Path
import stat
import sys

CONTROLLER = 'a600000.dwc3'
FLAG = '/data/jetlink-adb-handoff'


def read_small(path, limit=128):
  fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
  try:
    if not stat.S_ISREG(os.fstat(fd).st_mode):
      raise ValueError('nonregular attribute')
    data = os.read(fd, limit + 1)
    if len(data) > limit:
      raise ValueError('oversized attribute')
    return data.decode('ascii').strip()
  finally:
    os.close(fd)


class AdbHandoff:
  def __init__(self, root=Path('/')):
    # 별도 root는 합성 검사에서만 주입하며 CLI에는 경로 재지정 옵션이 없다.
    self.root = Path(root)
    self.gadgets = self.path('/sys/kernel/config/usb_gadget')
    self.reason = ''
    self.stage = 'permission'

  def deny(self, code, detail):
    self.reason = f'[adb-handoff:{code}] {detail}'
    return False

  def failed_io(self, exc):
    return self.deny('io_error', f'{self.stage}: {type(exc).__name__}, errno={getattr(exc, "errno", None)}')

  def path(self, name):
    return self.root / name.lstrip('/')

  def permitted(self, controller):
    self.reason = ''
    try:
      self.stage = 'opt-in marker'
      try:
        flag = self.path(FLAG).lstat()
      except FileNotFoundError:
        return self.deny('opt_in_missing', f'ADB handoff permission is missing ({FLAG})')
      if controller != CONTROLLER:
        return self.deny('controller', f'Unsupported controller {ascii(controller[:48])}')
      if (not stat.S_ISREG(flag.st_mode) or flag.st_nlink != 1 or
          flag.st_uid != os.geteuid() or flag.st_mode & 0o022):
        return self.deny('opt_in_invalid', f'Unsafe permission marker: uid={flag.st_uid}, mode={stat.S_IMODE(flag.st_mode):04o}, links={flag.st_nlink}')
      self.stage = 'platform'
      if not self.path('/TICI').is_file():
        return self.deny('platform', 'Required /TICI platform marker is missing')
      self.stage = 'road state'
      offroad = read_small(self.path('/data/params/d/IsOffroad'))
      onroad = read_small(self.path('/data/params/d/IsOnroad'))
      if offroad != '1' or onroad != '0':
        return self.deny('offroad', f'Offroad required: IsOffroad={ascii(offroad[:16])}, IsOnroad={ascii(onroad[:16])}')
      self.stage = 'USB role'
      role = read_small(self.path('/sys/class/usbpd/usbpd0/current_dr'))
      if role != 'ufp':
        return self.deny('usb_role', f'USB device role required: current_dr={ascii(role[:24])}')
      adb = self.gadgets / 'g1'
      self.stage = 'g1 identity'
      if adb.is_symlink():
        return self.deny('gadget_identity', 'g1 is a symbolic link')
      vendor, product = read_small(adb / 'idVendor').lower(), read_small(adb / 'idProduct').lower()
      if vendor not in ('0x04d8', '04d8') or product not in ('0x1234', '1234'):
        return self.deny('gadget_identity', f'Expected ADB 04d8:1234, got {ascii(vendor[:16])}/{ascii(product[:16])}')
      self.stage = 'g1 functions'
      functions = list((adb / 'functions').iterdir())
      if ({p.name for p in functions} != {'ffs.adb', 'ncm.0'} or
          any(p.is_symlink() or not p.is_dir() for p in functions)):
        names = ','.join(sorted(p.name for p in functions))[:64]
        return self.deny('gadget_functions', f'Expected ffs.adb+ncm.0 directories, got {ascii(names)}')
      self.stage = 'gadget owners'
      entries = list(self.gadgets.iterdir())
      if len(entries) > 16:
        return self.deny('gadget_tree', f'Too many gadget entries ({len(entries)})')
      for entry in entries:
        if entry.is_symlink() or not entry.is_dir():
          return self.deny('gadget_tree', f'Unsafe gadget entry {ascii(entry.name[:48])}')
        self.stage = f'UDC {ascii(entry.name[:32])}'
        bound = read_small(entry / 'UDC')
        if entry.name == 'g1':
          if bound != controller:
            return self.deny('controller_changed', f'g1 controller changed to {ascii(bound[:48])}')
        elif bound:
          return self.deny('other_owner', f'Another gadget is bound: {ascii(entry.name[:32])} -> {ascii(bound[:48])}')
      return True
    except (OSError, ValueError) as exc:
      return self.failed_io(exc)

  def release(self, controller):
    if not self.permitted(controller):
      return False
    self.stage = 'handoff lock'
    lock = os.open(self.path('/dev/shm/jetlink-adb-handoff.lock'),
                   os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
      info = os.fstat(lock)
      if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or info.st_nlink != 1:
        return self.deny('lock_invalid', 'Handoff lock has unsafe ownership or file type')
      try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
      except BlockingIOError:
        return self.deny('lock_busy', 'Another ADB handoff is already running')
      self.stage = 'open g1 UDC'
      fd = os.open(self.gadgets / 'g1/UDC', os.O_WRONLY | os.O_NOFOLLOW)
      try:
        # 확인과 sysfs 쓰기는 여러 파일에 걸쳐 원자적이지 않다. 직전에 다시 검사한다.
        if not stat.S_ISREG(os.fstat(fd).st_mode):
          return self.deny('udc_attribute', 'g1 UDC is not a regular configfs attribute')
        if not self.permitted(controller):
          self.reason += ' (pre-write recheck)'
          return False
        self.stage = 'unbind g1 UDC'
        if os.write(fd, b'\n') != 1:
          return self.deny('write_failed', 'g1 UDC unbind write was incomplete')
      finally:
        os.close(fd)
      self.stage = 'verify g1 UDC'
      if read_small(self.gadgets / 'g1/UDC'):
        return self.deny('still_bound', 'g1 UDC remains bound after the unbind write')
      return True
    finally:
      os.close(lock)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--udc', required=True)
  args = parser.parse_args()
  handoff = AdbHandoff()
  if os.geteuid() != 0:
    handoff.deny('not_root', 'ADB handoff helper requires root')
  else:
    try:
      if handoff.release(args.udc):
        return 0
    except (OSError, ValueError) as exc:
      handoff.failed_io(exc)
  print(handoff.reason, file=sys.stderr)
  return 1


if __name__ == '__main__':
  raise SystemExit(main())
