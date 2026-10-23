"""명시적으로 허용한 Konik 기본 ADB gadget만 정차 상태에서 인계한다."""
import argparse
import fcntl
import os
from pathlib import Path
import stat

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

  def path(self, name):
    return self.root / name.lstrip('/')

  def permitted(self, controller):
    try:
      flag = self.path(FLAG).lstat()
      if (controller != CONTROLLER or not stat.S_ISREG(flag.st_mode) or flag.st_nlink != 1 or
          flag.st_uid != os.geteuid() or flag.st_mode & 0o022 or
          not self.path('/TICI').is_file()):
        return False
      if (read_small(self.path('/data/params/d/IsOffroad')) != '1' or
          read_small(self.path('/data/params/d/IsOnroad')) != '0' or
          read_small(self.path('/sys/class/usbpd/usbpd0/current_dr')) != 'ufp'):
        return False
      adb = self.gadgets / 'g1'
      if (adb.is_symlink() or
          read_small(adb / 'idVendor').lower() not in ('0x04d8', '04d8') or
          read_small(adb / 'idProduct').lower() not in ('0x1234', '1234')):
        return False
      functions = list((adb / 'functions').iterdir())
      if ({p.name for p in functions} != {'ffs.adb', 'ncm.0'} or
          any(p.is_symlink() or not p.is_dir() for p in functions)):
        return False
      entries = list(self.gadgets.iterdir())
      if len(entries) > 16:
        return False
      for entry in entries:
        if entry.is_symlink() or not entry.is_dir():
          return False
        bound = read_small(entry / 'UDC')
        if entry.name == 'g1':
          if bound != controller:
            return False
        elif bound:
          return False
      return True
    except (OSError, ValueError):
      return False

  def release(self, controller):
    if not self.permitted(controller):
      return False
    lock = os.open(self.path('/dev/shm/jetlink-adb-handoff.lock'),
                   os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
      info = os.fstat(lock)
      if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or info.st_nlink != 1:
        return False
      try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
      except BlockingIOError:
        return False
      fd = os.open(self.gadgets / 'g1/UDC', os.O_WRONLY | os.O_NOFOLLOW)
      try:
        # 확인과 sysfs 쓰기는 여러 파일에 걸쳐 원자적이지 않다. 직전에 다시 검사한다.
        if not stat.S_ISREG(os.fstat(fd).st_mode) or not self.permitted(controller):
          return False
        if os.write(fd, b'\n') != 1:
          return False
      finally:
        os.close(fd)
      return not read_small(self.gadgets / 'g1/UDC')
    finally:
      os.close(lock)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--udc', required=True)
  args = parser.parse_args()
  if os.geteuid() != 0:
    return 1
  try:
    return 0 if AdbHandoff().release(args.udc) else 1
  except (OSError, ValueError):
    return 1


if __name__ == '__main__':
  raise SystemExit(main())
