"""운영자가 확인한 Konik A1M·Orin NX 16GB 조합만 선택한다."""
import json
import os
from pathlib import Path
import stat

PROFILE = 'konik-a1m-orin-nx16-j4012'
DIRECTORY = Path('/data/carrot-usb-internet')
MARKER = 'profile.json'
VALUE = {'schema': 1, 'profile': PROFILE, 'operator_confirmed': True}


def enabled(root=Path('/'), *, owner=0, machine=None, release=None):
  """기본값은 꺼짐이며 파일 이름만으로 장치 종류를 추정하지 않는다."""
  if os.name != 'posix':
    return False
  machine = machine if machine is not None else os.uname().machine
  release = release if release is not None else os.uname().release
  if machine != 'aarch64' or not isinstance(release, str) or not release.startswith('4.9.'):
    return False
  directory = Path(root) / str(DIRECTORY).lstrip('/')
  directory_fd = file_fd = None
  try:
    directory_fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    before = os.fstat(directory_fd)
    if before.st_uid != owner or stat.S_IMODE(before.st_mode) != 0o755:
      return False
    file_fd = os.open(MARKER, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory_fd)
    info = os.fstat(file_fd)
    if not (stat.S_ISREG(info.st_mode) and info.st_uid == owner and info.st_nlink == 1
            and stat.S_IMODE(info.st_mode) == 0o644 and 0 < info.st_size <= 512):
      return False
    raw = os.read(file_fd, 513)
    after = os.fstat(file_fd)
    named = os.stat(MARKER, dir_fd=directory_fd, follow_symlinks=False)
    fields = ('st_dev', 'st_ino', 'st_size', 'st_mtime_ns', 'st_ctime_ns', 'st_mode', 'st_uid', 'st_nlink')
    if len(raw) != info.st_size or any(getattr(info, key) != getattr(after, key) for key in fields):
      return False
    if (named.st_dev, named.st_ino) != (info.st_dev, info.st_ino):
      return False
    current = directory.lstat()
    if (current.st_dev, current.st_ino) != (before.st_dev, before.st_ino):
      return False
    value = json.loads(raw)
    return (isinstance(value, dict) and set(value) == set(VALUE) and type(value['schema']) is int
            and value['schema'] == 1 and value['profile'] == PROFILE and value['operator_confirmed'] is True)
  except (OSError, ValueError, TypeError):
    return False
  finally:
    if file_fd is not None:
      os.close(file_fd)
    if directory_fd is not None:
      os.close(directory_fd)
