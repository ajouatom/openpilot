"""명시적으로 선택한 Konik A1M·NX16 조합의 USB 인터넷을 준비한다. 기본은 계획 출력이다."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import stat
import tempfile
import time

HERE = Path(__file__).resolve().parent
PROFILE = 'konik-a1m-orin-nx16-j4012'
PROFILE_VALUE = {'schema': 1, 'profile': PROFILE, 'operator_confirmed': True}
SOURCE_SHA = {
  'sharing.py': 'e5fb77d061d3d9a3382588f9e2b516c52fbe78ddf9217164c1e2cf27ad4abe67',
  'host_client.py': '38b09876921b015a8ccd46d3f74c9e2f70e2bcb5117ee3ef11b61becbaf68877',
  'boot.py': '9743a2aa7e88e7d389698168ee33b8fcf0bdadc357bd3260987fae00221fdb76',
  'sharing.service': '3c8abdc884ee8a03faa82737734906429ed7aca3b42ce227a0fbd3b9f01a631d',
}
DATA = '/data/carrot-usb-internet'
CONTINUE = '/data/continue.sh'
TRANSPORT = '/data/jetlink-transport'
MARKER = DATA + '/profile.json'
STATE = DATA + '/profile-install'
JOURNAL = STATE + '/journal.json'
LOCK_PARENT = '/run'
LOCK = LOCK_PARENT + '/carrot-konik-nx16-usb-install.lock'
TARGETS = (DATA + '/program.py', DATA + '/runtime.service', CONTINUE, TRANSPORT, MARKER)


class Refused(RuntimeError):
  pass


def need(value, code):
  if not value:
    raise Refused(code)


def digest(raw):
  return hashlib.sha256(raw).hexdigest()


def encode(value):
  return (json.dumps(value, sort_keys=True, indent=2) + '\n').encode()


def load_source(name):
  path = HERE / name
  need(path.is_file() and not path.is_symlink(), 'source_type_changed')
  raw = path.read_bytes()
  need(digest(raw) == SOURCE_SHA[name], 'reviewed_source_changed')
  module = importlib.util.module_from_spec(importlib.util.spec_from_file_location('konik_usb_' + path.stem, path))
  exec(compile(raw, str(path), 'exec'), module.__dict__)
  return module


def target_platform(role, root=Path('/'), *, machine=None, release=None):
  root = Path(root)
  need(role in ('konik', 'jetson'), 'unknown_role')
  need(hasattr(os, 'uname') or machine is not None and release is not None, 'linux_required')
  machine = machine if machine is not None else os.uname().machine
  release = release if release is not None else os.uname().release
  need(machine == 'aarch64', 'arm64_required')
  if role == 'konik':
    need(release.startswith('4.9.'), 'konik_kernel_family_required')
    return {'role': role, 'hardware_identity': 'operator_confirmed_konik_a1m', 'runtime_family_checked': True}
  need(release == '5.15.148-tegra-j401-r3647', 'reviewed_j401_kernel_required')
  compatible = (root / 'proc/device-tree/compatible').read_bytes().decode('ascii').split('\0')
  need(any(re.search(r'(?:^|,)p3767-0000(?:\+|$)', item) for item in compatible), 'orin_nx16_device_tree_required')
  return {'role': role, 'hardware_identity': 'device_tree_declares_p3767_0000',
          'carrier_identity': 'operator_confirmed_recomputer_j4012', 'runtime_family_checked': True}


class KonikInstall:
  """DATA만 고치며 기존과 다른 파일은 지우거나 덮지 않는다."""

  def __init__(self, root=Path('/'), *, owner=0):
    self.root = Path(root)
    self.owner = owner
    self.boot = load_source('boot.py')

  def path(self, name):
    need(name in (*TARGETS, STATE, JOURNAL, LOCK, LOCK_PARENT, DATA)
         or re.fullmatch(re.escape(STATE) + r'/before-[0-4]', name) is not None,
         'path_not_owned')
    return self.root / name.lstrip('/')

  def read(self, name, *, optional=True, identity=False):
    path = self.path(name)
    if not path.exists() and not path.is_symlink():
      need(optional, 'required_file_missing')
      return None, None
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
      info = os.fstat(fd)
      need(stat.S_ISREG(info.st_mode) and info.st_nlink == 1 and 0 <= info.st_size <= 131072, 'unsafe_file_type')
      raw = os.read(fd, 131073)
      after = os.fstat(fd)
      named = path.lstat()
      fields = ('st_dev', 'st_ino', 'st_size', 'st_mtime_ns', 'st_ctime_ns', 'st_uid', 'st_gid', 'st_mode', 'st_nlink')
      need(len(raw) == info.st_size and all(getattr(info, k) == getattr(after, k) for k in fields)
           and (named.st_dev, named.st_ino) == (info.st_dev, info.st_ino), 'file_changed_while_reading')
      fact = {'sha256': digest(raw), 'uid': info.st_uid, 'gid': info.st_gid, 'mode': stat.S_IMODE(info.st_mode)}
      if identity:
        fact.update(device=info.st_dev, inode=info.st_ino, size=info.st_size,
                    mtime_ns=info.st_mtime_ns, ctime_ns=info.st_ctime_ns)
      return raw, fact
    finally:
      os.close(fd)

  def write(self, name, raw, expected, *, mode=0o644, uid=None, gid=None):
    path = self.path(name)
    need(self.read(name)[1] == expected, 'destination_changed')
    need(path.parent.is_dir() and not path.parent.is_symlink(), 'parent_not_directory')
    fd, temporary = tempfile.mkstemp(prefix='.konik-usb-', dir=path.parent)
    try:
      with os.fdopen(fd, 'wb') as stream:
        os.fchmod(stream.fileno(), mode)
        if os.geteuid() == 0:
          os.fchown(stream.fileno(), self.owner if uid is None else uid, 0 if gid is None else gid)
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())
      need(self.read(name)[1] == expected, 'destination_changed_before_replace')
      if expected is None:
        os.link(temporary, path, follow_symlinks=False)
      else:
        os.replace(temporary, path)
      folder = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
      try:
        os.fsync(folder)
      finally:
        os.close(folder)
    finally:
      Path(temporary).unlink(missing_ok=True)
    return self.read(name)[1]

  def safe_base(self, *, create=False):
    base = self.path(DATA)
    if not base.exists():
      need(create and base.parent.is_dir(), 'data_base_missing')
      try:
        base.mkdir(mode=0o755)
      except FileExistsError:
        pass
      else:
        fd = os.open(base, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
          need(os.fstat(fd).st_uid == self.owner, 'data_base_ownership')
          os.fchmod(fd, 0o755)
        finally:
          os.close(fd)
    info = base.lstat()
    need(stat.S_ISDIR(info.st_mode) and info.st_uid == self.owner and stat.S_IMODE(info.st_mode) == 0o755, 'data_base_ownership')

  @contextmanager
  def operation_lock(self, *, create=False):
    """두 명령이 같은 DATA inode와 같은 잠금 파일을 공유한 채 끝까지 직렬화된다."""
    import fcntl
    need(os.geteuid() == self.owner, 'root_required')
    directory = os.open(self.path(LOCK_PARENT), os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    lock_fd = None
    try:
      parent = os.fstat(directory)
      need(stat.S_ISDIR(parent.st_mode) and parent.st_uid == self.owner
           and stat.S_IMODE(parent.st_mode) == 0o755, 'lock_parent_ownership')
      lock_fd = os.open(Path(LOCK).name, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC,
                        0o600, dir_fd=directory)
      info = os.fstat(lock_fd)
      need(stat.S_ISREG(info.st_mode) and info.st_uid == self.owner and info.st_nlink == 1 and info.st_size == 0
           and stat.S_IMODE(info.st_mode) == 0o600, 'operation_lock_ownership')
      deadline = time.monotonic() + 8
      while True:
        try:
          fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
          break
        except BlockingIOError:
          need(time.monotonic() < deadline, 'operation_lock_timeout')
          time.sleep(0.02)

      self.safe_base(create=create)
      data = self.path(DATA).lstat()

      def unchanged():
        named_parent = self.path(LOCK_PARENT).lstat()
        named_data = self.path(DATA).lstat()
        named = os.stat(Path(LOCK).name, dir_fd=directory, follow_symlinks=False)
        current = os.fstat(lock_fd)
        need((named_parent.st_dev, named_parent.st_ino) == (parent.st_dev, parent.st_ino)
             and named_parent.st_uid == self.owner and stat.S_IMODE(named_parent.st_mode) == 0o755,
             'operation_lock_parent_changed')
        need((named_data.st_dev, named_data.st_ino) == (data.st_dev, data.st_ino)
             and stat.S_ISDIR(named_data.st_mode) and named_data.st_uid == self.owner
             and stat.S_IMODE(named_data.st_mode) == 0o755, 'operation_data_parent_changed')
        need(stat.S_ISREG(current.st_mode) and current.st_uid == self.owner and current.st_nlink == 1 and current.st_size == 0
             and stat.S_IMODE(current.st_mode) == 0o600
             and (named.st_dev, named.st_ino) == (info.st_dev, info.st_ino)
             and (current.st_dev, current.st_ino) == (info.st_dev, info.st_ino), 'operation_lock_identity_changed')

      unchanged()
      yield unchanged
    finally:
      if lock_fd is not None:
        os.close(lock_fd)
      os.close(directory)

  def payloads(self):
    program = (HERE / 'sharing.py').read_bytes()
    template = (HERE / 'sharing.service').read_bytes()
    need(digest(program) == SOURCE_SHA['sharing.py'] and digest(template) == SOURCE_SHA['sharing.service'], 'payload_changed')
    unit = self.boot.make_unit(template, digest(program))
    original, _ = self.read(CONTINUE, optional=False)
    transport, _ = self.read(TRANSPORT)
    need(transport is None or transport.strip() in (b'usb', b'auto'), 'transport_mode_not_reviewed')
    return {TARGETS[0]: program, TARGETS[1]: unit,
            CONTINUE: self.boot.make_continue(original, digest(unit), digest(program)),
            TRANSPORT: b'auto\n', MARKER: encode(PROFILE_VALUE)}

  def plan(self):
    values = self.payloads()
    for name in (TARGETS[0], TARGETS[1], MARKER):
      current, fact = self.read(name)
      need(current is None or current == values[name] and fact['uid'] == self.owner and fact['mode'] == 0o644,
           'existing_owned_destination_differs')
    return values

  def install(self):
    with self.operation_lock(create=True) as lock_check:
      return self._install_locked(lock_check)

  def _install_locked(self, lock_check):
    need(os.geteuid() == self.owner, 'root_required')
    values = self.plan()
    self.safe_base(create=True)
    state = self.path(STATE)
    if state.exists():
      info = state.lstat()
      need(stat.S_ISDIR(info.st_mode) and info.st_uid == self.owner and stat.S_IMODE(info.st_mode) == 0o700, 'state_directory_ownership')
      journal_raw, journal_fact = self.read(JOURNAL, optional=False)
      need(journal_fact['uid'] == self.owner and journal_fact['mode'] == 0o600, 'journal_ownership')
      journal = json.loads(journal_raw)
      need(journal.get('phase') == 'installed' and journal.get('profile') == PROFILE, 'incomplete_installation_preserved')
      need(all(self.read(name)[1] == journal['entries'][name]['after'] for name in TARGETS), 'installed_files_changed')
      lock_check()
      return {'status': 'already_installed', 'operating_files_changed': 0, 'reboot_or_restart_requested': False}
    state.mkdir(mode=0o700)
    state.chmod(0o700)
    entries = {}
    for index, name in enumerate(TARGETS):
      before, fact = self.read(name)
      backup = STATE + '/before-' + str(index)
      if before is not None:
        self.write(backup, before, None, mode=0o600)
      entries[name] = {'before': fact, 'backup': backup if before is not None else None, 'after': None,
                       'desired': {'sha256': digest(values[name]), 'uid': fact['uid'] if fact else self.owner,
                                   'gid': fact['gid'] if fact else 0, 'mode': fact['mode'] if fact else 0o644}}
    journal = {'schema': 1, 'profile': PROFILE, 'phase': 'installing', 'entries': entries}
    self.write(JOURNAL, encode(journal), None, mode=0o600)
    changed = 0
    new_marker_identity = None
    try:
      for name, raw in values.items():
        fact = entries[name]['before']
        if fact and fact['sha256'] == digest(raw):
          after = fact
        else:
          after = self.write(name, raw, fact, uid=fact['uid'] if fact else self.owner,
                             gid=fact['gid'] if fact else 0, mode=fact['mode'] if fact else 0o644)
          changed += 1
        entries[name]['after'] = after
        if name == MARKER and entries[name]['before'] is None:
          new_marker_identity = self.read(MARKER, identity=True)[1]
        self.write(JOURNAL, encode(journal), self.read(JOURNAL)[1], mode=0o600)
      lock_check()
      need(all(self.read(name)[1] == entries[name]['after'] for name in TARGETS), 'installed_postimage_changed')
      journal['phase'] = 'installed'
      self.write(JOURNAL, encode(journal), self.read(JOURNAL)[1], mode=0o600)
    except BaseException:
      # 미완료 설치가 새 선택 마커를 켜 둔 채 남지 않게 본인 inode만 거둔다.
      if new_marker_identity is not None and self.read(MARKER, identity=True)[1] == new_marker_identity:
        self.path(MARKER).unlink()
        entries[MARKER]['after'] = None
      journal['phase'] = 'incomplete_preserved'
      self.write(JOURNAL, encode(journal), self.read(JOURNAL)[1], mode=0o600)
      raise
    return {'status': 'installed_next_normal_start', 'operating_files_changed': changed,
            'rootfs_remounted': False, 'reboot_or_restart_requested': False, 'model_or_vehicle_settings_changed': False}

  def rollback(self, runner=None):
    with self.operation_lock() as lock_check:
      return self._rollback_locked(lock_check, runner)

  def _rollback_locked(self, lock_check, runner=None):
    self.safe_base()
    info = self.path(STATE).lstat()
    need(stat.S_ISDIR(info.st_mode) and info.st_uid == self.owner and stat.S_IMODE(info.st_mode) == 0o700, 'state_directory_ownership')
    journal_raw, fact = self.read(JOURNAL, optional=False)
    need(fact['uid'] == self.owner and fact['mode'] == 0o600, 'journal_ownership')
    journal = json.loads(journal_raw)
    need(journal.get('profile') == PROFILE and journal.get('phase') in ('installed', 'installing', 'incomplete_preserved'),
         'rollback_phase_invalid')
    entries = journal['entries']
    need(set(entries) == set(TARGETS), 'rollback_targets_changed')
    current_identity = {name: self.read(name, identity=True)[1] for name in TARGETS}
    current = {name: ({key: value[key] for key in ('sha256', 'uid', 'gid', 'mode')} if value is not None else None)
               for name, value in current_identity.items()}
    for name, item in entries.items():
      allowed = (item['after'],) if journal['phase'] == 'installed' else (
        item['before'], item['desired'], *((item['after'],) if item['after'] is not None else ()))
      need(current[name] in allowed, 'rollback_foreign_changes_preserved')
    backups = {}
    for name, item in entries.items():
      if item['before'] is None or item['before'] == current[name]:
        continue
      raw, backup = self.read(item['backup'], optional=False, identity=True)
      need(digest(raw) == item['before']['sha256'] and backup['mode'] == 0o600
           and backup['uid'] == self.owner, 'backup_changed')
      backups[name] = (raw, backup)

    def snapshots_unchanged():
      lock_check()
      need(self.read(JOURNAL)[1] == fact, 'rollback_journal_changed')
      need(all(self.read(name, identity=True)[1] == current_identity[name] for name in TARGETS), 'rollback_destination_changed')
      need(all(self.read(entries[name]['backup'], optional=False, identity=True)[1] == value[1]
               for name, value in backups.items()), 'rollback_backup_changed')

    snapshots_unchanged()
    runtime_stopped = False
    if current[TARGETS[1]] is not None and entries[TARGETS[1]]['before'] != current[TARGETS[1]]:
      result = self.boot.rollback(current[TARGETS[1]]['sha256'], root=self.root, owner=self.owner, runner=runner)
      runtime_stopped = result['status'] == 'runtime_removed'
    for name, item in reversed(list(entries.items())):
      snapshots_unchanged()
      if item['before'] == current[name]:
        continue
      if item['before'] is None:
        self.path(name).unlink()
        current[name] = None
        current_identity[name] = None
      else:
        raw, _backup = backups[name]
        before = item['before']
        current[name] = self.write(name, raw, current[name], mode=before['mode'], uid=before['uid'], gid=before['gid'])
        current_identity[name] = self.read(name, identity=True)[1]
    snapshots_unchanged()
    journal['phase_before_rollback'] = journal['phase']
    journal['phase'] = 'rolled_back'
    self.write(JOURNAL, encode(journal), self.read(JOURNAL)[1], mode=0o600)
    return {'status': 'owned_data_files_restored', 'runtime_service_stopped': runtime_stopped,
            'preexisting_runtime_preserved': entries[TARGETS[1]]['before'] == entries[TARGETS[1]]['after'],
            'backups_preserved': True}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('role', choices=('konik', 'jetson'))
  parser.add_argument('--profile', choices=(PROFILE,), required=True)
  parser.add_argument('--apply', action='store_true')
  parser.add_argument('--rollback', action='store_true')
  parser.add_argument('--confirm-hardware-match', action='store_true')
  parser.add_argument('--confirm-offroad', action='store_true')
  args = parser.parse_args()
  try:
    hardware = target_platform(args.role)
    if not args.apply:
      print(json.dumps({'status': 'plan_only', 'hardware': hardware, 'profile': PROFILE,
                        'apply_requires_hardware_and_offroad_confirmation': True, 'system_changed': False}))
      return 0
    need(args.confirm_hardware_match and args.confirm_offroad, 'explicit_hardware_and_stationary_confirmation_required')
    need(os.geteuid() == 0, 'root_required')
    os.umask(0o077)
    if args.role == 'konik':
      installer = KonikInstall()
      result = installer.rollback() if args.rollback else installer.install()
    else:
      host = load_source('host_client.py')
      with host.lock():
        client = host.Client(release=os.uname().release)
        result = client.rollback() if args.rollback else client.install(HERE / 'host_client.py', SOURCE_SHA['host_client.py'])
    print(json.dumps(result, sort_keys=True))
    return 0
  except (OSError, RuntimeError, ValueError) as error:
    print(json.dumps({'status': 'stopped_preserved', 'code': str(error) if isinstance(error, Refused) else type(error).__name__}))
    return 2


if __name__ == '__main__':
  raise SystemExit(main())
