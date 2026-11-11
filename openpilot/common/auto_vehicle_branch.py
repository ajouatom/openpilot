"""Startup-only, two-branch handoff. No vehicle control or forced Git reset."""

from __future__ import annotations

import os
import re
from pathlib import Path
import subprocess
import time
from urllib.parse import urlsplit

from openpilot.common.auto_vehicle import PORTER, IONIQ5
from openpilot.common.repo_update import child_lock_kwargs, repo_lock

BRANCH_SWITCH_PROTOCOL = 1
TARGET_BRANCHES = {PORTER: 'c3-pht-porter', IONIQ5: 'c3-pht'}
MODULE_PATH = 'openpilot/common/auto_vehicle_branch.py'
STILL_SECONDS = 1.0
FRESH_SECONDS = 0.3
REQUEST_SECONDS = 0.8


class HandoffError(RuntimeError):
  pass


def boot_id():
  return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def session_required(params):
  return params.get_bool('AutoVehicleSwitch') or bool(params.get('AutoVehicleBranchHandoff'))


def set_status(params, text):
  if params.get('AutoVehicleBranchStatus') != text:
    params.put('AutoVehicleBranchStatus', text)


class ParkGuard:
  """Require fresh Park and four exactly-zero wheels, including valid counters."""

  def __init__(self, vehicle):
    if vehicle not in TARGET_BRANCHES:
      raise HandoffError('unsupported vehicle')
    self.vehicle = vehicle
    self.seen = {}
    self.counters = {}
    self.safe_since = None

  def update(self, frames, now):
    wheel_addr, gear_addr = (0x386, 0x372) if self.vehicle == PORTER else (0xA0, 0x35)
    for msg in frames:
      if msg.src != 0 or msg.address not in (wheel_addr, gear_addr):
        continue
      data = bytes(msg.dat)
      expected = 8 if self.vehicle == PORTER else (24 if msg.address == wheel_addr else 32)
      valid = len(data) == expected
      counter = None
      if valid and self.vehicle == IONIQ5:
        from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum

        valid = int.from_bytes(data[:2], 'little') == hkg_can_fd_checksum(msg.address, None, bytearray(data))
        counter = data[2]
      elif valid and msg.address == wheel_addr:
        checksum = ((data[5] >> 6) & 3) | (((data[7] >> 6) & 3) << 2)
        valid = checksum == ((sum((b & 0x3F if i % 2 else b).bit_count() for i, b in enumerate(data)) ^ 9) & 15)
        counter = ((data[1] >> 6) & 3) | (((data[3] >> 6) & 3) << 2)
      if valid and counter is not None:
        previous = self.counters.get(msg.address)
        modulo = 16 if self.vehicle == PORTER else 256
        valid = previous is None or (counter - previous) % modulo in (1, 2)
        self.counters[msg.address] = counter
      if valid:
        if msg.address == wheel_addr:
          offsets = (0, 2, 4, 6) if self.vehicle == PORTER else (8, 10, 12, 14)
          mask = 0x3FFF if self.vehicle == PORTER else 0xFFFF
          valid = all((int.from_bytes(data[i : i + 2], 'little') & mask) == 0 for i in offsets)
        else:
          valid = ((data[2] & 15) == 0) if self.vehicle == PORTER else ((data[24] & 7) == 0 and data[5] == 0)
      self.seen[msg.address] = (now, valid)
      if not valid:
        self.safe_since = None
    safe = all(address in self.seen and self.seen[address][1] and 0 <= now - self.seen[address][0] <= FRESH_SECONDS for address in (wheel_addr, gear_addr))
    if not safe:
      self.safe_since = None
    elif self.safe_since is None:
      self.safe_since = now
    return safe and now - self.safe_since >= STILL_SECONDS


def persistent_keys(header):
  return {match[1] for line in header.splitlines() if (match := re.match(r'\s*\{"([^"]+)",\s*\{(.*)', line)) and re.search(r'\bPERSISTENT\b', match[2])}


class Repository:
  def __init__(self, root):
    self.root = Path(root)
    self.module = (self.root / MODULE_PATH).read_bytes()
    self.persistent = persistent_keys((self.root / 'openpilot/common/params_keys.h').read_text())

  def git(self, *args, timeout=10):
    env = dict(os.environ, GIT_TERMINAL_PROMPT='0')
    result = subprocess.run(['git', *args], cwd=self.root, env=env, capture_output=True, timeout=timeout, **child_lock_kwargs())
    if result.returncode:
      # Git output can contain remote credentials; expose only a fixed diagnosis.
      raise HandoffError(f'git {args[0]} failed ({result.returncode})')
    return result.stdout.decode().strip()

  def branch(self):
    branch = self.git('symbolic-ref', '--quiet', '--short', 'HEAD')
    if branch not in TARGET_BRANCHES.values():
      raise HandoffError('current branch is outside the two-vehicle pair')
    return branch

  def clean(self):
    if self.git('status', '--porcelain', '--untracked-files=no'):
      raise HandoffError('tracked changes or conflicts: preserve and resolve them manually')

  def compatible(self, ref):
    try:
      same = self.git('show', f'{ref}:{MODULE_PATH}').encode() == self.module.rstrip(b'\n')
      target_keys = persistent_keys(self.git('show', f'{ref}:openpilot/common/params_keys.h'))
      return same and self.persistent <= target_keys
    except HandoffError:
      return False

  def target(self, branch):
    # Prefer installed compatible branches, permitting offline round trips.
    try:
      local = self.git('rev-parse', '--verify', f'refs/heads/{branch}')
    except HandoffError:
      local = None
    if f'branch refs/heads/{branch}' in self.git('worktree', 'list', '--porcelain').splitlines():
      raise HandoffError('target branch is checked out in another worktree')
    if local is not None and self.compatible(local):
      return local, local
    remote = None
    for name in self.git('remote').splitlines():
      raw = self.git('remote', 'get-url', name)
      if raw.startswith('git@github.com:'):
        raw = 'https://github.com/' + raw.split(':', 1)[1]
      url = urlsplit(raw)
      if (
        url.scheme in ('https', 'ssh')
        and url.hostname == 'github.com'
        and not url.query
        and not url.fragment
        and url.path.rstrip('/').removesuffix('.git') == '/parkhantae/openpilot'
      ):
        remote = name
        break
    if remote is None:
      raise HandoffError('parkhantae/openpilot remote not found')
    ref = f'refs/remotes/{remote}/{branch}'
    self.tracking_ref = f'{remote}/{branch}'
    depth = '--depth=1' if local is None else '--depth=50'
    self.git('fetch', '--no-tags', depth, remote, f'refs/heads/{branch}:{ref}', timeout=30)
    tip = self.git('rev-parse', '--verify', ref)
    if not self.compatible(tip):
      raise HandoffError('target branch does not contain the same handoff protocol; update both branches')
    if local is not None:
      self.git('merge-base', '--is-ancestor', local, tip)
    return tip, local

  def checkout(self, branch, tip, local):
    if local is None:
      self.git('switch', '--create', branch, tip, timeout=30)
    else:
      if local != tip:
        self.git('update-ref', f'refs/heads/{branch}', tip, local)
      self.git('switch', branch, timeout=30)
    if getattr(self, 'tracking_ref', None):
      self.git('branch', '--set-upstream-to', self.tracking_ref, branch)
    if self.branch() != branch or self.git('rev-parse', 'HEAD') != tip:
      raise HandoffError('checkout verification failed')
    self.clean()


def valid_request(params, vehicle, now):
  request = params.get('AutoVehicleBranchRequest')
  return (
    isinstance(request, dict)
    and request.get('vehicle') == vehicle
    and request.get('parked') is True
    and request.get('boot') == boot_id()
    and isinstance(request.get('time'), (int, float))
    and 0 <= now - request['time'] <= REQUEST_SECONDS
  )


def perform_handoff(params, repo, vehicle, guard):
  """Run only in manager's worker with control processes held and Panda noOutput."""
  branch = TARGET_BRANCHES[vehicle]
  with repo_lock():
    if not guard():
      raise HandoffError('Park / zero speed / Panda noOutput confirmation expired')
    source = repo.branch()
    repo.clean()
    tip, local = repo.target(branch)
    if not guard():
      raise HandoffError('vehicle state changed during target preparation')
    if source == branch:
      raise HandoffError('handoff request does not match current branch')
    if repo.branch() != source:
      raise HandoffError('source branch changed during preparation')
    record = {
      'version': 1,
      'boot': boot_id(),
      'vehicle': vehicle,
      'source': source,
      'source_head': repo.git('rev-parse', 'HEAD'),
      'target': branch,
      'target_head': tip,
      'state': 'prepared',
    }
    params.put('AutoVehicleBranchHandoff', record)
    if params.get('AutoVehicleBranchHandoff') != record:
      raise HandoffError('handoff journal did not persist')
    # No forced checkout/reset/stash; Git also protects untracked collisions.
    repo.checkout(branch, tip, local)
    record['state'] = 'checked_out'
    params.put('AutoVehicleBranchHandoff', record)
    return tip


def await_correct_branch(params, candidate, can_recv, *, root=None):
  try:
    return _await_correct_branch(params, candidate, can_recv, root=root)
  except Exception as exc:
    params.put_bool('AutoVehicleBranchPending', True)
    detail = str(exc) if isinstance(exc, HandoffError) else type(exc).__name__
    set_status(params, f'blocked: {detail}; inspect while parked and restart')
    raise


def _await_correct_branch(params, candidate, can_recv, *, root=None):
  """Remain before interface/CarParams construction until manager reboots."""
  from openpilot.common.basedir import BASEDIR

  repo = Repository(root or BASEDIR)
  record = params.get('AutoVehicleBranchHandoff') or {}
  current_boot = boot_id()
  current = repo.branch()
  target = TARGET_BRANCHES[candidate]
  if record and record.get('version') != 1:
    raise HandoffError('invalid handoff journal')
  if record and record.get('boot') != current_boot:
    repo.clean()
    if current not in (record.get('source'), record.get('target')):
      raise HandoffError('unexpected branch after interrupted handoff')
    expected = record.get('target_head') if current == record.get('target') else record.get('source_head')
    if repo.git('rev-parse', 'HEAD') != expected:
      raise HandoffError('checkout changed after interrupted handoff; inspect before enabling control')
  if current == target and not (record and record.get('boot') == current_boot):
    # Recovery is complete only after fresh identification in a new OS boot.
    set_status(params, f'ready: {candidate} / {target}')
    return
  params.put_bool('AutoVehicleBranchPending', True)
  params.remove('ControlsReady')
  guard = ParkGuard(candidate)
  last_write = float('-inf')
  last_parked = None
  while True:
    now = time.monotonic()
    frames = [frame for packet in can_recv(wait_for_one=True) for frame in packet]
    parked = guard.update(frames, now)
    if parked != last_parked or (parked and now - last_write >= 0.2):
      params.put('AutoVehicleBranchRequest', {'vehicle': candidate, 'boot': current_boot, 'time': now, 'parked': parked})
      last_write = now
      last_parked = parked
    # CAN callback has a timeout; avoid a hot loop on an empty/synthetic source.
    if not frames:
      time.sleep(0.01)


def finish_identification(params, candidate):
  record = params.get('AutoVehicleBranchHandoff')
  if record and candidate in TARGET_BRANCHES and record.get('boot') != boot_id():
    params.remove('AutoVehicleBranchHandoff')
