from pathlib import Path
import subprocess

import pytest

from openpilot.common import auto_vehicle_branch as avb, repo_update
from openpilot.common.auto_vehicle import IONIQ5, PORTER
from openpilot.common.params import Params
from opendbc.can import CANPacker
from opendbc.car.can_definitions import CanData


def git(root, *args):
  return subprocess.run(['git', *args], cwd=root, check=True, capture_output=True, text=True).stdout.strip()


@pytest.fixture
def params(tmp_path):
  return Params(str(tmp_path / 'params'))


@pytest.fixture
def repo(tmp_path, monkeypatch):
  root = tmp_path / 'repo'
  root.mkdir()
  git(root, 'init', '--initial-branch=c3-pht-porter')
  git(root, 'config', 'user.email', 'test@example.invalid')
  git(root, 'config', 'user.name', 'Test')
  module = root / avb.MODULE_PATH
  module.parent.mkdir(parents=True)
  module.write_bytes(Path(avb.__file__).read_bytes())
  header = root / 'openpilot/common/params_keys.h'
  header.write_text((Path(avb.__file__).parent / 'params_keys.h').read_text())
  (root / 'version').write_text('porter')
  git(root, 'add', '.')
  git(root, 'commit', '-m', 'porter')
  git(root, 'switch', '-c', 'c3-pht')
  (root / 'version').write_text('ioniq')
  git(root, 'commit', '-am', 'ioniq')
  git(root, 'switch', 'c3-pht-porter')
  monkeypatch.setattr(repo_update, 'LOCK_PATH', str(tmp_path / 'repo.lock'))
  return avb.Repository(root)


class Frames:
  def __init__(self, vehicle):
    self.vehicle = vehicle
    self.packer = CANPacker('hyundai_canfd_generated' if vehicle == IONIQ5 else 'hyundai_kia_generic')
    self.counter = 0

  def parked(self, *, speed=0, park=True):
    self.counter += 1
    if self.vehicle == IONIQ5:
      values = {'COUNTER': self.counter % 256, **{f'WHEEL_SPEED_{n}': speed for n in range(1, 5)}}
      wheel = self.packer.make_can_msg('WHEEL_SPEEDS', 0, values)
      gear = self.packer.make_can_msg('ACCELERATOR', 0, {'COUNTER': self.counter % 256, 'GEAR': 0 if park else 5})
    else:
      values = {f'WHL_SPD_{n}': speed for n in ('FL', 'FR', 'RL', 'RR')}
      values.update(WHL_SPD_AliveCounter_LSB=self.counter % 4, WHL_SPD_AliveCounter_MSB=(self.counter % 16) // 4)
      address, raw, bus = self.packer.make_can_msg('WHL_SPD11', 0, values)
      data = bytearray(raw)
      checksum = (sum((b & 0x3F if i % 2 else b).bit_count() for i, b in enumerate(data)) ^ 9) & 15
      data[5] |= (checksum & 3) << 6
      data[7] |= (checksum >> 2) << 6
      wheel = (address, bytes(data), bus)
      gear = self.packer.make_can_msg('ELECT_GEAR', 0, {'Elect_Gear_Shifter': 0 if park else 5})
    return [CanData(a, d, b) for a, d, b in (wheel, gear)]


@pytest.mark.parametrize('vehicle', [PORTER, IONIQ5])
def test_park_guard_requires_continuous_fresh_zero_wheels_and_park(vehicle):
  guard, frames = avb.ParkGuard(vehicle), Frames(vehicle)
  assert not guard.update([], 10.0)
  for i in range(120):
    ready = guard.update(frames.parked(), 10 + i * 0.01)
  assert ready
  assert not guard.update(frames.parked(speed=0.03125), 11.2)
  assert not guard.update(frames.parked(), 11.21)
  for i in range(120):
    ready = guard.update(frames.parked(), 12 + i * 0.01)
  assert ready
  assert not guard.update(frames.parked(park=False), 13.2)
  assert not guard.update(frames.parked(), 13.21)
  assert not guard.update([], 14)


@pytest.mark.parametrize('vehicle', [PORTER, IONIQ5])
@pytest.mark.parametrize('fault', ['checksum', 'dlc', 'wrong_bus', 'frozen_counter', 'missing_gear'])
def test_bad_or_missing_vehicle_input_cannot_arm_handoff(vehicle, fault):
  guard, frames = avb.ParkGuard(vehicle), Frames(vehicle)
  frozen = frames.parked()
  for i in range(150):
    values = frames.parked()
    if fault == 'checksum':
      a, d, b = values[0]
      values[0] = CanData(a, bytes([d[0] ^ 1]) + d[1:], b)
    elif fault == 'dlc':
      values[0] = values[0]._replace(dat=b'')
    elif fault == 'wrong_bus':
      values = [v._replace(src=2) for v in values]
    elif fault == 'frozen_counter':
      values = frozen
    elif fault == 'missing_gear':
      values = values[:1]
    assert not guard.update(values, 10 + i * 0.01)


def test_safe_checkout_and_new_boot_identity_complete_round_trip(repo, params, monkeypatch):
  current_boot = ['first']
  monkeypatch.setattr(avb, 'boot_id', lambda: current_boot[0])
  params.put_bool('AutoVehicleSwitch', True)
  avb.perform_handoff(params, repo, IONIQ5, lambda: True)
  assert repo.branch() == 'c3-pht'
  assert (repo.root / 'version').read_text() == 'ioniq'
  assert params.get('AutoVehicleBranchHandoff')['state'] == 'checked_out'
  avb.finish_identification(params, IONIQ5)
  assert params.get('AutoVehicleBranchHandoff')  # same OS boot cannot finish
  current_boot[0] = 'second'
  avb.await_correct_branch(params, IONIQ5, lambda **kw: [], root=repo.root)
  assert params.get('AutoVehicleBranchHandoff')  # settings/CP must finish too
  avb.finish_identification(params, IONIQ5)
  assert not params.get('AutoVehicleBranchHandoff')
  avb.perform_handoff(params, repo, PORTER, lambda: True)
  assert repo.branch() == 'c3-pht-porter'
  current_boot[0] = 'third'
  avb.await_correct_branch(params, PORTER, lambda **kw: [], root=repo.root)
  avb.finish_identification(params, PORTER)
  assert not params.get('AutoVehicleBranchHandoff')


@pytest.mark.parametrize('fault', ['dirty', 'untracked_collision', 'incompatible', 'moving', 'other_branch', 'other_worktree'])
def test_failures_preserve_source_work_and_do_not_force_checkout(repo, params, fault, tmp_path):
  if fault == 'dirty':
    (repo.root / 'version').write_text('my changes')
  elif fault == 'untracked_collision':
    git(repo.root, 'switch', 'c3-pht')
    (repo.root / 'mine').write_text('target')
    git(repo.root, 'add', 'mine')
    git(repo.root, 'commit', '-m', 'target file')
    git(repo.root, 'switch', 'c3-pht-porter')
    (repo.root / 'mine').write_text('keep me')
  elif fault == 'incompatible':
    git(repo.root, 'switch', 'c3-pht')
    (repo.root / avb.MODULE_PATH).write_text('older protocol')
    git(repo.root, 'commit', '-am', 'old protocol')
    git(repo.root, 'switch', 'c3-pht-porter')
  elif fault == 'other_branch':
    git(repo.root, 'switch', '-c', 'unrelated')
  elif fault == 'other_worktree':
    git(repo.root, 'worktree', 'add', str(tmp_path / 'other'), 'c3-pht')
  head = git(repo.root, 'rev-parse', 'HEAD')
  with pytest.raises(avb.HandoffError):
    avb.perform_handoff(params, repo, IONIQ5, lambda: fault != 'moving')
  assert git(repo.root, 'rev-parse', 'HEAD') == head
  if fault == 'dirty':
    assert (repo.root / 'version').read_text() == 'my changes'
  if fault == 'untracked_collision':
    assert (repo.root / 'mine').read_text() == 'keep me'


def test_state_expiry_during_target_preparation_does_not_checkout(repo, params, monkeypatch):
  real = repo.target
  valid = [True]

  def slow_target(branch):
    result = real(branch)
    valid[0] = False
    return result

  monkeypatch.setattr(repo, 'target', slow_target)
  with pytest.raises(avb.HandoffError):
    avb.perform_handoff(params, repo, IONIQ5, lambda: valid[0])
  assert repo.branch() == 'c3-pht-porter'
  assert not params.get('AutoVehicleBranchHandoff')


def test_new_boot_ref_mismatch_never_releases_startup(repo, params, monkeypatch):
  monkeypatch.setattr(avb, 'boot_id', lambda: 'old')
  avb.perform_handoff(params, repo, IONIQ5, lambda: True)
  git(repo.root, 'commit', '--allow-empty', '-m', 'changed externally')
  monkeypatch.setattr(avb, 'boot_id', lambda: 'new')
  with pytest.raises(avb.HandoffError):
    avb.await_correct_branch(params, IONIQ5, lambda **kw: [], root=repo.root)
  assert params.get('AutoVehicleBranchHandoff')


def test_pending_card_restart_does_not_construct_interface_before_real_reboot(repo, params, monkeypatch):
  monkeypatch.setattr(avb, 'boot_id', lambda: 'same')
  avb.perform_handoff(params, repo, IONIQ5, lambda: True)

  class Reboot(Exception):
    pass

  def can_recv(**kw):
    raise Reboot

  with pytest.raises(Reboot):
    avb.await_correct_branch(params, IONIQ5, can_recv, root=repo.root)
  assert params.get_bool('AutoVehicleBranchPending')
  assert not params.get_bool('AutoVehicleReady')


def test_request_must_be_fresh_parked_same_vehicle_and_boot(params, monkeypatch):
  monkeypatch.setattr(avb, 'boot_id', lambda: 'boot')
  for vehicle, boot, stamp, parked in ((PORTER, 'boot', 10, True), (IONIQ5, 'other', 10, True), (IONIQ5, 'boot', 8, True), (IONIQ5, 'boot', 10, False)):
    params.put('AutoVehicleBranchRequest', {'vehicle': vehicle, 'boot': boot, 'time': stamp, 'parked': parked})
    assert not avb.valid_request(params, IONIQ5, 10)
  params.put('AutoVehicleBranchRequest', {'vehicle': IONIQ5, 'boot': 'boot', 'time': 10, 'parked': True})
  assert avb.valid_request(params, IONIQ5, 10.1)
  assert not avb.valid_request(params, IONIQ5, 9.9)


def test_bootstrap_fetch_creates_tracking_branch_without_reset(repo, params, tmp_path, monkeypatch):
  bare = tmp_path / 'remote.git'
  git(repo.root, 'clone', '--bare', str(repo.root), str(bare))
  git(repo.root, 'branch', '-D', 'c3-pht')
  git(repo.root, 'remote', 'add', 'origin', 'https://github.com/parkhantae/openpilot.git')
  original = repo.git

  def local_transport(*args, **kwargs):
    if args[0] == 'fetch':
      assert args[1:4] == ('--no-tags', '--depth=1', 'origin')
      return original('fetch', '--no-tags', '--depth=1', str(bare), *args[4:], **kwargs)
    return original(*args, **kwargs)

  monkeypatch.setattr(repo, 'git', local_transport)
  avb.perform_handoff(params, repo, IONIQ5, lambda: True)
  assert repo.branch() == 'c3-pht'
  assert original('rev-parse', '--abbrev-ref', '@{upstream}') == 'origin/c3-pht'


@pytest.mark.parametrize(
  'url', ['https://github.com/other/openpilot.git', 'https://evil.example/parkhantae/openpilot.git', 'file://github.com/parkhantae/openpilot.git']
)
def test_unknown_remote_cannot_bootstrap_checkout(repo, params, url):
  git(repo.root, 'branch', '-D', 'c3-pht')
  git(repo.root, 'remote', 'add', 'origin', url)
  with pytest.raises(avb.HandoffError, match='remote not found'):
    avb.perform_handoff(params, repo, IONIQ5, lambda: True)
  assert repo.branch() == 'c3-pht-porter'


def test_target_cannot_delete_stored_persistent_settings(repo, params):
  git(repo.root, 'switch', 'c3-pht')
  header = repo.root / 'openpilot/common/params_keys.h'
  header.write_text('')
  git(repo.root, 'commit', '-am', 'incompatible registry')
  git(repo.root, 'switch', 'c3-pht-porter')
  with pytest.raises(avb.HandoffError):
    avb.perform_handoff(params, repo, IONIQ5, lambda: True)
  assert repo.branch() == 'c3-pht-porter'


def test_manager_clear_preserves_porter_gap_values_and_interrupted_handoff(params):
  from openpilot.common.params import ParamKeyFlag

  params.put_int('PorterGap1', 37)
  params.put_int('PorterLeadReactionTime', 55)
  params.put_bool('ManualTurnIndicator', True)
  params.put_bool('AutoVehicleBranchPending', True)
  params.put('AutoVehicleBranchHandoff', {'version': 1, 'state': 'prepared'})
  params.clear_all(ParamKeyFlag.CLEAR_ON_MANAGER_START)
  assert params.get_int('PorterGap1') == 37
  assert params.get_int('PorterLeadReactionTime') == 55
  assert params.get_bool('ManualTurnIndicator')
  assert not params.get_bool('AutoVehicleBranchPending')
  assert avb.session_required(params)


def test_waiting_in_drive_does_not_rewrite_flash_on_every_can_frame(repo, params):
  class Stop(Exception):
    pass

  class CountingParams:
    requests = 0

    def __getattr__(self, key):
      return getattr(params, key)

    def put(self, key, value):
      if key == 'AutoVehicleBranchRequest':
        self.requests += 1
      return params.put(key, value)

  counted, frames = CountingParams(), Frames(IONIQ5)
  calls = 0

  def can_recv(**kwargs):
    nonlocal calls
    calls += 1
    if calls > 100:
      raise Stop
    return [frames.parked(park=False)]

  with pytest.raises(Stop):
    avb.await_correct_branch(counted, IONIQ5, can_recv, root=repo.root)
  assert counted.requests == 1
