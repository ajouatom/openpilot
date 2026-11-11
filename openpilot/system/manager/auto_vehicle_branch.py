"""Manager-side handoff worker; keep all controls blocked through OS restart."""

from concurrent.futures import ThreadPoolExecutor
import time

from openpilot.common.auto_vehicle_branch import (
  HandoffError,
  Repository,
  TARGET_BRANCHES,
  boot_id,
  perform_handoff,
  set_status,
  valid_request,
)
from openpilot.common.basedir import BASEDIR
from openpilot.common.repo_update import repo_lock

# Git writers and model/route readers cannot restart across a live checkout.
HANDOFF_PROCESSES = frozenset({'card', 'pandad', 'hardwared', 'ui', 'logmessaged', 'timed'})


def pandas_no_output(sm):
  from openpilot.cereal import car, log

  return (
    sm.all_checks(['deviceState', 'pandaStates'])
    and len(sm['pandaStates']) > 0
    and all(
      ps.pandaType != log.PandaState.PandaType.unknown and ps.safetyModel == car.CarParams.SafetyModel.noOutput and not ps.controlsAllowed
      for ps in sm['pandaStates']
    )
  )


class AutoVehicleBranchManager:
  def __init__(self, params, root=BASEDIR):
    self.params = params
    self.root = root
    self.future = None
    self.executor = None
    self.failed = False
    self.safety_time = float('-inf')
    self.vehicle = None
    self.epoch = None

  def guard(self):
    now = time.monotonic()
    return (
      self.params.get_bool('AutoVehicleBranchPending')
      and not self.failed
      and now - self.safety_time <= 0.8
      and self.vehicle in TARGET_BRANCHES
      and valid_request(self.params, self.vehicle, now)
    )

  def update(self, now, *, no_output, workers_stopped):
    try:
      self._update(now, no_output=no_output, workers_stopped=workers_stopped)
    except Exception as exc:
      self.failed = True
      detail = str(exc) if isinstance(exc, HandoffError) else type(exc).__name__
      set_status(self.params, f'blocked: {detail}; resolve while parked and restart')

  def _update(self, now, *, no_output, workers_stopped):
    if not self.params.get_bool('AutoVehicleBranchPending'):
      return
    if no_output and workers_stopped:
      self.safety_time = now
    else:
      self.safety_time = float('-inf')
    request = self.params.get('AutoVehicleBranchRequest') or {}
    candidate = request.get('vehicle')
    if candidate not in TARGET_BRANCHES:
      return
    if self.vehicle is None and candidate in TARGET_BRANCHES:
      self.vehicle = candidate
      self.epoch = request.get('boot')
    if candidate != self.vehicle or request.get('boot') != self.epoch:
      return
    if self.failed:
      return
    record = self.params.get('AutoVehicleBranchHandoff') or {}
    # Recovery of a manager/card restart in the checkout boot must never
    # release controls or start another checkout. A real OS restart is needed.
    if record.get('boot') == boot_id() and self.future is None:
      repo = Repository(self.root)
      if repo.branch() == record.get('target') and repo.git('rev-parse', 'HEAD') == record.get('target_head'):
        repo.clean()
        record['state'] = 'checked_out'
        self.params.put('AutoVehicleBranchHandoff', record)
        self._reboot_if_safe()
        return
    if self.future is None:
      if not self.guard():
        set_status(self.params, 'waiting: Park, zero wheels and Panda noOutput required')
        return
      self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='vehicle-branch')
      self.future = self.executor.submit(perform_handoff, self.params, Repository(self.root), self.vehicle, self.guard)
      set_status(self.params, f'switching: {TARGET_BRANCHES[self.vehicle]}')
    elif self.future.done():
      try:
        self.future.result()
        self._reboot_if_safe()
      except Exception as exc:
        self.failed = True
        # Never relay Git output or remote URLs into Params/logs.
        detail = str(exc) if isinstance(exc, HandoffError) else type(exc).__name__
        set_status(self.params, f'blocked: {detail}; resolve while parked and restart')
      finally:
        if self.executor is not None:
          self.executor.shutdown(wait=False)
          self.executor = None

  def _reboot_if_safe(self):
    if not self.guard():
      set_status(self.params, 'switched: waiting for Park and zero speed before reboot')
      return
    with repo_lock():
      repo = Repository(self.root)
      record = self.params.get('AutoVehicleBranchHandoff') or {}
      if repo.branch() != record.get('target') or repo.git('rev-parse', 'HEAD') != record.get('target_head'):
        raise HandoffError('checkout changed before reboot')
      repo.clean()
      if self.guard():
        set_status(self.params, f'rebooting: {record["target"]}')
        self.params.put_bool('DoReboot', True)
