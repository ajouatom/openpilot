"""Manual vehicle presets, with parked selections deferred until startup."""

import math
from pathlib import Path

PORTER = 'HYUNDAI_PORTER_II_EV'
IONIQ5 = 'HYUNDAI_IONIQ_5'
VEHICLES = (PORTER, IONIQ5)
CAR_LABELS = {PORTER: 'Hyundai Porter II EV 2024', IONIQ5: 'Hyundai Ioniq 5 (with HDA II) 2022-24'}
PRESETS = {
  PORTER: {'HyundaiCameraSCC': 0, 'EnableRadarTracks': 0, 'CanfdHDA2': 0, 'CustomSR': 202},
  IONIQ5: {'HyundaiCameraSCC': 2, 'EnableRadarTracks': 3, 'CanfdHDA2': 1, 'CustomSR': 140},
}
PENDING = 'ManualVehicleSelectionPending'


def _boot_id():
  value = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
  if not value:
    raise RuntimeError('Cannot identify the current boot')
  return value


def selected_vehicle_index(params):
  pending = params.get(PENDING)
  if isinstance(pending, dict) and pending.get('version') == 2 and pending.get('vehicle') in VEHICLES:
    return VEHICLES.index(pending['vehicle'])
  selected = params.get('CarSelected3')
  return next((i for i, car in enumerate(VEHICLES) if selected == CAR_LABELS[car]), -1)


def vehicle_selection_parked(sm, now):
  """Require fresh, valid Park and stopped CAN feedback for ignition-on selection."""
  if not sm.seen['carState'] or not sm.valid['carState']:
    return False
  age = now - sm.recv_time['carState']
  if not math.isfinite(age) or not 0 <= age <= 0.25:
    return False
  cs = sm['carState']
  speeds = (cs.vEgo, cs.vEgoRaw, cs.wheelSpeeds.fl, cs.wheelSpeeds.fr, cs.wheelSpeeds.rl, cs.wheelSpeeds.rr)
  return bool(
    cs.canValid and not cs.canTimeout and cs.gearShifter == 'park' and cs.standstill and all(math.isfinite(speed) and abs(speed) <= 0.1 for speed in speeds)
  )


def finish_pending_selection(params, *, startup=False):
  """Recover offroad writes, or apply a parked choice after an OS reboot."""
  pending = params.get(PENDING)
  if not pending:
    return
  if (
    not isinstance(pending, dict)
    or type(pending.get('version')) is not int
    or pending.get('version') not in (1, 2)
    or not isinstance(pending.get('vehicle'), str)
    or pending.get('vehicle') not in PRESETS
  ):
    raise ValueError('Invalid manual vehicle selection')
  required = {'version', 'vehicle'} if pending['version'] == 1 else {'version', 'vehicle', 'boot'}
  if set(pending) != required or (pending['version'] == 2 and (not isinstance(pending['boot'], str) or not pending['boot'])):
    raise ValueError('Invalid manual vehicle selection')
  # Version 2 is an ignition-on selection. Card/ignition restarts must keep the
  # running profile; only manager_init may consume it before processes start.
  if pending['version'] == 2 and (not startup or pending['boot'] == _boot_id()):
    return
  vehicle = pending['vehicle']
  values = {**PRESETS[vehicle], 'CarSelected3': CAR_LABELS[vehicle], 'CarName': vehicle}
  for key, value in values.items():
    params.put(key, value)
    if params.get(key) != value:
      raise OSError(f'Manual vehicle setting did not persist: {key}')
  params.remove(PENDING)


def select_manual_vehicle(params, index, *, parked=False):
  if not parked and (params.get_bool('IsOnroad') or not params.get_bool('IsOffroad')):
    raise RuntimeError('Select the vehicle only while offroad or parked')
  if type(index) is not int or not 0 <= index < len(VEHICLES):
    raise ValueError('Unsupported manual vehicle selection')
  # Journal the explicit request before changing the vehicle preset. This
  # also recovers a power loss without restoring unrelated manual settings.
  pending = {'version': 2 if parked else 1, 'vehicle': VEHICLES[index]}
  if parked:
    pending['boot'] = _boot_id()
  params.put(PENDING, pending)
  if params.get(PENDING) != pending:
    raise OSError('Manual vehicle selection did not persist')
  finish_pending_selection(params)


def retire_automatic_switch(params):
  """Ignore old auto-session/handoff latches; retain the old backup as data."""
  params.put_bool('AutoVehicleSwitch', False)
  for key in (
    'AutoVehicleSession',
    'AutoVehicleReady',
    'AutoVehicleResolved',
    'AutoVehicleBranchPending',
    'AutoVehicleBranchRequest',
    'AutoVehicleBranchHandoff',
    'AutoVehicleBranchStatus',
  ):
    params.remove(key)
