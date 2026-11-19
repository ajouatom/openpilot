"""Offroad manual vehicle presets. Never identify a car or change branches."""

PORTER = 'HYUNDAI_PORTER_II_EV'
IONIQ5 = 'HYUNDAI_IONIQ_5'
VEHICLES = (PORTER, IONIQ5)
CAR_LABELS = {PORTER: 'Hyundai Porter II EV 2024', IONIQ5: 'Hyundai Ioniq 5 (with HDA II) 2022-24'}
PRESETS = {
  PORTER: {'HyundaiCameraSCC': 0, 'EnableRadarTracks': 0, 'CanfdHDA2': 0},
  IONIQ5: {'HyundaiCameraSCC': 2, 'EnableRadarTracks': 3, 'CanfdHDA2': 1},
}
PENDING = 'ManualVehicleSelectionPending'


def selected_vehicle_index(params):
  selected = params.get('CarSelected3')
  return next((i for i, car in enumerate(VEHICLES) if selected == CAR_LABELS[car]), -1)


def finish_pending_selection(params):
  """Finish an interrupted explicit selection before constructing CarParams."""
  pending = params.get(PENDING)
  if not pending:
    return
  if not isinstance(pending, dict) or set(pending) != {'version', 'vehicle'} or pending['version'] != 1 or pending['vehicle'] not in PRESETS:
    raise ValueError('Invalid manual vehicle selection')
  vehicle = pending['vehicle']
  values = {**PRESETS[vehicle], 'CarSelected3': CAR_LABELS[vehicle], 'CarName': vehicle}
  for key, value in values.items():
    params.put(key, value)
    if params.get(key) != value:
      raise OSError(f'Manual vehicle setting did not persist: {key}')
  params.remove(PENDING)


def select_manual_vehicle(params, index):
  if params.get_bool('IsOnroad') or not params.get_bool('IsOffroad'):
    raise RuntimeError('Select the vehicle only while offroad')
  if type(index) is not int or not 0 <= index < len(VEHICLES):
    raise ValueError('Unsupported manual vehicle selection')
  # Journal the explicit request before changing any of the five fields. This
  # also recovers a power loss without restoring unrelated manual settings.
  pending = {'version': 1, 'vehicle': VEHICLES[index]}
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
