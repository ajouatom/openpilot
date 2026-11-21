"""Execute the device toggle's actual selection and refresh callbacks."""

import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from openpilot.common.params import Params
from openpilot.common.vehicle_selection import (
  select_manual_vehicle,
  selected_vehicle_index,
  vehicle_selection_parked,
  finish_pending_selection,
  PENDING,
  CAR_LABELS,
  IONIQ5,
  PORTER,
)

SOURCE = Path(__file__).resolve().parents[1] / 'layouts/settings/toggles.py'


@pytest.fixture
def menu(tmp_path, monkeypatch):
  from openpilot.common import vehicle_selection

  monkeypatch.setattr(vehicle_selection, '_boot_id', lambda: 'boot-a')
  params = Params(str(tmp_path / 'params'))
  params.put_bool('IsOffroad', True)
  params.put_int('CustomSR', 202)
  tree = ast.parse(SOURCE.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TogglesLayout')
  methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('_set_manual_vehicle', '_refresh_vehicle_selection')]
  obj = NS(_params=params)

  class Messages(dict):
    seen = {'carState': False}
    valid = {'carState': False}
    recv_time = {'carState': 1000.0}

  ui = NS(started=False, sm=Messages())
  errors = []
  namespace = {
    'ui_state': ui,
    'select_manual_vehicle': select_manual_vehicle,
    'selected_vehicle_index': selected_vehicle_index,
    'vehicle_selection_parked': vehicle_selection_parked,
    'time': NS(monotonic=lambda: 1000.0),
    'tr': lambda value: value,
    'DESCRIPTIONS': {'ManualVehicle': 'manual description'},
    'cloudlog': NS(exception=lambda msg: errors.append(msg)),
  }
  exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), 'exec'), namespace)
  for method in methods:
    setattr(obj, method.name, lambda *args, name=method.name: namespace[name](obj, *args))

  def factory(*args, **kw):
    action = NS(selected_button=kw['selected_index'], enabled=True)
    action.set_enabled = lambda value: setattr(action, 'enabled', value)
    item = NS(action_item=action, callback=kw['callback'], buttons=kw['buttons'])
    item.set_description = lambda value: setattr(item, 'description', value)
    return item

  node = next(n for n in ast.walk(cls) if isinstance(n, ast.Assign) and ast.unparse(n.targets[0]) == 'self._vehicle_selection')
  exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), 'exec'), dict(namespace, self=obj, multiple_button_item=factory))
  return obj, params, ui, errors


def test_both_buttons_apply_vehicle_settings_and_steering_ratio(menu):
  obj, params, _, errors = menu
  assert len(obj._vehicle_selection.buttons) == 2
  for index, vehicle, values in ((0, PORTER, (0, 0, 0)), (1, IONIQ5, (2, 3, 1))):
    obj._vehicle_selection.callback(index)
    assert params.get('CarSelected3') == CAR_LABELS[vehicle]
    assert tuple(params.get_int(k) for k in ('HyundaiCameraSCC', 'EnableRadarTracks', 'CanfdHDA2')) == values
    assert params.get_int('CustomSR') == (202 if vehicle == PORTER else 140)
    assert obj._vehicle_selection.action_item.selected_button == index
  assert not errors


@pytest.mark.parametrize('ui_started,stored_onroad', [(True, False), (False, True)])
def test_onroad_callbacks_cannot_change_the_selected_vehicle(menu, ui_started, stored_onroad):
  obj, params, ui, _ = menu
  obj._vehicle_selection.callback(0)
  ui.started = ui_started
  params.put_bool('IsOnroad', stored_onroad)
  obj._vehicle_selection.callback(1)
  assert params.get('CarName') == PORTER


@pytest.mark.parametrize('locale', ['ko', 'en', 'zh-CHS'])
def test_manual_vehicle_translations_load_in_the_production_parser(locale):
  from openpilot.system.ui.lib.multilang import load_translations

  path = SOURCE.parents[2] / 'translations' / f'app_{locale}.po'
  translated, _ = load_translations(path)
  assert translated['Vehicle (restart)']
  assert translated['Porter II EV']
  assert translated['Ioniq 5 HDA2']
  assert translated['Vehicle selection could not finish. Restart before driving.']
  tree = ast.parse(SOURCE.read_text())
  description = next(
    value
    for node in ast.walk(tree)
    if isinstance(node, ast.Dict)
    for key, value in zip(node.keys, node.values, strict=True)
    if isinstance(key, ast.Constant) and key.value == 'ManualVehicle'
  )
  # Read the exact concatenated tr_noop argument without importing the GUI.
  message = eval(compile(ast.Expression(description.args[0]), str(SOURCE), 'eval'), {'__builtins__': {}})
  assert translated[message]


@pytest.mark.parametrize('target', [0, 1])
def test_ignition_on_park_enables_choice_but_keeps_current_profile_until_restart(menu, monkeypatch, target):
  from openpilot.common import vehicle_selection
  from opendbc.car.structs import CarState

  obj, params, ui, errors = menu
  obj._vehicle_selection.callback(0)
  ui.started = True
  params.put_bool('IsOnroad', True)
  params.put_bool('IsOffroad', False)
  ui.sm['carState'] = CarState.new_message(canValid=True, standstill=True, gearShifter='park')
  ui.sm.seen['carState'] = ui.sm.valid['carState'] = True
  obj._refresh_vehicle_selection()
  assert obj._vehicle_selection.action_item.enabled
  obj._vehicle_selection.callback(target)
  assert obj._vehicle_selection.action_item.selected_button == target
  assert params.get('CarName') == PORTER
  assert params.get_int('CustomSR') == 202
  assert params.get(PENDING) == {'version': 2, 'vehicle': (PORTER, IONIQ5)[target], 'boot': 'boot-a'}
  finish_pending_selection(params)
  assert params.get('CarName') == PORTER
  finish_pending_selection(params, startup=True)
  assert params.get(PENDING)
  assert params.get('CarName') == PORTER
  monkeypatch.setattr(vehicle_selection, '_boot_id', lambda: 'boot-b')
  finish_pending_selection(params, startup=True)
  assert params.get('CarName') == (PORTER, IONIQ5)[target]
  assert not errors


@pytest.mark.parametrize('gear', ['drive', 'reverse', 'neutral', 'unknown'])
def test_ignition_on_nonpark_cannot_select(menu, gear):
  from opendbc.car.structs import CarState

  obj, params, ui, _ = menu
  ui.started = True
  params.put_bool('IsOnroad', True)
  params.put_bool('IsOffroad', False)
  ui.sm['carState'] = CarState.new_message(canValid=True, standstill=True, gearShifter=gear)
  ui.sm.seen['carState'] = ui.sm.valid['carState'] = True
  obj._refresh_vehicle_selection()
  assert not obj._vehicle_selection.action_item.enabled
  obj._vehicle_selection.callback(1)
  assert not params.get(PENDING)


@pytest.mark.parametrize('invalid', ['stale', 'future', 'unseen', 'invalid', 'can-invalid', 'timeout', 'moving', 'wheel-moving', 'nan', 'not-stopped'])
def test_invalid_park_feedback_does_not_allow_selection(invalid):
  from opendbc.car.structs import CarState

  class Messages(dict):
    seen = {'carState': True}
    valid = {'carState': True}
    recv_time = {'carState': 1000.0}

  cs = CarState.new_message(canValid=True, standstill=True, gearShifter='park')
  sm = Messages(carState=cs)
  assert vehicle_selection_parked(sm, 1000.0)
  if invalid == 'stale':
    sm.recv_time['carState'] = 999.0
  elif invalid == 'future':
    sm.recv_time['carState'] = 1001.0
  elif invalid == 'unseen':
    sm.seen['carState'] = False
  elif invalid == 'invalid':
    sm.valid['carState'] = False
  elif invalid == 'can-invalid':
    cs.canValid = False
  elif invalid == 'timeout':
    cs.canTimeout = True
  elif invalid == 'moving':
    cs.vEgoRaw = 0.2
  elif invalid == 'wheel-moving':
    cs.wheelSpeeds.rr = 0.2
  elif invalid == 'nan':
    cs.vEgo = float('nan')
  elif invalid == 'not-stopped':
    cs.standstill = False
  assert not vehicle_selection_parked(sm, 1000.0)


@pytest.mark.parametrize('change', ['drive', 'stale'])
def test_click_rechecks_park_even_when_button_was_enabled(menu, change):
  from opendbc.car.structs import CarState

  obj, params, ui, _ = menu
  ui.started = True
  params.put_bool('IsOnroad', True)
  params.put_bool('IsOffroad', False)
  ui.sm['carState'] = CarState.new_message(canValid=True, standstill=True, gearShifter='park')
  ui.sm.seen['carState'] = ui.sm.valid['carState'] = True
  obj._refresh_vehicle_selection()
  assert obj._vehicle_selection.action_item.enabled
  if change == 'drive':
    ui.sm['carState'].gearShifter = 'drive'
  else:
    ui.sm.recv_time['carState'] = 999.0
  obj._vehicle_selection.callback(1)
  assert not params.get(PENDING)
  assert not obj._vehicle_selection.action_item.enabled
