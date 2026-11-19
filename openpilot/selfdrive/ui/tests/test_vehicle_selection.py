"""Execute the device toggle's actual selection and refresh callbacks."""

import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from openpilot.common.params import Params
from openpilot.common.vehicle_selection import select_manual_vehicle, selected_vehicle_index, CAR_LABELS, IONIQ5, PORTER

SOURCE = Path(__file__).resolve().parents[1] / 'layouts/settings/toggles.py'


@pytest.fixture
def menu(tmp_path):
  params = Params(str(tmp_path / 'params'))
  params.put_bool('IsOffroad', True)
  params.put_int('CustomSR', 202)
  tree = ast.parse(SOURCE.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TogglesLayout')
  methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('_set_manual_vehicle', '_refresh_vehicle_selection')]
  obj = NS(_params=params)
  ui = NS(started=False)
  errors = []
  namespace = {
    'ui_state': ui,
    'select_manual_vehicle': select_manual_vehicle,
    'selected_vehicle_index': selected_vehicle_index,
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


def test_both_buttons_write_exact_three_values_and_keep_manual_tuning(menu):
  obj, params, _, errors = menu
  assert len(obj._vehicle_selection.buttons) == 2
  for index, vehicle, values in ((0, PORTER, (0, 0, 0)), (1, IONIQ5, (2, 3, 1))):
    obj._vehicle_selection.callback(index)
    assert params.get('CarSelected3') == CAR_LABELS[vehicle]
    assert tuple(params.get_int(k) for k in ('HyundaiCameraSCC', 'EnableRadarTracks', 'CanfdHDA2')) == values
    assert params.get_int('CustomSR') == 202
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
  assert translated['Vehicle (manual)']
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
