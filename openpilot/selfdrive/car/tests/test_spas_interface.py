import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from openpilot.cereal import car
from opendbc.car.hyundai.spas import compatible, SPAS
from opendbc.car.hyundai.values import HyundaiFlags, HyundaiSafetyFlags


@pytest.mark.parametrize('bus', [0, 1, 4, 5])
@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('valid_spas', [False, True])
def test_actual_enabled_interface_assigns_serializable_flags(bus, enabled, valid_spas):
  source = Path('opendbc_repo/opendbc/car/hyundai/interface.py')
  tree = ast.parse(source.read_text(encoding='utf-8'))
  node = next(n for n in ast.walk(tree) if isinstance(n, ast.If) and 'compatible' in ast.unparse(n.test))
  ret = car.CarParams.new_message(brand='hyundai', flags=HyundaiFlags.CANFD.value)
  ret.init('safetyConfigs', 1)
  ret.safetyConfigs[0].safetyModel = 'hyundaiCanfd'
  ns = {'ret': ret, 'params': NS(get_bool=lambda key: enabled), 'compatible': compatible,
            'fingerprint': {bus: SPAS if valid_spas else {0x165:16,0x16a:32}}, 'CAN': NS(ECAN=bus),
            'HyundaiFlags': HyundaiFlags, 'HyundaiSafetyFlags': HyundaiSafetyFlags}
  exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),ns)
  with car.CarParams.from_bytes(ret.to_bytes()) as cp:
    assert bool(cp.flags & HyundaiFlags.CANFD_SPAS_BLINKERS)==(enabled and valid_spas)
    assert bool(cp.safetyConfigs[0].safetyParam & HyundaiSafetyFlags.CANFD_SPAS_BLINKERS)==(enabled and valid_spas)


@pytest.mark.parametrize('camera', [False, True])
@pytest.mark.parametrize('kick_car', [False, True])
@pytest.mark.parametrize('kick', [False, True])
@pytest.mark.parametrize('control', [False, True])
def test_blinker_kick_needs_kick_car_and_camera_scc(camera, kick_car, kick, control):
  # CanfdBlinkerKick and CanfdBlinkerControl both pause APRK and send 0x16A. Kick runs only on a KICK_CARS model with the
  # camera-SCC harness: there it clears CanfdBlinkerControl at boot and sets CANFD_BLINKER_KICK, panda's Kick allowlist.
  # Anywhere else Kick is inert: CanfdBlinkerControl is left alone and the flag stays clear.
  source = Path('opendbc_repo/opendbc/car/hyundai/interface.py')
  tree = ast.parse(source.read_text(encoding='utf-8'))
  assign = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign) and ast.unparse(n.targets[0]) == 'kick')
  clear = next(n for n in ast.walk(tree) if isinstance(n, ast.If) and 'CanfdBlinkerControl' in ast.unparse(n.test)
               and 'kick' in {x.id for x in ast.walk(n.test) if isinstance(x, ast.Name)})
  flag = next(n for n in ast.walk(tree) if isinstance(n, ast.If) and ast.unparse(n.test) == 'kick')
  ret = car.CarParams.new_message(brand='hyundai', flags=HyundaiFlags.CANFD.value | (HyundaiFlags.CAMERA_SCC.value if camera else 0))
  ret.init('safetyConfigs', 1)
  ret.safetyConfigs[0].safetyModel = 'hyundaiCanfd'
  store = {'CanfdBlinkerKick': kick, 'CanfdBlinkerControl': control}
  params = NS(get_bool=store.get, put_bool=store.__setitem__)
  scope = {'params': params, 'docs': False, 'candidate': 'KICK' if kick_car else 'OTHER', 'KICK_CARS': {'KICK'}, 'ret': ret,
           'HyundaiFlags': HyundaiFlags, 'HyundaiSafetyFlags': HyundaiSafetyFlags}
  exec(compile(ast.Module(body=[assign, clear, flag], type_ignores=[]), str(source), 'exec'), scope)
  active = kick and kick_car and camera
  assert store['CanfdBlinkerControl'] == (control and not active)
  with car.CarParams.from_bytes(ret.to_bytes()) as cp:
    assert bool(cp.safetyConfigs[0].safetyParam & HyundaiSafetyFlags.CANFD_BLINKER_KICK) == active
