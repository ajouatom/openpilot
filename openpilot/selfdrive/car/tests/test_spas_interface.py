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


@pytest.mark.parametrize('kick', [False, True])
@pytest.mark.parametrize('control', [False, True])
def test_blinker_kick_switches_spas_control_off(kick, control):
  # joongyu01: CanfdBlinkerKick on -> CanfdBlinkerControl cleared at boot, so the two never pause APRK together
  source = Path('opendbc_repo/opendbc/car/hyundai/interface.py')
  tree = ast.parse(source.read_text(encoding='utf-8'))
  node = next(n for n in ast.walk(tree) if isinstance(n, ast.If) and 'CanfdBlinkerKick' in ast.unparse(n.test))
  store = {'CanfdBlinkerKick': kick, 'CanfdBlinkerControl': control}
  params = NS(get_bool=store.get, put_bool=store.__setitem__)
  exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), {'params': params, 'docs': False})
  assert store['CanfdBlinkerControl'] == (control and not kick)
