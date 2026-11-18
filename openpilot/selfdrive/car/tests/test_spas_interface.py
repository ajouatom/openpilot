import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from openpilot.cereal import car
from opendbc.car.hyundai.spas import compatible, kick_compatible, SPAS
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


# carrot-wip 6af3073e tests that hold on this branch. Its protocol-gated Kick tests (CANFD_BLINKER_KICK) do not: this
# branch's Kick is not limited to kick_compatible and its CANFD_BLINKER_KICK is 0 (values.py).
@pytest.mark.parametrize('missing', [0x165, 0x16A, 0x3C1])
@pytest.mark.parametrize('wrong_length', [None, 8, 16, 24, 32])
def test_kick_rejects_missing_or_wrong_contract(missing, wrong_length):
  contract = {**SPAS, 0x3C1: 8}
  expected = contract[missing]
  if wrong_length is None:
    del contract[missing]
  else:
    contract[missing] = wrong_length
  assert kick_compatible({0: contract}, 0, True) == (wrong_length == expected)


@pytest.mark.parametrize('fingerprint', ['HYUNDAI_IONIQ_5_PE', 'KIA_EV4', 'OTHER_MODEL'])
@pytest.mark.parametrize('passive', [False, True])
def test_model_observers_do_not_gate_on_model_name(fingerprint, passive):
  source = Path('openpilot/selfdrive/modeld/modeld.py')
  tree = ast.parse(source.read_text(encoding='utf-8'))
  call = next(n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'DesireHelper')
  cp = NS(brand='hyundai', passive=passive, carFingerprint=fingerprint)
  expr = next(k.value for k in call.keywords if k.arg == 'boundary_indicator_supported')
  assert eval(compile(ast.Expression(expr), str(source), 'eval'), {'CP': cp}) == (not passive)


@pytest.mark.parametrize('camera', [False, True])
@pytest.mark.parametrize('length', [None, 8, 16, 32])
def test_hold_capability_follows_camera_message(camera, length):
  from opendbc.car.hyundai.values import HyundaiExtFlags
  source = Path('opendbc_repo/opendbc/car/hyundai/interface.py')
  tree = ast.parse(source.read_text(encoding='utf-8'))
  node = next(n for n in ast.walk(tree) if isinstance(n, ast.If) and 'CANFD_INDICATOR_HOLD' in ast.unparse(n) and len(n.body) == 1)
  ret = NS(flags=HyundaiFlags.CAMERA_SCC if camera else 0, extFlags=0)
  exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),
       {'ret':ret, 'fingerprint':{2:{0x1EA:length}}, 'CAN':NS(CAM=2), 'HyundaiFlags':HyundaiFlags, 'HyundaiExtFlags':HyundaiExtFlags})
  assert bool(ret.extFlags & HyundaiExtFlags.CANFD_INDICATOR_HOLD) == (camera and length == 32)
