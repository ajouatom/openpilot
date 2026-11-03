import ast
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from opendbc.car import CanData
from opendbc.car.hyundai.spas import APRK, RESPONSE, EXTENDED, DEFAULT
from opendbc.car.hyundai.values import HyundaiFlags as F
from openpilot.selfdrive.car.aprk_diag import AprkDiag, KEY, RESULT, CONFLICTS, RESERVED


class Params(dict):
  def get_bool(self, key):
    return bool(self.get(key, False))

  def get_int(self, key):
    return int(self.get(key, 0))

  def put_bool(self, key, value):
    self[key] = value

  def put_nonblocking(self, key, value):
    self[key] = value


class Messages(dict):
  pass


@pytest.fixture
def rig():
  params = Params()
  cp = NS(brand='hyundai', flags=F.CANFD | F.CANFD_HDA2 | F.CANFD_CAMERA_SCC, passive=False,
          safetyConfigs=[NS(safetyModel='hyundaiCanfd', safetyParam=136)])
  cs = NS(canValid=True, gearShifter='park', standstill=True, vEgo=0., gasPressed=False)
  panda = NS(controlsAllowed=False, safetyTxBlocked=0, safetyModel='hyundaiCanfd', safetyParam=136)
  for i in range(3):
    setattr(panda, f'canState{i}', NS(busOff=False, errorWarning=False, errorPassive=False,
                                   busOffCnt=0, totalErrorCnt=0, totalTxLostCnt=0, totalRxLostCnt=0))
  sm = Messages(carControl=NS(enabled=False, latActive=False, longActive=False),
                selfdriveState=NS(enabled=False, active=False), pandaStates=[panda])
  sm.valid, sm.alive, sm.recv_time = dict.fromkeys(sm, True), dict.fromkeys(sm, True), dict.fromkeys(sm, 10.)
  events, sent = [], []
  diag = AprkDiag(cp, params, 'mici', lambda name, **data: events.append((name, json.dumps(data))))

  def tick(now, **kwargs):
    sm.recv_time = dict.fromkeys(sm, now)
    packets = diag.update(cs, sm, now, kwargs.pop('can_now', now), kwargs.pop('initialized', True), **kwargs)
    sent.extend(packets)
    # These are the only two permitted diagnostic operations in every test path.
    assert all(m.address == APRK and m.dat in (EXTENDED, DEFAULT) and m.src in (0, 1) for m in packets)
    return packets

  def receive(now, data='065003003201f400', bus=None, address=RESPONSE):
    diag.observe([(round(now * 1e9), [(address, bytes.fromhex(data), diag.bus if bus is None else bus)])], now)

  return NS(diag=diag, params=params, cp=cp, cs=cs, sm=sm, panda=panda, events=events, sent=sent, tick=tick, receive=receive)


def test_off_then_complete_once_on_each_bus(rig):
  r = rig
  assert r.tick(10.) == []
  assert not r.events
  r.params[KEY] = True
  assert r.tick(10.2) == [CanData(APRK, EXTENDED, 0)]
  r.receive(10.21, EXTENDED.hex(), bus=128, address=APRK)
  r.receive(10.22, '0000000000000000', address=0x16A)
  r.receive(10.23)
  assert r.tick(10.24) == [CanData(APRK, DEFAULT, 0)]
  r.receive(10.25, '065001003201f400')
  assert r.tick(10.26) == [CanData(APRK, EXTENDED, 1)]
  r.receive(10.27)
  assert r.tick(10.28) == [CanData(APRK, DEFAULT, 1)]
  r.receive(10.29, '065001003201f400')
  assert r.tick(10.3) == []
  assert not r.params[KEY] and not r.diag.busy
  result = json.loads(r.params[RESULT])
  assert result['status'] == 'complete'
  assert result['buses'][0]['session']['echo'] == result['buses'][0]['stock_frames'] == 1
  assert all(b['session']['status'] == b['restore']['status'] == 'positive' for b in result['buses'])
  assert r.tick(12.) == []
  assert len(r.sent) == 4


@pytest.mark.parametrize('device,brand,flags,passive', [
  ('tici', 'hyundai', F.CANFD | F.CANFD_HDA2 | F.CANFD_CAMERA_SCC, False),
  ('tizi', 'hyundai', F.CANFD | F.CANFD_HDA2 | F.CANFD_CAMERA_SCC, False),
  ('pc', 'hyundai', F.CANFD | F.CANFD_HDA2 | F.CANFD_CAMERA_SCC, False),
  ('mici', 'toyota', F.CANFD | F.CANFD_HDA2 | F.CANFD_CAMERA_SCC, False),
  ('mici', 'hyundai', F.CANFD | F.CANFD_CAMERA_SCC, False),
  ('mici', 'hyundai', F.CANFD | F.CANFD_HDA2, False),
  ('mici', 'hyundai', F.CANFD | F.CANFD_HDA2 | F.CANFD_CAMERA_SCC, True),
  ('mici', 'hyundai', F.CANFD | F.CANFD_HDA2 | F.CANFD_CAMERA_SCC | F.CANFD_SPAS_BLINKERS, False),
  ('mici', 'hyundai', F.CANFD | F.CANFD_HDA2 | F.CANFD_CAMERA_SCC | F.ENABLE_BLINKERS, False),
])
def test_unsupported_consumes_request_without_sending(rig, device, brand, flags, passive):
  r = rig
  r.cp.brand, r.cp.flags, r.cp.passive = brand, flags, passive
  diag = AprkDiag(r.cp, r.params, device, lambda *a, **k: None)
  r.params[KEY] = True
  assert diag.update(r.cs, r.sm, 10., 10., True) == []
  assert not r.params[KEY]
  assert diag.result['status'] == 'unsupported_vehicle_or_device'


@pytest.mark.parametrize('bad', ['drive', 'moving', 'nan', 'standstill', 'gas', 'can', 'cc', 'ss', 'controls',
                                'busoff', 'warning', 'passive', 'safety', 'safety_param', 'two_pandas',
                                'not_ready', 'owner', 'stale', 'invalid'])
def test_invalid_requests_cannot_start_later(rig, bad):
  r = rig
  r.params[KEY] = True
  if bad == 'drive':
    r.cs.gearShifter = 'drive'
  elif bad in ('moving', 'nan'):
    r.cs.vEgo = .1 if bad == 'moving' else float('nan')
  elif bad in ('standstill', 'can'):
    setattr(r.cs, 'standstill' if bad == 'standstill' else 'canValid', False)
  elif bad == 'gas':
    r.cs.gasPressed = True
  elif bad == 'cc':
    r.sm['carControl'].latActive = True
  elif bad == 'ss':
    r.sm['selfdriveState'].enabled = True
  elif bad == 'controls':
    r.panda.controlsAllowed = True
  elif bad in ('busoff', 'warning', 'passive'):
    setattr(r.panda.canState0, {'busoff': 'busOff', 'warning': 'errorWarning', 'passive': 'errorPassive'}[bad], True)
  elif bad == 'safety':
    r.panda.safetyModel = 'noOutput'
  elif bad == 'safety_param':
    r.panda.safetyParam += 1
  elif bad == 'two_pandas':
    r.sm['pandaStates'].append(r.panda)
  elif bad == 'stale':
    r.sm.recv_time['carControl'] = 0.
  elif bad == 'invalid':
    r.sm.valid['pandaStates'] = False
  assert r.diag.update(r.cs, r.sm, 10., 10., bad != 'not_ready', owner_busy=bad == 'owner') == []
  assert not r.params[KEY] and not r.diag.busy
  assert r.tick(11.) == []


@pytest.mark.parametrize('key', CONFLICTS)
def test_other_test_settings_are_exclusive(rig, key):
  r = rig
  r.params.update({KEY: True, key: 1})
  assert r.tick(10.) == []
  assert r.diag.result['status'] == 'other_spas_test_active'


def test_timeout_distinguishes_echo_rejection_and_wrong_bus_from_reply(rig):
  r = rig
  r.params[KEY] = True
  r.tick(10.)
  r.receive(10.01, EXTENDED.hex(), bus=128, address=APRK)
  r.receive(10.02, EXTENDED.hex(), bus=192, address=APRK)
  r.receive(10.03, bus=1)
  r.receive(9.9)  # old packet before the request
  r.receive(10.04, '105003003201f400')  # not an ISO-TP single frame
  assert r.tick(10.5) == []
  assert r.tick(10.81) == [CanData(APRK, DEFAULT, 0)]
  assert r.tick(11.62) == [CanData(APRK, EXTENDED, 1)]
  assert r.tick(12.43) == [CanData(APRK, DEFAULT, 1)]
  assert r.tick(13.24) == []
  assert not r.params[KEY]
  first = r.diag.result['buses'][0]['session']
  assert first['status'] == 'timeout' and first['echo'] == first['rejected'] == 1
  assert r.diag.result['status'] == 'complete_restore_unconfirmed'


def test_negative_response_is_recorded_and_restored(rig):
  r = rig
  r.params[KEY] = True
  r.tick(10.)
  r.receive(10.1, '037f101200000000')
  assert r.tick(10.11) == [CanData(APRK, DEFAULT, 0)]
  assert r.diag.result['buses'][0]['session']['nrc'] == 0x12


def test_pending_has_a_hard_deadline(rig):
  r = rig
  r.params[KEY] = True
  r.tick(10.)
  for now in (10.5, 11., 11.5, 11.9):
    r.receive(now, '037f107800000000')
    assert r.tick(now) == []
  assert r.tick(12.01) == [CanData(APRK, DEFAULT, 0)]


@pytest.mark.parametrize('phase', ['session', 'restore'])
@pytest.mark.parametrize('cancel', ['switch', 'move', 'controls', 'counter', 'blocked', 'conflict', 'stale'])
def test_cancel_restores_current_bus_only(rig, phase, cancel):
  r = rig
  r.params[KEY] = True
  r.tick(10.)
  if phase == 'restore':
    r.receive(10.01)
    r.tick(10.02)
  if cancel == 'switch':
    r.params[KEY] = False
  elif cancel == 'move':
    r.cs.gearShifter = 'drive'
  elif cancel == 'controls':
    r.sm['carControl'].enabled = True
  elif cancel == 'counter':
    r.panda.canState1.totalErrorCnt += 1
  elif cancel == 'blocked':
    r.panda.safetyTxBlocked += 1
  elif cancel == 'conflict':
    r.params['CanfdBlinkerKick'] = True
  else:
    r.sm.alive['carControl'] = False
  r.tick(10.2)
  r.tick(11.1)
  assert r.sent == [CanData(APRK, EXTENDED, 0), CanData(APRK, DEFAULT, 0)]
  assert not r.params[KEY] and not r.diag.busy
  assert r.diag.result['buses'][0]['restore']['status'] == 'timeout'
  assert 'session' not in r.diag.result['buses'][1]


def test_unconfirmed_restore_after_positive_ack_stops_bus_scan(rig):
  r = rig
  r.params[KEY] = True
  r.tick(10.)
  r.receive(10.01)
  r.tick(10.02)
  r.tick(10.9)
  assert len(r.sent) == 2
  assert not r.params[KEY] and r.diag.result['status'] == 'restore_unconfirmed'


def test_registered_key_and_lab_menu():
  root = Path(__file__).resolve().parents[3]
  keys = (root / 'common/params_keys.h').read_text(encoding='utf-8')
  assert '"CanfdAprkDiag", {CLEAR_ON_MANAGER_START, BOOL, "0"}' in keys
  assert '"CanfdAprkDiagResult", {PERSISTENT, STRING, "{}"}' in keys
  catalog = json.loads((root / 'selfdrive/carrot_settings.json').read_text(encoding='utf-8'))
  def objects(value):
    if isinstance(value, dict):
      yield value
      for child in value.values():
        yield from objects(child)
    elif isinstance(value, list):
      for child in value:
        yield from objects(child)
  lab = next(o for o in objects(catalog) if o.get('id') == 'VEH_TEST')
  assert lab['ko'] == 'Joongyu Lab' and KEY in lab['params']
  setting = next(o for o in objects(catalog) if o.get('name') == KEY)
  assert setting['group'] == 'Joongyu Lab' and setting['default'] == 0


def test_card_filters_conflicting_frames_only_during_probe():
  path = Path(__file__).parents[1] / 'card.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'controls_update')
  guard = next(n for n in ast.walk(method) if isinstance(n, ast.If) and ast.unparse(n.test) == 'self.aprk_diag.busy')
  packets = [CanData(addr, b'\x00' * 8, 0) for addr in (*RESERVED, 0x12A, 0x1A0)]
  for busy in (True, False):
    ns = {'self': NS(aprk_diag=NS(busy=busy)), 'can_sends': packets, 'APRK_DIAG_RESERVED': RESERVED}
    exec(compile(ast.Module(body=[guard], type_ignores=[]), str(path), 'exec'), ns)
    assert ns['can_sends'] == (packets[-2:] if busy else packets)


@pytest.mark.parametrize('mode', ['normal', 'passive', 'startup', 'owner', 'steering_owner'])
def test_card_step_runs_probe_and_consumes_invalid_requests(rig, mode):
  from openpilot.cereal import log
  from openpilot.selfdrive.pandad import can_list_to_can_capnp
  r = rig
  r.cp.passive = mode == 'passive'
  r.diag = AprkDiag(r.cp, r.params, 'mici', lambda *a, **k: None)
  r.params[KEY] = True
  r.cs.buttonEvents = []
  r.sm['onroadEvents'] = []
  r.sm.seen = {'onroadEvents': True}
  sent, controls = [], []
  controller = NS(CC=NS(aprk_state=1 if mode == 'owner' else 0, steer_angle_test=30 if mode == 'steering_owner' else 0))
  card = NS(CP=r.cp, CI=controller, aprk_diag=r.diag, sm=r.sm, initialized_prev=mode != 'startup',
            state_update=lambda: r.cs, state_publish=lambda cs: None,
            cruise_main_toggle=NS(update=lambda *a: False), card_diag_stage_current={}, card_diag_recv_ns=10_000_000_000,
            pm=NS(send=lambda name, msg: sent.append(msg)), runtime_diagnostics=NS(record=lambda **k: None),
            controls_update=lambda *a: controls.append(a))
  path = Path(__file__).parents[1] / 'card.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'step')
  ns = {'time': NS(monotonic=lambda: 10., monotonic_ns=lambda: 10_000_000_000, thread_time=lambda: 0.,
                   clock_gettime=lambda clock: 70., CLOCK_BOOTTIME=7), 'log': log,
        'EventName': NS(selfdriveInitializing='initializing'), 'CAN_BACKEND': 'test',
        'can_list_to_can_capnp': can_list_to_can_capnp}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), ns)
  ns['step'](card)
  if mode == 'normal':
    assert len(sent) == 1
    assert sent[0].logMonoTime == 70_000_000_000
    assert [(m.address, m.dat, m.src) for m in sent[0].sendcan] == [(APRK, EXTENDED, 0)]
  else:
    assert sent == []
  assert r.params[KEY] == (mode == 'normal')
  assert len(controls) == (0 if mode == 'passive' else 1)


def test_probe_after_suspend_uses_can_clock_and_cancels_on_new_suspend(rig):
  r = rig
  r.params[KEY] = True
  assert r.tick(10., can_now=70.) == [CanData(APRK, EXTENDED, 0)]
  r.receive(70.01, EXTENDED.hex(), bus=128, address=APRK)
  r.receive(70.02)
  assert r.tick(10.03, can_now=70.03) == [CanData(APRK, DEFAULT, 0)]
  assert r.diag.result['buses'][0]['session']['status'] == 'positive'
  assert r.diag.result['buses'][0]['session']['echo'] == 1
  r.tick(10.04, can_now=80.04)
  assert not r.diag.busy and not r.params[KEY]
  assert r.diag.result['status'] == 'suspend_during_probe'
  assert len(r.sent) == 2


def test_new_steering_test_is_inhibited_until_diag_finishes():
  path = Path(__file__).resolve().parents[4] / 'opendbc_repo/opendbc/car/hyundai/carcontroller.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  expr = next(n.value for n in ast.walk(tree) if isinstance(n, ast.Assign) and
              any(isinstance(t, ast.Name) and t.id == 'steer_test' for t in n.targets))
  for busy in (True, False):
    ns = {'self': NS(aprk_diag_active=busy, steer_angle_test=30), 'angle_control': True,
          'CS': NS(out=NS(standstill=True, steeringPressed=False)), 'park_assist_active': lambda cs: False}
    assert eval(compile(ast.Expression(body=expr), str(path), 'eval'), ns) == (not busy)
