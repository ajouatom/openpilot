from types import SimpleNamespace as NS
import pytest

import openpilot.selfdrive.car.hyundai_spas as h
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.car.hyundai.spas import frame, EXTENDED


class Messages(dict):
  pass


def setup(monkeypatch):
  monkeypatch.setattr(h, 'CanBus', lambda cp: NS(ECAN=0))
  params = NS(value=0, get_int=lambda key: params.value)
  cp = NS(brand='hyundai', flags=HyundaiFlags.CANFD_SPAS_BLINKERS, passive=False, carFingerprint='HYUNDAI_IONIQ_5_PE')
  sig = h.SpasSignals(cp, params)
  sig.worker = NS(latest=lambda: None, send=lambda value: None)
  cs = NS(
    canValid=True,
    vEgo=0.0,
    gearShifter='park',
    standstill=True,
    leftBlinker=False,
    rightBlinker=False,
    blinkerLever=0,
    leftBlinkerStalkCount=0,
    rightBlinkerStalkCount=0,
  )
  sm = Messages(carControl=NS(latActive=False, longActive=False), selfdriveState=NS(active=False, enabled=False), pandaStates=[NS(controlsAllowed=False)])
  sm.valid = sm.alive = dict.fromkeys(sm, True)
  sm.recv_time = dict.fromkeys(sm, 10.0)
  sig.driver_at = 10.0
  sig.driver_count = 3
  return sig, params, cs, sm


def warm(sig):
  for i in range(10):
    for addr in (0x165, 0x16A):
      m = frame(addr, i, None, 0)
      sig.session.observe(addr, 0, m.dat, 9.82 + i * 0.02)


@pytest.mark.parametrize('setting', [1, 2, 3, 4])
def test_parked_trial_is_one_shot_and_three_seconds(monkeypatch, setting):
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.update(cs, sm, 10.0, True)
  p.value = setting
  # Keep stock fresh until the settings poll; driver data is fresh too.
  warm(s)
  s.settings_at = 0
  sent = s.update(cs, sm, 10.01, True)
  assert sent[0].dat == EXTENDED
  assert s.test_until == 13.01
  assert s.test_direction == ('left' if setting in (1, 3) else 'right')
  assert s.test_pulse == (setting >= 3)


@pytest.mark.parametrize('bad', ['moving', 'drive', 'active', 'controls', 'panda', 'stale', 'can', 'stalk', 'other_hold'])
def test_invalid_trial_edge_is_consumed_without_later_start(monkeypatch, bad):
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.update(cs, sm, 10.0, True)
  p.value = 3
  s.settings_at = 0
  if bad == 'moving':
    cs.vEgo = 0.1
  elif bad == 'drive':
    cs.gearShifter = 'drive'
  elif bad == 'active':
    sm['selfdriveState'].active = True
  elif bad == 'controls':
    sm['carControl'].latActive = True
  elif bad == 'panda':
    sm['pandaStates'][0].controlsAllowed = True
  elif bad == 'stale':
    sm.recv_time['carControl'] = 0
  elif bad == 'can':
    cs.canValid = False
  elif bad == 'stalk':
    s.driver_count = 0
  else:
    s.hold_supported = False
  assert s.update(cs, sm, 10.01, True) == []
  assert s.test_request is None
  cs.vEgo = 0
  cs.gearShifter = 'park'
  cs.canValid = True
  sm['selfdriveState'].active = False
  sm['carControl'].latActive = False
  sm['pandaStates'][0].controlsAllowed = False
  sm.recv_time['carControl'] = 10.0
  s.driver_count = 3
  s.hold_supported = True
  assert s.update(cs, sm, 10.02, True) == []


@pytest.mark.parametrize('confirmed', [False, True])
def test_pulse_removes_spas_direction_and_uses_only_confirmed_hold(monkeypatch, confirmed):
  s, p, cs, sm = setup(monkeypatch)
  p.value = s.test_setting = 3
  s.settings_at = 100
  s.test_request = 'park:1'
  s.test_until = 13.0
  s.test_direction = 'left'
  s.test_pulse = True
  s.session.state = 'active'
  s.session.request = 'park:1'
  s.session.direction = 'left'
  s.session.started = 9.9
  s.session.duration = 3.0
  s.session.last_update = 9.99
  s.session.previous = 'park:1'
  s.session.keepalive = 11.0
  cs.leftBlinker = confirmed
  s.pulse_started = 9.7
  msgs = s.update(cs, sm, 10.0, True)
  assert next(m for m in msgs if m.address == 0x16A).dat[16] == 0
  assert s.hold_value == (2 if confirmed else 1)


def test_non_hyundai_or_unselected_has_no_worker_or_bus(monkeypatch):
  monkeypatch.setattr(h, 'CanBus', lambda cp: pytest.fail('disabled path accessed CAN'))
  for brand, flags in [('tesla', HyundaiFlags.CANFD_SPAS_BLINKERS), ('hyundai', 0)]:
    s = h.SpasSignals(NS(brand=brand, flags=flags, passive=False, carFingerprint='x'), None)
    assert not s.enabled and s.update(None, None, 10.0, False) == []
    s.observe([(1, [])], 10.0)


def test_driver_override_restores_instead_of_retaining_pulse_hold(monkeypatch):
  s, p, cs, sm = setup(monkeypatch)
  p.value = s.test_setting = 3
  s.settings_at = 100
  s.test_request = 'park:1'
  s.test_until = 13.0
  s.test_direction = 'left'
  s.test_pulse = True
  s.session.state = 'active'
  s.session.request = 'park:1'
  s.session.direction = 'left'
  s.session.started = 9.9
  s.session.duration = 3.0
  s.session.last_update = 9.99
  s.session.previous = 'park:1'
  s.session.keepalive = 11.0
  cs.leftBlinker = True
  cs.blinkerLever = 1
  sent = s.update(cs, sm, 10.0, True)
  assert s.session.state == 'restore_enable' and s.hold_value == 1
  assert all(m.address == 0x7B1 for m in sent)
