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


def warm(sig, end=10.):
  for i in range(10):
    for addr in (0x165, 0x16A):
      m = frame(addr, i, None, 0)
      sig.session.observe(addr, 0, m.dat, end-.18 + i * .02)


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
  s.test_hold = True
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
  s.test_hold = True
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


def test_actual_idle_stalk_event_counter_is_not_a_cycle_counter(monkeypatch):
  s, _, _, _ = setup(monkeypatch)
  s.driver_count = 0
  s.driver_at = -100.
  data = bytes.fromhex('12b0000000000000')
  for i in range(4):
    s.observe([(1, [(0x3c1, data, 0)])], 10.+i*.2)
  assert s.driver_count == 4 and not s.driver_active
  s.observe([(1, [(0x3c1, bytes([data[0] ^ 1])+data[1:], 0)])], 10.8)
  assert s.driver_count == 0


def test_unrelated_same_address_on_other_bus_cannot_arm_driver_input(monkeypatch):
  s, _, _, _ = setup(monkeypatch)
  s.driver_count = 0
  for i in range(4):
    s.observe([(1, [(0x3c1, bytes.fromhex('12b0000000000000'), 1)])], 10.+i*.2)
  assert s.driver_count == 0


@pytest.mark.parametrize('setting', range(5, 11))
def test_direct_comparison_has_no_uds_and_distinguishes_pulse_hold(monkeypatch, setting):
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.update(cs, sm, 10., True)
  p.value = setting
  s.settings_at = 0
  sent = s.update(cs, sm, 10.01, True)
  assert {m.address for m in sent} == {0x165, 0x16A}
  assert s.session.state == 'idle'
  assert s.direct_cooldown == 40.01
  assert s.hold_value == 1
  direction = 'left' if setting % 2 else 'right'
  setattr(cs, direction+'Blinker', True)
  warm(s, 10.11)
  s.update(cs, sm, 10.11, True)
  warm(s, 10.22)
  s.driver_at = 10.22
  sm.recv_time = dict.fromkeys(sm, 10.22)
  sent = s.update(cs, sm, 10.23, True)
  assert bool(sent) == (setting <= 6)
  assert s.hold_value == ((2 if direction == 'left' else 3) if setting in (7, 8) else 1)
  assert all(m.address in (0x165, 0x16A) for m in sent)
  p.value = 0
  s.settings_at = 0
  assert s.update(cs, sm, 10.24, True) == []
  assert s.hold_value == 1
  p.value = setting
  s.settings_at = 0
  setattr(cs, direction+'Blinker', False)
  assert s.update(cs, sm, 10.25, True) == []  # cooldown


@pytest.mark.parametrize('cause', ['loop_gap', 'panda_rejection'])
def test_direct_trial_stops_on_stall_or_panda_rejection(monkeypatch, cause):
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.update(cs, sm, 10., True)
  p.value = 7
  s.settings_at = 0
  assert len(s.update(cs, sm, 10.01, True)) == 2
  now = 10.24 if cause == 'loop_gap' else 10.04
  warm(s, now)
  sm.recv_time = dict.fromkeys(sm, now)
  s.driver_at = now
  cs.leftBlinker = True
  if cause == 'panda_rejection':
    sm['pandaStates'][0].safetyTxBlocked = 1
  assert s.update(cs, sm, now, True) == []
  assert s.hold_value == 1


@pytest.mark.parametrize('bad', ['moving', 'gear', 'driver', 'stock', 'stale', 'panda', 'lamp', 'unsupported'])
def test_direct_trial_rejects_guard_or_contract_failure(monkeypatch, bad):
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.update(cs, sm, 10., True)
  p.value = 7
  s.settings_at = 0
  if bad == 'moving':
    cs.vEgo = .1
  if bad == 'gear':
    cs.gearShifter = 'drive'
  if bad == 'driver':
    cs.blinkerLever = 1
  if bad == 'stock':
    s.session.stock_direction = 3
  if bad == 'stale':
    sm.recv_time['carControl'] = 0
  if bad == 'panda':
    sm['pandaStates'][0].controlsAllowed = True
  if bad == 'lamp':
    cs.leftBlinker = True
  if bad == 'unsupported':
    s.hold_supported = False
  assert s.update(cs, sm, 10.01, True) == []
  assert s.hold_value == 1


def test_direct_trial_guard_loss_cannot_resume(monkeypatch):
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.update(cs, sm, 10., True)
  p.value = 5
  s.settings_at = 0
  assert len(s.update(cs, sm, 10.01, True)) == 2
  cs.vEgo = .1
  assert s.update(cs, sm, 10.04, True) == []
  cs.vEgo = 0.
  warm(s)
  assert s.update(cs, sm, 10.05, True) == []
  assert s.hold_value == 1


@pytest.mark.parametrize('setting', [7, 8])
@pytest.mark.parametrize('confirmed', [False, True])
def test_ten_second_hold_sends_only_start_pulse_and_expires(monkeypatch, setting, confirmed):
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.update(cs, sm, 10., True)
  p.value = setting
  s.settings_at = 0
  start = 10.01
  assert len(s.update(cs, sm, start, True)) == 2
  assert s.test_until == start+10.
  direction = 'left' if setting == 7 else 'right'
  setattr(cs, direction+'Blinker', confirmed)
  for tick in range(1, 102):
    now = start+tick*.1
    warm(s, now)
    s.driver_at = now
    sm.recv_time = dict.fromkeys(sm, now)
    sent = s.update(cs, sm, now, True)
    if tick >= 3:
      assert sent == []  # no repeated SPAS throughout the hold or after expiry
    assert s.hold_value == ((2 if setting == 7 else 3) if confirmed and tick < 100 else 1)
  assert s.test_request is None  # unchanged selection must not restart
