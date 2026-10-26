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
  for i in range(3):
    setattr(sm['pandaStates'][0], f'canState{i}', NS(busOff=False, errorWarning=False, errorPassive=False,
            totalErrorCnt=0, busOffCnt=0, totalTxLostCnt=0, totalRxLostCnt=0))
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


@pytest.mark.parametrize('setting', [11, 12])
def test_phased_start_is_fresh_bounded_then_existing_hold(monkeypatch, setting):
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.update(cs, sm, 10., True)
  p.value = setting
  s.settings_at = 0
  assert s.update(cs, sm, 10.01, True) == []  # cannot send against old stock
  assert s.test_until == pytest.approx(20.01)
  packets = [(a, frame(a, 10, None, 0).dat, 0) for a in (0x165, 0x16A)]
  s.observe([(10_020_000_000, packets)], 10.021)
  sent = s.update(cs, sm, 10.022, True)
  assert [m.address for m in sent] == [0x165, 0x16A]
  assert all(m.dat[2] == 11 for m in sent)
  assert sent[1].dat[16] == (0x60 if setting == 11 else 0x80)
  assert s.hold_value == 1  # no lamp acknowledgement yet
  setattr(cs, ('left' if setting == 11 else 'right')+'Blinker', True)
  for i in range(1, 101):
    now = 10.022+i*.1
    warm(s, now)
    s.stock_received = dict.fromkeys((0x165, 0x16A), now)
    s.driver_at = now
    sm.recv_time = dict.fromkeys(sm, now)
    sent = s.update(cs, sm, now, True)
    assert bool(sent) == (i == 1)
    assert s.hold_value == ((2 if setting == 11 else 3) if now < 20.01 else 1)


@pytest.mark.parametrize('age', [-.001, .006, .020])
def test_phased_start_does_not_send_on_future_or_delayed_stock(monkeypatch, age):
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.update(cs, sm, 10., True)
  p.value = 11
  s.settings_at = 0
  assert s.update(cs, sm, 10.01, True) == []
  warm(s, 10.05)
  s.stock_received = dict.fromkeys((0x165, 0x16A), 10.05-age)
  assert s.update(cs, sm, 10.05, True) == []


@pytest.mark.parametrize('setting', [11, 12])
def test_phased_pulse_uses_new_receipt_once_and_retains_counter_rate(monkeypatch, setting):
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.update(cs, sm, 10., True)
  p.value = setting
  s.settings_at = 0
  s.update(cs, sm, 10.01, True)
  frames = []
  for i in range(30):
    now = 10.02+i*.02
    packets = [(a, frame(a, 10+i, None, 0).dat, 0) for a in (0x165, 0x16A)]
    s.observe([(round(now*1e9), packets)], now+.001)
    s.driver_at = now
    sm.recv_time = dict.fromkeys(sm, now)
    sent = s.update(cs, sm, now+.002, True)
    frames.extend(sent)
    assert s.update(cs, sm, now+.003, True) == []  # same RX cannot retrigger
  assert len(frames) == 50
  for address in (0x165, 0x16A):
    assert [m.dat[2] for m in frames if m.address == address] == list(range(11, 36))


@pytest.mark.parametrize('setting', [13, 14])
def test_phased_continuous_ten_seconds_without_existing_hold(monkeypatch, setting):
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.update(cs, sm, 10., True)
  p.value = setting
  s.settings_at = 0
  assert s.update(cs, sm, 10.01, True) == []
  assert not s.test_pulse and not s.test_hold
  assert s.test_until == pytest.approx(20.01)
  setattr(cs, ('left' if setting == 13 else 'right')+'Blinker', True)
  packets = []
  for i in range(510):
    now = 10.02+i*.02
    stock = [(a, frame(a, (10+i)%256, None, 0).dat, 0) for a in (0x165, 0x16A)]
    s.observe([(round(now*1e9), stock)], now+.001)
    s.driver_at = now
    sm.recv_time = dict.fromkeys(sm, now)
    sent = s.update(cs, sm, now+.002, True)
    assert bool(sent) == (i < 500)
    packets.extend(sent)
    assert s.hold_value == 1
    assert s.update(cs, sm, now+.003, True) == []
  for address in (0x165, 0x16A):
    selected = [m for m in packets if m.address == address]
    assert [m.dat[2] for m in selected] == [(11+i)%256 for i in range(500)]
    if address == 0x16A:
      assert all(m.dat[16] == (0x60 if setting == 13 else 0x80) for m in selected)
  assert s.test_request is None


@pytest.mark.parametrize('field', ['totalErrorCnt', 'busOffCnt', 'totalTxLostCnt', 'totalRxLostCnt', 'errorWarning', 'errorPassive', 'busOff'])
@pytest.mark.parametrize('controller', [0, 1, 2])
def test_direct_trial_stops_on_any_physical_can_health_change(monkeypatch, field, controller):
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.update(cs, sm, 10., True)
  p.value = 5
  s.settings_at = 0
  assert len(s.update(cs, sm, 10.01, True)) == 2
  health = getattr(sm['pandaStates'][0], f'canState{controller}')
  old = getattr(health, field)
  setattr(health, field, True if isinstance(old, bool) else old+1)
  assert s.update(cs, sm, 10.04, True) == []
  assert s.hold_value == 1
  setattr(health, field, old)
  assert s.update(cs, sm, 10.05, True) == []  # no replay after recovery


@pytest.mark.parametrize('fault', ['warning', 'missing'])
def test_direct_trial_never_starts_with_unhealthy_or_missing_can_health(monkeypatch, fault):
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.update(cs, sm, 10., True)
  if fault == 'warning':
    sm['pandaStates'][0].canState2.errorWarning = True
  else:
    del sm['pandaStates'][0].canState2
  p.value = 13
  s.settings_at = 0
  assert s.update(cs, sm, 10.01, True) == []
  assert s.test_request is None


@pytest.mark.parametrize('setting', [15, 16])
@pytest.mark.parametrize('stop', ['expiry', 'can_error', 'moving', 'cancel'])
def test_patch_mode_uses_original_uds_and_continues_without_stock_then_restores(monkeypatch, setting, stop):
  from opendbc.car.hyundai.spas import APRK, PATCH_DISABLE, DEFAULT
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.update(cs, sm, 10., True, 90)
  p.value = setting
  s.settings_at = 0
  assert s.update(cs, sm, 10.01, True, 91)[0].dat == EXTENDED
  assert s.update(cs, sm, 10.06, True, 96)[0].dat == PATCH_DISABLE
  # No ACK and no fresh stock after arm: original protocol still transmits.
  sent = []
  for i in range(97, 1093):
    now = 10.01+(i-91)*.01
    sm.recv_time = dict.fromkeys(sm, now)
    s.driver_at = now
    if stop != 'expiry' and i == 130:
      if stop == 'can_error': sm['pandaStates'][0].canState2.totalErrorCnt += 1
      if stop == 'moving': cs.vEgo = .1
      if stop == 'cancel': p.value = 0; s.settings_at = 0
    batch = s.update(cs, sm, now, True, i)
    sent.extend(batch)
    assert s.hold_value == 1
  assert sent[-1].address == APRK and sent[-1].dat == DEFAULT
  assert sum(m[0] == APRK and m[1] == DEFAULT for m in sent) == 1
  assert any(m[0] == 0x16A and m[1][16] == (0x60 if setting == 15 else 0x80) for m in sent)
  assert s.patch.state == 0 and s.test_request is None


@pytest.mark.parametrize('fault', ['none', 'cancel', 'driver', 'stale', 'error', 'knob', 'timeout'])
def test_stalk_trial_isolated_and_revoked(monkeypatch, fault):
  s, p, cs, sm = setup(monkeypatch)
  warm(s)
  s.driver_data = bytes.fromhex('f880300000000000')
  s.update(cs, sm, 10., True)
  p.value = 17
  s.settings_at = 0
  sent = s.update(cs, sm, 10.01, True)
  assert len(sent) == 1 and sent[0].address == 0x3C1
  assert sent[0].dat.hex() == 'de90304000000000'
  assert s.hold_value == 1 and s.session.state == 'idle'
  assert s.update(cs, sm, 10.02, True) == []
  if fault == 'cancel':
    p.value = 0
    s.settings_at = 0
  elif fault == 'driver':
    cs.blinkerLever = 2
  elif fault == 'stale':
    s.driver_at = 0
  elif fault == 'error':
    sm['pandaStates'][0].canState2.totalErrorCnt = 1
  elif fault == 'knob':
    s.driver_data = bytes(8)
  elif fault == 'timeout':
    s.test_until = 10.05
  sent = s.update(cs, sm, 10.07, True)
  assert bool(sent) == (fault == 'none')
  if fault != 'none':
    assert s.test_request is None
    assert s.update(cs, sm, 10.13, True) == []
