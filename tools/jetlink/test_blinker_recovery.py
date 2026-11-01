"""Regressions for recorded approach completion and bounded lamp recovery."""
import pytest

from blink_policy import AutomaticSignals, Policy, MAX_HOLD
from test_blinker_auto import update


@pytest.mark.parametrize('direction,sign', [('left', 1), ('right', -1)])
def test_approach_bend_does_not_complete_turn_still_ahead(direction, sign):
  a = AutomaticSignals()
  a.update(10, 1, True, 0., 'none', 'none', 'none')
  def step(t, distance, angle=0., heading=0., speed=3., atc=None):
    return a.update(t, 1, True, 11., direction, atc or 'turn '+direction, 'none',
                    distance=distance, angle=angle*sign, heading=heading*-sign, speed=speed)
  assert step(11, 42)
  assert step(12, 2, 250, 1.)
  assert step(13, 25, 5, 1.37)
  assert step(14, 19, 5, 1.37)
  assert step(15, 17, 5, 1.37, speed=0.)
  assert step(75, 17, 5, 1.37, speed=0.)
  assert step(76, 5, 200, 2.)
  assert step(77, 1000, 5, 3., atc='straight')
  assert step(77.41, 1000, 5, 3., atc='straight') is None


def owned_turn():
  p = Policy()
  update(p)
  update(p, t=11., automatic=('turn', 'left'))
  return p


def test_relight_once_only_after_confirmed_sustained_lamp_loss():
  p = owned_turn()
  old = p.hold['id']
  assert not p.relight(12., old, True, False, False)
  assert not p.relight(12.6, old, True, True, False)  # normal flash resets absence
  assert not p.relight(13., old, True, False, False)
  assert not p.relight(14.49, old, True, False, False)
  assert p.relight(14.51, old, True, False, False)
  new = p.hold['id']
  assert new != old and p.hold['time'] == 14.51 and p.started == 11.
  assert not p.relight(20., new, True, False, False)
  assert not p.relight(30., new, True, False, False)
  assert update(p, t=11.+MAX_HOLD, automatic=('turn', 'left'))[1] is None


@pytest.mark.parametrize('fault', ['unconfirmed', 'stale', 'opposite', 'manual', 'lane', 'cancel', 'driver', 'expiry'])
def test_relight_does_not_override_other_conditions(fault):
  p = owned_turn()
  ident = p.hold['id']
  if fault in ('manual', 'lane'):
    p.hold['source'] = 'bluetooth' if fault == 'manual' else 'lane'
  elif fault == 'cancel':
    update(p, t=12., automatic=None)
  elif fault == 'driver':
    p.acknowledged(12., ident, driver_override=True)
  for t in (12., 14., 20.):
    assert not p.relight(t if fault != 'expiry' else 312., None if fault == 'unconfirmed' else ident,
                         fault != 'stale', False, fault == 'opposite')


def test_loss_of_ack_restarts_absence_timer():
  p = owned_turn()
  ident = p.hold['id']
  assert not p.relight(12., ident, True, False, False)
  assert not p.relight(13., ident, False, False, False)
  assert not p.relight(14., ident, True, False, False)
  assert not p.relight(15., ident, True, False, False)
  assert p.relight(15.51, ident, True, False, False)


def test_relight_uses_existing_one_touch_receiver_and_new_ack(tmp_path):
  from test_blinker import setup
  clock, bus, receiver = setup(tmp_path)
  p = owned_turn()
  def deliver():
    h = p.hold
    receiver.accept({'token':'session', 'ok':True, 'guard_deadline':clock[0]+.3, 'events':[],
                     'hold':{**h, 'deadline':h['time']+.39}})
    receiver.tick()
  def finish():
    start = receiver.pending['start']
    while receiver.pending is not None:
      offset, _ = receiver.pending['schedule'][receiver.pending['index']]
      clock[0] = max(clock[0], start+offset)
      bus.latest[0x3c1] = (clock[0], bus.original)
      deliver()
  deliver()
  finish()
  first = p.hold['id']
  assert receiver.status['accepted_id'] == first and len(bus.sent) == 6
  assert not p.relight(12., first, True, False, False)
  assert p.relight(13.51, first, True, False, False)
  clock[0] = 13.51
  bus.latest[0x3c1] = (clock[0], bus.original)
  deliver()
  assert receiver.status['accepted_id'] is None
  finish()
  assert receiver.status['accepted_id'] == p.hold['id'] != first
  assert len(bus.sent) == 12 and bus.modes == [True, False, True, False]


def test_diagnostic_transitions_are_bounded_and_excluded_from_wire(monkeypatch):
  import json
  from types import SimpleNamespace as NS
  import blink_protocol as protocol
  clock, sent, records = [1.], [], []
  monkeypatch.setattr(protocol.time, 'monotonic', lambda:clock[0])
  source = {'stamp':1., 'ok':True, 'events':[], 'activated':True, 'relights':1,
            'turn_phase':'approach', 'turn_position':True,
            'hold':{'id':'secret-id', 'time':1., 'source':'turn', 'direction':'left'}}
  monkeypatch.setattr(protocol, 'Process', lambda *a:NS(latest=lambda:source, close=lambda:None, send=lambda _:True))
  client = NS(last_state={'carrot_blinker':{'token':'secret', 'clock':100., 'state':'ready', 'accepted_id':'secret-id'}},
              _next_seq=lambda:1, _expect=lambda *a:NS(payload=protocol.encode({'ready':True,'token':'secret','clock':100.})),
              t=NS(send_json=lambda *a:None, send=lambda *a:sent.append(a)))
  sender = protocol.Sender(client, {'carrot_host':'jetson', protocol.CAPABILITY:True})
  sender.report = lambda name, **kw:records.append((name, kw))
  for t in (1., 1.1, 1.3):
    clock[0] = t
    source['stamp'] = t
    sender.send(client)
  assert len(records) == 1 and records[0][1]['accepted'] and records[0][1]['activated']
  assert 'secret' not in str(records)
  for _, _, parts in sent:
    wire = json.loads(parts[0])
    assert 'activated' not in wire and 'relights' not in wire
    assert 'turn_phase' not in wire and 'turn_position' not in wire
  clock[0] = 1.6
  source['stamp'] = 1.6
  client.last_state['carrot_blinker']['state'] = 'late_request'
  sender.send(client)
  assert records[-1][1]['peer_state'] == 'late_request'


def test_completed_receiver_send_can_retry_initial_unlit_once():
  p = owned_turn()
  original = p.hold['id']
  assert not p.relight(11.4, None, True, False, False, retry_unlit=True)
  assert not p.relight(12.89, None, True, False, False, retry_unlit=True)
  assert p.relight(12.91, None, True, False, False, retry_unlit=True)
  assert p.hold['id'] != original and p.started == 11
  assert not p.relight(15, None, True, False, False, retry_unlit=True)
  assert p.acknowledged(16.92, None) is None


@pytest.mark.parametrize('fault', ['rejected', 'opposite', 'cancelled', 'driver', 'stale', 'lane'])
def test_initial_retry_never_bypasses_ownership_or_ack(fault):
  p = owned_turn()
  if fault == 'cancelled':
    update(p, t=11.5, automatic=None)
  if fault == 'driver':
    p.acknowledged(11.5, None, True)
  if fault == 'lane':
    p.hold['source'] = 'lane'
  for t in (12, 14, 16):
    assert not p.relight(t, None, fault not in ('rejected', 'stale'), False, fault == 'opposite', retry_unlit=True)


@pytest.mark.parametrize('sign,direction,next_direction', [(1, 'left', 'right'), (-1, 'right', 'left')])
def test_next_turn_cue_waits_for_physical_completion(sign, direction, next_direction):
  a, p = AutomaticSignals(), Policy()
  a.update(10, 1, True, 0, 'none', '', 'none')
  update(p, t=10)
  def step(t, cue, d, angle, heading):
    auto = a.update(t, 1, True, cue, d, 'turn '+d, 'turn'+d.title(),
                    angle=angle*sign, heading=heading*-sign, speed=5, distance=20)
    return update(p, t=t, automatic=auto)[1]
  assert step(11, 11, direction, 0, 0)['direction'] == direction
  assert step(11.5, 11, direction, 60, .6)['direction'] == direction
  assert step(12, 12, next_direction, 50, .8)['direction'] == direction
  assert step(12.5, 12, next_direction, 5, 1)['direction'] == direction
  assert step(12.91, 12, next_direction, 5, 1)['direction'] == next_direction


def test_next_turn_pending_expires_and_never_replays():
  a = AutomaticSignals()
  a.update(10, 1, True, 0, 'none', '', 'none')
  a.update(11, 1, True, 11, 'left', 'turn left', 'none')
  a.update(11.5, 1, True, 11, 'left', 'turn left', 'none', speed=5, angle=60, heading=-.6)
  a.update(12, 1, True, 12, 'right', 'turn right', 'none', speed=5, angle=50, heading=-.8)
  assert a.pending
  a.update(15.1, 1, True, 12, 'right', 'turn right', 'none', speed=5, angle=5, heading=-1)
  assert a.pending is None
  assert a.update(15.6, 1, True, 12, 'right', 'turn right', 'none', speed=5, angle=5, heading=-1) is None
  assert a.update(16, 1, True, 12, 'right', 'turn right', 'none') is None
