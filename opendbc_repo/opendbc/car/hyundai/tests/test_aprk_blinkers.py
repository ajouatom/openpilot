from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.carcontroller import CarController

D, P = structs.CarState.GearShifter.drive, structs.CarState.GearShifter.park
EXT, QUIET, DEFAULT = carcontroller.APRK_EXT_SESSION, carcontroller.APRK_COM_DISABLE, carcontroller.APRK_DEFAULT_SESSION
TICK = 10_000_000  # one controls frame in CAN nanos


class Crossing:
  """LaneCrossing stand-in: reports `side` once, on the first allowed call after it is set."""
  def __init__(self):
    self.side = 0

  def update(self, md, t, allowed, torque=None):
    side, self.side = (self.side, 0) if allowed else (0, self.side)
    return side


class Ecu:
  """Parking ECU stand-in: own SPAS frames every 20 ms; answers 10 03 one frame later if `answers`, stops its frames
  after 28 83 01 if `obeys`, resumes on 10 01."""
  def __init__(self, answers=True, obeys=True):
    self.answers, self.obeys = answers, obeys
    self.quiet, self.stock, self.ack = False, 0, 0

  def tick(self, now, sent):
    for addr, dat, _ in sent:
      if addr == carcontroller.APRK_ADDR:
        if dat == EXT and self.answers:
          self.ack = now
        elif dat == QUIET:
          self.quiet = self.obeys
        elif dat == DEFAULT:
          self.quiet = False
    if not self.quiet and now % (2 * TICK) == 0:
      self.stock = now


def make_cc(control=True, test=0, spas=0, ecu=None):
  return SimpleNamespace(blinker_control=control, blinker_test=test, aprk_state=0, aprk_frame=0, aprk_nanos=0,
                         lane_crossing=Crossing(), blinker_side=0, blinker_pulse_end=0, frame=0, yellow_until=[0, 0],
                         CAN=SimpleNamespace(ECAN=0, ACAN=1), packer=CANPacker('hyundai_canfd_generated'),
                         spas_steer_test=spas, spas_angle=0., aprk_buses=(), ecu=ecu or Ecu())


def run(cc, frames, out):
  sent = []
  for _ in range(frames):
    now = (cc.frame + 1) * TICK
    CS = out()
    CS.can_nanos, CS.aprk_stock_nanos, CS.aprk_ack_nanos = now, cc.ecu.stock, cc.ecu.ack
    msgs = CarController.aprk_blinkers(cc, None, CS)
    sent += msgs
    cc.frame += 1
    cc.ecu.tick((cc.frame + 1) * TICK, msgs)
  return sent


def step(cc, frames, gear, kph, cross=0, lamp=False, tunnel=0, roundabout=False, left_line=10, right_line=10):
  cc.lane_crossing.side = cross or cc.lane_crossing.side
  return run(cc, frames, lambda: SimpleNamespace(out=SimpleNamespace(
    gearShifter=gear, vEgo=kph / 3.6, leftBlinker=lamp, rightBlinker=False, steeringTorque=0, naviTunnel=bool(tunnel),
    naviRoundabout=roundabout, leftLaneLine=left_line, rightLaneLine=right_line)))


def spas2(cc, sent):
  sig = cc.packer.dbc.name_to_msg['SPAS2'].sigs['BLINKER_CONTROL']
  return [get_raw_value(m[1], sig) for m in sent if m[0] == 0x16A]


def uds(sent):
  return [(m[1], m[2]) for m in sent if m[0] == carcontroller.APRK_ADDR]


def test_aprk_paused_only_around_a_crossing_kick():
  cc = make_cc()
  assert step(cc, 50, D, 60) == [] and cc.aprk_state == carcontroller.APRK_IDLE  # nobody crossing: ECU untouched
  sent = step(cc, 30, D, 5, cross=3)  # any speed in D
  assert uds(sent)[:4] == [(EXT, 0), (EXT, 1), (QUIET, 0), (QUIET, 1)] and cc.aprk_state == carcontroller.APRK_ACTIVE
  assert set(spas2(cc, step(cc, 10, D, 0))) == {3}
  sent = step(cc, carcontroller.BLINKER_PULSE_FRAMES, D, 60)
  assert uds(sent)[-2:] == [(DEFAULT, 0), (DEFAULT, 1)]
  assert cc.aprk_state == carcontroller.APRK_IDLE and step(cc, 100, D, 60) == []  # restored after the kick
  step(cc, 30, D, 60, cross=4)
  assert uds(step(cc, 1, P, 60)) == [(DEFAULT, 0), (DEFAULT, 1)]  # out of D: restored


def test_frames_only_after_the_ecu_answered_and_went_quiet():
  cc = make_cc()
  sent = step(cc, 30, D, 60, cross=3)
  first = next(i for i, m in enumerate(sent) if m[0] == 0x16A)
  assert any(m[0] == carcontroller.APRK_ADDR and m[1] == QUIET for m in sent[:first])
  # 10-30: the stock stream never stopped, every kick collided -> now no frame while it is on the bus
  assert cc.ecu.quiet


def test_no_answer_means_no_frames_and_the_ecu_is_restored():
  cc = make_cc(ecu=Ecu(answers=False))  # 10-26..30: no 0x7B9 reply on bus 0 to ~140 requests
  sent = step(cc, carcontroller.BLINKER_PULSE_FRAMES, D, 60, cross=3)
  assert spas2(cc, sent) == [] and not any(m[0] == 0x165 for m in sent)
  assert uds(sent) == [(EXT, 0), (EXT, 1), (DEFAULT, 0), (DEFAULT, 1)]  # one try, then given back for this kick
  assert cc.aprk_state == carcontroller.APRK_FAILED and step(cc, 1, D, 60) == []  # kick over: back to idle, quietly
  assert cc.aprk_state == carcontroller.APRK_IDLE


def test_ecu_that_keeps_sending_is_never_overwritten():
  cc = make_cc(ecu=Ecu(obeys=False))  # answers but its frames continue
  sent = step(cc, carcontroller.BLINKER_PULSE_FRAMES, D, 60, cross=4)
  assert spas2(cc, sent) == [] and uds(sent)[-2:] == [(DEFAULT, 0), (DEFAULT, 1)]


def test_stock_frames_coming_back_stop_the_kick_at_once():
  cc = make_cc()
  step(cc, 30, D, 60, cross=4)
  assert cc.aprk_state == carcontroller.APRK_ACTIVE
  cc.ecu.quiet = False  # the ECU resumes on its own
  cc.ecu.stock = (cc.frame + 1) * TICK
  sent = step(cc, 1, D, 60)
  assert spas2(cc, sent) == [] and uds(sent) == [(DEFAULT, 0), (DEFAULT, 1)]


def test_kick_is_short_and_follows_the_crossing_side():
  cc = make_cc()
  kick = spas2(cc, step(cc, carcontroller.BLINKER_PULSE_FRAMES + 30, D, 60, cross=4))
  assert kick and set(kick) == {4}
  assert spas2(cc, step(cc, 300, D, 60)) == []
  assert set(spas2(cc, step(cc, 30, D, 60, cross=3))) == {3}


def test_no_kick_while_the_lamp_is_on_or_setting_off():
  cc = make_cc()
  assert step(cc, 100, D, 60, cross=3, lamp=True) == []  # the driver already signalled
  cc = make_cc(control=False)
  assert step(cc, 100, D, 60, cross=3) == []


def test_menu_test_blinks_at_standstill_in_p():
  cc = make_cc(control=False, test=2)
  sent = step(cc, 30, P, 0)
  assert cc.aprk_state == carcontroller.APRK_ACTIVE and set(spas2(cc, sent)) == {4}
  cc.blinker_test = 0
  assert uds(step(cc, 1, P, 0)) == [(DEFAULT, 0), (DEFAULT, 1)] and cc.aprk_state == carcontroller.APRK_IDLE


def test_no_kick_in_a_tunnel():
  # CarState.naviTunnel: lane changes are banned there; the crossing stays pending until out of the tunnel
  cc = make_cc()
  assert 3 not in spas2(cc, step(cc, 50, D, 80, cross=3, tunnel=1))
  assert 3 in spas2(cc, step(cc, 50, D, 80, tunnel=0))


def test_no_kick_across_a_yellow_line():
  # 10-30 seg 1: kicked left over a yellow dashed centre line (20); the color drops to 0 while crossing
  cc = make_cc()
  step(cc, 50, D, 30, left_line=20)
  assert 3 not in spas2(cc, step(cc, 50, D, 30, cross=3, left_line=0))   # yellow 0.5 s ago: no kick, crossing consumed
  assert 4 in spas2(cc, step(cc, 50, D, 30, cross=4, left_line=20))      # the white right line still kicks
  step(cc, 300, D, 30, left_line=0)
  assert 3 in spas2(cc, step(cc, 50, D, 30, cross=3, left_line=0))       # yellow over 2 s ago


def test_no_kick_after_a_roundabout_sign():
  # CarState.naviRoundabout (0x162 SIGNS=24, 20 s, < 60 km/h): same ban as a tunnel
  cc = make_cc()
  assert 4 not in spas2(cc, step(cc, 50, D, 30, cross=4, roundabout=True))
  assert 4 in spas2(cc, step(cc, 50, D, 30))


def test_spas_steer_test_parked_only():
  cc = make_cc(control=False, spas=90)
  out = lambda held=False, gear=P: lambda: SimpleNamespace(out=SimpleNamespace(
    gearShifter=gear, standstill=True, steeringPressed=held, steeringAngleDeg=-10., leftBlinker=False,
    rightBlinker=False, steeringTorque=0, leftLaneLine=0, rightLaneLine=0))
  sent = run(cc, 130, out())
  assert uds(sent)[:4] == [(EXT, 0), (EXT, 1), (QUIET, 0), (QUIET, 1)]  # APRK asked on both buses
  msg = cc.packer.dbc.name_to_msg['SPAS1']
  spas1 = [m[1] for m in sent if m[0] == 0x165]
  modes = [get_raw_value(d, msg.sigs['NEW_SIGNAL_2']) for d in spas1]
  assert modes[:carcontroller.SPAS_STEER_ASK_FRAMES] == [4] * carcontroller.SPAS_STEER_ASK_FRAMES and set(modes[10:]) == {5}
  angles = [int.from_bytes(d[12:14], 'little', signed=True) * 0.1 for d in spas1]
  assert abs(angles[0] - -9.) < 0.11 and abs(angles[-1] - 90.) < 0.11  # ramps from the wheel at 100 deg/s
  assert all(d[3] == 0x4b and d[11] in (0x10, 0x14) and not any(d[4:11]) and not any(d[14:]) for d in spas1)
  # hands on the wheel: APRK back on both buses, no more SPAS1
  assert uds(run(cc, 1, out(held=True))) == [(DEFAULT, 0), (DEFAULT, 1)]
  assert run(cc, 1, out(gear=D)) == []


def test_carstate_sees_stock_frames_and_replies_without_touching_can_validity():
  from opendbc.can import CANParser
  from opendbc.car.hyundai.carstate import CarState
  cp, alt = CANParser('hyundai_canfd_generated', [], 0), CANParser('hyundai_canfd_generated', [], 1)
  cs = SimpleNamespace(aprk_watch=False, can_nanos=0, aprk_stock_nanos=0, aprk_ack_nanos=0)
  def feed(t, frames):
    for parser in (cp, alt):
      parser.update([(t, frames)])
    CarState._update_aprk(cs, cp, alt)
  feed(1 * TICK, [])
  # stock SPAS2 with a garbage checksum and jumping counter: still counts as "the ECU is sending"
  feed(2 * TICK, [(0x16A, bytes([1, 2, 7]) + bytes(29), 0)])
  feed(3 * TICK, [(0x16A, bytes([9, 9, 200]) + bytes(29), 0)])
  assert cs.aprk_stock_nanos == 3 * TICK and cs.aprk_ack_nanos == 0 and cp.can_valid
  feed(4 * TICK, [(0x7B9, bytes([0x06, 0x50, 0x03, 0, 0x32, 1, 0xF4, 0]), 1)])  # positive reply on ACAN
  assert cs.aprk_ack_nanos == 4 * TICK
  feed(5 * TICK, [(0x7B9, bytes([0x03, 0x7F, 0x10, 0x22, 0, 0, 0, 0]), 0)])  # a refusal is not an answer
  assert cs.aprk_ack_nanos == 4 * TICK
