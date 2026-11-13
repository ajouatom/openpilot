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

  def update(self, md, t, allowed, torque=None, torque_min=250):
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


def make_cc(control=True, test=0, spas=0, ecu=None, mode=1, model=True):
  cc = SimpleNamespace(blinker_control=control, blinker_test=test, blinker_kick_mode=mode, aprk_state=0, aprk_frame=0, aprk_nanos=0, aprk_acked=False,
                         lane_crossing=Crossing(), blinker_side=0, blinker_pulse_end=0, frame=0, yellow_until=[0, 0],
                         blinker_kick_model=model, lane_change_prev="off", model_pulse=False,
                         CAN=SimpleNamespace(ECAN=0, ACAN=1), packer=CANPacker('hyundai_canfd_generated'), params=SimpleNamespace(STEER_THRESHOLD=250),
                         spas_steer_test=spas, spas_angle=0., aprk_buses=(), ecu=ecu or Ecu())
  cc.kick_pulse = lambda CS: CarController.kick_pulse(cc, CS)
  return cc


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


def test_no_answer_still_blinks_behind_the_stock_frames():
  cc = make_cc(ecu=Ecu(answers=False))  # 10-24..11-05: no 0x7B9 reply to ~180 requests, stock stream stays on
  sent = step(cc, carcontroller.APRK_REPLY_FRAMES, D, 60, cross=3)
  assert spas2(cc, sent) == [] and uds(sent) == [(EXT, 0), (EXT, 1)]  # the reply window: nothing yet
  sent = step(cc, carcontroller.BLINKER_PULSE_FRAMES, D, 60)
  # a full pulse after the window, next to the stock frames (panda sends each frame right behind a stock one)
  assert set(spas2(cc, sent)) == {3} and len(spas2(cc, sent)) == carcontroller.BLINKER_PULSE_FRAMES - 1  # none on the switch frame
  assert cc.aprk_state == carcontroller.APRK_ACTIVE and not cc.aprk_acked
  assert uds(step(cc, 1, D, 60)) == [(DEFAULT, 0), (DEFAULT, 1)] and cc.aprk_state == carcontroller.APRK_IDLE


def test_kick_mode_picks_the_send_pattern_without_an_ack():
  # 0: as before fcd251a2 - one try, then given back for this kick
  cc = make_cc(ecu=Ecu(answers=False), mode=0)
  sent = step(cc, carcontroller.BLINKER_PULSE_FRAMES, D, 60, cross=3)
  assert spas2(cc, sent) == [] and uds(sent) == [(EXT, 0), (EXT, 1), (DEFAULT, 0), (DEFAULT, 1)]
  assert cc.aprk_state == carcontroller.APRK_FAILED
  # 2: direct at a quarter rate; 3: direct at full rate (panda decides piggyback vs direct from the safety flag)
  for mode, frames in ((2, carcontroller.BLINKER_PULSE_FRAMES // 4), (3, carcontroller.BLINKER_PULSE_FRAMES - 1)):
    cc = make_cc(ecu=Ecu(answers=False), mode=mode)
    step(cc, carcontroller.APRK_REPLY_FRAMES, D, 60, cross=4)
    n = len(spas2(cc, step(cc, carcontroller.BLINKER_PULSE_FRAMES, D, 60)))
    assert abs(n - frames) <= 1 and cc.aprk_state == carcontroller.APRK_ACTIVE
  # an ECU that answered and went quiet is never rate-limited
  cc = make_cc(mode=2)
  step(cc, 30, D, 60, cross=3)
  assert len(spas2(cc, step(cc, 20, D, 60))) == 20


STOCK_CGW1 = bytes([0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00])  # ignition on, nothing else


def cgw_step(cc, frames, gear=D, cross=0, raw=STOCK_CGW1, lamp=False, md=None):
  """-> [(frame, addr, dat, bus)] sent"""
  cc.lane_crossing.side = cross or cc.lane_crossing.side
  sent = []
  for _ in range(frames):
    CS = SimpleNamespace(cgw_test_raw=raw, modelV2=md, out=SimpleNamespace(
      gearShifter=gear, vEgo=15., leftBlinker=lamp, rightBlinker=False, steeringTorque=0, naviTunnel=False,
      naviRoundabout=False, leftLaneLine=0, rightLaneLine=0))
    sent += [(cc.frame, *m) for m in CarController.cgw_blinkers(cc, CS)]
    cc.frame += 1
  return sent


def test_palisade_kick_forges_cgw1_like_the_parked_test():
  cc = make_cc()
  assert cgw_step(cc, 50) == []  # nobody crossing
  start = cc.frame
  sent = cgw_step(cc, carcontroller.BLINKER_PULSE_FRAMES + 20, cross=3)
  pulse = range(start, start + carcontroller.BLINKER_PULSE_FRAMES)
  assert [f for f, *_ in sent] == [f for f in pulse if f % carcontroller.CGW_TX_FRAMES == 0]  # 60 ms for the 1 s pulse
  assert all(addr == 0x541 and bus == 0 and len(dat) == 8 for _, addr, dat, bus in sent)
  assert all((dat[5] >> 1) & 3 == 2 and dat[7] >> 6 == 0 for _, _, dat, _ in sent)  # left lever, right lamp untouched
  # 0.35 s on / 0.35 s off from the kick
  assert [(dat[2] >> 3) & 3 for _, _, dat, _ in sent] == [int((f - start) // carcontroller.CGW_FLASH_HALF_FRAMES % 2 == 0) for f, *_ in sent]
  assert all(bytes(dat[i] for i in (0, 1, 3, 4, 6)) == bytes(STOCK_CGW1[i] for i in (0, 1, 3, 4, 6)) for _, _, dat, _ in sent)
  cc = make_cc()
  sent = cgw_step(cc, 30, cross=4)
  assert sent and all(dat[7] >> 6 in (0, 1) and (dat[5] >> 1) & 3 == 1 and dat[2] & 0x18 == 0 for _, _, dat, _ in sent)  # right


def test_palisade_kick_yields_to_the_driver_and_needs_d():
  cc = make_cc()
  assert cgw_step(cc, 30, gear=P, cross=3) == []  # not in D: no pulse
  cc = make_cc()
  cgw_step(cc, 10, cross=3)
  assert cgw_step(cc, 20, gear=P) == []  # out of D mid-pulse: nothing goes to the parked direct path
  cc = make_cc()
  cgw_step(cc, 10, cross=3)
  assert cgw_step(cc, 10, raw=bytes([0, 0, 0, 0, 0x02, 0, 0, 0])) == [] and cc.blinker_pulse_end == 0  # hazard: pulse over
  cc = make_cc()
  cgw_step(cc, 10, cross=4)
  assert cgw_step(cc, 10, raw=bytes([0, 0, 0, 0, 0, 0x04, 0, 0])) == []  # the driver's own lever
  cc = make_cc()
  cgw_step(cc, 10, cross=3)
  assert cgw_step(cc, 10, raw=b"") == []  # no fresh stock frame to copy: nothing sent


def test_no_answer_never_steers():
  cc = make_cc(spas=30, ecu=Ecu(answers=False))
  parked = lambda: SimpleNamespace(out=SimpleNamespace(
    gearShifter=P, standstill=True, steeringPressed=False, steeringAngleDeg=-10., leftBlinker=False, rightBlinker=False,
    steeringTorque=0, leftLaneLine=0, rightLaneLine=0))
  sent = run(cc, carcontroller.APRK_REPLY_FRAMES + 1, parked)
  assert not any(m[0] == 0x165 for m in sent) and cc.aprk_state == carcontroller.APRK_FAILED


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


# CanfdBlinkerKickModel: a model lane change (remote, app, voice, navigation) kicks on its laneChangeStarting edge, only
# with DesireHelper's lamp ownership (indicatorIsolationMask, 1 left 2 right) in the same modelV2
def lc(state="off", direction="none", mask=0):
  return SimpleNamespace(meta=SimpleNamespace(laneChangeState=state, laneChangeDirection=direction, indicatorIsolationMask=mask))


OFF, LEFT, RIGHT = lc(), lc("laneChangeStarting", "left", 1), lc("laneChangeStarting", "right", 2)


def model_step(cc, frames, md, gear=D, lamp=False, cancel=False, tunnel=False, cross=0, left_line=10):
  cc.lane_crossing.side = cross or cc.lane_crossing.side
  return run(cc, frames, lambda: SimpleNamespace(modelV2=md, out=SimpleNamespace(
    gearShifter=gear, vEgo=20., leftBlinker=lamp, rightBlinker=False, steeringTorque=0, naviTunnel=tunnel,
    naviRoundabout=False, leftLaneLine=left_line, rightLaneLine=10, cruiseCancelState=cancel)))


def test_model_lane_change_start_kicks_with_the_mask():
  for md, side in ((LEFT, 3), (RIGHT, 4)):
    cc = make_cc()
    assert model_step(cc, 50, OFF) == []
    # the model decided: no yellow-line guard (that one is for the driver's own crossing)
    sent = model_step(cc, 30, md, left_line=20)
    assert set(spas2(cc, sent)) == {side} and cc.aprk_state == carcontroller.APRK_ACTIVE
    # one 1 s pulse for the edge, however long Starting lasts
    sent = model_step(cc, carcontroller.BLINKER_PULSE_FRAMES + 100, md)
    assert uds(sent)[-2:] == [(DEFAULT, 0), (DEFAULT, 1)] and cc.aprk_state == carcontroller.APRK_IDLE
    assert model_step(cc, 100, md) == []


def test_palisade_model_lane_change_kick_on_cgw1():
  cc = make_cc()
  assert cgw_step(cc, 20, md=OFF) == []
  sent = cgw_step(cc, carcontroller.BLINKER_PULSE_FRAMES + 20, md=RIGHT)
  assert sent and all(addr == 0x541 and (dat[5] >> 1) & 3 == 1 for _, addr, dat, _ in sent)  # right lever
  assert sent[-1][0] < carcontroller.BLINKER_PULSE_FRAMES + 20


def test_no_model_kick_without_ownership_or_outside_the_rules():
  cases = {
    "mask of the other side": dict(md=lc("laneChangeStarting", "left", 2)),
    "no mask": dict(md=lc("laneChangeStarting", "right", 0)),
    "no direction": dict(md=lc("laneChangeStarting", "none", 3)),
    "lamp on": dict(lamp=True),
    "cruise CANCEL latch": dict(cancel=True),
    "in P": dict(gear=P),
    "tunnel": dict(tunnel=True),
    "KickModel 0": dict(model=False),
    "Kick 0": dict(control=False),
  }
  for name, case in cases.items():
    cc = make_cc(control=case.pop("control", True), model=case.pop("model", True))
    model_step(cc, 10, OFF)
    assert model_step(cc, 50, case.pop("md", LEFT), **case) == [] and cc.blinker_pulse_end == 0, name


def test_model_kick_only_on_the_edge():
  cc = make_cc()
  assert set(spas2(cc, model_step(cc, 30, LEFT))) == {3} and cc.model_pulse  # Starting when first seen: one kick
  model_step(cc, carcontroller.BLINKER_PULSE_FRAMES + 50, LEFT)
  assert model_step(cc, 50, LEFT) == []  # repeated Starting frames: no second kick
  # modelV2 None (not alive / invalid) is no transition: Starting -> None -> Starting does not kick again
  assert model_step(cc, 20, None) == [] and cc.lane_change_prev == "laneChangeStarting"
  assert model_step(cc, 50, LEFT) == []
  # off -> None -> Starting still kicks once
  cc = make_cc()
  model_step(cc, 10, OFF)
  model_step(cc, 10, None)
  assert set(spas2(cc, model_step(cc, 30, LEFT))) == {3}


def test_model_kick_cut_when_the_change_is_cancelled():
  cc = make_cc()
  model_step(cc, 10, OFF)
  assert set(spas2(cc, model_step(cc, 30, LEFT))) == {3}
  sent = model_step(cc, 1, OFF)  # cancelled: the lamp stops with it
  assert spas2(cc, sent) == [] and uds(sent) == [(DEFAULT, 0), (DEFAULT, 1)]
  assert model_step(cc, 50, OFF) == []
  # Palisade: same cut
  cc = make_cc()
  cgw_step(cc, 10, md=OFF)
  assert cgw_step(cc, 10, md=LEFT)
  assert cgw_step(cc, 50, md=OFF) == []
  # a cut never ends the driver's own crossing kick
  cc = make_cc()
  model_step(cc, 10, OFF, cross=4)
  assert not cc.model_pulse and set(spas2(cc, model_step(cc, 30, OFF))) == {4}


def test_crossing_kick_waits_out_a_model_lane_change():
  # no ownership for the driver-crossing kick: it must not light up a change DesireHelper is running
  cc = make_cc(model=False)
  for state in ("laneChangeStarting", "laneChangeFinishing"):
    assert model_step(cc, 50, lc(state, "left", 0), cross=3) == []
  assert cc.lane_crossing.side == 3  # still pending in the stand-in
  assert set(spas2(cc, model_step(cc, 30, lc("preLaneChange", "left")))) == {3}
  cc = make_cc()
  assert set(spas2(cc, model_step(cc, 30, None, cross=4))) == {4}  # no modelV2: as before
