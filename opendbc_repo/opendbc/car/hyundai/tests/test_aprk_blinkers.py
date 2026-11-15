import pytest
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.values import CAR, KICK_CARS, HyundaiSafetyFlags

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


def make_cc(control=True, ecu=None, mode=1, model=True):
  cc = SimpleNamespace(blinker_control=control, blinker_kick_mode=mode, aprk_state=0, aprk_frame=0, aprk_nanos=0, aprk_acked=False,
                       lane_crossing=Crossing(), blinker_side=0, blinker_pulse_end=0, frame=0, yellow_until=[0, 0],
                       blinker_kick_model=model, lane_change_prev="off", model_pulse=False, blinker_pulse_start=0, model_hold={},
                       model_flashes=0, CAN=SimpleNamespace(ECAN=0, ACAN=1), packer=CANPacker('hyundai_canfd_generated'),
                       params=SimpleNamespace(STEER_THRESHOLD=250), aprk_buses=(), ecu=ecu or Ecu())
  cc.kick_pulse = lambda CS: CarController.kick_pulse(cc, CS)
  cc.restart_pulse = lambda: CarController.restart_pulse(cc)
  return cc


def run(cc, frames, out):
  sent = []
  for _ in range(frames):
    now = (cc.frame + 1) * TICK
    CS = out()
    CS.can_nanos, CS.aprk_stock_nanos, CS.aprk_ack_nanos = now, cc.ecu.stock, cc.ecu.ack
    msgs = CarController.aprk_blinkers(cc, CS)
    sent += msgs
    cc.frame += 1
    cc.ecu.tick((cc.frame + 1) * TICK, msgs)
  return sent


def step(cc, frames, gear, kph, cross=0, lamp=False, left_line=10, right_line=10):
  cc.lane_crossing.side = cross or cc.lane_crossing.side
  return run(cc, frames, lambda: SimpleNamespace(out=SimpleNamespace(
    gearShifter=gear, vEgo=kph / 3.6, leftBlinker=lamp, rightBlinker=False, steeringTorque=0,
    leftLaneLine=left_line, rightLaneLine=right_line)))


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


def test_kick_mode_without_an_ack():
  # 0: as before fcd251a2 - one try, then given back for this kick
  cc = make_cc(ecu=Ecu(answers=False), mode=0)
  sent = step(cc, carcontroller.BLINKER_PULSE_FRAMES, D, 60, cross=3)
  assert spas2(cc, sent) == [] and uds(sent) == [(EXT, 0), (EXT, 1), (DEFAULT, 0), (DEFAULT, 1)]
  assert cc.aprk_state == carcontroller.APRK_FAILED
  # 1 and a stored 2/3 (the retired direct modes): every frame of a full pulse, panda piggybacks each one
  for mode in (1, 2, 3):
    cc = make_cc(ecu=Ecu(answers=False), mode=mode)
    step(cc, carcontroller.APRK_REPLY_FRAMES, D, 60, cross=4)
    sent = spas2(cc, step(cc, carcontroller.BLINKER_PULSE_FRAMES, D, 60))
    assert len(sent) == carcontroller.BLINKER_PULSE_FRAMES - 1 and cc.aprk_state == carcontroller.APRK_ACTIVE


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


def test_no_kick_across_a_yellow_line():
  # 10-30 seg 1: kicked left over a yellow dashed centre line (20); the color drops to 0 while crossing
  cc = make_cc()
  step(cc, 50, D, 30, left_line=20)
  assert 3 not in spas2(cc, step(cc, 50, D, 30, cross=3, left_line=0))   # yellow 0.5 s ago: no kick, crossing consumed
  assert 4 in spas2(cc, step(cc, 50, D, 30, cross=4, left_line=20))      # the white right line still kicks
  step(cc, 300, D, 30, left_line=0)
  assert 3 in spas2(cc, step(cc, 50, D, 30, cross=3, left_line=0))       # yellow over 2 s ago


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


# CanfdBlinkerKickModelMode (not 1): a model lane change (remote, app, voice, navigation) kicks on its laneChangeStarting edge, only
# with DesireHelper's lamp ownership (indicatorIsolationMask, 1 left 2 right) in the same modelV2
def lc(state="off", direction="none", mask=0):
  return SimpleNamespace(meta=SimpleNamespace(laneChangeState=state, laneChangeDirection=direction, indicatorIsolationMask=mask))


OFF, LEFT, RIGHT = lc(), lc("laneChangeStarting", "left", 1), lc("laneChangeStarting", "right", 2)


def model_step(cc, frames, md, gear=D, lamp=False, cancel=False, cross=0, left_line=10):
  cc.lane_crossing.side = cross or cc.lane_crossing.side
  return run(cc, frames, lambda: SimpleNamespace(modelV2=md, out=SimpleNamespace(
    gearShifter=gear, vEgo=20., leftBlinker=lamp, rightBlinker=False, steeringTorque=0,
    leftLaneLine=left_line, rightLaneLine=10, cruiseCancelState=cancel)))


def test_model_lane_change_start_kicks_with_the_mask():
  for md, side in ((LEFT, 3), (RIGHT, 4)):
    cc = make_cc()
    assert model_step(cc, 50, OFF) == []
    # the model decided: no yellow-line guard (that one is for the driver's own crossing)
    sent = model_step(cc, 30, md, left_line=20)
    assert set(spas2(cc, sent)) == {side} and cc.aprk_state == carcontroller.APRK_ACTIVE
    # held through Starting and Finishing (no lane lines here: no crossing seen), off with the change
    assert set(spas2(cc, model_step(cc, 200, md))) == {side}
    fin = lc("laneChangeFinishing", md.meta.laneChangeDirection, md.meta.indicatorIsolationMask)
    sent = model_step(cc, 100, fin)
    assert set(spas2(cc, sent)) == {side} and (DEFAULT, 0) not in uds(sent)
    sent = model_step(cc, 1, OFF)
    assert spas2(cc, sent) == [] and uds(sent) == [(DEFAULT, 0), (DEFAULT, 1)] and cc.aprk_state == carcontroller.APRK_IDLE
    assert model_step(cc, 100, OFF) == []


def lines(target, other):
  """modelV2 with lane lines for a left change: line 1 (left, the target) and line 2 (right) at the car, m"""
  md = lc("laneChangeStarting", "left", 1)
  md.laneLines = [SimpleNamespace(y=[y]) for y in (-5.4, -target, other, 5.4)]
  return md


def test_model_kick_released_once_the_car_is_across():
  cc = make_cc()
  model_step(cc, 10, OFF)
  assert set(spas2(cc, model_step(cc, 150, lines(1.8, 1.8)))) == {3}  # past the 1 s minimum: still held
  assert set(spas2(cc, model_step(cc, 10, lines(0.5, 3.1)))) == {3}  # at the line
  assert set(spas2(cc, model_step(cc, 10, lines(3.5, 0.3)))) == {3}  # relabelled: across, not 60 % in yet
  sent = model_step(cc, 40, lines(3.3, 0.5))  # 60 % in: released, the gap runs out
  assert 0 < len(spas2(cc, sent)) <= carcontroller.MODEL_HOLD_GAP_FRAMES + 1 and uds(sent)[-2:] == [(DEFAULT, 0), (DEFAULT, 1)]
  assert model_step(cc, 100, lines(3.3, 0.5)) == []  # Starting goes on: no new kick, no new hold


def test_model_kick_hold_has_a_ceiling():
  cc = make_cc()
  model_step(cc, 10, OFF)
  sent = model_step(cc, carcontroller.MODEL_HOLD_MAX_FRAMES + 200, LEFT)
  n = len(spas2(cc, sent))
  assert carcontroller.MODEL_HOLD_MAX_FRAMES - 60 < n <= carcontroller.MODEL_HOLD_MAX_FRAMES + carcontroller.MODEL_HOLD_GAP_FRAMES


@pytest.mark.parametrize("mode", [2, 3, 4])
def test_model_kick_counted_flashes_outlast_a_completed_change(mode):
  # CanfdBlinkerKickModelMode 2/3/4: 3/5/7 stock flash cycles, finished even after a short change, like the stock one-touch
  cc = make_cc()
  cc.model_flashes = carcontroller.MODEL_FLASH_COUNTS[mode]
  model_step(cc, 10, OFF)
  start = cc.frame
  fin = lc("laneChangeFinishing", "left", 1)
  sent = model_step(cc, 50, LEFT) + model_step(cc, 50, fin)
  # no hold, a fixed length counted from the end of the APRK handshake (the lamp cannot start before it)
  assert cc.blinker_pulse_end - cc.blinker_pulse_start == cc.model_flashes * carcontroller.MODEL_FLASH_FRAMES
  assert start < cc.blinker_pulse_start <= start + carcontroller.APRK_REPLY_FRAMES + 1
  after = model_step(cc, cc.blinker_pulse_end - cc.frame, OFF)  # the change is over, the flashes are not
  assert set(spas2(cc, sent + after)) == {3} and len(spas2(cc, after)) > 0 and not cc.model_pulse
  assert spas2(cc, model_step(cc, 50, OFF)) == []


def test_model_kick_counted_flashes_outlast_a_change_that_queues_the_next():
  # a queued next change (navigation ATC, a second one-touch) leaves Finishing through preLaneChange, not off
  cc = make_cc()
  cc.model_flashes = 7
  model_step(cc, 10, OFF)
  fin, pre = lc("laneChangeFinishing", "left", 1), lc("preLaneChange", "left", 1)
  model_step(cc, 50, LEFT)
  model_step(cc, 50, fin)
  assert len(spas2(cc, model_step(cc, 20, pre) + model_step(cc, 20, OFF))) == 40 and not cc.model_pulse


def test_model_kick_counted_flashes_cut_when_cancelled():
  cc = make_cc()
  cc.model_flashes = 7
  model_step(cc, 10, OFF)
  assert set(spas2(cc, model_step(cc, 30, LEFT))) == {3}
  assert spas2(cc, model_step(cc, 1, OFF)) == []  # Starting -> off: cancelled, not completed
  assert model_step(cc, 50, OFF) == []


def test_no_model_kick_without_ownership_or_outside_the_rules():
  cases = {
    "mask of the other side": dict(md=lc("laneChangeStarting", "left", 2)),
    "no mask": dict(md=lc("laneChangeStarting", "right", 0)),
    "no direction": dict(md=lc("laneChangeStarting", "none", 3)),
    "lamp on": dict(lamp=True),
    "cruise CANCEL latch": dict(cancel=True),
    "in P": dict(gear=P),
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
  model_step(cc, carcontroller.MODEL_HOLD_MAX_FRAMES + 50, LEFT)  # held up to the ceiling
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


@pytest.mark.parametrize("flashes", [0, 3, 5, 7])
def test_driver_crossing_pulse_is_never_counted(flashes):
  # carState reads the real lamp back as the driver's blinker: the unowned driver pulse stays 1 s in every model mode
  cc = make_cc()
  cc.model_flashes = flashes
  step(cc, 5, D, 60, cross=3)
  assert cc.blinker_pulse_end - cc.blinker_pulse_start == carcontroller.BLINKER_PULSE_FRAMES


@pytest.mark.parametrize("camera_scc", [0, 2])
@pytest.mark.parametrize("car", [CAR.HYUNDAI_IONIQ_5_PE, CAR.KIA_EV6])
def test_kick_is_limited_to_kick_cars(monkeypatch, car, camera_scc):
  # values.KICK_CARS on the camera-SCC harness only: anything else ignores CanfdBlinkerKick, at fingerprint (panda
  # flag CANFD_BLINKER_KICK) and in carcontroller, which follows that flag
  from unittest.mock import patch
  from opendbc.car import gen_empty_fingerprint
  from opendbc.car.hyundai.interface import CarInterface
  from opendbc.car.hyundai.tests.test_steering_handover_controller import setup_controller, step as control_step

  class Settings:
    def __init__(self, *a, **k):
      pass

    def get_bool(self, key):
      return key == "CanfdBlinkerKick"

    def get_int(self, key):
      return {"HyundaiCameraSCC": camera_scc, "MaxAngleFrames": 89}.get(key, 0)

  with patch("opendbc.car.hyundai.interface.Params", Settings):
    CP = CarInterface.get_params(car, gen_empty_fingerprint(), [], False, False, False)
  active = car in KICK_CARS and camera_scc > 0
  assert bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_BLINKER_KICK) == active

  controller, cs, cc, _ = setup_controller(monkeypatch)
  controller.CP = structs.CarParams(carFingerprint=car, flags=controller.CP.flags, wheelbase=3.0, steerRatio=14.26,
                                    safetyConfigs=[cfg.to_dict() for cfg in CP.safetyConfigs])
  cs.modelV2.meta, cs.modelV2.laneLines = OFF.meta, []
  monkeypatch.setattr(carcontroller, "Params", Settings)
  control_step(controller, cs, cc)
  assert controller.blinker_control == active

