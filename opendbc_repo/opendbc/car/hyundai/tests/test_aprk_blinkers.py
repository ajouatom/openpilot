"""SPAS transport accepts only the common policy's leased direction."""
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
TICK = 10_000_000


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


def make_cc(ecu=None, mode=1):
  return SimpleNamespace(blinker_control=True, kick_output_ready=True, kick_direction=0,
    blinker_kick_mode=mode, aprk_state=0, aprk_frame=0, aprk_nanos=0, aprk_acked=False,
    blinker_side=0, frame=0, CAN=SimpleNamespace(ECAN=0, ACAN=1),
    packer=CANPacker('hyundai_canfd_generated'), aprk_buses=(), ecu=ecu or Ecu())


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


def step(cc, frames, direction=0, gear=D, left=False, right=False):
  cc.kick_direction = direction
  return run(cc, frames, lambda: SimpleNamespace(out=SimpleNamespace(
    gearShifter=gear, leftBlinker=left, rightBlinker=right)))


def spas2(cc, sent):
  sig = cc.packer.dbc.name_to_msg['SPAS2'].sigs['BLINKER_CONTROL']
  return [get_raw_value(m[1], sig) for m in sent if m[0] == 0x16A]


def uds(sent):
  return [(m[1], m[2]) for m in sent if m[0] == carcontroller.APRK_ADDR]


@pytest.mark.parametrize('side', [3, 4])
def test_common_hold_controls_start_duration_and_release(side):
  cc = make_cc()
  assert step(cc, 50) == []
  sent = step(cc, 30, side)
  assert uds(sent)[:4] == [(EXT, 0), (EXT, 1), (QUIET, 0), (QUIET, 1)]
  assert set(spas2(cc, sent)) == {side}
  # A long common-policy turn survives the old ten-second model cap.
  assert set(spas2(cc, step(cc, 1200, side, left=side == 3, right=side == 4))) == {side}
  sent = step(cc, 1)
  assert not spas2(cc, sent)
  assert uds(sent) == [(DEFAULT, 0), (DEFAULT, 1)]
  assert step(cc, 100) == []


@pytest.mark.parametrize('guard', ['owner', 'setting', 'park', 'opposite'])
def test_guard_loss_restores_ecu_and_stops_sending(guard):
  cc = make_cc()
  step(cc, 30, 3)
  cc.kick_output_ready = guard != 'owner'
  cc.blinker_control = guard != 'setting'
  sent = step(cc, 1, 3, gear=P if guard == 'park' else D, right=guard == 'opposite')
  assert not spas2(cc, sent)
  assert uds(sent) == [(DEFAULT, 0), (DEFAULT, 1)]


def test_no_answer_uses_existing_piggyback_after_wait():
  cc = make_cc(ecu=Ecu(answers=False))
  assert not spas2(cc, step(cc, carcontroller.APRK_REPLY_FRAMES, 3))
  assert set(spas2(cc, step(cc, 100, 3))) == {3}
  assert not cc.aprk_acked
  assert uds(step(cc, 1)) == [(DEFAULT, 0), (DEFAULT, 1)]


def test_ack_only_mode_does_not_retry_a_continuing_request():
  cc = make_cc(ecu=Ecu(answers=False), mode=0)
  sent = step(cc, 200, 3)
  assert not spas2(cc, sent)
  assert uds(sent) == [(EXT, 0), (EXT, 1), (DEFAULT, 0), (DEFAULT, 1)]
  assert cc.aprk_state == carcontroller.APRK_FAILED
  assert step(cc, 100, 3) == []
  step(cc, 1)
  assert uds(step(cc, 1, 4)) == [(EXT, 0), (EXT, 1)]


def test_acknowledged_ecu_must_stay_quiet():
  cc = make_cc(ecu=Ecu(obeys=False))
  assert not spas2(cc, step(cc, 200, 3))
  assert cc.aprk_state == carcontroller.APRK_FAILED
  cc = make_cc()
  assert spas2(cc, step(cc, 30, 3))
  cc.ecu.quiet = False
  sent = step(cc, 5, 3)
  assert uds(sent)[-2:] == [(DEFAULT, 0), (DEFAULT, 1)]
  assert not spas2(cc, step(cc, 50, 3))


def test_direction_change_releases_previous_session_first():
  cc = make_cc()
  step(cc, 30, 3)
  sent = step(cc, 1, 4)
  assert uds(sent) == [(DEFAULT, 0), (DEFAULT, 1), (EXT, 0), (EXT, 1)]
  assert not spas2(cc, sent)
  assert set(spas2(cc, step(cc, 30, 4))) == {4}


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
  cs.modelV2.laneLines = []
  monkeypatch.setattr(carcontroller, "Params", Settings)
  control_step(controller, cs, cc)
  assert controller.blinker_control == active
