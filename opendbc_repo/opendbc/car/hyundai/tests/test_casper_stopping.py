"""Classic CAN Casper stopping regressions. No hardware or CAN transmission."""
import copy
from types import SimpleNamespace as NS
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai import hyundaican as candidate


def reference_commands(*args, **kwargs):
  # The generic camera-SCC path is the unchanged reference for all fields.
  cs = copy.deepcopy(kwargs['CS'])
  cs.CP.carFingerprint = CAR.HYUNDAI_SONATA
  kwargs['CS'] = cs
  return candidate.create_acc_commands_scc(*args, **kwargs)


baseline = NS(create_acc_commands_scc=reference_commands)


def make_state(platform=CAR.HYUNDAI_CASPER):
  return NS(CP=NS(carFingerprint=platform), paddle_button_prev=0, softHoldActive=0,
            out=NS(brakeHoldActive=False, brakePressed=False, gasPressed=False,
                   cruiseState=NS(available=True)), scc11={'DriverAlertDisplay': 1},
            scc12={'CF_VSM_ConfMode': 1}, scc14={}, fca11=None)


def encode(module, cs, **overrides):
  args = dict(enabled=True, accel=-0.5, stopping=True, long_override=False, idx=7, soft_hold_mode=0)
  args.update(overrides)
  return module.create_acc_commands_scc(
    CANPacker('hyundai_kia_generic'), **args,
    jerk=NS(carrot_cruise=0, cb_upper=0.8, cb_lower=0.7, jerk_u=0.5, jerk_l=1.0),
    hud_control=NS(leadDistance=2.5, leadRelSpeed=0.0, leadDistanceBars=2, leadVisible=True),
    set_speed=80, suppress_casper_ev_fca=False, CS=cs)


def decode(messages):
  parser = CANParser('hyundai_kia_generic', [('SCC12', 50)], 0)
  parser.update([1_000_000_000, messages])
  return dict(parser.vl['SCC12'])


class TestCasperStopping(unittest.TestCase):
  def test_stop_changes_only_request_and_checksum(self):
    for accel in [-0.5, -1.24, -2.0]:
      for idx in range(30):
        with self.subTest(accel=accel, idx=idx):
          old = encode(baseline, make_state(), accel=accel, idx=idx)
          new = encode(candidate, make_state(), accel=accel, idx=idx)
          a, b = decode(old), decode(new)
          self.assertAlmostEqual(a['aReqRaw'], accel)
          self.assertAlmostEqual(b['aReqRaw'], accel)
          self.assertAlmostEqual(b['aReqValue'], accel)
          self.assertEqual((b['StopReq'], b['ACCMode']), (0, 1))
          self.assertEqual({k:v for k,v in a.items() if k not in ('StopReq','CR_VSM_ChkSum')},
                           {k:v for k,v in b.items() if k not in ('StopReq','CR_VSM_ChkSum')})
          self.assertEqual([m for m in old if m[0] != 1057], [m for m in new if m[0] != 1057])
          msg = next(m for m in new if m[0] == 1057)
          n = int.from_bytes(msg[1], 'little')
          self.assertAlmostEqual(((n >> 24) & 2047) * .01 - 10.23, accel)
          self.assertEqual((n >> 15) & 1, 0)
          self.assertAlmostEqual(((n >> 37) & 2047) * .01 - 10.23, accel)
          self.assertEqual(sum((x >> 4) + (x & 15) for x in msg[1]) % 16, 0)

  def test_other_platforms_identical(self):
    for platform in CAR:
      if platform != CAR.HYUNDAI_CASPER:
        with self.subTest(platform=platform):
          self.assertEqual(encode(baseline, make_state(platform)), encode(candidate, make_state(platform)))

  def test_non_stopping_and_positive_requests_identical(self):
    for args in [dict(stopping=False, accel=1.0), dict(stopping=False, accel=-0.5),
                 dict(accel=0), dict(accel=1.0), dict(enabled=False), dict(long_override=True)]:
      with self.subTest(args=args):
        self.assertEqual(encode(baseline, make_state(), **args), encode(candidate, make_state(), **args))

  def test_driver_inputs_and_hold_identical(self):
    for field in ['brakePressed', 'gasPressed', 'brakeHoldActive']:
      cs = make_state()
      setattr(cs.out, field, True)
      self.assertEqual(encode(baseline, cs), encode(candidate, cs))
    for hold in [1, 2]:
      for mode in [0, 1, 2]:
        for enabled in [False, True]:
          cs = make_state()
          cs.softHoldActive = hold
          self.assertEqual(encode(baseline, cs, enabled=enabled, soft_hold_mode=mode),
                           encode(candidate, cs, enabled=enabled, soft_hold_mode=mode))

  def test_departure_and_cancel_do_not_latch(self):
    cs = make_state()
    for _ in range(20):
      self.assertEqual(decode(encode(candidate, cs))['StopReq'], 0)
    for args in [dict(stopping=False, accel=1), dict(enabled=False, stopping=False, accel=0)]:
      self.assertEqual(encode(baseline, cs, **args), encode(candidate, cs, **args))

  def test_missing_messages_and_input_immutability(self):
    for field in ['scc11', 'scc12', 'scc14']:
      cs = make_state()
      setattr(cs, field, None)
      original = copy.deepcopy(cs)
      messages = encode(candidate, cs)
      self.assertEqual(cs, original)
      if field == 'scc12':
        self.assertEqual(messages, encode(baseline, cs))


if __name__ == '__main__':
  unittest.main(verbosity=2)
