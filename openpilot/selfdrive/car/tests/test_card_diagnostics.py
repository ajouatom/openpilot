import time
import unittest
from types import SimpleNamespace

import pytest

from opendbc.can.parser import MessageState

from openpilot.selfdrive.car.card_diagnostics import (
  invalid_can_messages,
  CARD_DIAG_LOOP_WARN_US,
  CARD_DIAG_PROCESS_WARN_US,
  should_log_card_diagnostics,
)


class TestCardDiagnostics(unittest.TestCase):
  def test_normal_card_timing_does_not_log(self):
    # Representative upper bounds from recent multi-vehicle route analysis.
    self.assertFalse(should_log_card_diagnostics(loop_max_us=25_865, process_max_us=10_798, can_timeouts=0))

  def test_card_problem_logs(self):
    cases = [
      (CARD_DIAG_LOOP_WARN_US, 0, 0),
      (0, CARD_DIAG_PROCESS_WARN_US, 0),
      (0, 0, 1),
    ]
    for loop_max_us, process_max_us, can_timeouts in cases:
      with self.subTest(loop_max_us=loop_max_us, process_max_us=process_max_us, can_timeouts=can_timeouts):
        self.assertTrue(should_log_card_diagnostics(loop_max_us, process_max_us, can_timeouts))


def _parser(*states):
  return SimpleNamespace(message_states={s.address: s for s in states}, _last_update_nanos=10**9, bus_timeout=False)


def _state(address, name, last_nanos):
  state = MessageState(address=address, name=name, size=8, signals=[])
  state.timestamps.append(last_nanos)  # default timeout_threshold is 1e5 ns
  return state


class TestInvalidCanMessages(unittest.TestCase):
  def test_names_timed_out_message(self):
    cp = _parser(_state(0x1A0, "OK", 10**9), _state(0x1CF, "CRUISE_BUTTONS", 0))
    self.assertEqual(invalid_can_messages([cp]), "CRUISE_BUTTONS 0x1CF")

  def test_caps_names_and_empty_when_valid(self):
    lost = _parser(*[_state(0x100 + i, f"M{i}", 0) for i in range(6)])
    self.assertEqual(invalid_can_messages([lost]), "M0 0x100, M1 0x101")
    self.assertEqual(invalid_can_messages([_parser(_state(0x1A0, "OK", 10**9))]), "")


@pytest.mark.parametrize("hda2", [1, 0])  # CanfdHDA2: HDA2 and non-HDA2 branches of the ALT_BUTTONS check
def test_ioniq5pe_missing_1cf_keeps_cruise_buttons(monkeypatch, hda2):
  from opendbc.car import gen_empty_fingerprint
  from opendbc.car.hyundai import carstate, interface
  from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiSafetyFlags
  values = {"HyundaiCameraSCC": 2, "CanfdHDA2": hda2}

  class FakeParams:
    def get_int(self, k, *a, **kw): return values.get(k, 0)
    def get(self, k, *a, **kw): return str(gen_empty_fingerprint()) if k == "FingerPrints" else None
    def get_bool(self, k, *a, **kw): return False
    def __getattr__(self, name): return lambda *a, **kw: 0

  fake = FakeParams()
  monkeypatch.setattr(interface, "Params", lambda: fake)
  monkeypatch.setattr(carstate, "Params", lambda: fake)
  # Another CAN-FD car without 0x1CF is still the ALT_BUTTONS variant
  ev6 = interface.CarInterface.get_params(CAR.KIA_EV6, gen_empty_fingerprint(), [], False, False, False)
  assert ev6.flags & HyundaiFlags.CANFD_ALT_BUTTONS
  CP = interface.CarInterface.get_params(CAR.HYUNDAI_IONIQ_5_PE, gen_empty_fingerprint(), [], False, False, False)
  assert not CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS
  assert not any(cfg.safetyParam & HyundaiSafetyFlags.CANFD_ALT_BUTTONS for cfg in CP.safetyConfigs)

  # 0x1CF stays registered, so its absence becomes canValid=False (canError) instead of a silent variant switch
  CI = interface.CarInterface(CP)
  assert any(0x1CF in cp.message_states for cp in CI.can_parsers.values())
  t = time.monotonic_ns() + 3 * 10**9  # past the parser's 2 s grace for never-seen messages
  for i in range(10):
    CS = CI.update([(t + i * 10**7, [])])
  assert not CS.canValid
  assert invalid_can_messages(CI.can_parsers.values())


if __name__ == "__main__":
  unittest.main()
