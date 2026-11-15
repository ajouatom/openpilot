import unittest
from types import SimpleNamespace

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


if __name__ == "__main__":
  unittest.main()
