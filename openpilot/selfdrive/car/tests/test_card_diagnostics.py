import unittest

from openpilot.selfdrive.car.card_diagnostics import (
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


if __name__ == "__main__":
  unittest.main()
