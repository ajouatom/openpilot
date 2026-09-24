import unittest

from sof_history import classify_sof_history


class TestSofHistory(unittest.TestCase):
  def test_hardware_frame_between_callbacks(self):
    self.assertEqual(classify_sof_history(1_000_000_000, 1_100_000_000, 1_050_000_000, 1_100_000_000),
                     "intervening_hardware_sof")

  def test_hardware_itself_records_long_interval(self):
    self.assertEqual(classify_sof_history(1_000_000_000, 1_100_000_000, 1_000_000_000, 1_100_000_000),
                     "hardware_sof_gap")

  def test_register_rollover_during_read_cannot_establish_loss(self):
    self.assertEqual(classify_sof_history(1_000_000_000, 1_100_000_000, 1_100_000_000, 1_150_000_000),
                     "unstable_register_sample")

  def test_missing_previous_register_is_not_sensor_failure(self):
    self.assertEqual(classify_sof_history(1_000_000_000, 1_100_000_000, 0, 1_100_000_000),
                     "insufficient_history")

  def test_duplicate_is_separate_from_frame_gap(self):
    self.assertEqual(classify_sof_history(1_000_000_000, 1_000_000_000, 950_000_000, 1_000_000_000),
                     "nonadvancing_hardware_timestamp")

  def test_normal_cadence_and_invalid_history(self):
    self.assertEqual(classify_sof_history(1_000_000_000, 1_050_000_000, 1_000_000_000, 1_050_000_000),
                     "normal_observed_interval")
    self.assertEqual(classify_sof_history(1_000_000_000, 1_100_000_000, 1_150_000_000, 1_100_000_000),
                     "invalid_hardware_history")


if __name__ == "__main__":
  unittest.main()
