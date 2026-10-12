import numpy as np

from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.selfdrived.impact_detector import ImpactDetector


def test_changed_pose_calibration_and_invalid_inputs_do_not_reuse_old_transform():
  rng = np.random.default_rng(108)
  detector = ImpactDetector()
  for i in range(250):
    if i % 5 == 0:
      orientation = rng.uniform(-.4, .4, 3)
    if i % 23 == 0:
      calibration = rng.uniform(-.2, .2, 3)
    sensor = rng.uniform(-20., 20., 3)
    now = 1.+i*.01
    if i % 31 == 0:
      result = detector.update(now=now, timestamp=now, sensor=[float('nan')]*3,
                               orientation=orientation, calibration=calibration, valid=True)
      assert result == (False, False, None)
    expected = rot_from_euler(calibration).T @ (-sensor[::-1] - rot_from_euler(orientation).T @ [0.,0.,-9.81])
    result = detector.update(now=now, timestamp=now, sensor=sensor,
                             orientation=orientation, calibration=calibration, valid=True)
    np.testing.assert_array_equal(result[2], expected)
