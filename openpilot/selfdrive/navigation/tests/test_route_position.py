import numpy as np
import pytest

from openpilot.selfdrive.navigation.route import RouteTracker


def test_rejected_gps_cannot_poison_progress_and_block_recovery():
  route = np.array([[127., 37.], [127., 37.01]])
  tracker = RouteTracker(route)
  expected = tracker.update_pose(127., 37.001)
  progress = tracker.progress
  # This far-away fix projects forward on the same long segment but must
  # never become the monotonic progress floor for later, correct fixes.
  assert tracker.update_pose(127.01, 37.008) is None
  assert tracker.progress == progress
  assert tracker.update_pose(127., 37.001) == expected


def test_invalid_first_position_does_not_seed_route_progress():
  tracker = RouteTracker(np.array([[127., 37.], [127., 37.01]]))
  assert tracker.update_pose(128., 38.) is None
  assert tracker.progress == -np.inf
  assert tracker.update_pose(127., 37.001) is not None


def test_valid_positions_preserve_original_projection_and_extrapolation():
  route = np.array([[127., 37.], [127., 37.01]])
  original, guarded = RouteTracker(route), RouteTracker(route)
  for lat in np.linspace(37., 37.005, 50):
    expected = original.pose(original.project(127., lat) + 3.)
    assert guarded.update_pose(127., lat, 3.) == pytest.approx(expected)
    assert guarded.progress == original.progress


def test_extrapolation_rejection_also_retains_previous_progress():
  tracker = RouteTracker(np.array([[127., 37.], [127., 37.01]]))
  tracker.update_pose(127., 37.001)
  progress = tracker.progress
  assert tracker.update_pose(127., 37.002, 100.) is None
  assert tracker.progress == progress
