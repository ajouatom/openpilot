import pytest

from openpilot.selfdrive.carrot.traffic_stop import (
  get_traffic_stop_obstacle_distance,
  is_traffic_stop_entry_allowed,
)


def test_signal_obstacle_is_unmodified_outside_cruise_safe_distance():
  assert get_traffic_stop_obstacle_distance(130.0, 120.0, 0.0) == pytest.approx(130.0)


def test_signal_obstacle_matches_historical_behavior_at_release_distance():
  assert get_traffic_stop_obstacle_distance(50.0, 120.0, 0.0) == pytest.approx(50.0)
  assert get_traffic_stop_obstacle_distance(40.0, 120.0, 0.0) == pytest.approx(40.0)


def test_signal_obstacle_releases_smoothly_between_cruise_and_50m():
  cruise_obstacle = 120.0
  distances = [120.0, 110.0, 100.0, 90.0, 80.0, 70.0, 60.0, 50.0]
  obstacles = [get_traffic_stop_obstacle_distance(distance, cruise_obstacle, 0.0) for distance in distances]

  assert obstacles[0] == pytest.approx(cruise_obstacle)
  assert obstacles[-1] == pytest.approx(50.0)
  assert obstacles == sorted(obstacles, reverse=True)
  for signal_distance, obstacle_distance in zip(distances, obstacles, strict=True):
    assert signal_distance <= obstacle_distance <= cruise_obstacle


def test_signal_obstacle_begins_with_a_small_constraint_and_strengthens_gradually():
  cruise_obstacle = 120.0
  assert get_traffic_stop_obstacle_distance(110.0, cruise_obstacle, 0.0) == pytest.approx(118.5714286)
  assert get_traffic_stop_obstacle_distance(85.0, cruise_obstacle, 0.0) == pytest.approx(102.5)
  assert get_traffic_stop_obstacle_distance(60.0, cruise_obstacle, 0.0) == pytest.approx(68.5714286)


def test_configured_distance_adjust_is_applied_before_release():
  assert get_traffic_stop_obstacle_distance(100.0, 120.0, -2.0) == pytest.approx(113.0857143)
  assert get_traffic_stop_obstacle_distance(1.0, 120.0, -2.0) == 0.0


@pytest.mark.parametrize("steering_angle_deg", [-49.9, -20.0, 0.0, 20.0, 49.9])
def test_traffic_stop_entry_is_allowed_below_50_degrees(steering_angle_deg):
  assert is_traffic_stop_entry_allowed(steering_angle_deg)


@pytest.mark.parametrize("steering_angle_deg", [-50.0, 50.0, -90.0, 90.0])
def test_traffic_stop_entry_is_blocked_at_or_above_50_degrees(steering_angle_deg):
  assert not is_traffic_stop_entry_allowed(steering_angle_deg)
