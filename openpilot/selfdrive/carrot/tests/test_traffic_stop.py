import pytest

from openpilot.selfdrive.carrot.traffic_stop import (
  TrafficStopDistanceTracker,
  get_traffic_stop_accel_floor,
  get_traffic_stop_obstacle_distance,
  get_traffic_stop_reference_speed,
  get_virtual_traffic_stop_distance,
  is_traffic_stop_entry_allowed,
  should_limit_traffic_stop_accel,
)


@pytest.mark.parametrize("speed_kph", [0.0, 30.0, 60.0, 100.0, 140.0])
def test_virtual_stop_distance_never_exceeds_model(speed_kph):
  for model_distance in (0.0, 10.0, 15.0, 30.0, 60.0, 120.0):
    adjusted = get_virtual_traffic_stop_distance(model_distance, speed_kph)
    assert 0.0 <= adjusted <= model_distance


def test_virtual_stop_distance_is_unmodified_when_stopped_and_fades_near_line():
  assert get_virtual_traffic_stop_distance(100.0, 0.0) == pytest.approx(100.0)
  assert get_virtual_traffic_stop_distance(15.0, 100.0) == pytest.approx(13.65)


def test_virtual_stop_distance_advances_early_and_fades_near_line():
  # Around the supplied route's 62 km/h signal-stop onset, retain the historical
  # correction strength while making it available to the MPC from the first frame.
  assert get_virtual_traffic_stop_distance(110.0, 62.0) == pytest.approx(89.54)
  assert get_virtual_traffic_stop_distance(40.0, 62.0) == pytest.approx(34.048)
  assert get_virtual_traffic_stop_distance(20.0, 62.0) == pytest.approx(18.512)


def test_virtual_stop_distance_is_monotonic_in_model_distance():
  adjusted = [get_virtual_traffic_stop_distance(distance, 80.0) for distance in range(0, 151)]
  assert adjusted == sorted(adjusted)


def test_signal_stop_reference_speed_does_not_relax_during_deceleration():
  reference_speed = get_traffic_stop_reference_speed(69.23, None)
  assert get_traffic_stop_reference_speed(34.84, reference_speed) == pytest.approx(69.23)
  assert get_traffic_stop_reference_speed(71.0, reference_speed) == pytest.approx(71.0)


def test_latched_entry_speed_keeps_mid_stop_distance_advanced():
  model_distance = 40.0
  current_speed_distance = get_virtual_traffic_stop_distance(model_distance, 30.0)
  entry_speed_distance = get_virtual_traffic_stop_distance(model_distance, 69.23)
  assert current_speed_distance == pytest.approx(37.12)
  assert entry_speed_distance == pytest.approx(33.35392)
  assert entry_speed_distance < current_speed_distance


def test_configured_obstacle_adjustment_is_used_at_all_speeds():
  assert get_traffic_stop_obstacle_distance(100.0, -1.5) == pytest.approx(98.5)
  assert get_traffic_stop_obstacle_distance(1.0, -1.5) == 0.0
  assert get_traffic_stop_obstacle_distance(0.0, -2.0) == 0.0


def test_signal_stop_accel_floor_limits_early_braking_with_margin():
  # Supplied d76 route: 78.52 km/h and about 138.5 m available at detection.
  accel_floor = get_traffic_stop_accel_floor(78.52 / 3.6, 138.5, 5.5)
  assert accel_floor == pytest.approx(-2.2314, abs=1e-4)


def test_signal_stop_accel_floor_limits_d85_onset_to_comfortable_braking():
  # Supplied d85 route: 76.41 km/h with about 137 m to the physical stop line.
  accel_floor = get_traffic_stop_accel_floor(76.41 / 3.6, 137.0, 5.5)
  assert accel_floor == -2.2


def test_signal_stop_accel_floor_releases_when_distance_is_short():
  accel_floor = get_traffic_stop_accel_floor(62.0 / 3.6, 60.0, 5.5)
  assert accel_floor < -3.8
  assert get_traffic_stop_accel_floor(20.0, 10.0, 5.5) == -4.0


def test_signal_stop_accel_floor_fails_safe_for_invalid_distance():
  assert get_traffic_stop_accel_floor(20.0, float("nan"), 5.5) == -4.0


def test_signal_stop_accel_floor_releases_monotonically_as_margin_shrinks():
  floors = [get_traffic_stop_accel_floor(20.0, distance, 5.5) for distance in range(20, 201)]
  assert all(-4.0 <= floor <= -2.2 for floor in floors)
  assert floors == sorted(floors)


def test_stop_distance_tracker_rejects_a_transient_nearer_line_in_world_coordinates():
  tracker = TrafficStopDistanceTracker(sample_count=4)
  assert tracker.update(100.0, 0.0) == pytest.approx(100.0)
  assert tracker.update(99.0, 1.0) == pytest.approx(99.0)
  assert tracker.update(70.0, 1.0) == pytest.approx(98.0)
  assert tracker.update(97.0, 1.0) == pytest.approx(97.0)


def test_stop_distance_tracker_accepts_a_persistent_nearer_line():
  tracker = TrafficStopDistanceTracker(sample_count=3)
  tracker.update(100.0, 0.0)
  assert tracker.update(70.0, 1.0) == pytest.approx(99.0)
  assert tracker.update(69.0, 1.0) == pytest.approx(98.0)
  assert tracker.update(68.0, 1.0) == pytest.approx(68.0)


def test_stop_distance_tracker_accepts_a_farther_line_immediately():
  tracker = TrafficStopDistanceTracker(sample_count=4)
  tracker.update(100.0, 0.0)
  assert tracker.update(110.0, 1.0) == pytest.approx(110.0)


@pytest.mark.parametrize("source", ["cruise", "e2e"])
def test_signal_stop_accel_limit_covers_both_signal_sources(source):
  assert should_limit_traffic_stop_accel(True, source)


@pytest.mark.parametrize("source", ["lead0", "lead1"])
def test_signal_stop_accel_limit_yields_to_real_lead_sources(source):
  assert not should_limit_traffic_stop_accel(True, source)
  assert not should_limit_traffic_stop_accel(False, "e2e")


@pytest.mark.parametrize("steering_angle_deg", [-49.9, -20.0, 0.0, 20.0, 49.9])
def test_traffic_stop_entry_is_allowed_below_50_degrees(steering_angle_deg):
  assert is_traffic_stop_entry_allowed(steering_angle_deg)


@pytest.mark.parametrize("steering_angle_deg", [-50.0, 50.0, -90.0, 90.0])
def test_traffic_stop_entry_is_blocked_at_or_above_50_degrees(steering_angle_deg):
  assert not is_traffic_stop_entry_allowed(steering_angle_deg)
