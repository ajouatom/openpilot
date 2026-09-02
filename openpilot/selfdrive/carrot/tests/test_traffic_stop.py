import pytest

from openpilot.selfdrive.carrot.traffic_stop import (
  MODEL_LEAD_STOP_OFFSET_M,
  TrafficStopModelLeadMatcher,
  get_traffic_stop_distance_adjust,
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


def model_lead_match_update(matcher, *, stop_distance=50.0, lead_distance=52.0,
                            lead_probability=0.97, lead_velocity=0.1,
                            lead_x_std=2.0, lead_y_std=0.3, lead_v_std=0.2,
                            stop_active=True, allow_confirmation=True,
                            active_lead=False):
  return matcher.update(
    stop_active=stop_active,
    allow_confirmation=allow_confirmation,
    active_lead=active_lead,
    stop_distance=stop_distance,
    lead_probability=lead_probability,
    lead_distance=lead_distance,
    lead_velocity=lead_velocity,
    lead_x_std=lead_x_std,
    lead_y_std=lead_y_std,
    lead_v_std=lead_v_std,
  )


def test_stationary_model_lead_confirms_two_meter_stop_offset_after_five_frames():
  matcher = TrafficStopModelLeadMatcher()

  for _ in range(4):
    assert model_lead_match_update(matcher) == 0.0
  assert model_lead_match_update(matcher) == MODEL_LEAD_STOP_OFFSET_M


def test_stationary_model_lead_offset_latches_until_stop_or_real_lead_takes_over():
  matcher = TrafficStopModelLeadMatcher()
  for _ in range(5):
    model_lead_match_update(matcher)

  assert model_lead_match_update(matcher, lead_probability=0.0) == MODEL_LEAD_STOP_OFFSET_M
  assert model_lead_match_update(matcher, active_lead=True) == 0.0

  for _ in range(5):
    model_lead_match_update(matcher)
  assert model_lead_match_update(matcher, stop_active=False) == 0.0


@pytest.mark.parametrize("invalid_input", [
  {"lead_probability": 0.89},
  {"lead_velocity": 2.1},
  {"lead_distance": 54.0},
  {"lead_x_std": 5.1},
  {"lead_y_std": 0.76},
  {"lead_v_std": 1.51},
  {"lead_x_std": -0.1},
  {"allow_confirmation": False},
])
def test_stationary_model_lead_rejects_uncertain_or_unrelated_candidates(invalid_input):
  matcher = TrafficStopModelLeadMatcher()
  for _ in range(10):
    assert model_lead_match_update(matcher, **invalid_input) == 0.0


def test_model_lead_offset_replaces_signal_adjust_only_for_confirmed_vehicle_stop():
  assert get_traffic_stop_distance_adjust(-1.7, 10.0, 0.0) == pytest.approx(-1.7)
  assert get_traffic_stop_distance_adjust(-1.7, 0.05, 0.0) == pytest.approx(-2.0)
  assert get_traffic_stop_distance_adjust(-1.7, 10.0, MODEL_LEAD_STOP_OFFSET_M) == pytest.approx(2.0)
  assert get_traffic_stop_distance_adjust(-1.7, 0.05, MODEL_LEAD_STOP_OFFSET_M) == pytest.approx(2.0)


def test_elantra_log_geometry_uses_the_two_meter_gap_as_the_virtual_vehicle_position():
  # Near the supplied stop, the planner's remaining stop distance was 5.3 m
  # while the stationary model lead was about 7.2 m away. The ordinary stopped
  # signal adjustment placed the obstacle behind the ego target; +2 m restores
  # the approximate vehicle position and lets the 5.5 m fixed gap do its job.
  stop_distance = 5.3
  cruise_obstacle = 120.0
  ordinary_adjust = get_traffic_stop_distance_adjust(-1.7, 0.05, 0.0)
  vehicle_adjust = get_traffic_stop_distance_adjust(-1.7, 0.05, MODEL_LEAD_STOP_OFFSET_M)

  ordinary_obstacle = get_traffic_stop_obstacle_distance(stop_distance, cruise_obstacle, ordinary_adjust)
  vehicle_obstacle = get_traffic_stop_obstacle_distance(stop_distance, cruise_obstacle, vehicle_adjust)

  assert ordinary_obstacle == pytest.approx(3.3)
  assert vehicle_obstacle == pytest.approx(7.3)
  assert vehicle_obstacle - 5.5 == pytest.approx(1.8)


@pytest.mark.parametrize("steering_angle_deg", [-49.9, -20.0, 0.0, 20.0, 49.9])
def test_traffic_stop_entry_is_allowed_below_50_degrees(steering_angle_deg):
  assert is_traffic_stop_entry_allowed(steering_angle_deg)


@pytest.mark.parametrize("steering_angle_deg", [-50.0, 50.0, -90.0, 90.0])
def test_traffic_stop_entry_is_blocked_at_or_above_50_degrees(steering_angle_deg):
  assert not is_traffic_stop_entry_allowed(steering_angle_deg)
