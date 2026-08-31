import pytest

from opendbc.car.hyundai.jerk_limits import calculate_canfd_jerk_limits


def test_non_urgent_limits_keep_historical_jerk_behavior() -> None:
  upper, lower = calculate_canfd_jerk_limits(
    accel=-1.0, jerk=-1.0, braking_urgency=0.0,
  )

  assert upper == pytest.approx(1.0)
  assert lower == pytest.approx(4.0)


@pytest.mark.parametrize("accel", (0.0, -0.8, -1.5, -2.0, -3.2, -4.0))
def test_accel_request_alone_does_not_raise_lower_limit(accel: float) -> None:
  _, lower = calculate_canfd_jerk_limits(
    accel=accel, jerk=0.0, braking_urgency=0.0,
  )

  assert lower == pytest.approx(1.0)


def test_tracking_error_assist_is_preserved() -> None:
  _, lower = calculate_canfd_jerk_limits(
    accel=-2.0, jerk=0.0, tracking_error=0.75, braking_urgency=0.0,
  )

  assert lower == pytest.approx(3.0)


def test_brake_release_upper_limit_is_unchanged_by_urgency() -> None:
  upper, _ = calculate_canfd_jerk_limits(
    accel=-3.0, jerk=3.0, braking_urgency=1.0,
  )

  assert upper == pytest.approx(5.0)


def test_urgent_braking_overrides_smooth_comfort_cap() -> None:
  _, normal_lower = calculate_canfd_jerk_limits(
    accel=-1.0, jerk=0.0, braking_urgency=0.0,
  )
  _, urgent_lower = calculate_canfd_jerk_limits(
    accel=-1.0, jerk=0.0, braking_urgency=1.0,
  )

  assert normal_lower == pytest.approx(1.0)
  assert urgent_lower == pytest.approx(5.0)


def test_emergency_acceleration_request_reaches_full_lower_limit() -> None:
  _, lower = calculate_canfd_jerk_limits(
    accel=-4.0, jerk=0.0, braking_urgency=1.0,
  )

  assert lower == pytest.approx(5.0)
