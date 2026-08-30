import pytest

from opendbc.car.hyundai.jerk_limits import calculate_canfd_jerk_limits, calculate_lead_braking_urgency


def test_response_modes_only_shape_non_urgent_lower_jerk() -> None:
  smooth_upper, smooth_lower = calculate_canfd_jerk_limits(
    accel=-1.0, jerk=-1.0, response_mode=0,
  )
  balanced_upper, balanced_lower = calculate_canfd_jerk_limits(
    accel=-1.0, jerk=-1.0, response_mode=1,
  )
  sync_upper, sync_lower = calculate_canfd_jerk_limits(
    accel=-1.0, jerk=-1.0, response_mode=2,
  )

  assert smooth_upper == balanced_upper == sync_upper
  assert smooth_lower < balanced_lower < sync_lower


def test_response_mode_never_slows_brake_release() -> None:
  uppers = [
    calculate_canfd_jerk_limits(accel=-3.0, jerk=3.0, response_mode=mode)[0]
    for mode in (0, 1, 2)
  ]

  assert uppers == pytest.approx([5.0, 5.0, 5.0])


def test_urgent_braking_overrides_smooth_comfort_cap() -> None:
  _, normal_lower = calculate_canfd_jerk_limits(
    accel=-1.0, jerk=-2.0, braking_urgency=0.0, response_mode=0,
  )
  _, urgent_lower = calculate_canfd_jerk_limits(
    accel=-1.0, jerk=-2.0, braking_urgency=1.0, response_mode=0,
  )

  assert normal_lower == pytest.approx(1.8)
  assert urgent_lower == pytest.approx(5.0)


def test_lead_braking_urgency_treats_both_leads_equally() -> None:
  safe = (True, 60.0, -1.0)
  dangerous = (True, 12.0, -6.0)

  danger_as_one = calculate_lead_braking_urgency(20.0, (dangerous, safe))
  danger_as_two = calculate_lead_braking_urgency(20.0, (safe, dangerous))

  assert danger_as_one == pytest.approx(danger_as_two)
  assert danger_as_one > 0.8


@pytest.mark.parametrize("mode", (0, 1, 2))
def test_emergency_acceleration_request_reaches_full_lower_limit(mode: int) -> None:
  _, lower = calculate_canfd_jerk_limits(
    accel=-4.0, jerk=-2.0, braking_urgency=0.0, response_mode=mode,
  )

  assert lower == pytest.approx(5.0)
