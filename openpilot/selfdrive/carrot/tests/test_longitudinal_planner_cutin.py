from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.cutin_predecel import (
  apply_cutin_predecel_accel_limit,
  get_cutin_predecel_accel_limit,
)


def radar_state_with_risk(**overrides):
  values = {
    "status": True,
    "radar": True,
    "score": 0.9,
    "dRel": 20.0,
    "vRel": -6.5,
  }
  values.update(overrides)
  return SimpleNamespace(leadCutInRisk=SimpleNamespace(**values))


def test_cutin_predecel_requests_negative_acceleration_at_target_ttc() -> None:
  limit = get_cutin_predecel_accel_limit(radar_state_with_risk())

  assert limit == pytest.approx(-0.608, abs=0.002)


@pytest.mark.parametrize(
  "overrides",
  (
    {"status": False},
    {"radar": False},
    {"score": 0.1},
    {"dRel": -1.0},
    {"vRel": 0.0},
  ),
)
def test_cutin_predecel_ignores_invalid_or_weak_risk(overrides) -> None:
  assert get_cutin_predecel_accel_limit(
    radar_state_with_risk(**overrides),
  ) is None


def test_cutin_predecel_is_bounded_for_urgent_and_distant_risk() -> None:
  urgent = get_cutin_predecel_accel_limit(
    radar_state_with_risk(dRel=8.0, vRel=-8.0),
  )
  distant = get_cutin_predecel_accel_limit(
    radar_state_with_risk(dRel=40.0, vRel=-4.0),
  )

  assert urgent == pytest.approx(-0.65)
  assert distant == pytest.approx(-0.25)


def test_cutin_predecel_reduces_positive_acceleration_then_reaches_decel() -> None:
  acceleration = 0.96
  limits = []
  for _ in range(11):
    acceleration = apply_cutin_predecel_accel_limit(
      1.0,
      acceleration,
      -0.608,
    )
    limits.append(acceleration)

  assert limits[0] == pytest.approx(0.81)
  assert limits[6] == pytest.approx(-0.09)
  assert limits[-1] == pytest.approx(-0.608)


def test_normal_accel_limit_keeps_existing_small_step() -> None:
  assert apply_cutin_predecel_accel_limit(
    -1.0,
    0.5,
    None,
  ) == pytest.approx(0.45)
