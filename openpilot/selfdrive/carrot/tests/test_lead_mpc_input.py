from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.lead_mpc_input import (
  ApproachDistanceController,
  extrapolate_lead_motion,
)


DT = 0.05


def lead(track_id: int, d_rel: float, v_rel: float):
  return SimpleNamespace(
    status=True,
    radarTrackId=track_id,
    dRel=d_rel,
    vRel=v_rel,
  )


def update(controller, leads, desired=(10.0, 10.0), v_ego=10.0):
  return controller.update(
    base_t_follow=0.5,
    v_ego=v_ego,
    leads=leads,
    desired_distances=desired[:len(leads)],
  )


def test_steady_short_tf_is_unchanged() -> None:
  controller = ApproachDistanceController(DT)
  result = update(controller, (lead(1, 20.0, 0.0),))

  assert result.t_follow == pytest.approx(0.5)
  assert result.lead_active == (False,)


def test_non_closing_lead_does_not_create_a_stale_distance_floor() -> None:
  controller = ApproachDistanceController(DT)
  update(controller, (lead(1, 20.0, 1.0),), desired=(10.0,))
  result = update(controller, (lead(1, 22.0, 2.0),), desired=(5.0,))

  assert result.distance_floor == pytest.approx(5.0)
  assert result.t_follow == pytest.approx(0.5)


def test_approaching_preserves_the_pre_deceleration_gap() -> None:
  controller = ApproachDistanceController(DT)
  update(controller, (lead(1, 20.0, 0.0),))
  result = update(controller, (lead(1, 20.0, -0.3),))

  assert result.distance_floor == pytest.approx(20.0)
  assert result.t_follow == pytest.approx(1.5)
  assert result.lead_active == (True,)

  result = update(controller, (lead(1, 18.0, -0.4),), desired=(11.0,))
  assert result.distance_floor == pytest.approx(20.0)
  assert result.t_follow == pytest.approx(1.4)


def test_distance_floor_releases_after_relative_speed_settles() -> None:
  controller = ApproachDistanceController(DT)
  update(controller, (lead(1, 20.0, -0.3),))
  first_release = update(controller, (lead(1, 20.0, 0.0),))

  assert first_release.distance_floor == pytest.approx(19.9)
  assert first_release.t_follow == pytest.approx(1.49)
  assert first_release.lead_active == (False,)

  result = first_release
  for _ in range(100):
    result = update(controller, (lead(1, 20.0, 0.0),))
  assert result.t_follow == pytest.approx(0.5)


def test_track_change_does_not_reuse_an_old_distance_floor() -> None:
  controller = ApproachDistanceController(DT)
  update(controller, (lead(1, 25.0, -0.5),))
  result = update(controller, (lead(2, 14.0, -0.5),), desired=(11.0,))

  assert result.distance_floor == pytest.approx(14.0)
  assert result.t_follow == pytest.approx(0.8)


def test_lead_order_has_no_effect_for_stable_track_ids() -> None:
  lead_a = lead(11, 18.0, -0.4)
  lead_b = lead(22, 24.0, -0.8)
  controller_ab = ApproachDistanceController(DT)
  controller_ba = ApproachDistanceController(DT)

  result_ab = update(controller_ab, (lead_a, lead_b), desired=(10.0, 12.0))
  result_ba = update(controller_ba, (lead_b, lead_a), desired=(12.0, 10.0))

  assert result_ab.t_follow == pytest.approx(result_ba.t_follow)
  assert result_ab.distance_floor == pytest.approx(result_ba.distance_floor)


def test_duplicate_track_observations_are_order_independent() -> None:
  closing = lead(11, 18.0, -0.4)
  opening = lead(11, 17.0, 0.2)
  controller_co = ApproachDistanceController(DT)
  controller_oc = ApproachDistanceController(DT)

  result_co = update(controller_co, (closing, opening), desired=(10.0, 11.0))
  result_oc = update(controller_oc, (opening, closing), desired=(11.0, 10.0))

  assert result_co.t_follow == pytest.approx(result_oc.t_follow)
  assert result_co.distance_floor == pytest.approx(result_oc.distance_floor)
  assert result_co.lead_active == (True, True)
  assert result_oc.lead_active == (True, True)


def test_lead_motion_uses_acceleration_tau_and_physical_jerk_integration() -> None:
  time_indices = np.array((0.0, 0.5, 1.0))
  time_differences = np.array((0.0, 0.5, 0.5))
  position, velocity, acceleration = extrapolate_lead_motion(
    x_lead=20.0,
    v_lead=15.0,
    a_lead=-1.0,
    a_lead_tau=1.0,
    j_lead=-1.0,
    time_indices=time_indices,
    time_differences=time_differences,
  )

  assert acceleration[0] == pytest.approx(-1.5)
  assert acceleration[1] < -1.0
  assert velocity[-1] < 15.0
  assert position[-1] > position[0]
