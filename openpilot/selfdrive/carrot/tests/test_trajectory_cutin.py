from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar_motion.primary import RadarPointSnapshot
from openpilot.selfdrive.carrot.radar_motion.trajectory_cutin import (
  TrajectoryCutInDetector,
  prediction_horizon_s,
)


PATH = ((0.0, 0.0), (100.0, 0.0))
MODEL = SimpleNamespace(leadsV3=())


def point(
  track_id: int,
  source: str,
  d_rel: float,
  y_rel: float,
  *,
  v_ego: float = 10.0,
  v_rel: float = -2.0,
  yv_rel: float = 0.0,
) -> RadarPointSnapshot:
  return RadarPointSnapshot(
    track_id=track_id,
    source=source,
    d_rel=d_rel,
    y_rel=y_rel,
    v_rel=v_rel,
    a_rel=0.0,
    yv_rel=yv_rel,
    v_lead=v_ego + v_rel,
    a_lead=0.0,
    j_lead=0.0,
    measured=True,
  )


def corner_series(
  detector: TrajectoryCutInDetector,
  lateral_values: tuple[float, ...],
  *,
  d_rel: float = 12.0,
  v_ego: float = 10.0,
  v_rel: float = -2.0,
) -> object:
  estimate = None
  for index, y_rel in enumerate(lateral_values):
    corner = point(
      1200,
      "corner235",
      d_rel + v_rel * index * 0.1,
      y_rel,
      v_ego=v_ego,
      v_rel=v_rel,
      yv_rel=-0.8,
    )
    front = point(
      48,
      "frontRadar",
      corner.d_rel,
      y_rel,
      v_ego=v_ego,
      v_rel=v_rel,
    )
    estimate = detector.update(
      index * 0.1,
      v_ego,
      (corner,),
      PATH,
      MODEL,
      cross_sensor_matches={(corner.source, corner.track_id): front},
    )[0]
  assert estimate is not None
  return estimate


def test_low_speed_prediction_looks_farther_ahead() -> None:
  assert prediction_horizon_s(2.0) > prediction_horizon_s(25.0)


def test_higher_sensitivity_confirms_the_same_trajectory_earlier() -> None:
  def first_detection_index(sensitivity: int) -> int | None:
    detector = TrajectoryCutInDetector(sensitivity)
    for index in range(12):
      y_rel = 3.0 - 0.1 * index
      corner = point(
        1200, "corner235", 12.0 - 0.2 * index, y_rel, yv_rel=-0.8,
      )
      front = point(
        48, "frontRadar", corner.d_rel, y_rel,
      )
      estimate = detector.update(
        index * 0.1,
        10.0,
        (corner,),
        PATH,
        MODEL,
        cross_sensor_matches={(corner.source, corner.track_id): front},
      )[0]
      if estimate.confirmed_cutin:
        return index
    return None

  assert first_detection_index(5) < first_detection_index(1)


def test_corner_trajectory_detects_early_and_controls_when_relevant() -> None:
  estimate = corner_series(
    TrajectoryCutInDetector(),
    (3.0, 2.8, 2.55, 2.30, 2.10),
  )

  assert estimate.confirmed_cutin
  assert estimate.control_eligible
  assert estimate.future_d_path < estimate.d_path


def test_far_corner_trajectory_is_detected_and_promoted_to_l2() -> None:
  estimate = corner_series(
    TrajectoryCutInDetector(),
    (3.00, 2.91, 2.82, 2.73, 2.64, 2.55, 2.46, 2.37, 2.28, 2.19, 2.10),
    d_rel=32.0,
    v_rel=-2.0,
  )

  assert estimate.confirmed_cutin
  assert estimate.control_eligible


def test_confirmed_away_cutin_is_l2_without_predecel() -> None:
  estimate = corner_series(
    TrajectoryCutInDetector(),
    (3.0, 2.8, 2.55, 2.30, 2.10),
    d_rel=18.0,
    v_ego=10.0,
    v_rel=4.0,
  )

  assert estimate.confirmed_cutin
  assert estimate.control_eligible
  assert not estimate.predecel_risk


def test_cross_sensor_corner_slot_handoff_preserves_motion_history() -> None:
  detector = TrajectoryCutInDetector()
  continuity_ids = []
  estimate = None
  for index, y_rel in enumerate((3.0, 2.85, 2.70, 2.55, 2.40, 2.25)):
    corner = point(
      202 if index < 3 else 201,
      "corner235",
      7.0 - 0.1 * index,
      y_rel,
      v_rel=-1.0,
      yv_rel=-0.6,
    )
    front = point(
      59,
      "frontRadar",
      corner.d_rel + 0.1,
      y_rel + 0.1,
      v_rel=-1.0,
    )
    estimate = detector.update(
      index * 0.1,
      10.0,
      (corner,),
      PATH,
      MODEL,
      cross_sensor_matches={(corner.source, corner.track_id): front},
    )[0]
    continuity_ids.append(estimate.continuity_id)

  assert estimate is not None
  assert len(set(continuity_ids)) == 1
  assert estimate.history_s == 0.5
  assert estimate.inward_progress > 0.5
  assert estimate.confirmed_cutin


def test_far_cross_sensor_slot_change_keeps_raw_histories_separate() -> None:
  detector = TrajectoryCutInDetector()
  continuity_ids = []
  for index, track_id in enumerate((202, 201)):
    corner = point(
      track_id,
      "corner235",
      20.0,
      3.0 - 0.1 * index,
      v_rel=-1.0,
      yv_rel=-0.6,
    )
    front = point(
      40,
      "frontRadar",
      corner.d_rel,
      corner.y_rel,
      v_rel=-1.0,
    )
    estimate = detector.update(
      index * 0.1,
      10.0,
      (corner,),
      PATH,
      MODEL,
      cross_sensor_matches={(corner.source, corner.track_id): front},
    )[0]
    continuity_ids.append(estimate.continuity_id)

  assert continuity_ids[0] != continuity_ids[1]


def test_reversed_unsupported_trajectory_clears_confirmed_hold() -> None:
  detector = TrajectoryCutInDetector()
  estimate = None
  for index, y_rel in enumerate((3.0, 2.85, 2.70, 2.55, 2.40)):
    corner = point(
      3545,
      "corner235",
      20.0,
      y_rel,
      v_rel=-2.0,
      yv_rel=-0.8,
    )
    estimate = detector.update(
      index * 0.1, 10.0, (corner,), PATH, MODEL,
    )[0]

  assert estimate is not None and estimate.confirmed_cutin
  for offset, y_rel in enumerate((2.55, 2.70, 2.85, 3.0, 3.15), start=5):
    corner = point(
      3545,
      "corner235",
      20.0,
      y_rel,
      v_rel=1.0,
      yv_rel=0.8,
    )
    estimate = detector.update(
      offset * 0.1, 10.0, (corner,), PATH, MODEL,
    )[0]

  assert not estimate.raw_cutin
  assert not estimate.confirmed_cutin
  assert not estimate.predecel_risk


def test_far_inconsistent_corner_motion_stays_alert_only() -> None:
  detector = TrajectoryCutInDetector()
  estimate = None
  for index, y_rel in enumerate((3.20, 3.00, 2.78, 2.55, 2.30, 2.10)):
    corner = point(
      1106, "corner235", 30.0 - 0.1 * index, y_rel,
      v_ego=15.0, v_rel=-1.0, yv_rel=0.0,
    )
    front = point(
      51, "frontRadar", corner.d_rel, y_rel,
      v_ego=15.0, v_rel=-1.0,
    )
    estimate = detector.update(
      index * 0.1,
      15.0,
      (corner,),
      PATH,
      MODEL,
      cross_sensor_matches={(corner.source, corner.track_id): front},
    )[0]

  assert estimate is not None
  assert estimate.confirmed_cutin
  assert not estimate.control_eligible


def test_fast_unstable_side_motion_uses_shorter_prediction_horizon() -> None:
  detector = TrajectoryCutInDetector()
  estimate = None
  for index, y_rel in enumerate((3.2, 2.7, 3.1, 2.6, 3.0, 2.5)):
    corner = point(
      3001, "corner235", 18.0 + 0.8 * index, y_rel,
      v_ego=10.0, v_rel=8.0, yv_rel=-0.2,
    )
    front = point(
      45, "frontRadar", corner.d_rel, y_rel,
      v_ego=10.0, v_rel=8.0,
    )
    estimate = detector.update(
      index * 0.1,
      10.0,
      (corner,),
      PATH,
      MODEL,
      cross_sensor_matches={(corner.source, corner.track_id): front},
    )[0]

  assert estimate is not None
  assert estimate.jittering
  assert estimate.horizon_s < prediction_horizon_s(10.0)
  assert not estimate.confirmed_cutin


def test_corner_mode_unmatched_front_uses_strong_recent_history() -> None:
  detector = TrajectoryCutInDetector()
  estimate = None
  for index, y_rel in enumerate((3.40, 3.35, 3.30, 3.20, 3.05, 2.90)):
    estimate = detector.update(
      index * 0.1,
      10.0,
      (point(
        62, "frontRadar", 10.0 + 0.15 * index, y_rel,
        v_ego=10.0, v_rel=1.5,
      ),),
      PATH,
      MODEL,
      vision_required_front=True,
    )[0]

  assert estimate is not None
  assert estimate.front_history_supported
  assert estimate.confirmed_cutin
  assert not estimate.control_eligible


def test_corner_mode_unmatched_front_rejects_velocity_identity_jump() -> None:
  detector = TrajectoryCutInDetector()
  estimate = None
  v_rels = (2.0, 1.8, -2.8, 0.2, 0.8, 1.0)
  for index, (y_rel, v_rel) in enumerate(zip(
    (3.40, 3.35, 3.30, 3.20, 3.05, 2.90),
    v_rels,
    strict=True,
  )):
    estimate = detector.update(
      index * 0.1,
      10.0,
      (point(
        57, "frontRadar", 10.0, y_rel,
        v_ego=10.0, v_rel=v_rel,
      ),),
      PATH,
      MODEL,
      vision_required_front=True,
    )[0]

  assert estimate is not None
  assert estimate.recent_v_rel_spread > 1.0
  assert not estimate.front_history_supported
  assert not estimate.confirmed_cutin


def test_corner_mode_unmatched_front_rejects_recent_curve_motion() -> None:
  detector = TrajectoryCutInDetector()
  estimate = None
  for index, y_rel in enumerate((3.40, 3.35, 3.30, 3.20, 3.05, 2.90)):
    estimate = detector.update(
      index * 0.1,
      10.0,
      (point(
        50, "frontRadar", 10.0 + 0.15 * index, y_rel,
        v_ego=10.0, v_rel=1.5,
      ),),
      PATH,
      MODEL,
      yaw_rate_rad_s=0.03,
      vision_required_front=True,
    )[0]

  assert estimate is not None
  assert estimate.recent_abs_yaw_max >= 0.02
  assert not estimate.front_history_supported
  assert not estimate.confirmed_cutin


def test_close_cross_sensor_reflection_without_motion_is_rejected() -> None:
  detector = TrajectoryCutInDetector()
  estimate = None
  for index in range(5):
    corner = point(
      201, "corner235", 1.8 - 0.15 * index, 2.60,
      v_ego=8.0, v_rel=-2.0,
    )
    front = point(
      51, "frontRadar", corner.d_rel, 1.80,
      v_ego=8.0, v_rel=-2.0,
    )
    estimate = detector.update(
      index * 0.1,
      8.0,
      (corner,),
      PATH,
      MODEL,
      cross_sensor_matches={(corner.source, corner.track_id): front},
    )[0]

  assert estimate is not None
  assert not estimate.confirmed_cutin


def test_close_paired_outer_body_reflection_detects_with_inward_motion() -> None:
  detector = TrajectoryCutInDetector()
  estimate = None
  for index, y_rel in enumerate((-2.70, -2.62, -2.55, -2.48, -2.42)):
    corner = point(
      16687, "corner235", 8.0 - 0.25 * index, y_rel,
      v_ego=5.0, v_rel=-2.5, yv_rel=0.20,
    )
    front = point(
      49, "frontRadar", corner.d_rel + 0.8, y_rel,
      v_ego=5.0, v_rel=-2.5,
    )
    estimate = detector.update(
      index * 0.1,
      5.0,
      (corner,),
      PATH,
      MODEL,
      cross_sensor_matches={(corner.source, corner.track_id): front},
    )[0]

  assert estimate is not None
  assert estimate.confirmed_cutin
  assert estimate.control_eligible


def test_close_uncorroborated_projected_crossing_is_rejected() -> None:
  detector = TrajectoryCutInDetector()
  estimate = None
  for index, y_rel in enumerate((4.9, 4.5, 4.1, 3.8, 3.5)):
    estimate = detector.update(
      index * 0.1,
      10.0,
      (point(
        3442, "corner235", 4.5 - 0.5 * index, y_rel,
        v_ego=10.0, v_rel=-1.0, yv_rel=-1.0,
      ),),
      PATH,
      MODEL,
    )[0]

  assert estimate is not None
  assert not estimate.confirmed_cutin


def test_far_uncorroborated_away_corner_path_jump_is_rejected() -> None:
  detector = TrajectoryCutInDetector()
  estimate = None
  for index, y_rel in enumerate((2.75, 2.45, 2.05, 1.65, 1.30, 1.05)):
    estimate = detector.update(
      index * 0.1,
      14.0,
      (point(
        205, "corner235", 38.0 + 0.3 * index, y_rel,
        v_ego=14.0, v_rel=3.0, yv_rel=-0.6,
      ),),
      PATH,
      MODEL,
    )[0]

  assert estimate is not None
  assert not estimate.confirmed_cutin
  assert not estimate.predecel_risk


def test_front_parallel_drift_stays_clear_until_body_overlap() -> None:
  detector = TrajectoryCutInDetector()
  estimates = ()
  for index, y_rel in enumerate((2.70, 2.55, 2.40, 2.25, 2.05)):
    estimates = detector.update(
      index * 0.1,
      20.0,
      (point(44, "frontRadar", 40.0, y_rel, v_ego=20.0),),
      PATH,
      MODEL,
    )

  assert estimates
  assert not estimates[0].confirmed_cutin


def test_close_fast_side_pass_does_not_become_front_cutin() -> None:
  detector = TrajectoryCutInDetector()
  estimate = None
  for index, (d_rel, y_rel) in enumerate((
    (7.0, 2.40),
    (5.8, 2.30),
    (4.6, 2.12),
    (3.4, 2.00),
  )):
    estimate = detector.update(
      index * 0.1,
      12.0,
      (point(
        32,
        "frontRadar",
        d_rel,
        y_rel,
        v_ego=12.0,
        v_rel=-6.0,
      ),),
      PATH,
      MODEL,
    )[0]

  assert estimate is not None
  assert not estimate.confirmed_cutin


def test_close_born_uncorroborated_front_target_is_filtered() -> None:
  detector = TrajectoryCutInDetector()
  estimate = None
  for index, (d_rel, y_rel) in enumerate((
    (3.60, 2.30),
    (4.05, 2.10),
    (3.50, 1.90),
    (3.70, 1.70),
  )):
    estimate = detector.update(
      index * 0.1,
      10.0,
      (point(
        50,
        "frontRadar",
        d_rel,
        y_rel,
        v_ego=10.0,
        v_rel=-0.5,
      ),),
      PATH,
      MODEL,
      yaw_rate_rad_s=0.035,
    )[0]

  assert estimate is not None
  assert not estimate.close_front_supported
  assert not estimate.confirmed_cutin
  assert not estimate.control_eligible


def test_close_side_pass_uses_history_when_last_velocity_slows() -> None:
  detector = TrajectoryCutInDetector()
  estimate = None
  samples = (
    (4.55, 2.73, -2.20),
    (3.89, 2.73, -2.19),
    (2.79, 2.60, -2.57),
    (2.21, 2.24, -2.44),
    (1.78, 2.18, -3.52),
    (1.75, 2.02, -1.31),
  )
  for index, (d_rel, y_rel, v_rel) in enumerate(samples):
    estimate = detector.update(
      index * 0.29,
      8.0,
      (point(
        35,
        "frontRadar",
        d_rel,
        y_rel,
        v_ego=8.0,
        v_rel=v_rel,
      ),),
      PATH,
      MODEL,
    )[0]

  assert estimate is not None
  assert not estimate.confirmed_cutin


def test_alternating_corner_jitter_is_rejected() -> None:
  estimate = corner_series(
    TrajectoryCutInDetector(),
    (3.0, 2.55, 3.10, 2.50, 3.05, 2.60, 3.10),
  )

  assert estimate.jittering
  assert not estimate.confirmed_cutin
