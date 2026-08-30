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


def test_far_corner_trajectory_is_detected_without_l2_control() -> None:
  estimate = corner_series(
    TrajectoryCutInDetector(),
    (3.0, 2.8, 2.55, 2.30, 2.10),
    d_rel=32.0,
    v_rel=-2.0,
  )

  assert estimate.confirmed_cutin
  assert not estimate.control_eligible


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
