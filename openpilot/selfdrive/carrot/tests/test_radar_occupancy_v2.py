from dataclasses import replace

from openpilot.selfdrive.carrot.radar_motion.occupancy_v2 import (
  OccupancyEvidence,
  OccupancyStage,
  RadarOccupancyModelV2,
  early_control_eligible,
  occupancy_scores,
)


def evidence(**overrides: object) -> OccupancyEvidence:
  values = {
    "source": "corner235",
    "track_id": 3504,
    "continuity_id": 2,
    "d_rel": 5.5,
    "v_rel": -1.7,
    "v_lead": 9.9,
    "v_ego": 11.6,
    "d_path": -2.97,
    "d_path_rate_short": 0.59,
    "d_path_rate_long": 0.33,
    "reported_normal_speed": 0.55,
    "normal_speed_disagreement": 0.02,
    "directional_inward_displacement_m": 0.35,
    "directional_consistency": 0.91,
    "directional_inward_sample_ratio": 0.82,
    "motion_consistency": 0.93,
    "recent_motion_support": 0.94,
    "history_count": 24,
    "uncertainty": 0.42,
    "current_path_occupancy": False,
    "cross_sensor_confirmed": True,
    "vision_supported": True,
  }
  values.update(overrides)
  return OccupancyEvidence(**values)  # type: ignore[arg-type]


def test_parallel_adjacent_track_does_not_request_acceleration_limit() -> None:
  parallel = evidence(
    d_path=-3.2,
    d_path_rate_short=0.03,
    d_path_rate_long=0.01,
    reported_normal_speed=0.02,
    directional_inward_displacement_m=0.03,
    directional_consistency=0.55,
    directional_inward_sample_ratio=0.52,
  )

  risk, lead, *_ = occupancy_scores(parallel)

  assert risk < 0.10
  assert lead < 0.10


def test_low_speed_slow_entry_is_seen_before_it_becomes_control_urgent() -> None:
  slow_entry = evidence(
    v_ego=4.0,
    v_lead=3.6,
    v_rel=-0.4,
    d_rel=5.0,
    d_path=-2.55,
    d_path_rate_short=0.16,
    d_path_rate_long=0.14,
    reported_normal_speed=0.15,
    directional_inward_displacement_m=0.32,
    directional_consistency=0.96,
    directional_inward_sample_ratio=0.93,
  )

  estimate = RadarOccupancyModelV2().update(0.0, (slow_entry,))[0]

  # At low road speed, sustained position history exposes even a gentle lane
  # entry early. It is WATCH first; longitudinal urgency separately decides
  # when LIMIT or LEAD is justified.
  assert estimate.stage == OccupancyStage.WATCH
  assert estimate.occupancy_score > 0.30
  assert estimate.risk_score < 0.31
  assert estimate.lead_score < 0.48


def test_fused_committed_entry_advances_from_limit_to_lead() -> None:
  model = RadarOccupancyModelV2()
  early = evidence()
  committed = replace(
    early,
    d_path=-2.67,
    d_path_rate_short=0.83,
    d_path_rate_long=0.45,
    reported_normal_speed=0.90,
    directional_inward_displacement_m=0.59,
    directional_consistency=0.98,
    directional_inward_sample_ratio=0.95,
  )

  assert model.update(1.16, (early,))[0].stage < OccupancyStage.LIMIT
  assert model.update(1.27, (early,))[0].stage == OccupancyStage.LIMIT
  assert model.update(1.35, (committed,))[0].stage == OccupancyStage.LIMIT
  assert model.update(1.46, (committed,))[0].stage >= OccupancyStage.LEAD


def test_inconsistent_ghost_motion_stays_below_limit() -> None:
  ghost = evidence(
    cross_sensor_confirmed=False,
    vision_supported=False,
    d_path=-2.5,
    d_path_rate_short=0.9,
    d_path_rate_long=0.55,
    reported_normal_speed=-0.4,
    normal_speed_disagreement=0.9,
    directional_inward_displacement_m=0.12,
    directional_consistency=0.42,
    directional_inward_sample_ratio=0.45,
    motion_consistency=0.35,
    recent_motion_support=0.30,
    uncertainty=1.8,
  )

  risk, lead, *_ = occupancy_scores(ghost)

  assert risk < 0.20
  assert lead < 0.20


def test_quantized_single_sensor_motion_does_not_reach_lead() -> None:
  quantized = evidence(
    source="frontRadar",
    cross_sensor_confirmed=False,
    vision_supported=False,
    d_rel=30.0,
    d_path=-2.39,
    d_path_rate_short=1.67,
    d_path_rate_long=0.83,
    reported_normal_speed=0.61,
    normal_speed_disagreement=0.22,
    directional_inward_displacement_m=0.84,
    directional_consistency=0.84,
    directional_inward_sample_ratio=0.73,
    uncertainty=0.40,
  )

  _, lead, *_ = occupancy_scores(quantized)

  assert lead < 0.48


def test_lead_hysteresis_survives_one_weak_frame() -> None:
  model = RadarOccupancyModelV2()
  committed = evidence(
    d_path=-2.5,
    d_path_rate_short=0.9,
    d_path_rate_long=0.60,
    directional_inward_displacement_m=0.65,
    directional_consistency=0.98,
    directional_inward_sample_ratio=0.95,
  )
  weak = replace(
    committed,
    d_path_rate_short=0.05,
    d_path_rate_long=0.05,
    reported_normal_speed=0.05,
  )

  model.update(0.0, (committed,))
  assert model.update(0.11, (committed,))[0].stage >= OccupancyStage.LEAD
  assert model.update(0.16, (weak,))[0].stage >= OccupancyStage.LEAD


def test_early_control_requires_cross_sensor_close_or_urgent_entry() -> None:
  model = RadarOccupancyModelV2()
  close = evidence(d_rel=5.5, v_rel=-1.7)
  unsupported = replace(close, cross_sensor_confirmed=False)
  distant = replace(close, d_rel=20.0, v_rel=-0.4)
  outside_adjacent_lane = replace(close, d_rel=8.0, d_path=-4.2)

  close_estimate = model.update(0.0, (close,))[0]
  unsupported_estimate = model.update(0.1, (unsupported,))[0]
  distant_estimate = model.update(0.2, (distant,))[0]
  outside_estimate = model.update(0.3, (outside_adjacent_lane,))[0]

  assert early_control_eligible(close_estimate)
  assert not early_control_eligible(unsupported_estimate)
  assert not early_control_eligible(distant_estimate)
  assert not early_control_eligible(outside_estimate)
