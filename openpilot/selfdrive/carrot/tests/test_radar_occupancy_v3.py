from dataclasses import replace

from openpilot.selfdrive.carrot.radar_motion.occupancy_v2 import (
  OccupancyEstimate,
  OccupancyEvidence,
  OccupancyStage,
)
from openpilot.selfdrive.carrot.radar_motion.occupancy_v3 import (
  RadarOccupancyModelV3,
  V3Stage,
)


def occupancy(**overrides) -> OccupancyEstimate:
  evidence_values = {
    "source": "frontRadar",
    "track_id": 51,
    "continuity_id": 7,
    "d_rel": 14.8,
    "v_rel": -2.9,
    "v_lead": 17.1,
    "v_ego": 20.0,
    "d_path": -1.88,
    "d_path_rate_short": 1.75,
    "d_path_rate_long": 0.24,
    "reported_normal_speed": 0.04,
    "normal_speed_disagreement": 1.0,
    "directional_inward_displacement_m": 0.69,
    "directional_consistency": 0.79,
    "directional_inward_sample_ratio": 0.60,
    "motion_consistency": 1.0,
    "recent_motion_support": 1.0,
    "history_count": 40,
    "uncertainty": 0.4,
    "current_path_occupancy": False,
    "cross_sensor_confirmed": False,
    "vision_supported": False,
  }
  estimate_values = {
    "stage": OccupancyStage.CLEAR,
    "occupancy_score": 0.20,
    "risk_score": 0.18,
    "lead_score": 0.19,
    "intent_score": 0.59,
    "confidence": 0.42,
    "path_clearance_m": 0.0,
    "inward_rate_mps": 0.24,
    "time_to_overlap_s": 0.0,
    "time_gap_s": 0.74,
    "time_to_collision_s": 5.1,
    "control_urgency": 0.57,
  }
  for key, value in overrides.items():
    if key in evidence_values:
      evidence_values[key] = value
    else:
      estimate_values[key] = value
  return OccupancyEstimate(
    evidence=OccupancyEvidence(**evidence_values),
    **estimate_values,
  )


def test_v2_limit_is_preserved_as_v3_predecel_floor() -> None:
  model = RadarOccupancyModelV3()
  result = model.update(0.0, (
    occupancy(
      stage=OccupancyStage.LIMIT,
      directional_inward_displacement_m=0.0,
      directional_consistency=0.0,
      directional_inward_sample_ratio=0.0,
      d_path_rate_short=0.0,
      d_path_rate_long=0.0,
    ),
  ))[0]

  assert result.stage == V3Stage.PREDECEL
  assert result.reason == "baseline"


def test_front_position_history_recovers_missing_lateral_velocity() -> None:
  model = RadarOccupancyModelV3()
  values = [
    occupancy(
      directional_inward_displacement_m=0.40,
      directional_consistency=0.66,
      directional_inward_sample_ratio=0.52,
      d_path_rate_short=0.45,
      d_path_rate_long=0.12,
    ),
    occupancy(
      directional_inward_displacement_m=0.55,
      directional_consistency=0.73,
      directional_inward_sample_ratio=0.58,
      d_path_rate_short=0.75,
      d_path_rate_long=0.18,
    ),
    occupancy(),
  ]
  results = [
    model.update(index * 0.05, (value,))[0]
    for index, value in enumerate(values)
  ]

  assert results[0].stage == V3Stage.CAUTION
  assert results[1].stage == V3Stage.PREDECEL
  assert results[2].stage == V3Stage.LEAD
  assert results[2].reason == "front-position-history"


def test_fast_longitudinal_pass_is_not_a_cutin_extension() -> None:
  model = RadarOccupancyModelV3()
  result = model.update(0.0, (
    occupancy(time_to_collision_s=0.5, time_to_overlap_s=1.5),
  ))[0]

  assert result.extension_stage == V3Stage.CLEAR
  assert result.stage == V3Stage.CLEAR


def test_fused_corner_motion_enters_predecel_early() -> None:
  model = RadarOccupancyModelV3()
  base = occupancy(
    source="corner235",
    track_id=2091,
    continuity_id=2091,
    d_rel=21.1,
    d_path=3.17,
    v_rel=-6.2,
    d_path_rate_short=-0.52,
    d_path_rate_long=-0.41,
    reported_normal_speed=-0.73,
    directional_inward_displacement_m=0.34,
    directional_consistency=0.93,
    directional_inward_sample_ratio=0.81,
    cross_sensor_confirmed=True,
    confidence=0.86,
    control_urgency=0.84,
    risk_score=0.32,
    lead_score=0.05,
    time_to_overlap_s=1.87,
    time_to_collision_s=3.43,
  )
  result = model.update(0.0, (base,))[0]

  assert result.extension_stage == V3Stage.PREDECEL
  assert result.stage == V3Stage.PREDECEL
  assert result.reason == "fused-corner-motion"


def test_unfused_corner_does_not_get_v3_extension() -> None:
  model = RadarOccupancyModelV3()
  fused = occupancy(
    source="corner235",
    d_path=3.0,
    d_path_rate_short=-0.6,
    d_path_rate_long=-0.5,
    reported_normal_speed=-0.7,
    directional_consistency=0.95,
    directional_inward_sample_ratio=0.85,
    cross_sensor_confirmed=True,
    confidence=0.9,
    control_urgency=0.9,
    time_to_overlap_s=1.5,
    time_to_collision_s=3.0,
  )
  result = model.update(0.0, (
    replace(fused, evidence=replace(
      fused.evidence, cross_sensor_confirmed=False,
    )),
  ))[0]

  assert result.extension_stage == V3Stage.CLEAR
