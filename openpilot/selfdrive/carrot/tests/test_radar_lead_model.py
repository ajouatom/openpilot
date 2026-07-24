from dataclasses import replace

import numpy as np

from openpilot.selfdrive.carrot.radar.radar_lead_model import (
  MODEL_FEATURE_NAMES,
  RadarLeadContext,
  RadarLeadDecisionFilter,
  RadarLeadFeatureBuilder,
  RadarLeadPrediction,
  RadarLeadModel,
  anticipatory_eligibility,
)
from openpilot.selfdrive.carrot.radar.radar_object_fusion import FusedRadarObject


def fused(
  front_id: int | None = 40, corner_id: int | None = 1000,
  y_rel: float = 2.5, yv_rel: float = -0.8, d_rel: float = 20.0,
) -> FusedRadarObject:
  return FusedRadarObject(
    object_id="test", d_rel=d_rel, y_rel=y_rel, v_rel=-2.0, a_rel=0.0, yv_rel=yv_rel, v_lead=18.0,
    front_track_id=front_id, corner_track_id=corner_id, scc_track_id=None,
    front_d_rel=21.0 if front_id is not None else None,
    corner_d_rel=19.5 if corner_id is not None else None,
    front_y_rel=1.2 if front_id is not None else None,
    corner_y_rel=y_rel if corner_id is not None else None,
    front_v_rel=-2.0 if front_id is not None else None,
    corner_v_rel=-2.1 if corner_id is not None else None,
    distance_source="corner-weighted", lateral_source="corner180" if corner_id is not None else "frontRadar",
    match_confidence=0.9, pair_age=6,
  )


def context(time_s: float) -> RadarLeadContext:
  path = ((0.0, 0.0), (100.0, 0.0))
  lanes = (
    ((0.0, -5.4), (100.0, -5.4)),
    ((0.0, -1.8), (100.0, -1.8)),
    ((0.0, 1.8), (100.0, 1.8)),
    ((0.0, 5.4), (100.0, 5.4)),
  )
  return RadarLeadContext(time_s, 20.0, path, lanes, (0.9, 0.9, 0.9, 0.9), ())


def uncertain_lane_context(time_s: float) -> RadarLeadContext:
  sample = context(time_s)
  return replace(sample, lane_probs=(0.2, 0.45, 0.35, 0.2))


def test_anticipatory_eligibility_requires_sustained_lane_inward_motion() -> None:
  matrix = np.zeros((1, len(MODEL_FEATURE_NAMES)), dtype=np.float32)
  values = {
    "d_rel": 20.0, "v_lead": 15.0, "track_age": 12.0,
    "d_path": 2.4, "future_d_path": 1.8,
    "h8_present": 1.0, "h8_dt": 0.4, "h8_d_path": 2.7,
    "h12_present": 1.0, "h12_dt": 0.6, "h12_d_path": 2.9,
    "lane1_prob": 0.9, "lane2_prob": 0.9,
  }
  for name, value in values.items():
    matrix[0, MODEL_FEATURE_NAMES.index(name)] = value

  assert anticipatory_eligibility(matrix, np).tolist() == [True]

  matrix[0, MODEL_FEATURE_NAMES.index("h8_d_path")] = 2.4
  assert anticipatory_eligibility(matrix, np).tolist() == [False]


def test_anticipatory_probability_remap_preserves_decision_threshold() -> None:
  result = RadarLeadModel._remap_probability(np.asarray([0.9], dtype=np.float32), 0.9, 0.82)

  np.testing.assert_allclose(result, (0.82,), rtol=1e-6)


def test_feature_builder_keeps_identity_and_one_second_history() -> None:
  builder = RadarLeadFeatureBuilder()
  samples = ()
  for frame in range(22):
    samples = builder.update(context(frame * 0.05), (fused(),))

  assert len(samples) == 1
  sample = samples[0]
  assert sample.object_id == "front:40"
  assert sample.track_age == 22
  assert len(sample.values) == len(MODEL_FEATURE_NAMES)
  values = dict(zip(MODEL_FEATURE_NAMES, sample.values, strict=True))
  assert values["h20_present"] == 1.0
  assert 0.99 < values["h20_dt"] < 1.01


def test_fused_identity_inherits_front_only_history() -> None:
  builder = RadarLeadFeatureBuilder()
  front_only = fused(corner_id=None, y_rel=1.0, yv_rel=0.0)
  for frame in range(5):
    builder.update(context(frame * 0.05), (front_only,))
  sample = builder.update(context(0.25), (fused(),))[0]

  assert sample.object_id == "front:40"
  assert sample.track_age == 6


def test_reused_sensor_id_with_impossible_distance_jump_starts_new_history() -> None:
  builder = RadarLeadFeatureBuilder()
  previous = None
  for frame in range(10):
    previous = builder.update(context(frame * 0.05), (
      fused(corner_id=None, y_rel=-3.0, yv_rel=0.0, d_rel=65.0 - frame * 0.1),
    ))[0]

  current = builder.update(context(0.50), (
    fused(corner_id=None, y_rel=-2.4, yv_rel=0.0, d_rel=13.0),
  ))[0]

  assert previous is not None
  assert current.object_id != previous.object_id
  assert current.object_id.startswith("front:40@")
  assert current.track_age == 1


def cutin_prediction(
  builder: RadarLeadFeatureBuilder,
  frame: int,
  probability: float,
  *,
  d_path: float = 2.55,
  future_d_path: float = 2.20,
  d_rel: float = 20.0,
  yv_rel: float = -0.4,
  front_id: int | None = 40,
  corner_id: int | None = 1000,
  object_id: str | None = None,
  track_age: int = 10,
) -> RadarLeadPrediction:
  obj = fused(
    front_id=front_id, corner_id=corner_id, y_rel=d_path, yv_rel=yv_rel, d_rel=d_rel,
  )
  sample = builder.update(context(frame * 0.05), (obj,))[0]
  values = list(sample.values)
  for name, value in {
    "lane_half_width": 1.8,
    "lane1_prob": 0.9,
    "lane2_prob": 0.9,
    "h4_present": 1.0,
    "h4_dt": 0.2,
    "h4_d_path": d_path + 0.20,
    "h8_present": 1.0,
    "h8_dt": 0.4,
    "h8_d_path": d_path + 0.40,
  }.items():
    values[MODEL_FEATURE_NAMES.index(name)] = value
  sample = replace(
    sample,
    object_id=sample.object_id if object_id is None else object_id,
    values=tuple(values),
    track_age=track_age,
    d_path=d_path,
    d_path_future=future_d_path,
  )
  return RadarLeadPrediction(
    sample,
    lead_prob=0.0,
    cutin_prob=probability,
    risk_prob=probability,
    base_cutin_prob=probability,
    temporal_cutin_prob=probability,
  )


def test_cutin_filter_requires_two_model_positive_frames() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)

  first = decision_filter.update(0.0, (cutin_prediction(builder, 0, 0.86),))
  second = decision_filter.update(0.05, (cutin_prediction(builder, 1, 0.86),))

  assert not first.cutin_candidates
  assert len(second.cutin_candidates) == 1


def test_very_high_cutin_probability_activates_immediately() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)

  decision = decision_filter.update(0.0, (cutin_prediction(builder, 0, 0.96),))

  assert len(decision.cutin_candidates) == 1


def test_strong_predicted_lane_entry_activates_at_anticipatory_probability() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)

  decision = decision_filter.update(0.0, (cutin_prediction(
    builder, 0, 0.76, d_path=3.0, future_d_path=2.0,
  ),))

  assert len(decision.cutin_candidates) == 1


def test_corner_predicted_entry_does_not_override_temporal_model_with_base_evidence() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)

  predictions = []
  for frame in range(2):
    prediction = cutin_prediction(
      builder, frame, 0.10, d_path=3.0, future_d_path=2.0,
      front_id=None, corner_id=1013,
    )
    predictions.append(replace(
      prediction,
      base_cutin_prob=0.99,
      temporal_cutin_prob=0.15,
    ))

  first = decision_filter.update(0.0, (predictions[0],))
  second = decision_filter.update(0.05, (predictions[1],))

  assert not first.cutin_candidates
  assert not second.cutin_candidates


def test_corner_predicted_entry_rejects_base_evidence_without_temporal_support() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None

  for frame in range(3):
    prediction = cutin_prediction(
      builder, frame, 0.10, d_path=3.0, future_d_path=2.0,
      front_id=None, corner_id=1013,
    )
    prediction = replace(
      prediction,
      base_cutin_prob=0.99,
      temporal_cutin_prob=0.10,
    )
    decision = decision_filter.update(frame * 0.05, (prediction,))

  assert decision is not None
  assert not decision.cutin_candidates


def test_close_corner_entry_uses_strong_base_model_evidence() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)

  decisions = []
  for frame in range(2):
    prediction = cutin_prediction(
      builder, frame, 0.10, d_path=2.90, future_d_path=2.35,
      d_rel=4.8, front_id=None, corner_id=1013,
    )
    prediction = replace(
      prediction,
      base_cutin_prob=0.99,
      temporal_cutin_prob=0.10,
    )
    decisions.append(decision_filter.update(frame * 0.05, (prediction,)))

  assert not decisions[0].cutin_candidates
  assert decisions[1].cutin_candidates


def test_front_predicted_entry_does_not_use_base_evidence_override() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None

  for frame in range(3):
    prediction = cutin_prediction(
      builder, frame, 0.10, d_path=3.0, future_d_path=2.0,
      front_id=43, corner_id=None,
    )
    prediction = replace(
      prediction,
      base_cutin_prob=0.99,
      temporal_cutin_prob=0.10,
    )
    decision = decision_filter.update(frame * 0.05, (prediction,))

  assert decision is not None
  assert not decision.cutin_candidates


def test_near_corner_history_accepts_two_moderate_model_frames() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  for frame in range(2):
    prediction = cutin_prediction(
      builder, frame, 0.55, d_path=-3.20, future_d_path=-3.30,
      d_rel=8.0, yv_rel=-0.15, front_id=None, corner_id=1024,
    )
    values = list(prediction.features.values)
    values[MODEL_FEATURE_NAMES.index("h4_d_path")] = -3.35
    values[MODEL_FEATURE_NAMES.index("h8_d_path")] = -3.50
    prediction = replace(prediction, features=replace(prediction.features, values=tuple(values)))
    decision = decision_filter.update(frame * 0.05, (prediction,))

  assert decision is not None
  assert len(decision.cutin_candidates) == 1


def test_near_corner_weak_history_does_not_lower_cutin_threshold() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  for frame in range(3):
    prediction = cutin_prediction(
      builder, frame, 0.61, d_path=2.28, future_d_path=2.30,
      d_rel=6.2, yv_rel=0.0, front_id=None, corner_id=1071,
    )
    values = list(prediction.features.values)
    values[MODEL_FEATURE_NAMES.index("h4_d_path")] = 2.31
    values[MODEL_FEATURE_NAMES.index("h8_d_path")] = 2.33
    prediction = replace(prediction, features=replace(prediction.features, values=tuple(values)))
    decision = decision_filter.update(frame * 0.05, (prediction,))

  assert decision is not None
  assert not decision.cutin_candidates


def test_sub_five_meter_side_pass_needs_actual_body_entry() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  for frame in range(2):
    prediction = cutin_prediction(
      builder, frame, 0.70, d_path=-2.88, future_d_path=-2.70,
      d_rel=2.7, yv_rel=0.19, front_id=None, corner_id=1023,
    )
    values = list(prediction.features.values)
    values[MODEL_FEATURE_NAMES.index("h4_d_path")] = -3.20
    values[MODEL_FEATURE_NAMES.index("h8_d_path")] = -3.40
    prediction = replace(prediction, features=replace(prediction.features, values=tuple(values)))
    decision = decision_filter.update(frame * 0.05, (prediction,))

  assert decision is not None
  assert not decision.cutin_candidates


def test_low_model_probability_is_never_promoted_by_motion_rules() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  for frame in range(20):
    decision = decision_filter.update(
      frame * 0.05, (cutin_prediction(builder, frame, 0.20),),
    )

  assert decision is not None
  assert not decision.cutin_candidates


def test_vehicle_body_entry_allows_detection_before_center_crosses_lane_line() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)

  decision_filter.update(0.0, (cutin_prediction(
    builder, 0, 0.86, d_path=2.60, future_d_path=2.30,
  ),))
  decision = decision_filter.update(0.05, (cutin_prediction(
    builder, 1, 0.86, d_path=2.58, future_d_path=2.25,
  ),))

  assert decision.cutin_candidates
  assert abs(decision.cutin_candidates[0].features.d_path) > 1.8


def test_parallel_adjacent_vehicle_needs_model_and_inward_geometry() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  for frame in range(10):
    prediction = cutin_prediction(
      builder, frame, 0.99, d_path=2.60, future_d_path=2.60, yv_rel=0.0,
    )
    values = list(prediction.features.values)
    values[MODEL_FEATURE_NAMES.index("h4_d_path")] = 2.60
    values[MODEL_FEATURE_NAMES.index("h8_d_path")] = 2.60
    prediction = replace(prediction, features=replace(prediction.features, values=tuple(values)))
    decision = decision_filter.update(frame * 0.05, (prediction,))

  assert decision is not None
  assert not decision.cutin_candidates


def test_outward_vehicle_is_rejected_even_with_high_probability() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  for frame in range(3):
    prediction = cutin_prediction(
      builder, frame, 0.99, d_path=1.50, future_d_path=2.20, yv_rel=1.0,
    )
    values = list(prediction.features.values)
    values[MODEL_FEATURE_NAMES.index("h4_d_path")] = 1.30
    values[MODEL_FEATURE_NAMES.index("h8_d_path")] = 1.10
    prediction = replace(prediction, features=replace(prediction.features, values=tuple(values)))
    decision = decision_filter.update(frame * 0.05, (prediction,))

  assert decision is not None
  assert not decision.cutin_candidates


def test_corner_only_long_range_candidate_is_rejected() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  for frame in range(3):
    decision = decision_filter.update(frame * 0.05, (cutin_prediction(
      builder, frame, 0.99, d_rel=48.0, front_id=None, corner_id=1044,
    ),))

  assert decision is not None
  assert not decision.cutin_candidates


def test_front_only_near_field_requires_established_history() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)

  decision = decision_filter.update(0.0, (cutin_prediction(
    builder, 0, 0.99, d_rel=3.5, front_id=40, corner_id=None, track_age=6,
  ),))

  assert not decision.cutin_candidates


def test_front_only_point_below_five_meters_is_rejected_even_with_history() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)

  decision = decision_filter.update(0.0, (cutin_prediction(
    builder, 0, 0.99, d_rel=4.8, front_id=40, corner_id=None, track_age=23,
  ),))

  assert not decision.cutin_candidates


def test_sticky_cutin_holds_only_the_same_sensor_identity() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  active = cutin_prediction(builder, 0, 0.96)
  assert decision_filter.update(0.0, (active,)).cutin_candidates

  low = cutin_prediction(builder, 1, 0.01)
  assert decision_filter.update(0.10, (low,)).cutin_candidates

  reassociated = replace(
    low,
    features=replace(
      low.features,
      aliases=("front:41", "corner:1001"),
      radar_object=fused(front_id=41, corner_id=1001, y_rel=2.55, yv_rel=-0.4, d_rel=20.0),
    ),
  )
  assert not decision_filter.update(0.15, (reassociated,)).cutin_candidates


def test_sticky_cutin_expires_without_new_model_evidence() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  assert decision_filter.update(
    0.0, (cutin_prediction(builder, 0, 0.96),),
  ).cutin_candidates

  decision = None
  for frame in range(1, 20):
    decision = decision_filter.update(
      frame * 0.05, (cutin_prediction(builder, frame, 0.01),),
    )

  assert decision is not None
  assert not decision.cutin_candidates


def test_lead_and_external_heads_keep_independent_hysteresis() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter()
  decision = None
  for frame in range(2):
    prediction = cutin_prediction(
      builder, frame, 0.0, d_path=0.2, future_d_path=0.2, yv_rel=0.0,
    )
    prediction = replace(prediction, lead_prob=0.8, external_prob=0.8)
    decision = decision_filter.update(frame * 0.05, (prediction,))

  assert decision is not None
  assert decision.lead_candidates
  assert decision.external_candidates
  assert not decision.cutin_candidates
