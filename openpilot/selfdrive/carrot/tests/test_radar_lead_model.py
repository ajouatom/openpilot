from dataclasses import replace

from openpilot.selfdrive.carrot.radar_lead_model import (
  MODEL_FEATURE_NAMES,
  RadarLeadContext,
  RadarLeadDecisionFilter,
  RadarLeadFeatureBuilder,
  RadarLeadPrediction,
)
from openpilot.selfdrive.carrot.radar_object_fusion import FusedRadarObject


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


def test_cutin_filter_requires_persistence_and_holds_identity() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter()
  decision = None
  for frame in range(7):
    sample = builder.update(context(frame * 0.05), (fused(y_rel=2.5 - frame * 0.04),))[0]
    values = list(sample.values)
    values[MODEL_FEATURE_NAMES.index("h8_y_rate")] = -0.8
    sample = replace(sample, values=tuple(values))
    prediction = RadarLeadPrediction(sample, lead_prob=0.8, cutin_prob=0.8, risk_prob=0.8)
    decision = decision_filter.update(frame * 0.05, (prediction,))

  assert decision is not None
  assert len(decision.lead_candidates) == 1
  assert len(decision.cutin_candidates) == 1


def test_external_filter_is_independent_from_lead_and_cutin() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter()
  decision = None
  for frame in range(6):
    sample = builder.update(context(frame * 0.05), (fused(),))[0]
    prediction = RadarLeadPrediction(sample, 0.1, 0.1, 0.1, external_prob=0.8)
    decision = decision_filter.update(frame * 0.05, (prediction,))

  assert decision is not None
  assert not decision.lead_candidates
  assert not decision.cutin_candidates
  assert len(decision.external_candidates) == 1


def test_moving_away_cutin_is_released_quickly() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter()
  for frame in range(7):
    sample = builder.update(context(frame * 0.05), (fused(),))[0]
    values = list(sample.values)
    values[MODEL_FEATURE_NAMES.index("h8_y_rate")] = -0.8
    sample = replace(sample, values=tuple(values))
    decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.8, 0.8, 0.8),))

  moving_away = fused(y_rel=2.5, yv_rel=1.5)
  sample = builder.update(context(0.35), (moving_away,))[0]
  decision = decision_filter.update(0.35, (RadarLeadPrediction(sample, 0.8, 0.3, 0.3),))
  assert decision.cutin_candidates
  decision = decision_filter.update(0.50, (RadarLeadPrediction(sample, 0.8, 0.1, 0.1),))
  assert not decision.cutin_candidates


def test_outside_object_moving_away_never_activates_cutin() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  for frame in range(8):
    sample = builder.update(context(frame * 0.05), (fused(y_rel=2.45, yv_rel=0.1, d_rel=11.2),))[0]
    sample = replace(sample, d_path=2.45, d_path_future=2.54)
    decision = decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.0, 0.95, 0.95),))

  assert decision is not None
  assert not decision.cutin_candidates


def test_adjacent_object_that_will_not_reach_lane_never_activates_cutin() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  obj = fused(y_rel=-3.6, yv_rel=0.6, d_rel=5.0)
  for frame in range(8):
    sample = builder.update(context(frame * 0.05), (obj,))[0]
    sample = replace(sample, d_path=-3.6, d_path_future=-3.0)
    decision = decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.0, 0.99, 0.99),))

  assert decision is not None
  assert not decision.cutin_candidates


def test_close_parallel_object_with_tiny_lateral_motion_never_activates_cutin() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  obj = fused(y_rel=-3.1, yv_rel=0.05, d_rel=7.4)
  for frame in range(8):
    sample = builder.update(context(frame * 0.05), (obj,))[0]
    sample = replace(sample, d_path=-3.1, d_path_future=-3.0)
    decision = decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.0, 0.99, 0.99),))

  assert decision is not None
  assert not decision.cutin_candidates


def test_inside_object_moving_outward_never_activates_cutin() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  obj = fused(y_rel=0.3, yv_rel=0.6, d_rel=35.0)
  for frame in range(8):
    sample = builder.update(context(frame * 0.05), (obj,))[0]
    sample = replace(sample, d_path=0.3, d_path_future=0.95)
    decision = decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.0, 0.99, 0.99),))

  assert decision is not None
  assert not decision.cutin_candidates


def test_sticky_cutin_does_not_expose_low_probability_reassociation() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter()
  sample = None
  for frame in range(7):
    sample = builder.update(context(frame * 0.05), (fused(),))[0]
    decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.8, 0.8, 0.8),))

  assert sample is not None
  reassociated = replace(
    sample, aliases=("front:41", "corner:1001"), radar_object=fused(front_id=41, corner_id=1001),
  )
  decision = decision_filter.update(0.36, (RadarLeadPrediction(reassociated, 0.8, 0.01, 0.01),))
  assert not decision.cutin_candidates


def test_sticky_cutin_survives_same_sensor_fusion_dropout() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter()
  sample = None
  for frame in range(7):
    sample = builder.update(context(frame * 0.05), (fused(d_rel=6.0),))[0]
    values = list(sample.values)
    values[MODEL_FEATURE_NAMES.index("h8_y_rate")] = -0.8
    sample = replace(sample, values=tuple(values), d_path=2.0, d_path_future=1.5)
    decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.8, 0.8, 0.8),))

  assert sample is not None
  front_only = replace(sample, radar_object=fused(corner_id=None, y_rel=1.7, yv_rel=-0.1, d_rel=6.0))
  for frame in range(1, 12):
    front_only = replace(front_only, d_path=1.7, d_path_future=1.6)
    decision = decision_filter.update(0.35 + frame * 0.05, (RadarLeadPrediction(front_only, 0.8, 0.01, 0.01),))
    assert decision.cutin_candidates


def test_decision_filter_uses_one_sample_per_object_identity() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter()
  sample = None
  for frame in range(6):
    sample = builder.update(context(frame * 0.05), (fused(),))[0]
    decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.8, 0.1, 0.8),))

  assert sample is not None
  strong = RadarLeadPrediction(sample, 0.9, 0.1, 0.9)
  weak = RadarLeadPrediction(sample, 0.2, 0.1, 0.2)
  decision = decision_filter.update(0.35, (strong, weak))
  assert decision.lead_candidates == (strong,)


def test_close_inward_object_uses_lower_two_frame_threshold() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.87)
  decision = None
  for frame in range(6):
    sample = builder.update(context(frame * 0.05), (fused(y_rel=2.5 - frame * 0.04, d_rel=6.0),))[0]
    values = list(sample.values)
    values[MODEL_FEATURE_NAMES.index("h8_y_rate")] = -0.8
    sample = replace(sample, values=tuple(values))
    decision = decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.8, 0.68, 0.68),))

  assert decision is not None
  assert decision.cutin_candidates


def test_near_fused_inward_object_overrides_low_cutin_probability() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  obj = fused(y_rel=1.98, yv_rel=-0.2, d_rel=4.7)
  for frame in range(10):
    sample = builder.update(context(frame * 0.05), (obj,))[0]
    values = list(sample.values)
    values[MODEL_FEATURE_NAMES.index("h8_y_rate")] = -0.25
    sample = replace(sample, values=tuple(values), d_path=1.98, d_path_future=1.80)
    decision = decision_filter.update(frame * 0.05, (
      RadarLeadPrediction(sample, lead_prob=0.0, cutin_prob=0.01, risk_prob=0.01),
    ))

  assert decision is not None
  assert decision.cutin_candidates
  assert decision.cutin_candidates[0].cutin_prob >= 0.70


def test_midrange_fused_object_requires_sustained_inward_history() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  obj = fused(y_rel=2.7, yv_rel=-0.5, d_rel=13.0)
  for frame in range(10):
    sample = builder.update(context(frame * 0.05), (obj,))[0]
    values = list(sample.values)
    values[MODEL_FEATURE_NAMES.index("h8_y_rate")] = -0.6
    values[MODEL_FEATURE_NAMES.index("h12_y_rate")] = -0.6
    sample = replace(sample, values=tuple(values), d_path=2.7, d_path_future=2.15)
    decision = decision_filter.update(frame * 0.05, (
      RadarLeadPrediction(sample, lead_prob=0.0, cutin_prob=0.01, risk_prob=0.01),
    ))

  assert decision is not None
  assert decision.cutin_candidates


def test_front_only_close_probability_spike_requires_two_frames() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  obj = fused(corner_id=None, y_rel=1.7, yv_rel=-0.6, d_rel=3.5)
  sample = None
  for frame in range(5):
    sample = builder.update(context(frame * 0.05), (obj,))[0]
    values = list(sample.values)
    values[MODEL_FEATURE_NAMES.index("h8_y_rate")] = -0.6
    sample = replace(sample, values=tuple(values), d_path=1.6, d_path_future=1.0)
    decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.0, 0.05, 0.05),))

  assert sample is not None
  first = decision_filter.update(0.25, (RadarLeadPrediction(sample, 0.0, 0.99, 0.99),))
  low = decision_filter.update(0.30, (RadarLeadPrediction(sample, 0.0, 0.19, 0.19),))
  restarted = decision_filter.update(0.35, (RadarLeadPrediction(sample, 0.0, 0.99, 0.99),))
  second = decision_filter.update(0.40, (RadarLeadPrediction(sample, 0.0, 0.99, 0.99),))

  assert not first.cutin_candidates
  assert not low.cutin_candidates
  assert not restarted.cutin_candidates
  assert second.cutin_candidates


def test_close_cutin_evidence_does_not_accumulate_across_sensor_ids() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  first = builder.update(context(0.0), (fused(corner_id=None, y_rel=1.7, yv_rel=-0.6, d_rel=3.5),))[0]
  values = list(first.values)
  values[MODEL_FEATURE_NAMES.index("h8_y_rate")] = -0.6
  first = replace(first, values=tuple(values), track_age=5, d_path=1.6, d_path_future=1.0)
  second = replace(
    first,
    aliases=("front:42",),
    radar_object=fused(front_id=42, corner_id=None, y_rel=1.7, yv_rel=-0.6, d_rel=3.5),
  )

  first_decision = decision_filter.update(1.0, (RadarLeadPrediction(first, 0.0, 0.99, 0.99),))
  reassociated = decision_filter.update(1.05, (RadarLeadPrediction(second, 0.0, 0.99, 0.99),))
  confirmed = decision_filter.update(1.10, (RadarLeadPrediction(second, 0.0, 0.99, 0.99),))

  assert not first_decision.cutin_candidates
  assert not reassociated.cutin_candidates
  assert confirmed.cutin_candidates


def test_fused_inward_object_is_detected_beyond_twenty_meters() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  obj = fused(y_rel=2.3, yv_rel=-0.25, d_rel=34.0)
  for frame in range(10):
    sample = builder.update(context(frame * 0.05), (obj,))[0]
    values = list(sample.values)
    values[MODEL_FEATURE_NAMES.index("h8_y_rate")] = -0.25
    values[MODEL_FEATURE_NAMES.index("h12_y_rate")] = -0.25
    sample = replace(sample, values=tuple(values), d_path=2.05, d_path_future=1.8)
    decision = decision_filter.update(frame * 0.05, (
      RadarLeadPrediction(sample, lead_prob=0.0, cutin_prob=0.01, risk_prob=0.01),
    ))

  assert decision is not None
  assert decision.cutin_candidates


def test_distant_stationary_side_object_never_activates_cutin() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  obj = replace(fused(y_rel=-2.2, yv_rel=0.8, d_rel=24.0), v_lead=0.5)
  for frame in range(16):
    sample = builder.update(context(frame * 0.05), (obj,))[0]
    values = list(sample.values)
    values[MODEL_FEATURE_NAMES.index("h8_y_rate")] = 0.8
    values[MODEL_FEATURE_NAMES.index("h12_y_rate")] = 0.8
    sample = replace(sample, values=tuple(values), d_path=-2.0, d_path_future=-1.2)
    decision = decision_filter.update(frame * 0.05, (
      RadarLeadPrediction(sample, lead_prob=0.0, cutin_prob=0.99, risk_prob=0.99),
    ))

  assert decision is not None
  assert not decision.cutin_candidates


def test_uncertain_lanes_do_not_enable_midrange_projected_cutin() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  obj = fused(y_rel=-2.2, yv_rel=0.8, d_rel=24.0)
  for frame in range(16):
    sample = builder.update(uncertain_lane_context(frame * 0.05), (obj,))[0]
    values = list(sample.values)
    values[MODEL_FEATURE_NAMES.index("h8_y_rate")] = 0.8
    values[MODEL_FEATURE_NAMES.index("h12_y_rate")] = 0.8
    sample = replace(sample, values=tuple(values), d_path=-2.0, d_path_future=-1.2)
    decision = decision_filter.update(frame * 0.05, (
      RadarLeadPrediction(sample, lead_prob=0.0, cutin_prob=0.99, risk_prob=0.99),
    ))

  assert decision is not None
  assert not decision.cutin_candidates


def test_midrange_fused_object_far_from_ego_axis_is_not_forced_active() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  obj = fused(y_rel=-4.8, yv_rel=1.3, d_rel=39.0)
  for frame in range(10):
    sample = builder.update(context(frame * 0.05), (obj,))[0]
    values = list(sample.values)
    values[MODEL_FEATURE_NAMES.index("h8_y_rate")] = 1.0
    values[MODEL_FEATURE_NAMES.index("h12_y_rate")] = 1.0
    sample = replace(sample, values=tuple(values), d_path=-2.3, d_path_future=-1.1)
    decision = decision_filter.update(frame * 0.05, (
      RadarLeadPrediction(sample, lead_prob=0.0, cutin_prob=0.01, risk_prob=0.01),
    ))

  assert decision is not None
  assert not decision.cutin_candidates


def test_midrange_fast_adjacent_lane_change_is_not_forced_active() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  decision = None
  obj = fused(y_rel=3.04, yv_rel=-1.13, d_rel=28.0)
  for frame in range(10):
    sample = builder.update(context(frame * 0.05), (obj,))[0]
    values = list(sample.values)
    values[MODEL_FEATURE_NAMES.index("h8_y_rate")] = -1.2
    values[MODEL_FEATURE_NAMES.index("h12_y_rate")] = -0.9
    sample = replace(sample, values=tuple(values), d_path=2.37, d_path_future=1.20)
    decision = decision_filter.update(frame * 0.05, (
      RadarLeadPrediction(sample, lead_prob=0.0, cutin_prob=0.01, risk_prob=0.01),
    ))

  assert decision is not None
  assert not decision.cutin_candidates


def test_sticky_cutin_expires_after_last_directional_evidence() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.82)
  obj = fused(y_rel=2.1, yv_rel=-0.25, d_rel=34.0)
  decision = None
  for frame in range(10):
    sample = builder.update(context(frame * 0.05), (obj,))[0]
    values = list(sample.values)
    values[MODEL_FEATURE_NAMES.index("h8_y_rate")] = -0.25
    values[MODEL_FEATURE_NAMES.index("h12_y_rate")] = -0.25
    sample = replace(sample, values=tuple(values), d_path=2.05, d_path_future=1.80)
    decision = decision_filter.update(frame * 0.05, (
      RadarLeadPrediction(sample, lead_prob=0.0, cutin_prob=0.01, risk_prob=0.01),
    ))

  assert decision is not None and decision.cutin_candidates
  for frame in range(1, 25):
    time_s = 0.45 + frame * 0.05
    sample = builder.update(context(time_s), (replace(obj, yv_rel=0.0),))[0]
    sample = replace(sample, d_path=2.1, d_path_future=2.1)
    decision = decision_filter.update(time_s, (
      RadarLeadPrediction(sample, lead_prob=0.0, cutin_prob=0.01, risk_prob=0.01),
    ))

  assert not decision.cutin_candidates


def test_close_parallel_object_keeps_normal_threshold() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.87)
  decision = None
  for frame in range(8):
    sample = builder.update(context(frame * 0.05), (fused(yv_rel=0.0, d_rel=6.0),))[0]
    decision = decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.8, 0.68, 0.68),))

  assert decision is not None
  assert not decision.cutin_candidates
