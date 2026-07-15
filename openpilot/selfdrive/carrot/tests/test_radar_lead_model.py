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
    sample = builder.update(context(frame * 0.05), (fused(),))[0]
    prediction = RadarLeadPrediction(sample, lead_prob=0.8, cutin_prob=0.8, risk_prob=0.8)
    decision = decision_filter.update(frame * 0.05, (prediction,))

  assert decision is not None
  assert len(decision.lead_candidates) == 1
  assert len(decision.cutin_candidates) == 1


def test_moving_away_cutin_is_released_quickly() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter()
  for frame in range(7):
    sample = builder.update(context(frame * 0.05), (fused(),))[0]
    decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.8, 0.8, 0.8),))

  moving_away = fused(y_rel=2.5, yv_rel=1.5)
  sample = builder.update(context(0.35), (moving_away,))[0]
  decision = decision_filter.update(0.35, (RadarLeadPrediction(sample, 0.8, 0.3, 0.3),))
  assert decision.cutin_candidates
  decision = decision_filter.update(0.50, (RadarLeadPrediction(sample, 0.8, 0.1, 0.1),))
  assert not decision.cutin_candidates


def test_sticky_cutin_does_not_expose_low_probability_reassociation() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter()
  sample = None
  for frame in range(7):
    sample = builder.update(context(frame * 0.05), (fused(),))[0]
    decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.8, 0.8, 0.8),))

  assert sample is not None
  decision = decision_filter.update(0.36, (RadarLeadPrediction(sample, 0.8, 0.01, 0.01),))
  assert not decision.cutin_candidates


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
    sample = builder.update(context(frame * 0.05), (fused(d_rel=6.0),))[0]
    decision = decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.8, 0.68, 0.68),))

  assert decision is not None
  assert decision.cutin_candidates


def test_close_parallel_object_keeps_normal_threshold() -> None:
  builder = RadarLeadFeatureBuilder()
  decision_filter = RadarLeadDecisionFilter(cutin_threshold=0.87)
  decision = None
  for frame in range(8):
    sample = builder.update(context(frame * 0.05), (fused(yv_rel=0.0, d_rel=6.0),))[0]
    decision = decision_filter.update(frame * 0.05, (RadarLeadPrediction(sample, 0.8, 0.68, 0.68),))

  assert decision is not None
  assert not decision.cutin_candidates
