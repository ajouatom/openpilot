import csv
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.carrot.radar_lead_simulator import (
  MODEL_FEATURE_NAMES,
  Candidate,
  MLPLeadSelector,
  ManualLabels,
  ModelLead,
  RadarFrame,
  RadarPoint,
  RecordedLead,
  SimpleLeadSelector,
  Selection,
  ValidationReview,
  aligned_video_time_s,
  candidate_track_id,
  comparison_summary,
  _copy_track_points,
  _route_replay_module,
  export_training_dataset,
  qcamera_path_for_log,
  resolve_validation_case,
  resolved_recorded_track_id,
  validation_review_events,
)
from openpilot.selfdrive.carrot.radar_lead_train import (
  TrainingData,
  combine_training_and_validation,
  fit_probability_calibration,
  group_metrics,
  probability_calibration_metrics,
)


def point(track_id: int, d_rel: float, y_rel: float, v_lead: float, source: str = "frontRadar") -> RadarPoint:
  return RadarPoint(track_id, d_rel, y_rel, 0.0, 0.0, 0.0, v_lead, True, source)


def recorded(track_id: int = -1, status: bool = False) -> RecordedLead:
  return RecordedLead(status, status, track_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def frame(
  points: tuple[RadarPoint, ...],
  recorded_one: RecordedLead | None = None,
  recorded_two: RecordedLead | None = None,
) -> RadarFrame:
  return RadarFrame(
    mono_time_s=0.0,
    time_s=0.0,
    input_age_s=0.0,
    model_age_s=0.0,
    v_ego=20.0,
    points=points,
    path=((0.0, 0.0), (100.0, 0.0)),
    lane_lines=(),
    lane_probs=(),
    model_leads=(ModelLead(0.9, 31.52, -0.2, 20.0, 0.0, 1.0, 0.5, 1.0),),
    recorded_one=recorded_one or recorded(),
    recorded_two=recorded_two or recorded(),
  )


def test_simple_selector_matches_model_lead() -> None:
  selected = SimpleLeadSelector().select(frame((
    point(10, 30.0, 0.2, 20.0),
    point(11, 45.0, -2.0, 15.0),
  )))

  assert candidate_track_id(selected.lead_one) == 10


def test_validation_review_rearms_cutin_track_after_it_clears() -> None:
  frames = [replace(frame(()), mono_time_s=float(index), time_s=float(index), model_leads=()) for index in range(6)]
  lead_one = Candidate(10, 0.9, "MLP active lead")
  lead_two = Candidate(20, 0.9, "MLP active cutin")
  selections = (
    Selection(None, None),
    Selection(lead_one, None),
    Selection(lead_one, lead_two, active_cutin_candidates=(lead_two,)),
    Selection(lead_one, lead_two, active_cutin_candidates=(lead_two,)),
    Selection(lead_one, None),
    Selection(lead_one, lead_two, active_cutin_candidates=(lead_two,)),
  )

  class Selector:
    def select(self, _frame, frame_index=None):
      return selections[int(frame_index)]

  review = ValidationReview("case", "detect", "corner", 40.0, 50.0, "scene")

  assert validation_review_events(frames, Selector(), review) == {
    2: ("CUT-IN id 20",),
    5: ("CUT-IN id 20",),
  }


def test_validation_review_without_cutin_has_no_pause_events() -> None:
  frames = [replace(frame(()), mono_time_s=float(index), time_s=float(index), model_leads=()) for index in range(3)]

  class Selector:
    def select(self, _frame, frame_index=None):
      return Selection(None, None)

  review = ValidationReview("clear", "clear", "corner", 0.0, 2.0, "scene")
  assert validation_review_events(frames, Selector(), review) == {}


def test_stationary_review_pauses_on_selected_target_track() -> None:
  frames = [
    replace(frame((point(35, 108.0, 0.1, 0.0),)), mono_time_s=float(index), time_s=float(index))
    for index in range(3)
  ]

  class Selector:
    def select(self, _frame, frame_index=None):
      if frame_index == 0:
        return Selection(None, Candidate(35, 0.9, "MLP active stealth"))
      return Selection(Candidate(35, 0.9, "vision-radar Laplacian match"), None)

  review = ValidationReview("stopped", "stationary", "front", 0.0, 2.0, "scene", (35,))
  assert validation_review_events(frames, Selector(), review) == {
    0: ("STATIONARY leadTwo id 35 108m",),
  }


def test_numbered_rlog_uses_matching_qcamera_segment() -> None:
  assert qcamera_path_for_log(Path("route/rlog.1.zst")) == Path("route/qcamera.1.ts")
  assert qcamera_path_for_log(Path("route/rlog.zst")) == Path("route/qcamera.ts")


def test_video_time_aligns_model_frame_to_qcamera_start() -> None:
  assert aligned_video_time_s(1_000_000_000, 6_800_000_000) == 5.8
  assert aligned_video_time_s(0, 6_800_000_000) is None
  assert aligned_video_time_s(7_000_000_000, 6_800_000_000) is None


def test_validation_review_ignores_internal_cutin_on_current_lead_one() -> None:
  frames = [replace(frame(()), mono_time_s=float(index), time_s=float(index)) for index in range(2)]
  lead_one = Candidate(10, 0.9, "MLP active lead")
  internal_cutin = Candidate(10, 0.9, "MLP active cutin")

  class Selector:
    def select(self, _frame, frame_index=None):
      return Selection(lead_one, None, active_cutin_candidates=(internal_cutin,))

  review = ValidationReview("clear", "clear", "corner", 0.0, 2.0, "scene")
  assert validation_review_events(frames, Selector(), review) == {}


def test_validation_case_resolves_route_and_review_metadata(tmp_path: Path) -> None:
  cases_path = tmp_path / "cases.json"
  cases_path.write_text(json.dumps({"cases": [{
    "id": "sample-case", "vehicle_folder": "CAR", "log": "SEG/rlog.zst",
    "source": "front", "window": [8.0, 13.0], "expected": "detect", "scene": "test scene",
  }]}), encoding="utf-8")

  route, review = resolve_validation_case(cases_path, tmp_path / "routes", "sample")

  assert route == tmp_path / "routes" / "CAR" / "SEG" / "rlog.zst"
  assert review == ValidationReview("sample-case", "detect", "front", 8.0, 13.0, "test scene")


def test_simple_selector_uses_distinct_path_candidate_for_lead_two() -> None:
  selected = SimpleLeadSelector().select(frame((
    point(10, 30.0, 0.2, 20.0),
    point(21, 18.0, 0.6, 18.0, "corner235"),
  )))

  assert candidate_track_id(selected.lead_one) == 10
  assert candidate_track_id(selected.lead_two) == 21


def test_recorded_slot_id_resolves_to_stable_corner_track_by_kinematics() -> None:
  sample = frame((
    point(62, 60.8, -1.4, -2.0),
    point(1003, 56.1, -1.3, -1.7, "corner180"),
  ))
  lead = RecordedLead(True, True, 2445, 56.3, -1.4, -1.6, 0.0, 0.0, 0.0, 0.0)

  assert resolved_recorded_track_id(sample, lead) == 1003


def test_comparison_summary_counts_exact_radar_ids() -> None:
  sample = frame(
    (point(10, 30.0, 0.2, 20.0), point(21, 18.0, 0.6, 18.0, "corner235")),
    recorded(10, True),
    recorded(21, True),
  )

  summary = comparison_summary([sample], SimpleLeadSelector())

  assert summary["lead_one_matches"] == 1
  assert summary["lead_two_matches"] == 1


def test_manual_labels_round_trip_and_training_export(tmp_path: Path) -> None:
  frames = [frame((
    point(10, 30.0, 0.2, 20.0),
    point(21, 18.0, 0.6, 18.0, "corner235"),
  ))]
  labels = ManualLabels()
  labels.set(0, "leadOne", 10)
  labels.set(0, "leadTwo", None)
  label_path = tmp_path / "labels.json"
  labels.save(label_path, tmp_path / "rlog.zst", frames)

  loaded = ManualLabels.load(label_path, len(frames))
  assert loaded.get(0, "leadOne") == (True, 10)
  assert loaded.get(0, "leadTwo") == (True, None)

  dataset_path = tmp_path / "dataset.csv"
  stats = export_training_dataset(dataset_path, frames, loaded, manual_only=True)
  with dataset_path.open(newline="", encoding="utf-8") as source:
    rows = list(csv.DictReader(source))

  assert stats == {
    "groups": 1,
    "manual_groups": 1,
    "recorded_groups": 0,
    "teacher_groups": 0,
    "none_groups": 0,
    "rows": 2,
    "positives": 1,
    "skipped": 0,
    "duplicate_groups_skipped": 0,
  }
  assert sum(int(row["is_positive"]) for row in rows) == 1


def test_mlp_selector_runs_shared_model_per_radar_source(tmp_path: Path) -> None:
  model_path = tmp_path / "model.npz"
  input_size = len(MODEL_FEATURE_NAMES)
  np.savez_compressed(
    model_path,
    feature_names=np.asarray(MODEL_FEATURE_NAMES),
    mean=np.zeros(input_size, dtype=np.float32),
    std=np.ones(input_size, dtype=np.float32),
    thresholds=np.asarray([0.4], dtype=np.float32),
    w1=np.zeros((input_size, 2), dtype=np.float32),
    b1=np.zeros(2, dtype=np.float32),
    w2=np.zeros((2, 2), dtype=np.float32),
    b2=np.zeros(2, dtype=np.float32),
    w3=np.zeros((2, 1), dtype=np.float32),
    b3=np.zeros(1, dtype=np.float32),
  )
  frames = [frame((
    point(10, 30.0, 0.2, 20.0),
    point(11, 34.0, -0.2, 20.0),
    point(21, 18.0, 0.6, 18.0, "corner235"),
    point(22, 22.0, -0.6, 18.0, "corner180"),
  ))]

  selected = MLPLeadSelector(model_path, frames).select(frames[0], 0)

  assert candidate_track_id(selected.lead_one) == 10
  assert candidate_track_id(selected.lead_two) == 11
  assert [candidate.track_id for candidate in selected.front_candidates] == [10, 11]
  assert [candidate.track_id for candidate in selected.corner_candidates] == [21, 22]


def test_mlp_selector_preserves_ranked_candidates_below_decision_threshold(tmp_path: Path) -> None:
  model_path = tmp_path / "model.npz"
  input_size = len(MODEL_FEATURE_NAMES)
  np.savez_compressed(
    model_path,
    feature_names=np.asarray(MODEL_FEATURE_NAMES),
    mean=np.zeros(input_size, dtype=np.float32),
    std=np.ones(input_size, dtype=np.float32),
    thresholds=np.asarray([0.8, 0.8], dtype=np.float32),
    w1=np.zeros((input_size, 2), dtype=np.float32),
    b1=np.zeros(2, dtype=np.float32),
    w2=np.zeros((2, 2), dtype=np.float32),
    b2=np.zeros(2, dtype=np.float32),
    w3=np.zeros((2, 1), dtype=np.float32),
    b3=np.zeros(1, dtype=np.float32),
  )
  frames = [frame((point(10, 30.0, 0.2, 20.0), point(21, 18.0, 0.6, 18.0, "corner235")))]

  selected = MLPLeadSelector(model_path, frames).select(frames[0], 0)

  assert selected.lead_one is None
  assert selected.lead_two is None
  assert [candidate.track_id for candidate in selected.front_candidates] == [10]
  assert [candidate.track_id for candidate in selected.corner_candidates] == [21]
  assert not selected.front_candidates[0].eligible
  assert not selected.corner_candidates[0].eligible


def test_held_out_dataset_groups_do_not_overlap_training() -> None:
  def data(group: int) -> TrainingData:
    return TrainingData(
      features=np.zeros((1, len(MODEL_FEATURE_NAMES)), dtype=np.float32),
      labels=np.ones(1, dtype=np.float32),
      groups=np.asarray([group], dtype=np.int32),
      sources=np.zeros(1, dtype=np.int8),
      sample_weights=np.ones(1, dtype=np.float32),
      manual=np.zeros(1, dtype=np.bool_),
    )

  combined, training_indices, validation_indices = combine_training_and_validation(data(0), data(0))

  assert set(combined.groups[training_indices]).isdisjoint(set(combined.groups[validation_indices]))


def test_duplicate_recorded_leads_are_one_training_target(tmp_path: Path) -> None:
  sample = frame(
    (point(10, 30.0, 0.2, 20.0), point(21, 18.0, 0.6, 18.0)),
    recorded(10, True),
    recorded(10, True),
  )

  stats = export_training_dataset(tmp_path / "dataset.csv", [sample], ManualLabels())

  assert stats["groups"] == 1
  assert stats["positives"] == 1
  assert stats["duplicate_groups_skipped"] == 0


def test_recorded_lead_one_and_two_form_one_two_positive_group(tmp_path: Path) -> None:
  sample = frame(
    (point(10, 30.0, 0.2, 20.0), point(21, 18.0, 0.6, 18.0)),
    recorded(10, True),
    recorded(21, True),
  )
  dataset_path = tmp_path / "dataset.csv"

  stats = export_training_dataset(dataset_path, [sample], ManualLabels())
  with dataset_path.open(newline="", encoding="utf-8") as source:
    rows = list(csv.DictReader(source))

  assert stats["groups"] == 1
  assert stats["positives"] == 2
  assert {row["target_track_ids"] for row in rows} == {"10;21"}
  assert {int(row["track_id"]) for row in rows if row["is_positive"] == "1"} == {10, 21}


def test_group_metrics_accepts_two_probability_ranked_outputs() -> None:
  metrics = group_metrics(
    probabilities=np.asarray([0.9, 0.8, 0.1], dtype=np.float32),
    labels=np.asarray([1.0, 1.0, 0.0], dtype=np.float32),
    groups=np.zeros(3, dtype=np.int32),
    threshold=0.5,
  )

  assert metrics["exact"] == 1
  assert metrics["precision"] == 1.0
  assert metrics["recall"] == 1.0


def test_group_metrics_keep_two_outputs_from_each_radar_source() -> None:
  metrics = group_metrics(
    probabilities=np.asarray([0.9, 0.8, 0.85, 0.75, 0.1], dtype=np.float32),
    labels=np.asarray([1.0, 1.0, 1.0, 1.0, 0.0], dtype=np.float32),
    groups=np.zeros(5, dtype=np.int32),
    threshold=(0.5, 0.5),
    sources=np.asarray([0, 0, 1, 1, 1], dtype=np.int8),
  )

  assert metrics["exact"] == 1
  assert metrics["precision"] == 1.0
  assert metrics["recall"] == 1.0


def test_legacy_corner_track_ids_are_normalized() -> None:
  def raw_point(track_id: int, source: str = "frontRadar") -> SimpleNamespace:
    return SimpleNamespace(
      trackId=track_id,
      dRel=10.0,
      yRel=2.0,
      vRel=0.0,
      aRel=0.0,
      yvRel=0.0,
      vLead=20.0,
      measured=True,
      radarSource=source,
    )

  copied = _copy_track_points((raw_point(42), raw_point(205), raw_point(244)))

  assert [point.source for point in copied] == ["frontRadar", "corner235", "corner180"]


def test_reconstructed_corner_tracks_fill_only_missing_groups() -> None:
  route_replay = _route_replay_module()
  recorded = SimpleNamespace(trackId=205, radarSource="frontRadar")
  corner235 = route_replay.ReconstructedLiveTrack(1000, 10.0, 2.0, 0.0, 0.0, 0.0, 20.0, True, "corner235")
  corner180 = route_replay.ReconstructedLiveTrack(1001, 12.0, -2.0, 0.0, 0.0, 0.0, 20.0, True, "corner180")

  merged = route_replay.merge_recorded_and_reconstructed_tracks((recorded,), (corner235, corner180))

  assert [point.trackId for point in merged] == [205, 1001]

  preferred = route_replay.merge_recorded_and_reconstructed_tracks(
    (recorded,), (corner235, corner180), prefer_reconstructed_corner=True
  )

  assert [point.trackId for point in preferred] == [1000, 1001]

  front = SimpleNamespace(trackId=42, radarSource="frontRadar")
  raw_only = route_replay.merge_recorded_and_reconstructed_tracks(
    (front, recorded), (), raw_corner_only=True,
  )

  assert [point.trackId for point in raw_only] == [42]


def test_route_parser_accepts_explicit_front_cutin_source() -> None:
  parser = _route_replay_module().RouteLogParser(cutin_radar_source="front")

  assert parser.cutin_radar_source == "front"


def test_probability_calibration_corrects_weighted_logit_bias() -> None:
  logits = np.asarray([-1.0, 1.0, 3.0, 5.0], dtype=np.float32)
  labels = np.asarray([0.0, 0.0, 1.0, 1.0], dtype=np.float32)
  raw = 1.0 / (1.0 + np.exp(-logits))

  scale, bias = fit_probability_calibration(logits, labels)
  calibrated = 1.0 / (1.0 + np.exp(-(logits * scale + bias)))

  assert scale > 0.0
  assert probability_calibration_metrics(calibrated, labels)["brier"] < probability_calibration_metrics(raw, labels)["brier"]
