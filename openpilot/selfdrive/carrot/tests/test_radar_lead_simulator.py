import csv
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator as radar_simulator
from openpilot.selfdrive.carrot.radar.radar_vision_model_controller import RadarLeadModelOutput

from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import (
  MODEL_FEATURE_NAMES,
  Candidate,
  CurrentRadardTeacher,
  MLPLeadSelector,
  ManualLabels,
  ModelLead,
  RadarFrame,
  RadarPoint,
  RecordedLead,
  SimulatorUI,
  ProductionHybridLeadSelector,
  SimpleLeadSelector,
  Selection,
  ValidationReview,
  aligned_video_time_s,
  candidate_track_id,
  comparison_summary,
  cutin_stage_series,
  lead_comparison_series,
  radar_trajectory_series,
  load_review_probability,
  save_review_probability,
  _copy_track_points,
  preferred_radar_points,
  _route_replay_module,
  export_training_dataset,
  front_only_frames,
  qcamera_path_for_log,
  resolve_validation_case,
  resolve_validation_cases,
  resolved_recorded_track_id,
  trajectory_review_events,
  trajectory_model_review_events,
  upsert_trajectory_review_label,
  update_validation_case_label,
  validation_review_events,
)
from openpilot.selfdrive.carrot.radar.tools.radar_lead_validation_review import group_cases_by_log
from openpilot.selfdrive.carrot.radar.tools.radar_lead_train import (
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


def test_front_only_frames_remove_corner_points_without_changing_other_inputs() -> None:
  original = frame((
    point(10, 30.0, 0.2, 20.0),
    point(1010, 29.5, 0.3, 20.0, "corner235"),
    point(0, 31.0, 0.1, 20.0, "scc"),
  ))

  filtered, removed = front_only_frames([original])

  assert removed == 1
  assert [point.source for point in filtered[0].points] == ["frontRadar", "scc"]
  assert filtered[0].model_leads == original.model_leads
  assert filtered[0].recorded_one == original.recorded_one


def test_preferred_radar_points_uses_validation_sensor_source() -> None:
  radar_frame = frame((
    point(10, 30.0, 0.2, 20.0),
    point(1010, 29.5, 0.3, 20.0, "corner235"),
    point(0, 31.0, 0.1, 20.0, "scc"),
  ))

  assert [item.track_id for item in preferred_radar_points(radar_frame, "corner")] == [1010]
  assert [item.track_id for item in preferred_radar_points(radar_frame, "front")] == [10]


def test_validation_runner_groups_duplicate_log_cases() -> None:
  groups = group_cases_by_log([
    {"id": "first", "vehicle_folder": "CAR", "log": "SEG/rlog.zst"},
    {"id": "second", "vehicle_folder": "CAR", "log": "SEG/rlog.zst"},
    {"id": "third", "vehicle_folder": "CAR", "log": "OTHER/rlog.zst"},
  ])

  assert [[case["id"] for case in group] for group in groups] == [
    ["first", "second"],
    ["third"],
  ]


def test_trajectory_review_uses_corner_from_combined_source() -> None:
  frames = [
    replace(
      radar_center_frame((
        point(1010, 25.0, 4.5 - index * 0.5, 20.0, "corner235"),
        point(10, 25.0, -4.5 + index * 0.5, 20.0, "frontRadar"),
      )),
      mono_time_s=index * 0.25,
      time_s=index * 0.25,
      path_y_stds=((0.0, 0.15), (100.0, 0.15)),
      lane_stds=(0.2, 0.2, 0.2),
    )
    for index in range(4)
  ]
  trajectories = radar_trajectory_series(frames)

  assert trajectory_review_events(
    frames, trajectories, ("front+corner",), horizon_s=1.0,
  ) == {
    3: ("TRAJECTORY CORNER id 1010 p1.00 t0.19s",),
  }


def test_trajectory_review_merges_same_vehicle_after_track_id_change() -> None:
  frames = []
  trajectories = []
  for index in range(8):
    track_id = 1010 if index < 4 else 1011
    sample_point = point(
      track_id,
      25.0,
      4.5 - index * 0.5,
      20.0,
      "corner235",
    )
    frames.append(replace(
      radar_center_frame((sample_point,)),
      mono_time_s=index * 0.25,
      time_s=index * 0.25,
    ))
    is_candidate = index in (3, 7)
    trajectories.append({
      ("corner235", track_id): SimpleNamespace(
        history_count=4,
        reason="trajectory candidate" if is_candidate else "not entering",
        time_to_entry_s=0.25 if is_candidate else None,
        samples=(SimpleNamespace(horizon_s=1.0, occupancy_prob=1.0),),
      ),
    })

  assert trajectory_review_events(
    frames, trajectories, ("corner",), horizon_s=1.0,
  ) == {
    3: ("TRAJECTORY CORNER id 1010 p1.00 t0.25s",),
  }


def test_trajectory_model_review_uses_display_probability_threshold() -> None:
  frames = [
    replace(
      frame((point(1010, 25.0, 3.0, 20.0, "corner235"),)),
      mono_time_s=index * 0.25,
      time_s=index * 0.25,
    )
    for index in range(3)
  ]
  scores = (0.40, 0.70, 0.80)

  class Selector:
    def select(self, _frame, frame_index=None):
      score = scores[frame_index]
      candidate = Candidate(
        1010, score, "trajectory corner path-entry",
        d_rel=25.0, y_rel=3.0, horizon_scores=(score, score, score, score),
      )
      return Selection(None, None, cutin_diagnostics=(candidate,))

  assert trajectory_model_review_events(frames, Selector(), ("corner",), 0.75) == {
    2: ("TRAJECTORY MODEL CORNER id 1010 IN0.80 OUT0.00 raw 0.80/0.80/0.80/0.80",),
  }


def test_trajectory_model_review_ignores_candidate_behind_lead_one() -> None:
  frames = [
    replace(
      frame((point(1010, 40.0, 3.0, 20.0, "corner235"),)),
      mono_time_s=index * 0.25,
      time_s=index * 0.25,
    )
    for index in range(2)
  ]

  class Selector:
    def select(self, _frame, frame_index=None):
      candidate = Candidate(
        1010, 1.0, "trajectory corner path-entry",
        d_rel=40.0, y_rel=3.0, stage="BLOCK-LEAD",
      )
      return Selection(None, None, cutin_diagnostics=(candidate,))

  assert trajectory_model_review_events(frames, Selector(), ("corner",), 0.99) == {}


def test_review_probability_settings_round_trip(tmp_path: Path) -> None:
  settings = tmp_path / "review.json"

  assert load_review_probability(settings) == 0.5
  assert save_review_probability(0.65, settings) is True
  assert load_review_probability(settings) == 0.65


def test_device_review_mode_pauses_only_for_production_cutin_output() -> None:
  frames = [
    replace(
      frame((point(10, 20.0, 3.0, 15.0, "corner235"),)),
      mono_time_s=float(index),
      time_s=float(index),
      model_leads=(),
    )
    for index in range(2)
  ]
  raw = Candidate(
    10, 1.0, "trajectory corner path-entry",
    d_rel=20.0, y_rel=3.0, stage="DECISION",
  )
  selected = Candidate(20, 0.82, "MLP confirmed cutin")

  class Selector:
    def select(self, _frame, frame_index=None):
      if int(frame_index) == 1:
        return Selection(
          None,
          selected,
          cutin_diagnostics=(raw,),
          active_cutin_candidates=(selected,),
        )
      return Selection(None, None, cutin_diagnostics=(raw,))

  review = ValidationReview("device", "detect", "corner", 0.0, 1.0, "scene")
  ui = SimulatorUI.__new__(SimulatorUI)
  ui.frames = frames
  ui.selector = Selector()
  ui.review = review
  ui.reviews = (review,)
  ui.device_review_mode = True
  ui.min_candidate_probability = 0.5
  ui.trajectory_horizon_s = 1.0
  ui.trajectories = ()
  ui.trajectory_review_labels = {}

  ui._prepare_review_events()

  assert ui.review_events == {1: ("CUT-IN id 20",)}
  assert "DEVICE REVIEW ARMED" in ui.review_status


def test_user_rewind_rearms_and_pauses_same_review_event(monkeypatch) -> None:
  review = ValidationReview("case", "clear", "corner", 0.0, 3.0, "scene")
  ui = SimulatorUI.__new__(SimulatorUI)
  ui.frames = [frame(()) for _ in range(4)]
  ui.times = [0.0, 1.0, 2.0, 3.0]
  ui.index = 2
  ui.playback_time = 2.0
  ui.review = review
  ui.reviews = (review,)
  ui.review_events = {2: ("TRAJECTORY MODEL CORNER id 1010 p0.80",)}
  ui.review_handled = {2}
  ui.review_suppressed = set()
  ui.min_candidate_probability = 0.75
  ui.paused = True
  alerts = []
  monkeypatch.setattr(radar_simulator, "play_review_alert", alerts.append)

  ui._user_seek(0.5)
  assert ui.review_handled == set()

  ui._pause_for_review(0, 2)
  assert ui.paused is True
  assert ui.index == 2
  assert ui.review_handled == {2}
  assert alerts == [ui.review_events[2]]


def test_cutin_stage_series_keeps_model_to_selected_stages_separate() -> None:
  radar_frame = frame((point(20, 18.0, -1.5, 15.0, "corner235"),))
  model = Candidate(20, 0.61, "MLP corner cutin", stage="WAIT-CONFIRM")
  decision = Candidate(20, 0.72, "MLP decision cutin")
  output = Candidate(20, 0.81, "MLP output cutin")
  selected = Candidate(20, 0.90, "MLP confirmed cutin")

  class Selector:
    def select(self, _frame, frame_index=None):
      return Selection(
        None,
        selected,
        cutin_diagnostics=(model,),
        decision_cutin_candidates=(decision,),
        active_cutin_candidates=(output,),
      )

  stages = cutin_stage_series((radar_frame,), Selector())

  assert stages[0].model == model
  assert stages[0].decision == decision
  assert stages[0].output == output
  assert stages[0].selected == selected


def test_production_hybrid_selector_uses_device_controller_output(monkeypatch, tmp_path: Path) -> None:
  lead_one = {
    "status": True, "radarTrackId": 10, "modelProb": 0.9, "score": 0.9,
    "dRel": 30.0, "yRel": 0.2,
  }
  lead_two = {
    "status": True, "radarTrackId": 20, "modelProb": 0.8, "score": 0.8,
    "dRel": 18.0, "yRel": -1.5,
  }

  class FakeDeviceController:
    def __init__(self) -> None:
      self.runtime = None
      self.last_runtime_result = None

    def update(self, time_s, v_ego, points, model, car_state=None):
      assert points[0].trackId == 10
      assert model.leadsV3[0].prob == 0.9
      assert car_state is not None
      self.runtime.model = SimpleNamespace(thresholds=(0.5, 0.5, 0.5))
      self.last_runtime_result = SimpleNamespace(available=True, predictions=())
      return RadarLeadModelOutput(
        True, lead_one=lead_one, lead_two=lead_two, lead_cutin=lead_two, leads_cutin=(lead_two,),
      )

  monkeypatch.setattr(radar_simulator, "VisionModelRadarController", FakeDeviceController)
  selector = ProductionHybridLeadSelector(tmp_path / "unused-model.npz", [frame((
    point(10, 30.0, 0.2, 20.0),
  ))])
  selected = selector.select(frame(()), 0)

  assert selector.name.endswith(":front-only")
  assert candidate_track_id(selected.lead_one) == 10
  assert candidate_track_id(selected.lead_two) == 20
  assert selected.lead_two is not None and selected.lead_two.reason == "MLP confirmed cutin"
  assert selected.lead_two_tentative is False
  assert tuple(candidate.track_id for candidate in selected.active_cutin_candidates) == (20,)


def test_current_radard_teacher_rejects_adjacent_lane_distance_match() -> None:
  frames = [replace(frame((
      point(35, 9.6, 2.8, 13.8),
      point(36, 6.7, -0.6, 13.0),
    )), model_leads=(ModelLead(0.999, 11.12, -0.1, 13.5, 0.0, 1.0, 2.0, 2.0),))
    for _ in range(4)
  ]

  selected = CurrentRadardTeacher(frames).select(frames[-1], len(frames) - 1)

  assert candidate_track_id(selected.lead_one) == 36


def radar_center_frame(points: tuple[RadarPoint, ...]) -> RadarFrame:
  lane_xs = ((0.0, 1.8), (100.0, 1.8))
  lane_center = ((0.0, 0.0), (100.0, 0.0))
  lane_right = ((0.0, -1.8), (100.0, -1.8))
  return replace(
    frame(points),
    lane_lines=(lane_center, lane_xs, lane_right),
    lane_probs=(0.0, 1.0, 1.0),
    model_leads=(),
  )


def test_current_radard_teacher_rejects_far_unmatched_corner_center() -> None:
  frames = [radar_center_frame((point(1190, 68.0, 0.2, 12.0, "corner235"),)) for _ in range(8)]

  selected = CurrentRadardTeacher(frames).select(frames[-1], len(frames) - 1)

  assert selected.lead_two is None


def test_current_radard_teacher_keeps_near_corner_center() -> None:
  frames = [radar_center_frame((point(1190, 35.0, 0.2, 12.0, "corner235"),)) for _ in range(6)]

  selected = CurrentRadardTeacher(frames).select(frames[-1], len(frames) - 1)

  assert candidate_track_id(selected.lead_two) == 1190


def test_current_radard_teacher_rechecks_discontinuous_corner_track() -> None:
  frames = [radar_center_frame((point(1189, 40.0, 0.2, 12.0, "corner235"),)) for _ in range(5)]
  frames.append(radar_center_frame((point(1189, 33.0, 0.2, 25.0, "corner235"),)))

  selected = CurrentRadardTeacher(frames).select(frames[-1], len(frames) - 1)

  assert selected.lead_two is None


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

  review = ValidationReview("case", "detect", "corner", 0.0, 5.0, "scene")

  assert validation_review_events(frames, Selector(), review) == {
    2: ("CUT-IN id 20",),
    5: ("CUT-IN id 20",),
  }


def test_validation_review_labels_production_cutin_confirmation_state() -> None:
  frames = [replace(frame(()), mono_time_s=0.0, time_s=0.0, model_leads=())]
  cutin = Candidate(20, 0.9, "MLP tentative cutin")

  class Selector:
    def select(self, _frame, frame_index=None):
      return Selection(
        None,
        cutin,
        active_cutin_candidates=(cutin,),
        lead_two_tentative=True,
      )

  review = ValidationReview("case", "detect", "corner", 0.0, 1.0, "scene")
  assert validation_review_events(frames, Selector(), review) == {
    0: ("CUT-IN TENTATIVE id 20",),
  }


def test_validation_review_labels_confirmed_production_cutin() -> None:
  frames = [replace(frame(()), mono_time_s=0.0, time_s=0.0, model_leads=())]
  cutin = Candidate(21, 0.9, "MLP confirmed cutin")

  class Selector:
    def select(self, _frame, frame_index=None):
      return Selection(
        None,
        cutin,
        active_cutin_candidates=(cutin,),
        lead_two_tentative=False,
      )

  review = ValidationReview("case", "detect", "corner", 0.0, 1.0, "scene")
  assert validation_review_events(frames, Selector(), review) == {
    0: ("CUT-IN CONFIRMED id 21",),
  }


def test_validation_review_uses_internal_decision_stage_when_requested() -> None:
  frames = [replace(frame(()), mono_time_s=float(index), time_s=float(index), model_leads=()) for index in range(3)]
  internal_cutin = Candidate(1024, 0.42, "MLP decision cutin", track_aliases=(52,))

  class Selector:
    def select(self, _frame, frame_index=None):
      return (
        Selection(None, None, decision_cutin_candidates=(internal_cutin,))
        if int(frame_index) == 1
        else Selection(None, None)
      )

  review = ValidationReview(
    "decision", "detect", "corner", 0.0, 2.0, "scene",
    target_track_ids=(52, 1024), validation_stage="decision",
  )
  assert validation_review_events(frames, Selector(), review) == {
    1: ("CUT-IN id 1024",),
  }


def test_validation_review_covers_full_log_outside_labeled_window() -> None:
  frames = [replace(frame(()), mono_time_s=float(index), time_s=float(index), model_leads=()) for index in range(6)]
  cutin = Candidate(59, 0.9, "MLP active cutin")

  class Selector:
    def select(self, _frame, frame_index=None):
      if int(frame_index) >= 4:
        return Selection(None, cutin, active_cutin_candidates=(cutin,))
      return Selection(None, None)

  review = ValidationReview("early", "detect", "corner", 0.0, 2.0, "scene")
  assert validation_review_events(frames, Selector(), review) == {
    4: ("CUT-IN id 59",),
  }


def test_validation_review_does_not_rearm_low_probability_sticky_cutin() -> None:
  frames = [replace(frame(()), mono_time_s=float(index), time_s=float(index), model_leads=()) for index in range(3)]
  sticky = Candidate(59, 0.01, "MLP active cutin")

  class Selector:
    def select(self, _frame, frame_index=None):
      return Selection(None, sticky, active_cutin_candidates=(sticky,)) if int(frame_index) == 1 else Selection(None, None)

  review = ValidationReview("full", "detect", "corner", 0.0, 2.0, "scene")
  assert validation_review_events(frames, Selector(), review) == {}


def test_validation_review_without_cutin_has_no_pause_events() -> None:
  frames = [replace(frame(()), mono_time_s=float(index), time_s=float(index), model_leads=()) for index in range(3)]

  class Selector:
    def select(self, _frame, frame_index=None):
      return Selection(None, None)

  review = ValidationReview("clear", "clear", "corner", 0.0, 2.0, "scene")
  assert validation_review_events(frames, Selector(), review) == {}


def test_validation_review_pauses_on_forbidden_lead_two() -> None:
  frames = [replace(frame(()), mono_time_s=float(index), time_s=float(index), model_leads=()) for index in range(4)]
  false_lead = Candidate(40, 0.9, "MLP active stealth")

  class Selector:
    def select(self, _frame, frame_index=None):
      return Selection(None, false_lead) if int(frame_index) in (1, 2) else Selection(None, None)

  review = ValidationReview("tunnel", "clear", "front", 0.0, 3.0, "scene", forbidden_lead_two_ids=(40,))
  assert validation_review_events(frames, Selector(), review) == {
    1: ("FALSE leadTwo id 40",),
  }


def test_cutin_review_does_not_pause_for_lead_continuity_diagnostics() -> None:
  frames = [replace(frame(()), mono_time_s=float(index), time_s=float(index)) for index in range(3)]
  selections = (
    Selection(Candidate(43, 1.0, "vision-radar Laplacian match"), None),
    Selection(None, None),
    Selection(Candidate(43, 1.0, "vision-radar Laplacian match"), None),
  )

  class Selector:
    def select(self, _frame, frame_index=None):
      return selections[int(frame_index)]

  review = ValidationReview("early", "detect", "corner", 0.0, 2.0, "scene")
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


def test_all_rlog_variants_use_primary_qcamera() -> None:
  assert qcamera_path_for_log(Path("route/rlog.1.zst")) == Path("route/qcamera.ts")
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


def test_validation_cases_resolve_one_route_without_duplicate_replay(tmp_path: Path) -> None:
  cases_path = tmp_path / "cases.json"
  cases_path.write_text(json.dumps({"cases": [
    {
      "id": "first", "vehicle_folder": "CAR", "log": "SEG/rlog.zst",
      "source": "corner", "window": [8.0, 13.0], "expected": "detect",
      "scene": "first scene",
    },
    {
      "id": "second", "vehicle_folder": "CAR", "log": "SEG/rlog.zst",
      "source": "corner", "window": [20.0, 24.0], "expected": "clear",
      "scene": "second scene",
    },
  ]}), encoding="utf-8")

  route, reviews = resolve_validation_cases(
    cases_path, tmp_path / "routes", ("first", "second"),
  )

  assert route == tmp_path / "routes" / "CAR" / "SEG" / "rlog.zst"
  assert [review.case_id for review in reviews] == ["first", "second"]


def test_validation_case_resolves_human_verified_metadata(tmp_path: Path) -> None:
  cases_path = tmp_path / "cases.json"
  cases_path.write_text(json.dumps({"cases": [{
    "id": "verified", "vehicle_folder": "CAR", "log": "SEG/rlog.zst",
    "source": "corner", "window": [1.0, 2.0], "expected": "clear",
    "human_verified": True, "scene": "verified scene",
  }]}), encoding="utf-8")

  _, review = resolve_validation_case(cases_path, tmp_path / "routes", "verified")

  assert review.human_verified is True


def test_update_validation_case_label_marks_only_target_human_verified(tmp_path: Path) -> None:
  cases_path = tmp_path / "cases.json"
  cases_path.write_text(json.dumps({"cases": [
    {"id": "first", "expected": "clear", "scene": "first"},
    {"id": "second", "expected": "detect", "scene": "second"},
  ]}, indent=2) + "\n", encoding="utf-8")

  update_validation_case_label(cases_path, "first", "stationary")
  update_validation_case_label(cases_path, "first", "detect")

  payload = json.loads(cases_path.read_text(encoding="utf-8"))
  assert payload["cases"][0]["expected"] == "detect"
  assert payload["cases"][0]["human_verified"] is True
  assert "human_verified" not in payload["cases"][1]
  assert cases_path.read_text(encoding="utf-8").count('"human_verified": true') == 1


def test_review_label_save_resumes_playback(monkeypatch, tmp_path: Path) -> None:
  review = ValidationReview("case", "clear", "corner", 0.0, 2.0, "scene")
  ui = SimulatorUI.__new__(SimulatorUI)
  ui.review = review
  ui.reviews = (review,)
  ui.validation_cases_path = tmp_path / "cases.json"
  ui.playback_time = 1.0
  ui.index = 0
  ui.paused = True
  ui.review_events = {}
  ui._review_containing_time = lambda _: review
  ui._prepare_review_events = lambda: None
  monkeypatch.setattr(radar_simulator, "update_validation_case_label", lambda *_: None)

  assert ui._set_review_expected("detect") is True
  assert ui.paused is False
  assert ui.review.expected == "detect"


def test_review_label_save_failure_stays_paused(monkeypatch, tmp_path: Path) -> None:
  review = ValidationReview("case", "clear", "corner", 0.0, 2.0, "scene")
  ui = SimulatorUI.__new__(SimulatorUI)
  ui.review = review
  ui.reviews = (review,)
  ui.validation_cases_path = tmp_path / "cases.json"
  ui.playback_time = 1.0
  ui.index = 0
  ui.paused = True
  ui.review_events = {}
  ui._review_containing_time = lambda _: review
  monkeypatch.setattr(
    radar_simulator,
    "update_validation_case_label",
    lambda *_: (_ for _ in ()).throw(OSError("write failed")),
  )

  assert ui._set_review_expected("detect") is False
  assert ui.paused is True
  assert "LABEL SAVE FAILED" in ui.review_status


def test_trajectory_event_inside_case_saves_candidate_label(monkeypatch, tmp_path: Path) -> None:
  review = ValidationReview("case", "detect", "corner", 15.5, 17.5, "scene", human_verified=True)
  radar_frame = frame((point(1001, 7.0, 3.2, 20.0, "corner235"),))
  ui = SimulatorUI.__new__(SimulatorUI)
  ui.review = review
  ui.reviews = (review,)
  ui.validation_cases_path = tmp_path / "cutin_validation_cases.json"
  ui.log_path = tmp_path / "CAR" / "SEG" / "rlog.zst"
  ui.frames = [radar_frame]
  ui.index = 0
  ui.playback_time = 17.29
  ui.paused = True
  ui.trajectory_horizon_s = 1.0
  ui.trajectory_review_labels = {}
  ui.review_events = {0: ("TRAJECTORY CORNER id 1001 p0.67 t0.75s",)}
  ui._prepare_review_events = lambda: None
  monkeypatch.setattr(
    radar_simulator,
    "update_validation_case_label",
    lambda *_: (_ for _ in ()).throw(AssertionError("case label must not change")),
  )
  monkeypatch.setattr(
    radar_simulator,
    "upsert_trajectory_review_label",
    lambda *_: "manual-corner-1001",
  )

  assert ui._set_review_expected("clear") is True
  assert ui.paused is False
  assert ui.review.expected == "detect"
  assert ui.trajectory_review_labels[(0.0, 1001)] == "clear"


def test_trajectory_label_upsert_replaces_same_point_and_time(tmp_path: Path) -> None:
  labels_path = tmp_path / "radar_trajectory_labels.json"
  log_path = tmp_path / "CAR" / "SEG" / "rlog.zst"
  log_path.parent.mkdir(parents=True)
  sample = replace(
    frame((point(1010, 25.0, 3.0, 20.0, "corner235"),)),
    time_s=12.34,
  )

  first_id = upsert_trajectory_review_label(
    labels_path, log_path, sample, sample.points[0], "detect", 1.0,
  )
  second_id = upsert_trajectory_review_label(
    labels_path, log_path, sample, sample.points[0], "clear", 1.5,
  )
  labels = json.loads(labels_path.read_text(encoding="utf-8"))["labels"]

  assert first_id == second_id
  assert len(labels) == 1
  assert labels[0]["expected"] == "clear"
  assert labels[0]["prediction_horizon_s"] == 1.5
  assert labels[0]["human_verified"] is True


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


def test_lead_comparison_series_contains_current_radard_and_model_distances() -> None:
  sample = frame(
    (point(10, 31.0, 0.2, 20.0), point(21, 18.0, 0.6, 18.0, "corner235")),
    RecordedLead(True, True, 10, 30.5, 0.2, 0.0, 20.0, 0.0, 0.9, 1.0),
    RecordedLead(True, True, 22, 42.0, 0.4, 0.0, 18.0, 0.0, 0.8, 0.9),
  )

  class RadardSelector:
    def select(self, _frame, frame_index=None):
      return Selection(Candidate(21, 1.0, "current radard lead"), None)

  class ModelSelector:
    def select(self, _frame, frame_index=None):
      return Selection(
        Candidate(10, 1.0, "model lead", d_rel=30.8, y_rel=0.2),
        Candidate(21, 0.9, "model cutin"),
      )

  values = lead_comparison_series([sample], RadardSelector(), ModelSelector())[0]

  assert values.radard_one is not None and values.radard_one.d_rel == 18.0
  assert values.radard_two is None
  assert values.model_one is not None and values.model_one.d_rel == 30.8
  assert values.model_two is not None and values.model_two.d_rel == 18.0


def test_lead_comparison_series_keeps_model_history_without_radard_comparison() -> None:
  sample = frame((point(10, 31.0, 0.2, 20.0),))

  class ModelSelector:
    def select(self, _frame, frame_index=None):
      return Selection(Candidate(10, 1.0, "model lead", d_rel=30.8, y_rel=0.2), None)

  values = lead_comparison_series([sample], None, ModelSelector())[0]

  assert values.radard_one is None
  assert values.radard_two is None
  assert values.model_one is not None and values.model_one.d_rel == 30.8
  assert values.model_two is None


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
