from __future__ import annotations

import json
from dataclasses import replace
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.carrot.radar_motion import model_path_point_at_s
from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import (
  Candidate,
  CurrentRadardSelector,
  ModelLead,
  RadarFrame,
  RadarMotionShadowSelector,
  RadarPoint,
  RecordedLead,
  SimulatorUI,
  candidate_track_id,
  front_only_frames,
  is_position_only_reference,
  motion_points_at_model_time,
  preferred_radar_motion_sensor,
  radar_trajectory_series,
  resolve_validation_cases,
  trajectory_history_display_y,
  update_validation_case_label,
)
from openpilot.selfdrive.carrot.radar.tools.radar_lead_validation_review import (
  group_cases_by_log,
  simulator_command,
)
from openpilot.selfdrive.carrot.radar.tools.validate_radar_lead_model import _metrics


def point(
  track_id: int,
  d_rel: float,
  y_rel: float,
  *,
  source: str = "frontRadar",
  measured: bool = True,
) -> RadarPoint:
  return RadarPoint(
    track_id,
    d_rel,
    y_rel,
    0.0,
    0.0,
    0.0,
    20.0,
    measured,
    source,
  )


def recorded(track_id: int = -1, status: bool = False) -> RecordedLead:
  return RecordedLead(
    status,
    status,
    track_id,
    30.0,
    0.0,
    0.0,
    20.0,
    0.0,
    0.0,
    0.0,
  )


def frame(
  points: tuple[RadarPoint, ...],
  *,
  time_s: float = 0.0,
  one: RecordedLead | None = None,
  two: RecordedLead | None = None,
) -> RadarFrame:
  return RadarFrame(
    mono_time_s=time_s,
    time_s=time_s,
    input_age_s=0.0,
    model_age_s=0.0,
    v_ego=20.0,
    points=points,
    path=((0.0, 0.0), (100.0, 0.0)),
    lane_lines=(),
    lane_probs=(),
    model_leads=(ModelLead(0.9, 31.52, 0.0, 20.0, 0.0, 1.0, 0.5, 1.0),),
    recorded_one=one or recorded(),
    recorded_two=two or recorded(),
  )


def test_shadow_selector_does_not_import_existing_radard_lead_roles() -> None:
  frames = [
    frame(
      (
        point(10, 30.0, 0.0),
        point(1010, 25.0, 4.0 - index * 0.4, source="corner235"),
      ),
      time_s=index * 0.1,
      one=recorded(10, True),
    )
    for index in range(5)
  ]
  radard = CurrentRadardSelector(
    frames,
    [set(), set(), set(), {1010}, {1010}],
  )
  shadow = RadarMotionShadowSelector(frames)

  radard_selection = radard.select(frames[-1], len(frames) - 1)
  selection = shadow.select(frames[-1], len(frames) - 1)
  assert candidate_track_id(radard_selection.lead_one) == 10
  assert candidate_track_id(radard_selection.lead_two) == 1010
  assert selection.lead_one is None
  assert selection.lead_two is None
  assert selection.active_cutin_candidates == ()
  assert any(candidate.track_id == 1010 for candidate in selection.cutin_diagnostics)


def test_physical_shadow_is_diagnostic_only_when_radard_has_no_cutin() -> None:
  frames = [
    frame(
      (
        replace(
          point(1010, 25.0, 4.0 - index * 0.1, source="corner235"),
          yv_rel=-1.0,
        ),
      ),
      time_s=index * 0.1,
    )
    for index in range(15)
  ]
  shadow = RadarMotionShadowSelector(frames)
  selection = shadow.select(frames[-1], len(frames) - 1)

  assert selection.lead_two is None
  assert selection.active_cutin_candidates == ()
  assert selection.decision_cutin_candidates


def test_cutin_confirmation_survives_out_to_in_path_transition() -> None:
  frames = [
    frame(
      (
        replace(
          point(
            1010,
            25.0,
            2.6 - index * 0.08,
            source="corner235",
          ),
          yv_rel=-0.8,
        ),
      ),
      time_s=index * 0.1,
    )
    for index in range(20)
  ]
  shadow = RadarMotionShadowSelector(frames)

  confirmed_inside = [
    (
      shadow.trajectories[index][("corner235", 1010)],
      shadow.select(current, index).decision_cutin_candidates,
    )
    for index, current in enumerate(frames)
    if (
      shadow.trajectories[index][("corner235", 1010)].current_path_occupancy
      and shadow.select(current, index).decision_cutin_candidates
    )
  ]

  assert confirmed_inside
  assert confirmed_inside[0][1][0].score > 0.5


def test_front_only_frames_preserve_non_corner_inputs_and_leads() -> None:
  original = frame(
    (
      point(10, 30.0, 0.2),
      point(1010, 29.5, 0.3, source="corner235"),
      point(0, 31.0, 0.1, source="scc"),
    ),
    one=recorded(10, True),
  )

  filtered, removed = front_only_frames([original])

  assert removed == 1
  assert [value.source for value in filtered[0].points] == ["frontRadar", "scc"]
  assert filtered[0].recorded_one == original.recorded_one


def test_corner_motion_is_preferred_for_whole_log_when_available() -> None:
  frames = [
    frame((
      point(10, 30.0, 1.0),
      point(1010, 29.0, 2.8, source="corner235"),
    ), time_s=index * 0.1)
    for index in range(2)
  ]

  values = radar_trajectory_series(frames)

  assert preferred_radar_motion_sensor(frames) == "corner"
  assert set(values[-1]) == {("corner235", 1010)}


def test_front_motion_is_used_when_corner_measurements_are_absent() -> None:
  frames = [
    frame((
      point(10, 30.0, 1.0),
      point(0, 29.0, 0.5, source="scc"),
    ), time_s=index * 0.1)
    for index in range(2)
  ]

  values = radar_trajectory_series(frames)

  assert preferred_radar_motion_sensor(frames) == "front"
  assert set(values[-1]) == {("frontRadar", 10)}


def test_near_zero_vlead_is_exposed_only_as_position_reference() -> None:
  stopped = replace(point(10, 30.0, 1.0), v_lead=0.5)
  current = frame((stopped,))

  assert is_position_only_reference(current, stopped, "front")
  assert radar_trajectory_series((current,))[0] == {}


def test_radar_point_is_projected_to_model_timestamp_before_dpath() -> None:
  current = replace(
    frame((replace(point(10, 30.0, 3.0), v_rel=-2.0, yv_rel=-4.0),)),
    input_age_s=0.0,
    model_age_s=0.02,
  )

  aligned = motion_points_at_model_time(current, "front")

  assert aligned[0].d_rel == pytest.approx(30.04)
  assert aligned[0].y_rel == pytest.approx(3.08)
  prediction = radar_trajectory_series((current,))[0][("frontRadar", 10)]
  assert prediction.d_path == pytest.approx(3.08)


def test_shadow_metrics_ignore_labels_for_the_unselected_sensor() -> None:
  rows = [
    {
      "expected": "detect",
      "shadow_event": (1.0, 1001),
      "shadow_applicable": True,
    },
    {
      "expected": "detect",
      "shadow_event": None,
      "shadow_applicable": False,
    },
  ]

  metrics = _metrics(rows, "shadow_event")

  assert metrics["labels"] == 1
  assert metrics["tp"] == 1
  assert metrics["fn"] == 0


def test_unmeasured_points_are_absent_from_replay_trajectory_series() -> None:
  frames = [
    frame((point(10, 30.0, 2.0, measured=False),), time_s=0.0),
    frame((point(10, 30.0, 1.8),), time_s=0.1),
  ]

  values = radar_trajectory_series(frames)

  assert values[0] == {}
  assert values[1][("frontRadar", 10)].history_count == 1


def test_history_display_reprojects_saved_dpath_on_current_model_path() -> None:
  current = replace(
    frame(()),
    path=((0.0, 0.0), (100.0, 10.0)),
  )
  sample = type("Sample", (), {"path_x": 50.0, "d_path": 2.0})()

  expected_y = model_path_point_at_s(current.path, 50.0, 2.0)[1]
  assert trajectory_history_display_y(current, sample) == pytest.approx(
    expected_y,
  )


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


def test_validation_runner_launches_importable_module_and_advances_at_end(tmp_path) -> None:
  cases = tmp_path / "cases.json"
  command = simulator_command(
    [{"id": "case-a"}],
    tmp_path,
    cases,
    0.5,
    "1/40",
    True,
  )

  assert command[1:3] == [
    "-m",
    "openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator",
  ]
  assert "--exit-at-end" in command
  assert command[command.index("--review-position") + 1] == "1/40"
  assert command[-1] == "--front-only"


def test_predictor_event_pause_seeks_to_first_unhandled_marker() -> None:
  ui = object.__new__(SimulatorUI)
  ui.times = (0.0, 0.1, 0.2, 0.3)
  ui.index = 3
  ui.playback_time = 0.3
  ui.paused = False
  ui.events = {1: ("CUT-IN id 10",), 2: ("CUT-IN id 11",)}
  ui.handled_events = set()
  ui.status = ""

  assert ui._pause_for_event(0, 3)
  assert ui.index == 1
  assert ui.playback_time == 0.1
  assert ui.paused
  assert ui.handled_events == {1}
  assert ui.status == "자동 일시정지 @0.10초: CUT-IN id 10"


def test_birds_eye_radar_positive_left_is_drawn_left_of_ego() -> None:
  ui = object.__new__(SimulatorUI)
  rect = SimpleNamespace(x=0.0, y=0.0, width=200.0, height=200.0)

  left_x, _ = ui._screen(rect, 20.0, 2.0)
  center_x, _ = ui._screen(rect, 20.0, 0.0)
  right_x, _ = ui._screen(rect, 20.0, -2.0)

  assert left_x < center_x < right_x


def test_resolve_and_update_validation_case_without_model_arguments(tmp_path) -> None:
  cases = tmp_path / "cases.json"
  cases.write_text(json.dumps({
    "cases": [{
      "id": "case-a",
      "vehicle_folder": "CAR",
      "log": "SEG/rlog.zst",
      "source": "corner",
      "window": [1.0, 2.0],
      "expected": "detect",
      "scene": "test",
    }],
  }), encoding="utf-8")

  path, reviews = resolve_validation_cases(cases, tmp_path, ("case-a",))
  update_validation_case_label(cases, "case-a", "clear")

  assert path == tmp_path / "CAR" / "SEG" / "rlog.zst"
  assert reviews[0].case_id == "case-a"
  assert json.loads(cases.read_text(encoding="utf-8"))["cases"][0] == {
    "id": "case-a",
    "vehicle_folder": "CAR",
    "log": "SEG/rlog.zst",
    "source": "corner",
    "window": [1.0, 2.0],
    "expected": "clear",
    "scene": "test",
    "human_verified": True,
  }


def test_candidate_type_remains_available_to_validation_consumers() -> None:
  candidate = Candidate(10, 0.7, "physical shadow", decision_threshold=0.5)
  assert candidate.eligible
