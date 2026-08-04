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
  Selection,
  SimulatorUI,
  candidate_track_id,
  confirmed_cutin_overlap_at,
  corner_radar_display_points,
  front_only_frames,
  front_radar_display_points,
  is_position_only_reference,
  lead_continuity_segments,
  lead_one_rgb,
  load_validation_lookahead,
  load_validation_motion_mode,
  load_validation_probability,
  load_validation_sensitivity,
  motion_points_at_model_time,
  preferred_radar_motion_sensor,
  radar_trajectory_series,
  resolve_validation_cases,
  save_validation_motion_mode,
  save_validation_probability,
  save_validation_sensitivity,
  trajectory_history_display_y,
  trajectory_model_review_events,
  update_validation_case_label,
  vision_lead_continuity_segments,
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


def test_shadow_selector_uses_new_controller_not_recorded_radard_roles() -> None:
  frames = [
    frame(
      (
        point(10, 30.0, 0.0),
        point(1010, 25.0, 4.0 - index * 0.4, source="corner235"),
      ),
      time_s=index * 0.1,
      one=recorded(99, True),
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
  assert candidate_track_id(radard_selection.lead_one) == 99
  assert candidate_track_id(radard_selection.lead_two) == 1010
  assert candidate_track_id(selection.lead_one) == 10
  assert selection.lead_two is None
  assert selection.active_cutin_candidates == ()
  assert any(candidate.track_id == 1010 for candidate in selection.cutin_diagnostics)


def test_new_controller_publishes_confirmed_physical_lead_two() -> None:
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

  assert candidate_track_id(selection.lead_two) == 1010
  assert selection.active_cutin_candidates == ()
  assert selection.decision_cutin_candidates


def test_review_event_is_emitted_once_per_physical_continuity() -> None:
  frames = [frame((), time_s=index * 0.1) for index in range(3)]
  candidate = Candidate(
    1019,
    1.0,
    "physical corner dPath shadow",
    source="corner235",
  )
  selections = (
    Selection(None, None, decision_cutin_candidates=(candidate,)),
    Selection(None, None),
    Selection(None, None, decision_cutin_candidates=(candidate,)),
  )
  selector = SimpleNamespace(
    trajectories=tuple(
      {
        ("corner235", 1019): SimpleNamespace(continuity_id=15),
      }
      for _ in frames
    ),
    select=lambda _frame, index: selections[index],
  )

  events = trajectory_model_review_events(
    frames,
    selector,
    ("front+corner",),
    0.5,
  )

  assert tuple(events) == (0,)


def test_validation_threshold_is_passed_to_physical_decision_tracker() -> None:
  selector = RadarMotionShadowSelector(
    [frame((point(10, 30.0, 0.0),))],
    decision_threshold=0.45,
  )

  assert selector.decision_threshold == pytest.approx(0.45)


def test_validation_threshold_reuses_cached_physical_trajectories() -> None:
  frames = [
    frame(
      (point(1010, 25.0, 4.0 - index * 0.1, source="corner235"),),
      time_s=index * 0.1,
    )
    for index in range(5)
  ]
  original = RadarMotionShadowSelector(frames, decision_threshold=0.50)
  adjusted = RadarMotionShadowSelector(
    frames,
    decision_threshold=0.42,
    motion_points=original.motion_points,
    trajectories=original.trajectories,
    lead_one_outputs=original.lead_one_outputs,
  )

  assert adjusted.motion_points is original.motion_points
  assert adjusted.trajectories is original.trajectories
  assert adjusted.lead_one_outputs is original.lead_one_outputs
  assert adjusted.decision_threshold == pytest.approx(0.42)


def test_cutin_confirmation_survives_out_to_in_path_transition() -> None:
  frames = [
    frame(
      (
        point(10, 30.0, 0.0),
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


def test_front_radar_display_toggle_uses_only_measured_front_points() -> None:
  current = frame((
    point(10, 30.0, 0.2),
      point(11, -9.0, 0.2),
      point(12, 121.0, 0.2),
      point(14, 131.0, 0.2),
      point(13, 20.0, 0.2, measured=False),
    point(1010, 20.0, 2.0, source="corner235"),
    point(0, 20.0, 0.0, source="scc"),
  ))

  assert [
    value.track_id for value in front_radar_display_points(current)
  ] == [10, 11, 12]


def test_corner_radar_display_includes_all_measured_points_in_range() -> None:
  current = frame((
    point(1001, 20.0, 0.2, source="corner235"),
      point(1002, 85.0, 1.0, source="corner235"),
      point(1003, 121.0, 0.2, source="corner235"),
      point(1005, 131.0, 0.2, source="corner235"),
    point(
      1004, 30.0, 0.2,
      source="corner235", measured=False,
    ),
    point(10, 20.0, 0.0, source="frontRadar"),
  ))

  assert [
    value.track_id for value in corner_radar_display_points(current)
  ] == [1001, 1002, 1003]


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


def test_front_motion_mode_ignores_corner_points_even_when_present() -> None:
  frames = [
    frame((
      point(10, 30.0, 1.0),
      point(1010, 29.0, 2.8, source="corner235"),
    ), time_s=index * 0.1)
    for index in range(2)
  ]

  selector = RadarMotionShadowSelector(
    frames,
    motion_sensor="front",
  )

  assert selector.motion_sensor == "front"
  assert set(selector.trajectories[-1]) == {("frontRadar", 10)}


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


def test_corner_measurement_delay_is_added_to_timestamp_alignment() -> None:
  current = replace(
    frame((
      replace(
        point(1010, 30.0, 3.0, source="corner235"),
        v_rel=-2.0,
        yv_rel=-4.0,
      ),
    )),
    input_age_s=0.0,
    model_age_s=0.076,
  )

  aligned = motion_points_at_model_time(current, "corner")

  # Camera is 76 ms older, while the corner object itself represents a
  # measurement one 50 ms radar cycle before liveTracks publication.
  assert aligned[0].d_rel == pytest.approx(30.052)
  assert aligned[0].y_rel == pytest.approx(3.104)


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


def test_validation_runner_uses_saved_probability_when_not_overridden(tmp_path) -> None:
  command = simulator_command(
    [{"id": "case-a"}],
    tmp_path,
    tmp_path / "cases.json",
    None,
    "1/1",
    False,
  )

  assert "--prob" not in command


def test_validation_runner_forwards_device_sensitivity_override(tmp_path) -> None:
  command = simulator_command(
    [{"id": "case-a"}],
    tmp_path,
    tmp_path / "cases.json",
    None,
    "1/1",
    False,
    sensitivity=4,
  )

  assert command[command.index("--sensitivity") + 1] == "4"
  assert "--lookahead-s" not in command


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


def test_manual_seek_rearms_future_predictor_pauses() -> None:
  ui = object.__new__(SimulatorUI)
  ui.times = (0.0, 0.1, 0.2, 0.3)
  ui.frames = (None,) * 4
  ui.index = 3
  ui.playback_time = 0.3
  ui.paused = False
  ui.status = ""
  ui.handled_events = {1, 2, 3}
  axis = SimpleNamespace(x=0.0, width=100.0)

  ui._seek_from_time_axis(axis, 25.0, "seek bar")

  assert ui.index == 1
  assert ui.handled_events == set()
  assert "자동정지 재설정됨" in ui.status


def test_birds_eye_radar_positive_left_is_drawn_left_of_ego() -> None:
  ui = object.__new__(SimulatorUI)
  rect = SimpleNamespace(x=0.0, y=0.0, width=200.0, height=200.0)

  left_x, _ = ui._screen(rect, 20.0, 2.0)
  center_x, _ = ui._screen(rect, 20.0, 0.0)
  right_x, _ = ui._screen(rect, 20.0, -2.0)

  assert left_x < center_x < right_x


def test_birds_eye_distance_axis_covers_minus_30_to_130m() -> None:
  ui = object.__new__(SimulatorUI)
  rect = SimpleNamespace(x=0.0, y=0.0, width=200.0, height=200.0)

  _, top_y = ui._screen(rect, 130.0, 0.0)
  _, ego_y = ui._screen(rect, 0.0, 0.0)
  _, bottom_y = ui._screen(rect, -30.0, 0.0)

  assert top_y == pytest.approx(72.0)
  assert top_y < ego_y < bottom_y
  assert bottom_y == pytest.approx(182.0)


def test_lead_continuity_breaks_on_missing_frames_and_track_id_changes() -> None:
  frames = [frame((), time_s=index * 0.1) for index in range(5)]
  selections = (
    Selection(Candidate(10, 1.0, "L1", d_rel=30.0), None),
    Selection(Candidate(10, 1.0, "L1", d_rel=29.5), None),
    Selection(None, None),
    Selection(Candidate(10, 1.0, "L1", d_rel=28.5), None),
    Selection(Candidate(11, 1.0, "L1", d_rel=28.0), None),
  )

  segments = lead_continuity_segments(frames, selections, "lead_one")

  assert [[point[2] for point in segment] for segment in segments] == [
    [10, 10],
    [10],
    [11],
  ]


def test_vision_only_lead_one_uses_blue_instead_of_radar_orange() -> None:
  assert lead_one_rgb(-1) == (72, 145, 255)
  assert lead_one_rgb(56) == (246, 142, 55)
  assert lead_one_rgb(None) == (246, 142, 55)


def test_lead_continuity_splits_vision_and_radar_color_segments() -> None:
  frames = [frame((), time_s=index * 0.1) for index in range(2)]
  selections = (
    Selection(
      Candidate(
        -1, 1.0, "vision L1",
        d_rel=30.0, y_rel=0.1, v_lead=0.0,
      ),
      None,
    ),
    Selection(
      Candidate(
        41, 1.0, "radar L1",
        d_rel=29.5, y_rel=0.2, v_lead=0.1,
      ),
      None,
    ),
  )

  segments = lead_continuity_segments(frames, selections, "lead_one")

  assert [[point[2] for point in segment] for segment in segments] == [
    [-1],
    [41],
  ]


def test_vision_lead_graph_uses_point_four_probability_threshold() -> None:
  probabilities = (0.40, 0.39, 0.80)
  frames = [
    replace(
      frame((), time_s=index * 0.1),
      model_leads=(ModelLead(
        probability,
        31.52 + index,
        0.0,
        20.0,
        0.0,
        1.0,
        0.5,
        1.0,
      ),),
    )
    for index, probability in enumerate(probabilities)
  ]

  segments = vision_lead_continuity_segments(frames)

  assert len(segments) == 2
  assert segments[0][0] == pytest.approx((0.0, 30.0, 0.40))
  assert segments[1][0] == pytest.approx((0.2, 32.0, 0.80))


def test_lead_continuity_joins_physical_stationary_track_handoff() -> None:
  frames = [frame((), time_s=index * 0.1) for index in range(3)]
  selections = (
    Selection(
      Candidate(
        41, 1.0, "L1", d_rel=30.0, y_rel=0.1, v_lead=0.0,
      ),
      None,
    ),
    Selection(
      Candidate(
        48, 1.0, "L1", d_rel=29.5, y_rel=0.2, v_lead=0.1,
      ),
      None,
    ),
    Selection(
      Candidate(
        45, 1.0, "L1", d_rel=28.9, y_rel=0.1, v_lead=-0.1,
      ),
      None,
    ),
  )

  segments = lead_continuity_segments(frames, selections, "lead_one")

  assert [[point[2] for point in segment] for segment in segments] == [
    [41, 48, 45],
  ]


def test_lead_graph_and_seek_bar_share_one_time_axis() -> None:
  ui = object.__new__(SimulatorUI)
  ui.rl = SimpleNamespace(
    Rectangle=lambda x, y, width, height: SimpleNamespace(
      x=x,
      y=y,
      width=width,
      height=height,
    ),
  )

  timeline, panel, video, radar_map, continuity = ui._layout_rects(1440, 1080)
  continuity_axis = ui._continuity_time_axis_rect(continuity)

  assert timeline.x == pytest.approx(continuity_axis.x)
  assert timeline.width == pytest.approx(continuity_axis.width)
  assert timeline.width == pytest.approx(1360.0)
  assert continuity.width == pytest.approx(1416.0)
  assert continuity.height == pytest.approx(290.0)
  assert continuity.y == pytest.approx(701.0)
  assert video.height == pytest.approx(340.5)
  assert radar_map.height == pytest.approx(332.5)
  assert radar_map.height > continuity.height
  assert panel.y + panel.height == pytest.approx(693.0)
  assert panel.y + panel.height < continuity.y


def test_clicking_lead_graph_seeks_on_shared_time_axis() -> None:
  ui = object.__new__(SimulatorUI)
  ui.times = (0.0, 10.0, 20.0)
  ui.frames = (None, None, None)
  ui.index = 0
  ui.playback_time = 0.0
  ui.paused = False
  ui.status = ""
  axis = SimpleNamespace(x=50.0, width=800.0)

  ui._seek_from_time_axis(axis, 650.0, "L1/L2 그래프")

  assert ui.playback_time == pytest.approx(15.0)
  assert ui.paused
  assert ui.status.startswith("L1/L2 그래프 탐색 @15.00초")


def test_validation_probability_is_saved_outside_the_repository(tmp_path) -> None:
  settings = tmp_path / "radar_validation.json"

  assert load_validation_probability(
    settings, sensor="corner",
  ) == pytest.approx(0.30)
  assert load_validation_probability(
    settings, sensor="front",
  ) == pytest.approx(0.67)
  save_validation_probability(0.45, settings, sensor="corner")
  save_validation_probability(0.70, settings, sensor="front")
  save_validation_motion_mode("front", settings)

  assert load_validation_probability(
    settings, sensor="corner",
  ) == pytest.approx(0.45)
  assert load_validation_probability(
    settings, sensor="front",
  ) == pytest.approx(0.70)
  assert load_validation_motion_mode(settings) == "front"
  assert json.loads(settings.read_text(encoding="utf-8")) == {
    "corner_probability": 0.45,
    "front_probability": 0.7,
    "motion_mode": "front",
  }


def test_validation_sensitivity_is_saved_outside_repository(
  tmp_path,
) -> None:
  settings = tmp_path / "radar_validation.json"

  assert load_validation_sensitivity(settings) == 3
  settings.write_text(json.dumps({
    "corner_lookahead_s": 4.5,
    "front_lookahead_s": 3.5,
  }), encoding="utf-8")

  save_validation_sensitivity(4, settings)

  assert load_validation_sensitivity(settings) == 4
  assert load_validation_lookahead(
    settings, sensor="corner",
  ) == pytest.approx(5.0)
  assert load_validation_lookahead(
    settings, sensor="front",
  ) == pytest.approx(5.0)
  assert json.loads(settings.read_text(encoding="utf-8")) == {
    "cut_in_sensitivity": 4,
  }


def test_validation_corner_sensitivity_applies_from_cached_physical_history(
  tmp_path,
) -> None:
  frames = [
    frame(
      (point(1010, 25.0, 4.0 - index * 0.1, source="corner235"),),
      time_s=index * 0.1,
    )
    for index in range(5)
  ]
  selector = RadarMotionShadowSelector(
    frames,
    cut_in_sensitivity=3,
    motion_sensor="corner",
  )
  ui = SimulatorUI(
    frames,
    selector,
    "test",
    tmp_path / "rlog.zst",
    cut_in_sensitivity=3,
    settings_path=tmp_path / "radar_validation.json",
  )
  lead_one_outputs = selector.lead_one_outputs
  trajectories = selector.trajectories
  ui._request_sensitivity(4)

  assert ui.selector.cut_in_sensitivity == 4
  assert ui.selector.maximum_lookahead_s == pytest.approx(5.0)
  assert ui.selector.lead_one_outputs is lead_one_outputs
  assert ui.selector.trajectories is not trajectories
  assert ui.selector.motion_sensitivity.confirmation_s == pytest.approx(0.25)
  assert ui.status.startswith("CUT-IN 감도 4 민감")
  assert load_validation_sensitivity(
    tmp_path / "radar_validation.json",
  ) == 4


def test_confirmed_cutin_colors_complete_continuous_overlap() -> None:
  prediction = SimpleNamespace(
    current_path_occupancy=False,
    predicted_path_overlap_start_s=1.5,
    predicted_path_overlap_s=3.5,
  )

  assert not confirmed_cutin_overlap_at(prediction, 1.0)
  assert confirmed_cutin_overlap_at(prediction, 1.5)
  assert confirmed_cutin_overlap_at(prediction, 4.0)
  assert confirmed_cutin_overlap_at(prediction, 5.0)


def test_validation_mode_toggle_uses_same_sensitivity_and_fixed_horizon(
  tmp_path,
) -> None:
  frames = [
    frame((
      point(10, 30.0, 1.0),
      point(1010, 29.0, 2.8, source="corner235"),
    ), time_s=index * 0.1)
    for index in range(2)
  ]
  settings = tmp_path / "radar_validation.json"
  selector = RadarMotionShadowSelector(
    frames,
    cut_in_sensitivity=3,
    motion_sensor="corner",
  )
  ui = SimulatorUI(
    frames,
    selector,
    "test",
    tmp_path / "rlog.zst",
    cut_in_sensitivity=3,
    settings_path=settings,
  )

  ui._request_motion_mode("front")

  assert ui.motion_mode == "front"
  assert ui.selector.motion_sensor == "front"
  assert ui.selector.cut_in_sensitivity == 3
  assert ui.selector.decision_threshold == pytest.approx(0.67)
  assert ui.selector.maximum_lookahead_s == pytest.approx(5.0)
  assert load_validation_motion_mode(settings) == "front"


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
