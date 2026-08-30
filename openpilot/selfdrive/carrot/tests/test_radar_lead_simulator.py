from __future__ import annotations

import json
from dataclasses import replace
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.carrot.radar_motion import model_path_point_at_s
from openpilot.selfdrive.carrot.radar.tools import radar_lead_validation_review
from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import (
  Candidate,
  CurrentRadardSelector,
  ModelLead,
  ProductionDPathSelector,
  RadarFrame,
  RadarMotionShadowSelector,
  RadarOccupancyV2Selector,
  RadarOccupancyV3Selector,
  RadarPoint,
  RecordedLead,
  Selection,
  SimulatorUI,
  candidate_track_id,
  confirmed_cutin_overlap_at,
  corner_radar_display_points,
  frame_value_continuity_segments,
  front_only_frames,
  front_radar_display_points,
  is_position_only_reference,
  lead_continuity_segments,
  lead_speed_continuity_segments,
  lead_one_rgb,
  load_validation_lookahead,
  load_validation_motion_mode,
  load_validation_probability,
  load_validation_sensitivity,
  load_visual_replay_cache,
  motion_points_at_model_time,
  monotonic_log_events,
  preferred_radar_motion_sensor,
  predictor_reference_time_ns,
  radar_trajectory_series,
  resolve_validation_cases,
  save_validation_motion_mode,
  save_validation_probability,
  save_validation_sensitivity,
  save_visual_replay_cache,
  trajectory_history_display_y,
  trajectory_model_review_events,
  update_validation_case_label,
  vision_lead_continuity_segments,
  vision_lead_display_value,
  vision_lead_rgb,
  visual_replay_cache_path,
)
from openpilot.selfdrive.carrot.radar.tools.radar_lead_validation_review import (
  VALIDATION_PRELOAD_AHEAD,
  group_cases_by_log,
  rolling_preload_indexes,
  simulator_environment,
  simulator_command,
)
from openpilot.selfdrive.carrot.radar.tools.validate_radar_lead_model import (
  _candidate_matches_entry,
  _first_role_constraint_event,
  _lead_one_continuous,
  _lead_two_continuous,
  _metrics,
)


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


def test_visual_review_can_cycle_cached_v1_v2_and_v3(tmp_path) -> None:
  frames = [
    frame(
      (point(1010, 8.0, -3.0, source="corner235"),),
      time_s=index * 0.1,
    )
    for index in range(3)
  ]
  v1 = RadarMotionShadowSelector(
    frames,
    motion_sensor="corner",
    enable_radar_tracks=2,
  )
  v2 = RadarOccupancyV2Selector(
    frames,
    baseline=v1,
    enable_radar_tracks=2,
  )
  v3 = RadarOccupancyV3Selector(frames, enable_radar_tracks=2)
  ui = SimulatorUI(
    frames,
    v2,
    "test",
    tmp_path / "rlog.zst",
    settings_path=tmp_path / "radar_validation.json",
    v3_selector=v3,
  )

  assert ui.use_occupancy_v2
  assert ui.selector is v2
  assert v2.baseline is v1
  assert v2.enable_radar_tracks == v1.enable_radar_tracks
  assert v2.cut_in_sensitivity == v1.cut_in_sensitivity

  ui._toggle_occupancy_version()
  assert not ui.use_occupancy_v2
  assert ui.occupancy_version == 3
  assert ui.selector is v3
  assert "V3" in ui.status

  ui._toggle_occupancy_version()
  assert ui.occupancy_version == 1
  assert ui.selector is v1
  assert "V1" in ui.status

  ui._toggle_occupancy_version()
  assert ui.use_occupancy_v2
  assert ui.occupancy_version == 2
  assert ui.selector is v2
  assert "V2" in ui.status


def test_visual_review_starts_with_production_and_cycles_old_versions(
  tmp_path,
) -> None:
  frames = [
    frame(
      (point(1010, 8.0, -3.0, source="corner235"),),
      time_s=index * 0.1,
    )
    for index in range(3)
  ]
  v1 = RadarMotionShadowSelector(
    frames,
    motion_sensor="corner",
    enable_radar_tracks=2,
  )
  v2 = RadarOccupancyV2Selector(
    frames,
    baseline=v1,
    enable_radar_tracks=2,
  )
  v3 = RadarOccupancyV3Selector(frames, enable_radar_tracks=2)
  production = ProductionDPathSelector(
    frames,
    motion_sensor="corner",
    enable_radar_tracks=2,
  )
  ui = SimulatorUI(
    frames,
    v2,
    "test",
    tmp_path / "rlog.zst",
    settings_path=tmp_path / "radar_validation.json",
    v3_selector=v3,
    production_selector=production,
  )

  assert ui.occupancy_version == 4
  assert ui.selector is production
  assert ui.v1_selector is v1
  assert ui.v2_selector is v2

  expected = (
    (1, v1, "이전 V1"),
    (2, v2, "이전 V2"),
    (3, v3, "이전 V3"),
    (4, production, "현재 Trajectory"),
  )
  for version, selector, label in expected:
    ui._toggle_occupancy_version()
    assert ui.occupancy_version == version
    assert ui.selector is selector
    assert label in ui.status

  ui._request_sensitivity(4)
  assert ui.occupancy_version == 4
  assert ui.selector is ui.production_selector
  assert ui.selector.cut_in_sensitivity == 4

  ui._request_motion_mode("front")
  assert ui.occupancy_version == 4
  assert ui.selector is ui.production_selector
  assert ui.selector.motion_sensor == "front"


def test_visual_replay_cache_round_trip_and_exact_configuration(tmp_path) -> None:
  log_path = tmp_path / "rlog.zst"
  log_path.write_bytes(b"test-log-identity")
  frames = [
    frame(
      (point(1010, 8.0, -3.0, source="corner235"),),
      time_s=index * 0.1,
    )
    for index in range(3)
  ]
  v1 = RadarMotionShadowSelector(
    frames,
    motion_sensor="corner",
    enable_radar_tracks=2,
  )
  v2 = RadarOccupancyV2Selector(
    frames,
    baseline=v1,
    enable_radar_tracks=2,
  )
  v3 = RadarOccupancyV3Selector(frames, enable_radar_tracks=2)
  production = ProductionDPathSelector(
    frames,
    motion_sensor="corner",
    enable_radar_tracks=2,
  )
  cache_path = visual_replay_cache_path(
    tmp_path / "cache",
    log_path,
    motion_mode="normal",
    cut_in_sensitivity=3,
    probability_override=None,
    enable_radar_tracks=2,
  )

  save_visual_replay_cache(cache_path, frames, v2, v3, production)
  cached = load_visual_replay_cache(cache_path)

  assert cached is not None
  cached_frames, cached_v2, cached_v3, cached_production = cached
  assert cached_frames == frames
  assert cached_v2.selections == v2.selections
  assert cached_v2.baseline.selections == v1.selections
  assert cached_v3.selections == v3.selections
  assert cached_production.selections == production.selections
  assert visual_replay_cache_path(
    tmp_path / "cache",
    log_path,
    motion_mode="front",
    cut_in_sensitivity=3,
    probability_override=None,
    enable_radar_tracks=2,
  ) != cache_path


def test_visual_replay_cache_ignores_damaged_file(tmp_path) -> None:
  cache_path = tmp_path / "damaged.pickle"
  cache_path.write_bytes(b"not a pickle")

  assert load_visual_replay_cache(cache_path) is None


def test_shadow_selector_rejects_far_corner_only_tunnel_fixture() -> None:
  d_rels = (
    96.65, 96.15, 95.05, 94.50, 94.55, 94.25, 93.60,
    93.60, 92.65, 91.45, 90.80, 89.45, 88.35, 89.60,
    89.00, 88.90, 88.60, 88.00, 89.15, 88.65,
  )
  v_rels = (
    -8.85, -8.85, -9.85, -9.85, -11.10, -11.10, -11.10,
    -11.10, -10.75, -11.50, -11.50, -12.90, -12.95, -10.95,
    -10.95, -10.60, -10.60, -10.60, -8.90, -8.80,
  )
  frames = []
  for index, (d_rel, v_rel) in enumerate(zip(d_rels, v_rels, strict=True)):
    time_s = index * 0.05
    current = frame((
      replace(
        point(
          2809,
          d_rel,
          -0.55,
          source="corner235",
        ),
        v_rel=v_rel,
        v_lead=29.6 + v_rel,
      ),
    ), time_s=time_s)
    frames.append(replace(
      current,
      v_ego=29.6,
      model_leads=(
        ModelLead(
          0.02, 119.52, -0.2, 27.5, 0.0, 1.0, 0.5, 1.0,
        ),
      ),
    ))

  selector = RadarMotionShadowSelector(
    frames,
    enable_radar_tracks=1,
  )

  assert all(
    selector.select(current, index).lead_one is None
    for index, current in enumerate(frames)
  )


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


def test_shadow_selector_replays_option_two_scc_lead_two() -> None:
  frames = []
  for index in range(5):
    current = frame((
      replace(
        point(0, 105.0, 0.0, source="scc"),
        v_rel=-15.5,
        v_lead=4.5,
      ),
      replace(
        point(45, 103.5, 8.0),
        v_rel=-15.6,
        v_lead=4.4,
      ),
      replace(
        point(1618, 104.0, 7.5, source="corner235"),
        v_rel=-14.0,
        v_lead=6.0,
      ),
    ), time_s=index * 0.05)
    frames.append(replace(
      current,
      path=((0.0, 0.0), (120.0, -9.6)),
      model_leads=(
        ModelLead(0.20, 111.52, -8.0, 4.5, 0.0, 8.0, 1.5, 2.0),
      ),
    ))

  option_one = RadarMotionShadowSelector(
    frames,
    enable_radar_tracks=1,
  )
  option_two = RadarMotionShadowSelector(
    frames,
    enable_radar_tracks=2,
  )

  assert option_one.selections[-1].lead_two is None
  assert candidate_track_id(option_two.selections[-1].lead_two) == 45


def test_shadow_selector_preserves_lane_boundary_directional_entry() -> None:
  frames = [
    frame((
      point(10, 30.0, 0.0),
      replace(
        point(1010, 20.0, y_rel, source="corner235"),
        v_rel=-3.5,
        yv_rel=-0.5,
        v_lead=6.5,
      ),
    ), time_s=index * 0.1)
    for index, y_rel in enumerate((
      3.00, 2.95, 2.90, 2.85, 2.80, 2.75,
      2.70, 2.65, 2.60, 2.55, 2.50, 2.45,
    ))
  ]

  selector = RadarMotionShadowSelector(frames, motion_sensor="corner")

  assert selector.selections[-1].decision_cutin_candidates
  assert (
    selector.selections[-1].decision_cutin_candidates[0].track_id
    == 1010
  )


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


def test_predecel_and_confirmed_cutin_are_distinct_review_events() -> None:
  frames = [frame((), time_s=index * 0.1) for index in range(4)]
  predecel = Candidate(
    2091,
    0.87,
    "corner CUT-IN pre-deceleration risk",
    source="corner235",
    stage="PRE-DECEL",
  )
  cutin = Candidate(
    2091,
    0.79,
    "physical corner dPath shadow",
    source="corner235",
  )
  selections = (
    Selection(None, None, cutin_predecel_candidate=predecel),
    Selection(None, None, cutin_predecel_candidate=predecel),
    Selection(None, None, decision_cutin_candidates=(cutin,)),
    Selection(None, None, decision_cutin_candidates=(cutin,)),
  )
  selector = SimpleNamespace(
    trajectories=tuple(
      {
        ("corner235", 2091): SimpleNamespace(continuity_id=21),
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

  assert tuple(events) == (0, 2)
  assert events[0] == ("예비감속 위험 corner id 2091 위험도 0.87",)
  assert events[2] == ("물리 예측 CUT-IN corner id 2091 진입 0.79 이탈 0.00",)


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


def test_log_events_are_replayed_in_monotonic_timestamp_order() -> None:
  events = tuple(
    SimpleNamespace(logMonoTime=value)
    for value in (300, 100, 200, 200)
  )

  ordered = monotonic_log_events(events)

  assert [event.logMonoTime for event in ordered] == [100, 200, 200, 300]


def test_predictor_uses_model_exposure_timestamp_with_event_fallback() -> None:
  assert predictor_reference_time_ns(300, 200) == 200
  assert predictor_reference_time_ns(300, 0) == 300


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


def test_validation_role_constraint_finds_forbidden_lead_one() -> None:
  frames = [frame((), time_s=1.0, one=recorded(99, True))]
  selector = CurrentRadardSelector(frames, [set()])

  event = _first_role_constraint_event(
    selector,
    frames,
    {"window": [0.5, 1.5]},
    "lead_one",
    [99],
  )

  assert event == (1.0, 99)


def test_validation_spatial_target_survives_raw_track_id_changes() -> None:
  entry = {
    "target_track_ids": [1013],
    "target_spatial_match": {
      "d_rel": [3.5, 6.0],
      "y_rel": [-3.0, -2.0],
    },
  }

  assert _candidate_matches_entry(
    Candidate(202, 0.8, "physical", d_rel=4.8, y_rel=-2.5),
    entry,
  )
  assert not _candidate_matches_entry(
    Candidate(202, 0.8, "other", d_rel=12.0, y_rel=2.5),
    entry,
  )


def test_validation_requires_physical_lead_two_through_continuity_window() -> None:
  frames = [frame((), time_s=index * 0.1) for index in range(4)]
  matching = Candidate(
    202, 0.8, "physical", d_rel=4.8, y_rel=-2.5,
  )
  selector = SimpleNamespace(selections=(
    Selection(None, matching),
    Selection(None, replace(matching, track_id=203)),
    Selection(None, replace(matching, track_id=204)),
    Selection(None, matching),
  ))
  selector.select = lambda _frame, index: selector.selections[index]
  entry = {
    "lead_two_continuous_window": [0.0, 0.3],
    "target_track_ids": [1013],
    "target_spatial_match": {
      "d_rel": [3.5, 6.0],
      "y_rel": [-3.0, -2.0],
    },
  }

  assert _lead_two_continuous(selector, frames, entry)
  selector.selections = (
    *selector.selections[:2],
    Selection(None, None),
    selector.selections[3],
  )
  assert not _lead_two_continuous(selector, frames, entry)


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
  assert command[command.index("--enable-radar-tracks") + 1] == "2"


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


def test_validation_runner_can_preload_and_consume_private_cache(tmp_path) -> None:
  command = simulator_command(
    [{"id": "case-a"}],
    tmp_path,
    tmp_path / "cases.json",
    None,
    "2/40",
    False,
    cache_dir=tmp_path / "cache",
    preload_only=True,
    consume_cache=True,
  )

  assert command[command.index("--cache-dir") + 1] == str(tmp_path / "cache")
  assert "--preload-only" in command
  assert "--consume-cache" in command


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


def test_validation_runner_keeps_five_log_rolling_preload() -> None:
  route_available = [True] * 10
  scheduled: set[int] = set()
  started: list[int] = []

  for current_index in range(len(route_available)):
    scheduled.discard(current_index)
    new_indexes = rolling_preload_indexes(
      current_index,
      route_available,
      scheduled,
    )
    if current_index == 0:
      assert new_indexes == [1, 2, 3, 4, 5]
    elif current_index <= 4:
      assert new_indexes == [current_index + VALIDATION_PRELOAD_AHEAD]
    scheduled.update(new_indexes)
    started.extend(new_indexes)
    expected_ready = min(
      VALIDATION_PRELOAD_AHEAD,
      len(route_available) - current_index - 1,
    )
    assert len(scheduled) == expected_ready

  assert started == list(range(1, len(route_available)))


def test_validation_runner_rolling_preload_skips_missing_logs() -> None:
  route_available = [True, True, False, True, True, False, True, True]

  assert rolling_preload_indexes(
    0,
    route_available,
    set(),
  ) == [1, 3, 4, 6, 7]
  assert rolling_preload_indexes(
    1,
    route_available,
    {3, 4, 6, 7},
  ) == []


def test_validation_runner_replenishes_five_log_buffer_to_end(
  tmp_path,
  monkeypatch,
) -> None:
  cases = []
  for index in range(8):
    relative_log = f"segment-{index}/rlog.zst"
    route = tmp_path / "CAR" / relative_log
    route.parent.mkdir(parents=True, exist_ok=True)
    route.write_bytes(b"log")
    cases.append({
      "id": f"case-{index}",
      "vehicle_folder": "CAR",
      "log": relative_log,
      "expected": "detect",
    })
  cases_path = tmp_path / "cases.json"
  cases_path.write_text(json.dumps({"cases": cases}), encoding="utf-8")
  args = SimpleNamespace(
    root=tmp_path,
    cases=cases_path,
    case=[],
    expected="all",
    prob=None,
    sensitivity=3,
    enable_radar_tracks=2,
    front_only=False,
    motion_mode="normal",
    list=False,
  )
  preload_commands: list[list[str]] = []
  preload_schema_dirs: list[str] = []
  foreground_commands: list[list[str]] = []
  foreground_schema_dirs: list[str] = []

  class FakePreload:
    def __init__(self, command, **_kwargs) -> None:
      preload_commands.append(command)
      preload_schema_dirs.append(
        _kwargs["env"][radar_lead_validation_review.ROUTE_SCHEMA_CACHE_ENV],
      )

    def poll(self) -> int:
      return 0

    def wait(self, timeout=None) -> int:
      del timeout
      return 0

    def terminate(self) -> None:
      raise AssertionError("completed preload must not be terminated")

    def kill(self) -> None:
      raise AssertionError("completed preload must not be killed")

  def fake_run(command, *, check, env):
    assert not check
    foreground_commands.append(command)
    foreground_schema_dirs.append(
      env[radar_lead_validation_review.ROUTE_SCHEMA_CACHE_ENV],
    )
    return SimpleNamespace(
      returncode=9 if len(foreground_commands) == 1 else 0,
    )

  monkeypatch.setattr(radar_lead_validation_review, "parse_args", lambda: args)
  monkeypatch.setattr(
    radar_lead_validation_review.subprocess,
    "Popen",
    FakePreload,
  )
  monkeypatch.setattr(
    radar_lead_validation_review.subprocess,
    "run",
    fake_run,
  )

  assert radar_lead_validation_review.main() == 1
  preload_ids = [
    command[command.index("--validation-case") + 1]
    for command in preload_commands
  ]
  foreground_ids = [
    command[command.index("--validation-case") + 1]
    for command in foreground_commands
  ]
  assert preload_ids[:5] == [f"case-{index}" for index in range(1, 6)]
  assert preload_ids == [f"case-{index}" for index in range(1, 8)]
  assert foreground_ids == [f"case-{index}" for index in range(8)]
  assert len(set(preload_schema_dirs)) == 7
  assert len(set(foreground_schema_dirs)) == 8


def test_validation_runner_gives_each_loader_a_private_schema_dir(
  tmp_path,
) -> None:
  first = simulator_environment(tmp_path, 1)
  second = simulator_environment(tmp_path, 2)

  key = radar_lead_validation_review.ROUTE_SCHEMA_CACHE_ENV
  assert first[key] == str(tmp_path / "schema" / "log-001")
  assert second[key] == str(tmp_path / "schema" / "log-002")
  assert first[key] != second[key]


def test_lead_speed_continuity_converts_vlead_to_kph() -> None:
  frames = [frame((), time_s=index * 0.1) for index in range(3)]
  selections = (
    Selection(Candidate(10, 1.0, "L1", v_lead=20.0), None),
    Selection(Candidate(10, 1.0, "L1", v_lead=19.5), None),
    Selection(None, None),
  )

  segments = lead_speed_continuity_segments(frames, selections)

  assert len(segments) == 1
  assert segments[0][0] == (0.0, 72.0, 10)
  assert segments[0][1][1] == pytest.approx(70.2)


def test_frame_value_segments_break_when_scc_object_is_not_detected() -> None:
  frames = (
    replace(frame((), time_s=0.0), scc_distance_m=42.0),
    replace(frame((), time_s=0.1), scc_distance_m=40.0),
    replace(frame((), time_s=0.2), scc_distance_m=None),
    replace(frame((), time_s=0.3), scc_distance_m=35.0),
  )

  segments = frame_value_continuity_segments(frames, "scc_distance_m")

  assert segments == (((0.0, 42.0), (0.1, 40.0)), ((0.3, 35.0),))


def test_frame_value_segments_preserve_zero_scc_acceleration() -> None:
  frames = (
    replace(frame((), time_s=0.0), scc_a_req_raw=0.0),
    replace(frame((), time_s=0.1), scc_a_req_raw=0.0),
    replace(frame((), time_s=0.2), scc_a_req_raw=-1.0),
  )

  segments = frame_value_continuity_segments(frames, "scc_a_req_raw")

  assert segments == (((0.0, 0.0), (0.1, 0.0), (0.2, -1.0)),)


def test_validation_lead_one_continuity_rejects_a_single_missing_frame() -> None:
  frames = [frame((), time_s=index * 0.1) for index in range(4)]
  lead = Candidate(-1, 1.0, "vision L1", d_rel=30.0)
  selections = (
    Selection(lead, None),
    Selection(lead, None),
    Selection(None, None),
    Selection(lead, None),
  )
  selector = SimpleNamespace(
    select=lambda _frame, index: selections[index],
  )
  entry = {"lead_one_continuous_window": [0.0, 0.3]}

  assert not _lead_one_continuous(selector, frames, entry)
  assert _lead_one_continuous(
    selector,
    frames,
    {"lead_one_continuous_window": [0.0, 0.1]},
  )


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


def test_vision_lead_graph_shows_point_two_and_splits_weak_color() -> None:
  probabilities = (0.20, 0.19, 0.39, 0.40, 0.80)
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

  assert len(segments) == 3
  assert segments[0][0] == pytest.approx((0.0, 30.0, 0.20))
  assert segments[1][0] == pytest.approx((0.2, 32.0, 0.39))
  assert segments[2][0] == pytest.approx((0.3, 33.0, 0.40))
  assert vision_lead_rgb(0.39) != vision_lead_rgb(0.40)


def test_vision_lead_legend_keeps_subthreshold_probability_visible() -> None:
  weak = ModelLead(0.23, 81.52, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0)
  faint = replace(weak, probability=0.06)

  weak_text, weak_rgb, weak_distance = vision_lead_display_value(weak)
  faint_text, faint_rgb, faint_distance = vision_lead_display_value(faint)

  assert weak_text == "80.0m p0.23"
  assert weak_rgb == vision_lead_rgb(0.23)
  assert weak_distance == pytest.approx(80.0)
  assert faint_text == "-- p0.06"
  assert faint_rgb != weak_rgb
  assert faint_distance is None


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


def test_lead_continuity_joins_moving_raw_id_handoff_by_continuity_id() -> None:
  frames = [frame((), time_s=index * 0.1) for index in range(3)]
  selections = (
    Selection(
      None,
      Candidate(
        202, 1.0, "L2", d_rel=5.8, y_rel=-2.5, v_lead=6.3,
        source="corner235", continuity_id=40,
      ),
    ),
    Selection(
      None,
      Candidate(
        203, 1.0, "L2", d_rel=5.9, y_rel=-2.4, v_lead=6.2,
        source="corner235", continuity_id=40,
      ),
    ),
    Selection(
      None,
      Candidate(
        204, 1.0, "L2", d_rel=6.0, y_rel=-2.3, v_lead=6.1,
        source="corner235", continuity_id=41,
      ),
    ),
  )

  segments = lead_continuity_segments(frames, selections, "lead_two")

  assert [[point[2] for point in segment] for segment in segments] == [
    [202, 203],
    [204],
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
  assert continuity.height == pytest.approx(356.4)
  assert continuity.y == pytest.approx(634.6)
  assert video.height == pytest.approx(307.3)
  assert radar_map.height == pytest.approx(299.3)
  assert radar_map.height >= 150.0
  assert panel.y + panel.height == pytest.approx(626.6)
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
