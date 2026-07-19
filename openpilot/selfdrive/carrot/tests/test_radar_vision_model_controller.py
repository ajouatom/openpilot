from dataclasses import replace
from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar_lead_model import RadarLeadDecision, VisionLeadContext
from openpilot.selfdrive.carrot.radar_lead_runtime import RadarLeadRuntimeResult
from openpilot.selfdrive.carrot.radar_lead_simulator import (
  Candidate,
  Selection,
  cutin_continuity_series,
  validation_review_events,
)
from openpilot.selfdrive.carrot.radar_vision_model_controller import VisionModelRadarController, VisionRadarMatcher
from openpilot.selfdrive.carrot.tests.test_radar_lead_controller import prediction


def vision_model(d_rel: float, y_rel: float, v_lead: float, probability: float = 0.95):
  return SimpleNamespace(leadsV3=[SimpleNamespace(
    prob=probability,
    x=[d_rel + 1.52],
    y=[-y_rel],
    v=[v_lead],
    xStd=[1.0],
    yStd=[0.5],
    vStd=[1.0],
  )])


def test_laplacian_match_requires_all_sanity_gates() -> None:
  matcher = VisionRadarMatcher()
  sane = prediction(40, 0.2, 0.1, 0.1)
  vision = VisionLeadContext(0.95, 12.0, 0.2, 19.0, 0.0, 1.0, 0.5, 1.0)
  assert matcher.match_context(vision, (sane,), 20.0) is not None

  bad_distance = VisionLeadContext(0.95, 35.0, 0.2, 19.0, 0.0, 1.0, 0.5, 1.0)
  assert matcher.match_context(bad_distance, (sane,), 20.0) is None

  bad_lateral = VisionLeadContext(0.95, 12.0, 3.0, 19.0, 0.0, 1.0, 0.5, 1.0)
  assert matcher.match_context(bad_lateral, (sane,), 20.0) is None

  bad_velocity = VisionLeadContext(0.95, 12.0, 0.2, 10.0, 0.0, 1.0, 0.5, 1.0)
  assert matcher.match_context(bad_velocity, (sane,), 40.0) is None

  low_probability = VisionLeadContext(0.34, 12.0, 0.2, 19.0, 0.0, 1.0, 0.5, 1.0)
  assert matcher.match_context(low_probability, (sane,), 20.0) is None


def test_low_probability_hold_keeps_only_previous_sane_radar_alias() -> None:
  matcher = VisionRadarMatcher()
  selected = prediction(40, 0.2, 0.1, 0.1)
  other = prediction(41, 0.2, 0.1, 0.1)
  high = VisionLeadContext(0.95, 12.0, 0.2, 19.0, 0.0, 1.0, 0.5, 1.0)
  low = VisionLeadContext(0.43, 12.0, 0.2, 19.0, 0.0, 1.0, 0.5, 1.0)

  first = matcher.match_context(high, (selected, other), 20.0)
  assert first is not None and first.prediction.features.radar_object.front_track_id == 40
  held = matcher.match_context(low, (selected, other), 20.0)
  assert held is not None and held.prediction.features.radar_object.front_track_id == 40

  for _ in range(9):
    assert matcher.match_context(low, (selected, other), 20.0) is not None
  assert matcher.match_context(low, (selected, other), 20.0) is None


def test_previous_match_bridges_small_distance_gate_jitter() -> None:
  selected = prediction(40, 0.2, 0.1, 0.1)
  initial = VisionLeadContext(0.95, 12.0, 0.2, 19.0, 0.0, 1.0, 0.5, 1.0)
  boundary_jitter = VisionLeadContext(0.95, 17.5, 0.2, 19.0, 0.0, 1.0, 0.5, 1.0)

  fresh_matcher = VisionRadarMatcher()
  assert fresh_matcher.match_context(boundary_jitter, (selected,), 20.0) is None

  held_matcher = VisionRadarMatcher()
  assert held_matcher.match_context(initial, (selected,), 20.0) is not None
  held = held_matcher.match_context(boundary_jitter, (selected,), 20.0)
  assert held is not None and held.prediction.features.radar_object.front_track_id == 40


def test_high_probability_vision_can_replace_farther_previous_match() -> None:
  matcher = VisionRadarMatcher()
  farther = prediction(43, 0.41, 0.1, 0.1, d_rel=29.1, v_lead=9.2)
  closer = prediction(34, -0.07, 0.1, 0.1, d_rel=19.7, v_lead=8.5)
  farther = replace(farther, features=replace(
    farther.features,
    radar_object=replace(farther.features.radar_object, front_d_rel=29.1, front_v_rel=-0.9),
  ))
  closer = replace(closer, features=replace(
    closer.features,
    radar_object=replace(closer.features.radar_object, front_d_rel=19.7, front_v_rel=-1.5),
  ))

  initial = VisionLeadContext(0.99, 29.1, 0.41, 9.2, 0.0, 2.37, 0.25, 1.2)
  assert matcher.match_context(initial, (farther,), 10.0).prediction is farther

  motorcycle_scene = VisionLeadContext(0.993, 25.6, -0.075, 10.48, 0.0, 2.37, 0.25, 1.2)
  match = matcher.match_context(motorcycle_scene, (farther, closer), 10.0)
  assert match is not None
  assert match.prediction is closer


def test_stable_in_lane_closer_second_match_wins_for_small_target() -> None:
  matcher = VisionRadarMatcher()
  farther = prediction(43, 0.10, 0.1, 0.1, d_rel=29.4, v_lead=9.13)
  closer = prediction(34, -0.14, 0.1, 0.1, d_rel=20.3, v_lead=8.69)
  farther = replace(farther, features=replace(
    farther.features,
    radar_object=replace(farther.features.radar_object, front_d_rel=29.4, front_v_rel=-0.73),
  ))
  closer = replace(closer, features=replace(
    closer.features,
    radar_object=replace(closer.features.radar_object, front_d_rel=20.3, front_v_rel=-1.17),
  ))
  # The closer motorcycle is just outside the normal 25% distance gate, but
  # is a stable, sane second match more than halfway to the vision estimate.
  vision = VisionLeadContext(0.994, 27.94, -0.053, 10.16, 0.0, 2.54, 0.26, 1.22)

  match = matcher.match_context(vision, (farther, closer), 9.9)
  assert match is not None
  assert match.prediction is closer


def test_scc_can_supply_vision_matched_lead_one() -> None:
  matcher = VisionRadarMatcher()
  scc = prediction(0, 0.1, 0.1, 0.1, front=False, scc=True, d_rel=20.0, v_lead=18.0)
  vision = VisionLeadContext(0.95, 20.0, 0.1, 18.0, 0.0, 1.0, 0.5, 1.0)
  match = matcher.match_context(vision, (scc,), 20.0)
  assert match is not None
  assert match.prediction.features.radar_object.scc_track_id == 0


def test_controller_does_not_duplicate_primary_cutin_as_lead_two() -> None:
  shared = prediction(40, 0.2, 0.1, 0.95)

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(
        True,
        RadarLeadDecision((), (shared,), ()),
        (shared,),
        0.1,
      )

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(12.0, 0.2, 19.0))
  assert output.lead_one is not None and output.lead_one["radarTrackId"] == 40
  assert output.lead_two is None
  assert output.lead_one["radar"]
  assert output.leads_cutin == ()


def test_controller_does_not_duplicate_primary_external_as_lead_two() -> None:
  shared = prediction(40, 0.2, 0.95, 0.1, external_prob=0.95)

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(
        True,
        RadarLeadDecision((shared,), (), (shared,)),
        (shared,),
        0.1,
      )

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(12.0, 0.2, 19.0))
  assert output.lead_one is not None and output.lead_one["radarTrackId"] == 40
  assert output.lead_two is None


def test_controller_reports_control_unusable_cutin_without_lead_two() -> None:
  close_cutin = prediction(32, 1.2, 0.1, 0.99, front=False, d_rel=1.5)

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(
        True,
        RadarLeadDecision((), (close_cutin,)),
        (close_cutin,),
        0.1,
      )

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 1.0, (), vision_model(8.0, 0.0, 1.0))

  assert output.lead_two is None
  assert len(output.leads_cutin) == 1
  assert output.leads_cutin[0]["radarTrackId"] == 32


def test_controller_suppresses_low_speed_out_of_lane_cutin() -> None:
  side_cutin = prediction(49, 2.2, 0.1, 0.99, d_rel=3.0, v_lead=1.8)

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(
        True,
        RadarLeadDecision((), (side_cutin,)),
        (side_cutin,),
        0.1,
      )

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 0.45, (), vision_model(8.0, 0.0, 1.0))

  assert output.lead_two is None
  assert output.leads_cutin == ()


def test_controller_suppresses_submeter_cutin_return() -> None:
  bumper_return = prediction(46, 0.0, 0.1, 0.99, d_rel=0.25, v_lead=0.1)

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(
        True,
        RadarLeadDecision((), (bumper_return,)),
        (bumper_return,),
        0.1,
      )

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 5.0, (), vision_model(8.0, 0.0, 5.0))

  assert output.lead_two is None
  assert output.leads_cutin == ()


def test_controller_hides_cutin_behind_primary_lead() -> None:
  primary = prediction(35, -0.3, 0.95, 0.1, d_rel=3.55, v_lead=0.0)
  primary = replace(primary, features=replace(
    primary.features,
    radar_object=replace(primary.features.radar_object, front_d_rel=3.55, front_v_rel=0.0),
  ))
  hidden_cutin = prediction(33, 0.25, 0.1, 0.99, front=False, d_rel=7.65, v_lead=0.0)

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(
        True,
        RadarLeadDecision((primary,), (hidden_cutin,)),
        (primary, hidden_cutin),
        0.1,
      )

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 0.0, (), vision_model(3.55, -0.3, 0.0))

  assert output.lead_one is not None and output.lead_one["radarTrackId"] == 35
  assert output.lead_two is None
  assert output.leads_cutin == ()


def test_controller_never_uses_raw_vision_as_lead_one() -> None:
  far = prediction(40, 0.2, 0.1, 0.1)

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((), ()), (far,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(50.0, 0.2, 19.0))
  assert output.lead_one is None


def test_controller_uses_unmatched_in_lane_model_lead_as_stealth_lead_two() -> None:
  candidate = prediction(40, 0.2, 0.95, 0.1)
  candidate = replace(candidate, features=replace(
    candidate.features, d_path=0.1, in_lane_prob=0.9, track_age=12,
  ))

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((candidate,), ()), (candidate,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(50.0, 0.0, 19.0))
  assert output.lead_one is None
  assert output.lead_two is not None and output.lead_two["radarTrackId"] == 40
  assert output.lead_two["radar"]


def test_controller_rejects_out_of_lane_model_lead_as_stealth_lead_two() -> None:
  candidate = prediction(40, 2.0, 0.95, 0.1)
  candidate = replace(candidate, features=replace(
    candidate.features, d_path=1.2, in_lane_prob=0.2, track_age=12,
  ))

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((candidate,), ()), (candidate,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), vision_model(50.0, 0.0, 19.0))
  assert output.lead_one is None
  assert output.lead_two is None


def test_controller_rejects_stationary_front_external_as_lead_two() -> None:
  ghost = prediction(40, 0.2, 0.0, 0.0, external_prob=1.0, v_lead=0.0, d_rel=22.0)

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((), (), (ghost,)), (ghost,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 17.0, (), vision_model(75.0, 0.0, 18.0))

  assert output.lead_two is None
  assert output.lead_external is None


def test_controller_rejects_stationary_unmatched_front_stealth_lead_two() -> None:
  ghost = prediction(40, 0.2, 1.0, 0.0, v_lead=0.0, d_rel=22.0)
  ghost = replace(ghost, features=replace(
    ghost.features, d_path=0.1, in_lane_prob=0.9, track_age=12,
  ))

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((ghost,), ()), (ghost,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 17.0, (), vision_model(75.0, 0.0, 18.0))

  assert output.lead_two is None


def test_controller_holds_same_physically_sane_stealth_lead_briefly() -> None:
  candidate = prediction(40, 0.2, 0.95, 0.1)
  candidate = replace(candidate, features=replace(
    candidate.features, d_path=0.1, in_lane_prob=0.9, track_age=12,
  ))

  class Runtime:
    active = True

    def update(self, *_args):
      decision = RadarLeadDecision((candidate,), ()) if self.active else RadarLeadDecision((), ())
      return RadarLeadRuntimeResult(True, decision, (candidate,), 0.1)

  controller = VisionModelRadarController()
  runtime = Runtime()
  controller.runtime = runtime
  assert controller.update(0.0, 20.0, (), vision_model(50.0, 0.0, 19.0)).lead_two is not None

  runtime.active = False
  assert controller.update(0.3, 20.0, (), vision_model(50.0, 0.0, 19.0)).lead_two is not None
  assert controller.update(0.6, 20.0, (), vision_model(50.0, 0.0, 19.0)).lead_two is None


def test_controller_moves_recent_primary_to_lead_two_during_brief_vision_mismatch() -> None:
  candidate = prediction(40, 0.2, 0.95, 0.1)
  candidate = replace(candidate, features=replace(
    candidate.features, d_path=1.5, in_lane_prob=0.1, track_age=12,
  ))

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((), ()), (candidate,), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  matched = controller.update(0.0, 20.0, (), vision_model(12.0, 0.2, 19.0))
  assert matched.lead_one is not None and matched.lead_one["radarTrackId"] == 40

  held = controller.update(0.5, 20.0, (), vision_model(40.0, 0.0, 19.0))
  assert held.lead_one is None
  assert held.lead_two is not None and held.lead_two["radarTrackId"] == 40
  assert controller.update(0.8, 20.0, (), vision_model(40.0, 0.0, 19.0)).lead_two is None


def test_controller_keeps_displaced_primary_as_lead_two_during_vision_target_switch() -> None:
  near = prediction(40, 0.2, 0.95, 0.1)
  near = replace(near, features=replace(near.features, d_path=1.5, in_lane_prob=0.1, track_age=12))
  far = prediction(41, 0.2, 0.95, 0.1, d_rel=30.0)
  far = replace(far, features=replace(
    far.features,
    radar_object=replace(far.features.radar_object, front_d_rel=30.0),
    d_path=0.1,
    in_lane_prob=0.9,
    track_age=12,
  ))

  class Runtime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((), ()), (near, far), 0.1)

  controller = VisionModelRadarController()
  controller.runtime = Runtime()
  first = controller.update(0.0, 20.0, (), vision_model(12.0, 0.2, 19.0))
  assert first.lead_one is not None and first.lead_one["radarTrackId"] == 40

  switched = controller.update(0.1, 20.0, (), vision_model(30.0, 0.2, 19.0))
  assert switched.lead_one is not None and switched.lead_one["radarTrackId"] == 41
  assert switched.lead_two is not None and switched.lead_two["radarTrackId"] == 40


def test_review_pauses_only_when_lead_one_becomes_vision_only() -> None:
  frames = [SimpleNamespace(time_s=index * 0.1) for index in range(5)]
  selections = (
    Selection(Candidate(40, 1.0, "radar"), None),
    Selection(Candidate(-1, 1.0, "vision fallback"), None),
    Selection(Candidate(-1, 1.0, "vision fallback"), None),
    Selection(Candidate(41, 1.0, "radar"), None),
    Selection(Candidate(-1, 1.0, "vision fallback"), None),
  )

  class Selector:
    def select(self, _frame, frame_index):
      return selections[frame_index]

  events = validation_review_events(frames, Selector(), None)
  assert events == {1: ("leadOne VISION",), 4: ("leadOne VISION",)}


def test_review_pauses_when_high_probability_vision_has_no_sane_radar_match() -> None:
  frames = [
    SimpleNamespace(time_s=0.0, model_leads=(SimpleNamespace(probability=0.9),)),
    SimpleNamespace(time_s=0.1, model_leads=(SimpleNamespace(probability=0.9),)),
    SimpleNamespace(time_s=0.2, model_leads=(SimpleNamespace(probability=0.9),)),
  ]
  selections = (
    Selection(Candidate(40, 1.0, "radar"), None),
    Selection(None, None),
    Selection(None, None),
  )

  class Selector:
    def select(self, _frame, frame_index):
      return selections[frame_index]

  events = validation_review_events(frames, Selector(), None)
  assert events == {1: ("leadOne LOST (VISION)",)}


def test_review_distinguishes_initial_vision_unmatched_from_lead_one_loss() -> None:
  frames = [
    SimpleNamespace(time_s=0.0, model_leads=(SimpleNamespace(probability=0.9),)),
    SimpleNamespace(time_s=0.1, model_leads=(SimpleNamespace(probability=0.9),)),
  ]
  selections = (Selection(None, None), Selection(None, None))

  class Selector:
    def select(self, _frame, frame_index):
      return selections[frame_index]

  events = validation_review_events(frames, Selector(), None)
  assert events == {0: ("VISION UNMATCHED",)}


def test_review_pauses_when_lead_two_only_is_lost_with_vision_present() -> None:
  frames = [
    SimpleNamespace(time_s=0.0, model_leads=(SimpleNamespace(probability=0.9),)),
    SimpleNamespace(time_s=0.1, model_leads=(SimpleNamespace(probability=0.9),)),
    SimpleNamespace(time_s=0.2, model_leads=(SimpleNamespace(probability=0.9),)),
  ]
  selections = (
    Selection(None, Candidate(40, 0.9, "MLP active cutin")),
    Selection(None, None),
    Selection(None, None),
  )

  class Selector:
    def select(self, _frame, frame_index):
      return selections[frame_index]

  events = validation_review_events(frames, Selector(), None)
  assert events == {
    0: ("CUT-IN id 40", "VISION UNMATCHED"),
    1: ("leadTwo LOST (VISION)",),
  }


def test_review_does_not_pause_when_lead_one_replaces_lead_two_only() -> None:
  frames = [
    SimpleNamespace(time_s=0.0, model_leads=(SimpleNamespace(probability=0.9),)),
    SimpleNamespace(time_s=0.1, model_leads=(SimpleNamespace(probability=0.9),)),
  ]
  selections = (
    Selection(None, Candidate(40, 0.9, "MLP active cutin")),
    Selection(Candidate(40, 0.9, "radar"), None),
  )

  class Selector:
    def select(self, _frame, frame_index):
      return selections[frame_index]

  events = validation_review_events(frames, Selector(), None)
  assert events == {0: ("CUT-IN id 40", "VISION UNMATCHED")}


def test_cutin_continuity_bridges_short_lead_two_drop() -> None:
  frames = [SimpleNamespace(time_s=index * 0.1) for index in range(5)]
  cutin = Selection(None, Candidate(40, 0.9, "MLP active cutin"))
  selections = (cutin, cutin, Selection(None, None), cutin, cutin)

  class Selector:
    def select(self, _frame, frame_index):
      return selections[frame_index]

  continuity = cutin_continuity_series(frames, Selector(), bridge_gap_s=1.0, retain_s=0.0)
  assert continuity[2] is not None
  assert continuity[2].matched_frames == 2
  assert continuity[2].episode_frames == 3
  assert continuity[2].drop_runs == 1
  assert continuity[2].current_drop_s > 0.0
  assert continuity[4] is not None
  assert continuity[4].matched_frames == 4
  assert continuity[4].episode_frames == 5
  assert continuity[4].current_drop_s == 0.0
