import pytest

from openpilot.cereal import log
from openpilot.selfdrive.carrot.carrot_functions import (
  CarrotPlanner,
  DrivingMode,
  DrivingModeDetector,
  get_driving_mode_comfort_brake_factor,
  get_mode_lead_response,
)
from openpilot.selfdrive.carrot.t_follow import (
  get_t_follow_mode_factor, get_t_follow_mode_max, get_speed_t_follow_factor, get_lead_response_for_gap, ramp_t_follow,
)


DT_MDL = 0.05


def _apply_for_seconds(current: float, target: float, seconds: float, decel_extra: float = 0.0) -> float:
  for _ in range(round(seconds / DT_MDL)):
    current = ramp_t_follow(target, current, decel_extra, DT_MDL)
  return current


def test_gap_one_to_four_is_felt_within_two_seconds():
  assert _apply_for_seconds(1.1, 1.6, 1.7) == pytest.approx(1.6)


def test_gap_increase_is_faster_while_already_decelerating():
  assert _apply_for_seconds(1.1, 1.6, 0.85, decel_extra=0.03) == pytest.approx(1.6)


def test_gap_reduction_remains_immediate():
  assert ramp_t_follow(1.1, 1.6, 0.0, DT_MDL) == pytest.approx(1.1)


@pytest.mark.parametrize("lane_change", [True, False])
def test_gap_stays_at_baseline_without_dynamic_jerk_adjustment(lane_change):
  planner = _speed_tf_planner(0, 1.3)
  planner.speedTFFactor = 10
  planner.myTFollowFactor = 1.0
  planner.tFollowGap1 = planner.tFollowGap2 = planner.tFollowGap3 = planner.tFollowGap4 = 1.3
  planner.lane_change_active = lane_change
  for _ in range(100):
    assert planner.get_T_FOLLOW(v_ego=15., a_ego=0.) == pytest.approx(1.3)
    assert planner.t_follow_last == pytest.approx(1.3)


@pytest.mark.parametrize("boost", [5, 20, 50, 100])
def test_decel_boost_is_added_once_even_in_sustained_braking(boost):
  planner = _speed_tf_planner(0, 1.2)
  planner.speedTFFactor = 10
  planner.myTFollowFactor = 1.
  planner.tFollowGap2 = 1.2
  planner.tFollowGap4 = 1.8
  planner.tFollowDecelBoost = boost / 100.
  values = [planner.get_T_FOLLOW(v_ego=15., a_ego=-1.) for _ in range(200)]
  assert max(values) == pytest.approx(1.2 + .25 * boost / 100.)
  assert planner._tf_decel_base == pytest.approx(1.2)


def test_boost_release_does_not_drop_target_gap_in_one_cycle():
  planner = _speed_tf_planner(0, 1.2)
  planner.speedTFFactor = 10
  planner.myTFollowFactor = 1.
  planner.tFollowGap2 = 1.2
  planner.tFollowGap4 = 1.8
  planner.tFollowDecelBoost = 1.
  for _ in range(100):
    before = planner.get_T_FOLLOW(v_ego=15., a_ego=-1.)
  assert before == pytest.approx(1.45)
  after = planner.get_T_FOLLOW(v_ego=15., a_ego=0.)
  assert after == pytest.approx(before - .005)
  for _ in range(100):
    after = planner.get_T_FOLLOW(v_ego=15., a_ego=0.)
  assert after == pytest.approx(1.2)


def test_increased_braking_margin_is_not_delayed_by_release_filter():
  planner = _speed_tf_planner(0, 1.2)
  planner.tFollowDecelBoost = 1.
  planner._apply_decel_hold_and_boost_t_follow(1.2, -0.3)
  assert planner._apply_decel_hold_and_boost_t_follow(1.2, -2.5) == pytest.approx(1.7)


@pytest.mark.parametrize(
  ("comfort_factor", "t_follow_factor"),
  (
    (0.9, 1.1),
    (0.8, 1.2),
    (1.0, 1.0),
  ),
)
def test_comfort_mode_reduction_increases_t_follow(comfort_factor, t_follow_factor):
  assert get_t_follow_mode_factor(comfort_factor) == pytest.approx(t_follow_factor)


def test_safe_mode_increase_is_not_clipped_at_the_configured_normal_max():
  assert get_t_follow_mode_max(1.6, 1.2, 0.0) == pytest.approx(1.92)


def test_mode_and_deceleration_margin_do_not_clip_the_speed_scaled_target():
  assert get_t_follow_mode_max(1.8, 1.2, 0.1) == pytest.approx(2.26)


def test_safe_comfort_brake_uses_a_modest_reduction_only():
  assert get_driving_mode_comfort_brake_factor(DrivingMode.Safe) == pytest.approx(0.9)
  assert get_driving_mode_comfort_brake_factor(DrivingMode.Eco) == pytest.approx(1.0)
  assert get_driving_mode_comfort_brake_factor(DrivingMode.Normal) == pytest.approx(1.0)
  assert get_driving_mode_comfort_brake_factor(DrivingMode.High) == pytest.approx(1.0)


@pytest.mark.parametrize(
  ("auto_mode", "congested", "expected"),
  (
    (1, False, DrivingMode.Normal),
    (1, True, DrivingMode.Safe),
    (2, False, DrivingMode.Eco),
    (2, True, DrivingMode.Safe),
  ),
)
def test_automatic_driving_mode_mapping(auto_mode, congested, expected):
  detector = DrivingModeDetector()
  detector.congested = congested
  assert detector.get_mode(auto_mode) == expected


def test_safe_t_follow_does_not_compound_during_repeated_deceleration():
  planner = CarrotPlanner.__new__(CarrotPlanner)
  planner.speedTFFactor = 10
  planner.tFollowGap1 = 0.5
  planner.tFollowGap2 = 0.6
  planner.tFollowGap3 = 0.8
  planner.tFollowGap4 = 1.2
  planner.tFollowDecelBoost = 0.0
  planner.leadAccelResponse = 0
  planner.myDrivingMode = DrivingMode.Safe
  planner.myTFollowFactor = 1.2
  planner._tf_decel_extra = 0.0
  planner._tf_applied = 0.72
  planner.t_follow_last = 0.72

  values = [
    planner.get_T_FOLLOW(log.LongitudinalPersonality.standard, v_ego=10.0, a_ego=-1.0)
    for _ in range(100)
  ]
  assert values == pytest.approx([0.72] * 100)


def _speed_tf_planner(lead_accel_response: int, applied_t_follow: float) -> CarrotPlanner:
  planner = CarrotPlanner.__new__(CarrotPlanner)
  planner.speedTFFactor = 10
  planner.tFollowGap1 = 0.4
  planner.tFollowGap2 = 0.6
  planner.tFollowGap3 = 0.8
  planner.tFollowGap4 = 1.2
  planner.tFollowDecelBoost = 0.0
  planner.leadAccelResponse = lead_accel_response
  planner.myDrivingMode = DrivingMode.Safe
  planner.myTFollowFactor = 1.2
  planner._tf_decel_extra = 0.0
  planner._tf_applied = applied_t_follow
  planner.t_follow_last = applied_t_follow
  return planner


@pytest.mark.parametrize("setting,speed,expected", [
  (10, 0., 1.), (10, 200., 1.), (20, 0., 1.), (20, 50., 1.5),
  (20, 100., 2.), (20, 150., 2.5), (20, 200., 3.), (15, 100., 1.5),
  (0, 100., 1.), (50, 100., 3.), (20, -10., 1.),
])
def test_speed_factor_is_linear_and_normalized_at_zero(setting, speed, expected):
  assert get_speed_t_follow_factor(setting, speed) == pytest.approx(expected)


@pytest.mark.parametrize("personality,base", [
  (log.LongitudinalPersonality.aggressive, .4), (log.LongitudinalPersonality.standard, .6),
  (log.LongitudinalPersonality.relaxed, .8), (log.LongitudinalPersonality.moreRelaxed, 1.2),
])
@pytest.mark.parametrize("level", range(6))
@pytest.mark.parametrize("setting,speed", [(10, 100.), (20, 50.), (20, 100.), (20, 200.)])
def test_every_response_keeps_selected_tf_speed_factor_and_mode(personality, base, level, setting, speed):
  expected = base * get_speed_t_follow_factor(setting, speed) * 1.2
  planner = _speed_tf_planner(level, expected)
  planner.speedTFFactor = setting
  assert planner.get_T_FOLLOW(personality, v_ego=speed / 3.6) == pytest.approx(expected)


def test_speed_multiplier_survives_equal_configured_gaps_and_old_two_second_cap():
  planner = _speed_tf_planner(5, 3.0)
  planner.myTFollowFactor = 1.
  planner.speedTFFactor = 20
  planner.tFollowGap1 = planner.tFollowGap2 = planner.tFollowGap3 = planner.tFollowGap4 = 1.5
  assert planner.get_T_FOLLOW(v_ego=100 / 3.6) == pytest.approx(3.)


def test_deceleration_holds_speed_scaled_gap_without_compounding():
  planner = _speed_tf_planner(5, 1.44)
  planner.speedTFFactor = 20
  planner.get_T_FOLLOW(v_ego=100 / 3.6)
  for _ in range(100):
    assert planner.get_T_FOLLOW(v_ego=50 / 3.6, a_ego=-1.) == pytest.approx(1.44)
  assert planner.get_T_FOLLOW(v_ego=50 / 3.6) == pytest.approx(1.08)


@pytest.mark.parametrize("common", range(6))
@pytest.mark.parametrize("gap", range(4))
def test_gap_response_inherits_common_including_zero(common, gap):
  assert get_lead_response_for_gap(common, [-1] * 4, gap) == common


def test_equal_tf_can_use_different_responses_and_zero_is_an_override():
  overrides = [5, 3, 0, -1]
  assert [get_lead_response_for_gap(4, overrides, gap) for gap in range(4)] == [5, 3, 0, 4]
  assert get_lead_response_for_gap(2, overrides, 3) == 2


@pytest.mark.parametrize('mode,ceiling', [(DrivingMode.Eco, 2), (DrivingMode.Safe, 3),
                                        (DrivingMode.Normal, 5), (DrivingMode.High, 5)])
def test_actual_update_resolves_response_after_refresh_on_every_gap_change(mode, ceiling):
  import ast
  from pathlib import Path
  from types import SimpleNamespace
  source = Path(__file__).resolve().parents[1] / 'carrot_functions.py'
  tree = ast.parse(source.read_text(encoding='utf-8'))
  planner_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'CarrotPlanner')
  update = next(n for n in planner_class.body if isinstance(n, ast.FunctionDef) and n.name == 'update')
  # Run the production refresh/selection prefix without hardware or navigation.
  update.body = update.body[:3]
  namespace = {'get_lead_response_for_gap': get_lead_response_for_gap, 'get_mode_lead_response': get_mode_lead_response}
  exec(compile(ast.Module(body=[update], type_ignores=[]), str(source), 'exec'), namespace)
  planner = SimpleNamespace(leadAccelResponseBase=0, leadAccelResponseTF=[5, 3, 0, -1], myDrivingMode=mode)
  planner._update_driving_mode = lambda sm: None
  planner._params_update = lambda: setattr(planner, 'leadAccelResponseBase', 4)
  event = log.Event.new_message()
  state = event.init('selfdriveState')
  for gap, expected in [('aggressive', 5), ('standard', 3), ('relaxed', 0), ('moreRelaxed', 4), ('aggressive', 5)]:
    state.personality = gap
    namespace['update'](planner, {'selfdriveState': state}, 100., 'acc')
    assert planner.leadAccelResponse == min(expected, ceiling)
    assert planner.leadAccelResponseTF == [5, 3, 0, -1]
    assert planner.leadAccelResponseBase == 4


@pytest.mark.parametrize("personality", list(log.LongitudinalPersonality.schema.enumerants.values()))
def test_modes_share_the_same_baseline_jerk_cost(personality):
  planner = _speed_tf_planner(3, 1.2)
  factors = []
  for mode in DrivingMode:
    planner.myDrivingMode = mode
    planner._get_base_t_follow(personality, 10.)
    factors.append(planner.jerk_factor)
  assert factors == [factors[0]] * len(factors)


def test_mode_margin_releases_slowly_without_delaying_manual_gap_reduction():
  planner = _speed_tf_planner(3, 1.2)
  planner.tFollowGap1 = .5
  planner.tFollowGap2 = 1.
  assert planner.get_T_FOLLOW(v_ego=10.) == pytest.approx(1.2)
  planner.myTFollowFactor = 1.
  first = planner.get_T_FOLLOW(v_ego=10.)
  assert first == pytest.approx(1.1975)
  # User requests a smaller gap: only the mode multiplier retains its tail.
  assert planner.get_T_FOLLOW(log.LongitudinalPersonality.aggressive, v_ego=10.) == pytest.approx(.5 * 1.195)
  for _ in range(80):
    final = planner.get_T_FOLLOW(log.LongitudinalPersonality.aggressive, v_ego=10.)
  assert final == pytest.approx(.5)


def test_safe_reentry_during_mode_release_does_not_compound_margin():
  planner = _speed_tf_planner(3, 1.2)
  planner.tFollowGap2 = 1.
  planner.get_T_FOLLOW(v_ego=10.)
  for _ in range(5):
    planner.myTFollowFactor = 1.
    for _ in range(20):
      planner.get_T_FOLLOW(v_ego=10.)
    planner.myTFollowFactor = 1.2
    for _ in range(20):
      value = planner.get_T_FOLLOW(v_ego=10.)
      assert value <= 1.2 + 1e-9
    assert value == pytest.approx(1.2)


@pytest.mark.parametrize('automatic,base_mode', [(1, DrivingMode.Normal), (2, DrivingMode.Eco)])
def test_live_auto_selection_preserves_short_launch_and_manual_override(automatic, base_mode):
  from types import SimpleNamespace as NS

  class State(dict):
    def all_checks(self, services):
      assert services == ['carState', 'radarState']
      return self.valid

  sm = State(carState=NS(vEgo=0.), radarState=NS(leadOne=NS(
    status=True, dRel=8., vLead=0., vRel=0., aLeadK=0., radar=True, radarTrackId=42)))
  sm.valid = True
  planner = CarrotPlanner.__new__(CarrotPlanner)
  planner.myDrivingMode = base_mode
  planner.myDrivingModeAuto = automatic
  planner.myDrivingMode_disable_auto = False
  planner.drivingModeDetector = DrivingModeDetector()
  for _ in range(8):
    planner._update_driving_mode(sm)
  assert planner.myDrivingMode == DrivingMode.Safe
  sm['carState'].vEgo = 20 / 3.6
  lead = sm['radarState'].leadOne
  lead.dRel, lead.vLead, lead.vRel, lead.aLeadK = 35., 30 / 3.6, 10 / 3.6, 2.
  for _ in range(60):
    planner._update_driving_mode(sm)
  assert planner.myDrivingMode == DrivingMode.Safe
  for _ in range(62):
    planner._update_driving_mode(sm)
  assert planner.myDrivingMode == base_mode
  planner.myDrivingMode_disable_auto = True
  planner.myDrivingMode = DrivingMode.High
  sm['carState'].vEgo = 0.
  lead.dRel, lead.vLead, lead.vRel, lead.aLeadK = 8., 0., 0., 0.
  for _ in range(20):
    planner._update_driving_mode(sm)
  assert planner.myDrivingMode == DrivingMode.High


def test_stored_mode_change_still_disables_automatic_selection():
  from types import SimpleNamespace as NS
  values = {'MyDrivingMode': DrivingMode.Eco.value, 'MyDrivingModeAuto': 1, 'TrafficLightDetectMode': 1}
  planner = CarrotPlanner.__new__(CarrotPlanner)
  planner.params = NS(get_int=values.__getitem__)
  planner.frame = planner.params_count = 9
  planner.myDrivingMode_last = planner.myDrivingMode = DrivingMode.Normal
  planner.myDrivingMode_disable_auto = False
  planner.drivingModeDetector = DrivingModeDetector()
  planner.drivingModeDetector.congested = True
  planner._params_update()
  assert planner.myDrivingMode_disable_auto
  assert planner.myDrivingMode == DrivingMode.Eco
