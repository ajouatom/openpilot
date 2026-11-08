"""Native MPC and complete production-planner checks, with no live settings/IPC."""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.controls.lib.tesla_traffic_stop import STOP_BUFFER
from openpilot.selfdrive.controls.tests.test_tesla_traffic_stop import N, state
from opendbc.car.tesla.values import TeslaFlags


@pytest.fixture
def native():
  return pytest.importorskip("openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc")


def carrot_for(mode="acc", stop_distance=6.):
  return NS(mode=mode, comfort_brake=2.5, stop_distance=stop_distance, v_cruise=20., stop_dist=1000.,
            trafficStopDistanceAdjust=100., trafficStopModelLeadOffset=2.,
            get_T_FOLLOW=lambda *args: 1.45, jerk_factor=1., leadAccelResponse=0,
            autoNaviSpeedDecelRate=1., lane_change_active=False, myDrivingMode=3)


def solve(native, *, mode="acc", distance=None, stop_distance=6., lead=False, reset=False):
  mpc = native.LongitudinalMpc(mode=mode)
  mpc.set_accel_limits(-2., 1.6)
  mpc.set_cur_state(10., 0.)
  radar = log.RadarState.new_message()
  if lead:
    radar.leadOne = {"status": True, "radar": True, "radarTrackId": 1, "dRel": 18., "vLead": 0., "aLeadTau": 1.5}
  times = native.T_IDXS
  mpc.update(carrot_for(mode, stop_distance), reset, radar, 20., times * 10., np.full(13, 10.),
             np.zeros(13), np.zeros(13), tesla_stop_distance=distance)
  assert mpc.solution_status == 0
  return mpc


@pytest.mark.parametrize("mode", ["acc", "blended"])
@pytest.mark.parametrize("gap", [3., 6., 10.])
def test_native_mpc_uses_measured_endpoint_without_model_adjustments(native, mode, gap):
  baseline = solve(native, mode=mode, stop_distance=gap)
  stopping = solve(native, mode=mode, distance=28., stop_distance=gap)
  assert np.all(stopping.params[:, 2] <= 28. + gap)
  assert np.any(stopping.a_solution < baseline.a_solution - .1)
  np.testing.assert_array_equal(stopping.params[:, [0, 1, 4, 6, 7]], baseline.params[:, [0, 1, 4, 6, 7]])
  assert stopping.mode == "acc"
  assert stopping.source == "e2e"


@pytest.mark.parametrize("mode", ["acc", "blended"])
def test_other_obstacles_remain_stronger_and_can_green_cannot_remove_them(native, mode):
  lead_only = solve(native, mode="acc", lead=True)
  both = solve(native, mode=mode, distance=80., lead=True)
  np.testing.assert_array_equal(both.params, lead_only.params)
  np.testing.assert_array_equal(both.a_solution, lead_only.a_solution)
  assert both.source == lead_only.source


@pytest.mark.parametrize("invalid", [float("nan"), float("inf"), -1., 255.])
def test_invalid_optional_constraint_is_exactly_the_original_native_path(native, invalid):
  baseline = solve(native)
  invalid_stop = solve(native, distance=invalid)
  np.testing.assert_array_equal(invalid_stop.params, baseline.params)
  np.testing.assert_array_equal(invalid_stop.x_sol, baseline.x_sol)


def test_reset_does_not_apply_optional_constraint(native):
  baseline = solve(native, reset=True)
  reset_stop = solve(native, distance=0., reset=True)
  np.testing.assert_array_equal(reset_stop.params, baseline.params)
  np.testing.assert_array_equal(reset_stop.x_sol, baseline.x_sol)


@pytest.fixture
def planner_factory(native, monkeypatch):
  from openpilot.selfdrive.controls.lib import longitudinal_planner as module

  params = NS(get_int=lambda key: 0, get_float=lambda key: 50. if key == "VEgoStopping" else 20.)
  monkeypatch.setattr(module, "Params", lambda: params)
  monkeypatch.setattr(module.cloudlog, "event", lambda *args, **kwargs: None)
  clock = [N]
  monkeypatch.setattr(module.time, "clock_gettime_ns", lambda _: clock[0])

  class Messages(dict):
    healthy = True
    def all_checks(self, service_list):
      return self.healthy

  def create(brand="tesla", enabled=True, mode="acc", speed=10.):
    cp = car.CarParams.new_message(brand=brand, openpilotLongitudinalControl=True,
                                  flags=int(TeslaFlags.CAN_TRAFFIC_STOP) if enabled else 0)
    planner = module.LongitudinalPlanner(cp, init_v=speed)
    carrot = carrot_for(mode)
    carrot.soft_hold_active = False
    carrot.get_carrot_accel = lambda v: 1.6
    carrot.aChangeCostStarting = 10.
    carrot.start_decision = carrot.prepare_decision = None
    carrot.prepare_control_active = False
    carrot.events = set()
    carrot.update = lambda *args: 72.
    model = log.ModelDataV2.new_message()
    times = np.asarray(module.ModelConstants.T_IDXS)
    model.position.x = (speed * times).tolist()
    model.velocity.x = [speed] * len(times)
    model.acceleration.x = [0.] * len(times)
    model.action.desiredVelocity = speed
    model.action.desiredAcceleration = 1.
    cs = state(speed=speed)
    cs.vCruise = 72.
    cs.vCluRatio = 1.
    sm = Messages(carState=cs, modelV2=model.as_reader(), radarState=log.RadarState.new_message().as_reader(),
                  controlsState=log.ControlsState.new_message(longControlState="pid"),
                  selfdriveState=log.SelfdriveState.new_message(enabled=True, active=True, experimentalMode=mode == "blended"),
                  carControl=car.CarControl.new_message(longActive=True))
    sm['selfdriveState'].personality = "standard"

    def update(t, *, light="red", distance=30.):
      clock[0] = N + round(t * 1e9)
      sm['carState'].teslaTrafficControl.sampleMonoTime = clock[0]
      sm['carState'].teslaTrafficControl.light = light
      sm['carState'].teslaTrafficControl.stopLineDistance = distance
      carrot.events.clear()
      planner.update(sm, carrot)
      return planner
    return planner, sm, carrot, update
  return create


@pytest.mark.parametrize("brand", ["hyundai", "toyota", "volkswagen", "tesla"])
def test_complete_planner_disabled_and_foreign_misconfigured_switch_are_identical(planner_factory, brand):
  original, _, _, original_step = planner_factory(brand, False)
  modified, _, _, modified_step = planner_factory(brand, brand != "tesla")
  for i in range(12):
    original_step(.05 * i)
    modified_step(.05 * i)
    np.testing.assert_array_equal(modified.mpc.params, original.mpc.params)
    np.testing.assert_array_equal(modified.a_desired_trajectory, original.a_desired_trajectory)
    assert (modified.output_a_target, modified.output_v_target_now, modified.output_should_stop, modified.mpc.source) == (
      original.output_a_target, original.output_v_target_now, original.output_should_stop, original.mpc.source)
  assert modified.tesla_traffic_stop is None


@pytest.mark.parametrize("mode", ["acc", "blended"])
def test_complete_planner_delivers_can_stop_and_does_not_forge_model_or_radar(planner_factory, mode):
  planner, sm, _, update = planner_factory(mode=mode)
  original_model = sm['modelV2'].to_dict()
  original_radar = sm['radarState'].to_dict()
  for i in range(5):
    update(.05 * i, distance=30. - .5 * i)
  assert planner.tesla_traffic_decision.active
  assert np.all(planner.mpc.params[:, 2] <= 28. - STOP_BUFFER + 6.)
  assert np.any(planner.a_desired_trajectory < -.1)
  assert sm['modelV2'].to_dict() == original_model
  assert sm['radarState'].to_dict() == original_radar


@pytest.mark.parametrize("veto", ["stock_long", "long_inactive", "stale_input", "pedal"])
def test_complete_planner_checks_actual_longitudinal_permission(planner_factory, veto):
  planner, sm, _, update = planner_factory()
  if veto == "stock_long":
    planner.CP.openpilotLongitudinalControl = False
    # Factory normally prevents construction on this configuration.
    from openpilot.selfdrive.controls.lib.tesla_traffic_stop import create_tesla_traffic_stop
    planner.tesla_traffic_stop = create_tesla_traffic_stop(planner.CP)
  elif veto == "long_inactive":
    sm['carControl'].longActive = False
  elif veto == "stale_input":
    sm.healthy = False
  else:
    sm['carState'].gasPressed = True
  for i in range(8):
    update(.05 * i)
  assert planner.tesla_traffic_decision is None or not planner.tesla_traffic_decision.active


def test_final_hold_is_same_cycle_and_fault_reaches_existing_soft_disable(planner_factory, monkeypatch):
  from openpilot.selfdrive.selfdrived import events as events_module
  from openpilot.selfdrive.selfdrived.state import StateMachine, SOFT_DISABLE_TIME
  from openpilot.common.realtime import DT_CTRL
  monkeypatch.setattr(events_module, "tr", lambda text: text)
  planner, sm, carrot, update = planner_factory(speed=0.)
  for i in range(5):
    update(.05 * i, distance=1.)
  assert planner.output_should_stop and planner.output_v_target_now == 0.
  assert planner.output_a_target <= 0. and planner.coasting_target == 0.
  assert planner.tesla_traffic_decision.hold
  update(.25, distance=100.)
  assert log.OnroadEvent.EventName.teslaTrafficStopTakeover in carrot.events
  assert planner.tesla_traffic_decision.fault
  events = events_module.Events()
  for event in carrot.events:
    events.add(event)
  machine = StateMachine()
  machine.state = log.SelfdriveState.OpenpilotState.enabled
  machine.update(events)
  assert machine.state == log.SelfdriveState.OpenpilotState.softDisabling
  alerts = events.create_alerts([events_module.ET.SOFT_DISABLE],
                               [planner.CP, sm['carState'], sm, False, 100, log.LongitudinalPersonality.standard])
  assert len(alerts) == 1 and "Tesla CAN" in alerts[0].alert_text_2
  for _ in range(round(SOFT_DISABLE_TIME / DT_CTRL)):
    machine.update(events)
  assert machine.state == log.SelfdriveState.OpenpilotState.disabled
  # A green cannot clear another existing model/user stop.
  planner.apply_signal_start_guard(NS(start_decision=NS(hold=True)), 0.)
  assert planner.output_should_stop


def test_late_red_creates_warning_without_automatically_disabling_control(planner_factory, monkeypatch):
  from openpilot.selfdrive.selfdrived import events as events_module
  from openpilot.selfdrive.selfdrived.state import StateMachine
  monkeypatch.setattr(events_module, "tr", lambda text: text)
  speed = 80 / 3.6
  _, _, carrot, update = planner_factory(speed=speed)
  for i in range(5):
    update(.05 * i, distance=30. - speed * .05 * i)
  events = events_module.Events()
  for event in carrot.events:
    events.add(event)
  alert, = events.create_alerts([events_module.ET.WARNING])
  assert "BRAKE!" in alert.alert_text_1
  machine = StateMachine()
  machine.state = log.SelfdriveState.OpenpilotState.enabled
  machine.update(events)
  assert machine.state == log.SelfdriveState.OpenpilotState.enabled


def test_blended_red_uses_stop_solver_preserves_stronger_model_braking_and_restores_mode(planner_factory):
  planner, sm, _, update = planner_factory(mode="blended")
  model = sm['modelV2'].as_builder()
  model.action.desiredAcceleration = -2.5
  model.action.shouldStop = True
  sm['modelV2'] = model.as_reader()
  for i in range(5):
    update(.05 * i, distance=30. - .5 * i)
  assert planner.mpc.mode == "acc" and planner.tesla_traffic_blended
  assert planner.output_a_target <= -2.5
  for i in range(11):
    update(.25 + .05 * i, light="green", distance=27.5 - .5 * i)
  assert not planner.tesla_traffic_decision.active
  assert planner.mpc.mode == "blended" and not planner.tesla_traffic_blended
  assert planner.output_should_stop and planner.output_a_target <= -2.5


@pytest.mark.parametrize("mode", ["acc", "blended"])
@pytest.mark.parametrize("speed,distance", [(10., 60.), (50 / 3.6, 100.), (80 / 3.6, 125.)])
def test_idealized_native_feedback_loop_stops_before_line_despite_go_model(planner_factory, mode, speed, distance):
  planner, sm, _, update = planner_factory(mode=mode, speed=speed)
  acceleration = 0.
  for i in range(800):
    sm['carState'].vEgo = speed
    sm['carState'].aEgo = acceleration
    sm['carState'].standstill = speed < .01
    update(.05 * i, distance=max(0., round(distance)))
    assert planner.mpc.solution_status == 0
    assert planner.tesla_traffic_decision is None or not planner.tesla_traffic_decision.fault
    acceleration = float(np.clip(planner.output_a_target, -3.48, 2.))
    next_speed = max(0., speed + acceleration * .05)
    distance -= (speed + next_speed) * .025
    speed = next_speed
    assert distance > 0.
    if speed < .1 and planner.output_should_stop:
      break
  assert speed < .1 and planner.output_should_stop
  assert 0. < distance < 6.
