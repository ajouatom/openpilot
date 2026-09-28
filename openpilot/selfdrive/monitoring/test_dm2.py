import pytest

from openpilot.common.realtime import DT_DMON
from openpilot.selfdrive.monitoring.dm2 import DriverMonitoring2
from openpilot.selfdrive.monitoring.policy import DriverMonitoring, AlertLevel
from openpilot.selfdrive.monitoring.test_monitoring import make_msg


def wheel(dm, seconds, clear=False, response=False, enabled=True, strict=False):
  for _ in range(round(seconds / DT_DMON)):
    dm.configure_context(dm.now + DT_DMON, False, strict, clear)
    if response:
      dm.record_interaction(dm.now)
    dm.run_without_camera(response, enabled, False, False)


def camera(dm, seconds, msg=None, clear=False, response=False, held=False, strict=False):
  msg = make_msg(True, distracted=True) if msg is None else msg
  for _ in range(round(seconds / DT_DMON)):
    dm.configure_context(dm.now + DT_DMON, True, strict, clear)
    if response:
      dm.record_interaction(dm.now)
    dm._update_states(msg, [0, 0, 0], 20, True, False)
    dm._update_events(held, True, False, False)


@pytest.mark.parametrize('experimental,clear,factor', [(False, False, 1), (False, True, 1), (True, False, 1), (True, True, 2)])
def test_interaction_budgets_and_terminal_alert(experimental, clear, factor):
  dm = DriverMonitoring2(experimental=experimental)
  wheel(dm, 15 * factor - DT_DMON, clear)
  assert dm.alert_level == AlertLevel.none
  wheel(dm, 2 * DT_DMON, clear)
  assert dm.alert_level == AlertLevel.one
  wheel(dm, 15 * factor, clear)
  assert dm.alert_level == AlertLevel.two
  wheel(dm, 15 * factor, clear)
  assert dm.alert_level == AlertLevel.three and dm.alert_3_cnt == 1
  wheel(dm, 6, clear, response=True)
  assert dm.alert_level == AlertLevel.three and dm.no_response_cnt == 1
  assert dm.get_state_packet().driverMonitoringState.noResponseForceDecel
  wheel(dm, DT_DMON, enabled=False)
  assert dm.alert_level == AlertLevel.none


@pytest.mark.parametrize('experimental,clear', [(False, False), (True, False), (True, True)])
def test_interaction_resets_entire_budget_even_from_orange(experimental, clear):
  dm = DriverMonitoring2(experimental=experimental)
  factor = 2 if experimental and clear else 1
  wheel(dm, 31 * factor, clear)
  assert dm.alert_level == AlertLevel.two
  wheel(dm, DT_DMON, clear, response=True)
  assert dm.awareness == 1 and dm.alert_level == AlertLevel.none
  wheel(dm, 14 * factor, clear)
  assert dm.alert_level == AlertLevel.none


def test_traffic_override_preserves_elapsed_and_does_not_clear_orange():
  dm = DriverMonitoring2(experimental=True)
  wheel(dm, 32, clear=True)
  assert dm.alert_level == AlertLevel.one
  wheel(dm, DT_DMON, clear=True, strict=True)
  assert dm.alert_level == AlertLevel.two
  assert (1 - dm.awareness) * 45 == pytest.approx(32.05)
  wheel(dm, DT_DMON, clear=True)
  assert dm.wheel_factor == 1 and dm.alert_level == AlertLevel.two
  wheel(dm, DT_DMON, clear=True, response=True)
  wheel(dm, DT_DMON, clear=True)
  assert dm.wheel_factor == 2 and dm.alert_level == AlertLevel.none


def test_overdue_terminal_is_counted_once_after_budget_shrinks():
  dm = DriverMonitoring2(experimental=True)
  wheel(dm, 48, clear=True)
  wheel(dm, DT_DMON, strict=True)
  assert dm.alert_level == AlertLevel.three and dm.alert_3_cnt == 1
  wheel(dm, 10, clear=True, response=True)
  assert dm.alert_3_cnt == 1 and dm.no_response_cnt == 1


@pytest.mark.parametrize('clear,factor', [(False, 2), (True, 4)])
@pytest.mark.parametrize('cause', ['eye', 'sleep', 'phone'])
def test_camera_experimental_doubles_timing_but_not_detection_thresholds(clear, factor, cause):
  dm = DriverMonitoring2(experimental=True)
  msg = make_msg(True, distracted=(cause == 'eye'))
  if cause != 'eye':
    setattr(msg.leftDriverData, cause + 'Prob', 0.99)
  camera(dm, 5 * factor - .5, msg, clear)
  assert dm.distracted_types[cause] and dm.alert_level == AlertLevel.none
  camera(dm, 1, msg, clear)
  assert dm.alert_level == AlertLevel.one
  camera(dm, 3 * factor, msg, clear)
  assert dm.alert_level == AlertLevel.two
  camera(dm, 5 * factor, msg, clear)
  assert dm.alert_level == AlertLevel.three
  assert dm.settings._SLEEP_THRESH == .75 and dm.settings._PHONE_THRESH == .5


@pytest.mark.parametrize('clear,allowance', [(False, 45), (True, 90)])
def test_camera_input_starts_full_grace_then_camera_clock(clear, allowance):
  dm = DriverMonitoring2(experimental=True)
  camera(dm, DT_DMON, clear=clear, response=True)
  assert dm.awareness == 1
  camera(dm, allowance - 1, clear=clear, held=True)
  assert dm.alert_level == AlertLevel.none and dm.awareness == 1
  assert dm.grace_started == DT_DMON  # held gas/steering never renews it
  camera(dm, 1.5, clear=clear)
  assert dm.interaction_grace_remaining == 0 and dm.awareness < 1
  camera(dm, (20 if clear else 10), clear=clear)
  assert dm.alert_level == AlertLevel.one


def test_grace_shrinks_on_traffic_without_restarting_and_new_click_renews_it():
  dm = DriverMonitoring2(experimental=True)
  camera(dm, DT_DMON, clear=True, response=True)
  camera(dm, 50, clear=True)
  assert dm.interaction_grace_remaining == pytest.approx(40)
  camera(dm, DT_DMON, strict=True)
  assert dm.interaction_grace_remaining == 0
  assert dm.vision_factor == 2 and dm.awareness < 1
  camera(dm, DT_DMON, response=True, strict=True)
  assert dm.interaction_grace_remaining == 45 and dm.awareness == 1


def test_stale_duplicate_or_future_input_cannot_restart_grace():
  dm = DriverMonitoring2(experimental=True)
  dm.configure_context(10, True)
  dm.record_interaction(9)
  dm.record_interaction(11)
  assert dm.interaction_grace_remaining == 0
  dm.record_interaction(10)
  dm.configure_context(11, True)
  dm.record_interaction(10)
  assert dm.interaction_grace_remaining == 44


def test_expired_grace_cannot_restart_just_because_road_becomes_clear():
  dm = DriverMonitoring2(experimental=True)
  camera(dm, DT_DMON, response=True)
  camera(dm, 48)
  assert dm.interaction_grace_remaining == 0
  previous = dm.awareness
  camera(dm, DT_DMON, clear=True)
  assert dm.interaction_grace_remaining == 0 and dm.awareness < 1
  assert (1 - dm.awareness) * 52 >= (1 - previous) * 26


def test_camera_input_resets_orange_but_does_not_clear_terminal_or_lockout():
  dm = DriverMonitoring2(experimental=True)
  camera(dm, 18)
  assert dm.alert_level == AlertLevel.two
  camera(dm, DT_DMON, response=True)
  assert dm.awareness == 1 and dm.alert_level == AlertLevel.none
  camera(dm, 73)
  assert dm.alert_level == AlertLevel.three
  count = dm.alert_3_cnt
  camera(dm, 3, response=True, msg=make_msg(True))
  assert dm.alert_level == AlertLevel.three and dm.alert_3_cnt == count
  assert dm.interaction_grace_remaining == 0
  dm.too_distracted = True
  dm.alert_level = AlertLevel.none
  dm.awareness = .5
  dm.configure_context(dm.now + 1, True)
  dm.record_interaction(dm.now)
  assert dm.awareness == .5 and dm.interaction_grace_remaining == 0


def test_forward_attention_resets_after_two_seconds_without_starting_input_grace():
  dm = DriverMonitoring2(experimental=True)
  camera(dm, 18)
  for _ in range(39):
    camera(dm, DT_DMON, msg=make_msg(True))
    assert not dm.forward_recovery
  camera(dm, DT_DMON, msg=make_msg(True))
  assert dm.forward_recovery and dm.awareness == 1
  assert dm.interaction_grace_remaining == 0
  camera(dm, DT_DMON)
  assert not dm.forward_recovery and dm.forward_frames == 0


def test_pose_relaxation_restores_settings_and_keeps_strong_warning_thresholds():
  dm = DriverMonitoring2(experimental=True)
  dm.configure_context(1, True)
  dm.pose.yaw = dm.settings._YAW_NATURAL_OFFSET + .44
  dm.pose.pitch = dm.settings._PITCH_NATURAL_OFFSET
  dm._get_distracted_types()
  assert not dm.distracted_types['pose']
  assert dm.settings._POSE_YAW_THRESHOLD == .4020
  dm.alert_level = AlertLevel.two
  dm._get_distracted_types()
  assert dm.distracted_types['pose']


def test_camera_failure_recovery_preserves_progress_and_terminal():
  dm = DriverMonitoring2(experimental=True)
  dm.awareness = .5
  dm.configure_context(1, False, clear=True)
  assert dm.awareness == .5
  wheel(dm, 1, clear=True)
  prior = dm.awareness
  dm.configure_context(3, True)
  assert dm.awareness <= prior
  wheel(dm, 100, clear=True)
  assert dm.alert_level == AlertLevel.three
  camera(dm, 3, msg=make_msg(True), response=True)
  assert dm.alert_level == AlertLevel.three


def test_camera_traffic_shrink_retains_elapsed_and_counts_terminal_once():
  dm = DriverMonitoring2(experimental=True)
  camera(dm, 30, clear=True)
  assert dm.alert_level == AlertLevel.one
  camera(dm, DT_DMON, strict=True)
  assert dm.alert_level == AlertLevel.three and dm.alert_3_cnt == 1
  camera(dm, 7, clear=True, response=True)
  assert dm.alert_3_cnt == 1 and dm.no_response_cnt == 1


def test_mode_zero_camera_recovery_restores_stock_wheel_timing():
  dm = DriverMonitoring2()
  wheel(dm, 10)
  assert dm._timeouts('WHEELTOUCH') == (15, 30, 45)
  camera(dm, DT_DMON, msg=make_msg(True))
  assert dm._timeouts('WHEELTOUCH') == (5, 15, 25)
  assert dm._timeouts('VISION') == (5, 8, 13)


def test_mode_zero_camera_matches_stock_including_face_loss_and_inputs():
  standard, dm = DriverMonitoring(), DriverMonitoring2()
  sequence = ([make_msg(True)] * 30 + [make_msg(True, distracted=True)] * 80 +
              [make_msg(True)] * 70 + [make_msg(False)] * 40 + [make_msg(True)] * 60)
  for i, sample in enumerate(sequence):
    dm.configure_context(i * DT_DMON, True, strict=i % 2 == 0, clear=True)
    dm.record_interaction(i * DT_DMON)  # added BT/buttons must not affect mode 0
    for policy in (standard, dm):
      policy._set_pose_strictness(.2, 20)
      policy._update_states(sample, [0, 0, 0], 20, True, False)
      policy._update_events(i % 53 == 0, True, False, False)
    for field in ('awareness', 'alert_level', 'active_policy', 'alert_3_cnt', 'no_response_cnt', 'too_distracted'):
      assert getattr(dm, field) == getattr(standard, field)
