import pytest

from openpilot.common.realtime import DT_DMON
from openpilot.selfdrive.monitoring.dm2 import DriverMonitoring2
from openpilot.selfdrive.monitoring.policy import DriverMonitoring, AlertLevel
from openpilot.selfdrive.monitoring.test_monitoring import make_msg


def wheel(dm, seconds, factor=1.0, response=False, enabled=True):
  for _ in range(round(seconds / DT_DMON)):
    dm.run_without_camera(response, enabled, False, False, factor)


@pytest.mark.parametrize('factor', [1.0, 2.0, 2.4])
def test_wheel_budget_and_terminal_alert(factor):
  dm = DriverMonitoring2()
  wheel(dm, 5 * factor - DT_DMON, factor)
  assert dm.alert_level == AlertLevel.none
  wheel(dm, 2 * DT_DMON, factor)
  assert dm.alert_level == AlertLevel.one
  wheel(dm, 10 * factor, factor)
  assert dm.alert_level == AlertLevel.two
  wheel(dm, 10 * factor, factor)
  assert dm.alert_level == AlertLevel.three
  assert dm.alert_3_cnt == 1
  wheel(dm, 6, factor, response=True)
  assert dm.alert_level == AlertLevel.three
  assert dm.no_response_cnt == 1
  assert dm.get_state_packet().driverMonitoringState.noResponseForceDecel
  wheel(dm, DT_DMON, enabled=False)
  assert dm.alert_level == AlertLevel.none


def test_ten_second_override_preserves_elapsed_time_and_latches_orange():
  dm = DriverMonitoring2()
  wheel(dm, 16, 2)
  assert dm.alert_level == AlertLevel.one
  wheel(dm, DT_DMON, 1)
  assert dm.alert_level == AlertLevel.two
  assert (1 - dm.awareness) * 25 == pytest.approx(16.05)
  wheel(dm, DT_DMON, 2)
  assert dm.wheel_factor == 1
  assert dm.alert_level == AlertLevel.two
  wheel(dm, DT_DMON, 2, response=True)
  assert dm.alert_level == AlertLevel.none
  wheel(dm, DT_DMON, 2)
  assert dm.wheel_factor == 2


def test_strict_transition_overdue_red_counts_once_and_cannot_be_cleared_by_input():
  dm = DriverMonitoring2()
  wheel(dm, 28, 2)
  wheel(dm, DT_DMON, 1, response=True)
  assert dm.alert_level == AlertLevel.three
  assert dm.alert_3_cnt == 1
  wheel(dm, 10, 2, response=True)
  assert dm.alert_3_cnt == 1 and dm.no_response_cnt == 1


@pytest.mark.parametrize('cause', ['eye', 'sleep', 'phone'])
def test_relaxed_pose_keeps_other_detection_and_timing_identical(cause):
  standard, dm = DriverMonitoring(), DriverMonitoring2()
  dm.relax_pose = True
  msg = make_msg(True, distracted=(cause == 'eye'))
  if cause != 'eye':
    setattr(msg.leftDriverData, cause + 'Prob', 0.99)
  for _ in range(round(15 / DT_DMON)):
    for policy in (standard, dm):
      policy._update_states(msg, [0, 0, 0], 20, True, False)
      policy._update_events(False, True, False, False)
    assert dm.alert_level == standard.alert_level
    assert dm.awareness == standard.awareness
  assert dm.alert_level == AlertLevel.three


def test_pose_only_relaxation_restores_on_strict_or_orange():
  dm = DriverMonitoring2()
  dm.pose.yaw = dm.settings._YAW_NATURAL_OFFSET + 0.44
  dm.pose.pitch = dm.settings._PITCH_NATURAL_OFFSET
  dm._get_distracted_types()
  assert dm.distracted_types['pose']
  dm.relax_pose = True
  dm._get_distracted_types()
  assert not dm.distracted_types['pose']
  assert dm.settings._POSE_YAW_THRESHOLD == 0.4020
  dm.alert_level = AlertLevel.two
  dm._get_distracted_types()
  assert dm.distracted_types['pose']


def test_camera_failure_and_recovery_never_reset_progress_or_terminal_alert():
  dm = DriverMonitoring2()
  dm.awareness = 0.5
  dm.set_camera_available(False, 2)
  assert dm.awareness == 0.5
  wheel(dm, 0.05, 2)
  prior = dm.awareness
  dm.set_camera_available(True)
  assert dm.awareness <= prior
  dm.set_camera_available(False, 2)
  wheel(dm, 60, 2)
  assert dm.alert_level == AlertLevel.three
  count = dm.alert_3_cnt
  dm.set_camera_available(True)
  awake = make_msg(True)
  dm._update_states(awake, [0, 0, 0], 20, True, False)
  dm._update_events(True, True, False, False)
  assert dm.alert_level == AlertLevel.three
  assert dm.alert_3_cnt == count


def test_forward_score_requires_two_seconds_then_accelerates_recovery_only_in_mode_one():
  awake = make_msg(True)
  standard, dm = DriverMonitoring(), DriverMonitoring2()
  dm.relax_pose = True
  # Accumulate confidence without discarding an existing attention debt.
  for _ in range(40):
    dm._update_states(awake, [0, 0, 0], 20, True, False)
  standard.awareness = dm.awareness = 0.6
  for policy in (standard, dm):
    policy._update_states(awake, [0, 0, 0], 20, True, False)
    policy._update_events(False, True, False, False)
  assert dm.forward_score >= 0.9 and dm.forward_recovery
  assert dm.awareness - 0.6 == pytest.approx(1.5 * (standard.awareness - 0.6))
  dm.relax_pose = False  # traffic entry or mode 0 removes the recovery bonus
  dm._update_events(False, True, False, False)
  assert not dm.forward_recovery
  assert dm.settings._TIMEOUT_RECOVERY_FACTOR_MAX == standard.settings._TIMEOUT_RECOVERY_FACTOR_MAX


@pytest.mark.parametrize('cause', ['eye', 'sleep', 'phone'])
def test_forward_recovery_never_accelerates_sleep_eye_or_phone_debt(cause):
  dm = DriverMonitoring2()
  dm.relax_pose = True
  distracted = make_msg(True, distracted=(cause == 'eye'))
  if cause != 'eye':
    setattr(distracted.leftDriverData, cause + 'Prob', 0.99)
  dm._update_states(distracted, [0, 0, 0], 20, True, False)
  dm.awareness = 0.5
  for _ in range(45):
    dm._update_states(make_msg(True), [0, 0, 0], 20, True, False)
  dm._update_events(False, True, False, False)
  assert dm.forward_score >= 0.9 and not dm.forward_recovery


def test_camera_interaction_credit_is_bounded_fresh_and_rate_limited():
  dm = DriverMonitoring2()
  dm.relax_pose = True
  dm.awareness = 0.6
  dm.credit_camera_interaction(10, 10)
  assert dm.awareness == pytest.approx(0.6 + 2 / 13)
  assert dm.input_credit_seconds == pytest.approx(2)
  dm.credit_camera_interaction(10.05, 10)
  assert dm.input_credit_seconds == 0
  dm.credit_camera_interaction(10.1, 10.1)
  assert dm.input_credit_seconds == 0
  dm.credit_camera_interaction(12, 11)
  assert dm.input_credit_seconds == 0
  dm.awareness = 0.99
  dm.credit_camera_interaction(13, 13)
  assert dm.awareness == 1
  assert dm.input_credit_seconds < 2


@pytest.mark.parametrize('blocked', ['standard', 'eye', 'sleep', 'phone', 'orange', 'red'])
def test_controls_do_not_erase_protected_camera_alerts(blocked):
  dm = DriverMonitoring2()
  dm.relax_pose = blocked != 'standard'
  dm.awareness = 0.6
  if blocked in ('eye', 'sleep', 'phone'):
    dm.distracted_types[blocked] = True
  if blocked in ('orange', 'red'):
    dm.alert_level = AlertLevel.two if blocked == 'orange' else AlertLevel.three
  dm.credit_camera_interaction(10, 10)
  assert dm.awareness == 0.6
  assert dm.input_credit_seconds == 0


def test_mode_zero_camera_policy_matches_stock_through_distraction_and_recovery():
  standard, dm = DriverMonitoring(), DriverMonitoring2()
  sequence = ([make_msg(True)] * 30 + [make_msg(True, distracted=True)] * 80 +
              [make_msg(True)] * 70 + [make_msg(False)] * 40 + [make_msg(True)] * 60)
  for i, sample in enumerate(sequence):
    for policy in (standard, dm):
      policy._set_pose_strictness(0.2, 20)
      policy._update_states(sample, [0, 0, 0], 20, True, False)
      policy._update_events(i % 53 == 0, True, False, False)
    for field in ('awareness', 'alert_level', 'active_policy', 'alert_3_cnt', 'no_response_cnt', 'too_distracted'):
      assert getattr(dm, field) == getattr(standard, field)
