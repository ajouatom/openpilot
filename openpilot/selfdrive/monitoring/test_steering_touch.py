from types import SimpleNamespace

import pytest

from openpilot.selfdrive.monitoring.dm2_context import SteeringTouchEvidence
from openpilot.selfdrive.monitoring.dm2 import DriverMonitoring2
from openpilot.selfdrive.monitoring.policy import AlertLevel
from openpilot.selfdrive.monitoring.test_dm2 import camera, wheel


def sample(time, held=True, valid=True, available=True):
  return SimpleNamespace(sampleMonoTime=int(time * 1e9), touched=held, valid=valid, available=available)


def test_contact_requires_release_before_camera_edge_and_expires():
  monitor = SteeringTouchEvidence()
  assert monitor.update(10, sample(10), True) == (True, False)
  assert monitor.update(10.1, sample(10.1, held=False), True) == (False, False)
  assert monitor.update(10.2, sample(10.2), True) == (True, True)
  assert monitor.update(10.25, sample(10.2), True) == (True, False)
  assert monitor.update(10.5, sample(10.2), True) == (False, False)
  assert monitor.update(10.6, sample(10.6), True) == (True, False)


@pytest.mark.parametrize('reason', ['car', 'frame', 'unsupported', 'future', 'backwards'])
def test_invalid_contact_never_creates_an_edge_on_recovery(reason):
  monitor = SteeringTouchEvidence()
  monitor.update(10, sample(10, held=False), True)
  touch = sample(10.1, valid=reason != 'frame', available=reason != 'unsupported')
  if reason == 'future':
    touch.sampleMonoTime = int(11 * 1e9)
  if reason == 'backwards':
    touch.sampleMonoTime = int(9.9 * 1e9)
  assert monitor.update(10.1, touch, reason != 'car') == (False, False)
  assert monitor.update(10.2, sample(10.2), True) == (True, False)


@pytest.mark.parametrize('experimental', [False, True])
def test_held_contact_only_maintains_no_camera_monitoring_before_terminal(experimental):
  dm = DriverMonitoring2(experimental=experimental)
  wheel(dm, 20)
  assert dm.alert_level == AlertLevel.one
  # Dispatcher uses the held evidence as a wheel response, not repeated camera
  # record_interaction calls. Releasing begins the normal warning budget.
  for _ in range(1200):
    dm.configure_context(dm.now + .05, False)
    dm.run_without_camera(True, True, False, False)
  assert dm.alert_level == AlertLevel.none and dm.awareness == 1
  wheel(dm, 15.1)
  assert dm.alert_level == AlertLevel.one
  wheel(dm, 31)
  assert dm.alert_level == AlertLevel.three
  dm.run_without_camera(True, True, False, False)
  assert dm.alert_level == AlertLevel.three


@pytest.mark.parametrize('experimental', [False, True])
def test_single_touch_edge_cannot_permanently_suppress_camera(experimental):
  dm = DriverMonitoring2(experimental=experimental)
  camera(dm, 1)
  dm.record_interaction(dm.now)
  camera(dm, 80, held=True)
  assert dm.alert_level == AlertLevel.three
