import pytest

from openpilot.selfdrive.monitoring.config import configure_monitoring
from openpilot.selfdrive.monitoring.dm2_context import CameraAvailability, InteractionEdges, ObjectObservation, TrafficContext


def obj(x=30, y=0, speed=20, relative_speed=0):
  return ObjectObservation(x, y, speed, relative_speed)


def test_new_vehicle_strict_window_is_not_occupancy_timer():
  ctx = TrafficContext()
  for i in range(401):
    now = i / 20
    strict, clear = ctx.update(now, [obj()], True, True, True)
    assert strict == (now < 10)
    assert not clear
  assert ctx.update(20.05, [obj(), obj(60, 3)], True, True, True)[0]
  assert ctx.strict_until == 30.05


def test_dropout_lane_change_and_duplicate_do_not_restart_timer():
  ctx = TrafficContext()
  ctx.update(0, [obj()], True, True, True)
  ctx.update(1, [], True, True, True)
  ctx.update(1.5, [obj(31, 1), obj(31.2, 1.1)], True, True, True)
  ctx.update(2, [obj(32, 2.5)], True, True, True)
  assert ctx.strict_until == 10
  assert len(ctx.tracks) == 1


def test_genuinely_new_appearance_after_absence_retriggers():
  ctx = TrafficContext()
  ctx.update(0, [obj()], True, True, True)
  ctx.update(3, [], True, True, True)
  ctx.update(4, [obj()], True, True, True)
  assert ctx.strict_until == 14


def test_same_speed_traffic_is_moving_and_stationary_objects_block_clear():
  ctx = TrafficContext()
  assert ctx.update(0, [obj(speed=20, relative_speed=0)], True, True, True)[0]
  ctx = TrafficContext()
  for t in range(20):
    assert ctx.update(t, [obj(speed=0)], True, True, True) == (False, False)


@pytest.mark.parametrize('healthy,straight,coverage', [(False, True, True), (True, False, True), (True, True, False)])
def test_unknown_or_curved_road_never_earns_empty_road_bonus(healthy, straight, coverage):
  ctx = TrafficContext()
  for t in range(30):
    assert not ctx.update(t, [], healthy, straight, coverage)[1]


def test_clear_road_requires_continuous_ten_seconds_and_stale_data_revokes_it():
  ctx = TrafficContext()
  assert ctx.update(0, [], True, True, True) == (False, False)
  assert ctx.update(10, [], True, True, True) == (False, True)
  assert ctx.update(11, [], False, True, True) == (True, False)
  assert ctx.update(12, [], True, True, True) == (False, False)


def test_invalid_geometry_is_conservative():
  assert TrafficContext().update(0, [obj(x=float('nan'))], True, True, True) == (True, False)


def test_held_controls_and_repeated_button_packets_are_not_repeated_responses():
  edges = InteractionEdges()
  assert edges.update(0, True, False, True, [('accelCruise', True)])
  for t in range(1, 21):
    assert not edges.update(t, True, False, True, [('accelCruise', True)])
  assert edges.timeout_factor(20, True, False, False) == 2
  assert not edges.update(21, False, False, False, [('accelCruise', False)])
  assert edges.update(22, False, True, False, [('accelCruise', True)])
  assert edges.timeout_factor(22, True, False, False) == 2.2


def test_other_buttons_do_not_earn_pedal_bonus_and_strict_overrides_bonuses():
  edges = InteractionEdges()
  assert edges.update(0, False, False, False, [('gapAdjustCruise', True)])
  assert edges.timeout_factor(0, True, False, False) == 2
  edges.update(1, True, False, False, [])
  assert edges.timeout_factor(1, True, False, True) == pytest.approx(2.4)
  assert edges.timeout_factor(1, True, True, True) == 1
  assert edges.timeout_factor(1, False, False, True) == 1


class FakeParams:
  def __init__(self, values):
    self.values = dict(values)

  def get(self, name):
    return self.values.get(name)

  def get_int(self, name):
    return int(self.get(name) or 0)

  def get_bool(self, name):
    return self.get_int(name) == 1

  def put(self, name, value):
    self.values[name] = value

  def put_bool(self, name, value):
    self.put(name, int(value))


@pytest.mark.parametrize('old', [0, 1, 2])
def test_legacy_modes_never_migrate_to_experimental(old):
  params, env = FakeParams({'DisableDM': old}), {}
  configure_monitoring(params, env)
  assert params.get_int('DriverMonitoringMode') == 0
  assert params.get_bool('CarrotVisionEnabled') == (old == 2)
  assert env['CARROT_DM_MODE'] == '0'
  params.put_bool('CarrotVisionEnabled', False)
  params.put('DriverMonitoringMode', 1)
  configure_monitoring(params, env)
  assert not params.get_bool('CarrotVisionEnabled')
  assert env['CARROT_DM_MODE'] == '1'


def test_camera_failure_falls_back_immediately_and_recovery_needs_continuous_health():
  camera = CameraAvailability()
  assert not camera.update(0, False)
  assert not camera.update(1, True)
  assert camera.update(3, True)
  assert not camera.update(3.05, False)
  assert not camera.update(4, True)
  assert not camera.update(5, False)
  assert not camera.update(6, True)
  assert not camera.update(7.95, True)
  assert camera.update(8, True)
