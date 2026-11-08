import pytest

from openpilot.cereal import car
from openpilot.selfdrive.controls.lib.tesla_traffic_stop import TeslaTrafficStop, create_tesla_traffic_stop
from opendbc.car.tesla.values import TeslaFlags

N = 10_000_000_000


def state(*, sample=N, distance=80., light="red", speed=0.):
  cs = car.CarState.new_message(canValid=True, gearShifter="drive", vEgo=speed)
  cs.teslaTrafficControl = {"available": True, "valid": True, "sampleMonoTime": sample,
                           "stopLineDistance": distance, "light": light, "status": "app_traffic_control"}
  return cs


def step(guard, t=0., *, fresh=True, **kwargs):
  cs = state(sample=N + round(t * 1e9) if fresh else N, **kwargs)
  return guard.update(cs, N + round(t * 1e9), engaged=True, control_active=True)


def establish(guard, *, distance=80., speed=0.):
  for i in range(5):
    result = step(guard, i * .05, distance=distance - speed * i * .05, speed=speed)
  return result


@pytest.mark.parametrize("brand", ["hyundai", "toyota", "tesla"])
@pytest.mark.parametrize("enabled", [False, True])
@pytest.mark.parametrize("longitudinal", [False, True])
def test_factory_ignores_foreign_params_and_requires_longitudinal_permission(brand, enabled, longitudinal):
  cp = car.CarParams.new_message(brand=brand, openpilotLongitudinalControl=longitudinal,
                                flags=int(TeslaFlags.CAN_TRAFFIC_STOP) if enabled else 0)
  assert (create_tesla_traffic_stop(cp) is not None) == (brand == "tesla" and enabled and longitudinal)
  cp.dashcamOnly = True
  assert create_tesla_traffic_stop(cp) is None


@pytest.mark.parametrize("distance,expected", [(0, False), (125, True), (126, False), (127, False), (255, False)])
def test_red_entry_boundary(distance, expected):
  assert establish(TeslaTrafficStop(), distance=distance).active is expected


@pytest.mark.parametrize("light", ["unknown", "green", "yellow"])
def test_only_red_admits_a_new_stop(light):
  guard = TeslaTrafficStop()
  for i in range(12):
    assert not step(guard, i * .05, light=light).active


def test_repeated_cached_or_older_sample_cannot_confirm_red():
  guard = TeslaTrafficStop()
  for t in (0., .05, .1, .2, .3, .4):
    assert not step(guard, t, fresh=False).active
  assert not step(guard, .55).active
  assert not step(guard, .65).active
  assert step(guard, .75).active


def test_missing_samples_and_jumping_distance_reset_confirmation():
  guard = TeslaTrafficStop()
  assert not step(guard).active
  assert not step(guard, .7).active
  assert not step(guard, .8, distance=30).active
  assert not step(guard, .9, distance=30).active
  assert step(guard, 1., distance=30).active


def test_green_releases_only_after_unique_continuous_same_target_evidence():
  guard = TeslaTrafficStop()
  assert establish(guard).active
  for t in (.25, .35, .45, .55, .65):
    assert step(guard, t, light="green").active
  released = step(guard, .75, light="green")
  assert not released.active and released.reason == "green_released"
  assert not step(guard, .8, light="green").hold


def test_a_single_green_flicker_or_cached_green_does_not_release():
  guard = TeslaTrafficStop()
  establish(guard)
  assert step(guard, .25, light="green").active
  assert step(guard, .3).active
  for i in range(10):
    cs = state(sample=N + 350_000_000, light="green")
    result = guard.update(cs, N + 350_000_000 + i * 50_000_000, engaged=True, control_active=True)
    assert result.active


@pytest.mark.parametrize("speed,late", [(50 / 3.6, True), (80 / 3.6, True), (20 / 3.6, False)])
def test_late_green_to_red_admits_stop_and_warns_without_raising_braking_limits(speed, late):
  result = establish(TeslaTrafficStop(), distance=30, speed=speed)
  assert result.active and result.warn is late and not result.fault
  assert result.required_decel > 0


@pytest.mark.parametrize("light", ["unknown", "yellow"])
def test_unknown_or_yellow_does_not_turn_owned_red_into_go(light):
  guard = TeslaTrafficStop()
  establish(guard)
  for i in range(1, 13):
    result = step(guard, .2 + .05 * i, light=light)
    assert result.active
  assert result.fault and result.warn


def test_lost_signal_keeps_bounded_stop_target_and_requests_takeover():
  guard = TeslaTrafficStop()
  establish(guard, speed=5.)
  for i in range(1, 14):
    cs = state(sample=N + 200_000_000, distance=79., speed=5.)
    result = guard.update(cs, N + 200_000_000 + i * 50_000_000, engaged=True, control_active=True)
  assert result.active and result.fault and result.reason == "traffic_signal_lost"
  assert result.distance == pytest.approx(79 - 5 * .65)
  assert step(guard, .9, speed=5., distance=75.5, light="green").fault


def test_next_intersection_cannot_move_a_latched_stop_away_or_release_it():
  guard = TeslaTrafficStop()
  establish(guard, distance=10.)
  result = step(guard, .25, distance=100., light="green")
  assert result.active and result.fault and result.distance <= 10.
  assert result.reason == "stop_line_jump"


def test_driver_override_requires_disengagement_and_new_samples_before_rearming():
  guard = TeslaTrafficStop()
  establish(guard)
  cs = state(sample=N + 250_000_000)
  cs.gasPressed = True
  assert not guard.update(cs, N + 250_000_000, engaged=True, control_active=False).active
  for t in (.3, .4, .5):
    assert not step(guard, t).active
  guard.update(state(sample=N + 550_000_000), N + 550_000_000, engaged=False, control_active=False)
  for t in (.6, .7):
    assert not step(guard, t).active
  assert step(guard, .8).active


@pytest.mark.parametrize("field,value", [
  ("gearShifter", "reverse"), ("canValid", False), ("vehicleSensorsInvalid", True),
  ("doorOpen", True), ("seatbeltUnlatched", True), ("parkingBrake", True),
  ("accFaulted", True), ("espDisabled", True), ("vEgo", float("nan")), ("brakePressed", True),
])
def test_driver_and_vehicle_guards_revoke_the_constraint(field, value):
  guard = TeslaTrafficStop()
  establish(guard)
  cs = state(sample=N + 250_000_000)
  setattr(cs, field, value)
  assert not guard.update(cs, N + 250_000_000, engaged=True, control_active=True).active


def test_stationary_owned_red_holds_at_zero_but_zero_cannot_start_a_new_episode():
  guard = TeslaTrafficStop()
  assert establish(guard, distance=1.).hold
  result = step(guard, .25, distance=0.)
  assert result.active and result.hold
  assert not establish(TeslaTrafficStop(), distance=0.).active


@pytest.mark.parametrize("time", [.1, 1.])
def test_clock_discontinuity_cannot_release_an_owned_stop(time):
  guard = TeslaTrafficStop()
  establish(guard)
  result = step(guard, time)
  assert result.active and result.fault and result.reason == "clock_discontinuity"


def test_passed_target_is_not_replayed_as_a_new_stop_inside_intersection():
  guard = TeslaTrafficStop()
  establish(guard, distance=1.)
  for i in range(1, 20):
    cs = state(sample=N + 200_000_000, distance=1., speed=10.)
    result = guard.update(cs, N + 200_000_000 + i * 50_000_000, engaged=True, control_active=True)
  assert not result.active and result.fault and result.reason == "passed_stop_line"
  assert not step(guard, 1.2, distance=100.).active
