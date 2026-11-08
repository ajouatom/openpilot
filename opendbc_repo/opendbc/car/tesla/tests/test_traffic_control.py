from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.traffic_control import TeslaTrafficControl
from opendbc.car.tesla.values import CAR, TeslaFlags

N = 10_000_000_000


def frame(**values):
  fields = dict(APP_tcControlType=3, APP_tcFeatureState=2, APP_tcControlSource=2,
                APP_tcVisionLight=1, APP_tcControlDistance=80, APP_tcControlLightState=1)
  fields.update(values)
  return CANPacker("tesla_modely_hw4_perception").make_can_msg("APP_trafficControl", 2, fields)[1]


@pytest.fixture
def observer(monkeypatch):
  monkeypatch.setattr("opendbc.car.tesla.traffic_control.boot_time_ns", lambda: N)
  return TeslaTrafficControl()


def published(observer, now=N):
  cs = structs.CarState()
  observer.publish(cs.teslaTrafficControl, now)
  return cs.teslaTrafficControl


@pytest.mark.parametrize("color,expected", [(1, "red"), (2, "green"), (3, "yellow")])
def test_exact_app_profile_publishes_metres_color_and_original_timestamp(observer, color, expected):
  observer.observe(frame(APP_tcControlLightState=color), 2, N)
  result = published(observer)
  assert result.valid and result.available
  assert (result.stopLineDistance, result.light, result.sampleMonoTime) == (80, expected, N)
  assert not published(observer, N + 500_000_001).valid
  assert published(observer, N + 500_000_001).sampleMonoTime == N


@pytest.mark.parametrize("dlc", [0, 4, 6, 7, 9, 12])
def test_legacy_or_partial_profile_never_drives_stopping(observer, dlc):
  observer.observe(frame()[:dlc] + bytes(max(0, dlc - 8)), 2, N)
  assert not published(observer).valid
  assert published(observer).status == "unsupported_dlc"


@pytest.mark.parametrize("values", [
  {"APP_tcControlType": 2}, {"APP_tcControlType": 0},
  {"APP_tcFeatureState": 0}, {"APP_tcFeatureState": 1},
  {"APP_tcControlSource": 0}, {"APP_tcControlSource": 1}, {"APP_tcVisionLight": 0},
  {"APP_tcUnavailableReason": 1}, {"APP_tcWarningSuppressionReason": 1},
  {"APP_tcControlLightState": 0}, {"APP_tcControlLightState": 4}, {"APP_tcControlLightState": 7},
  {"APP_tcControlDistance": 255},
])
def test_invalid_or_map_only_signal_does_not_inherit_previous_red(observer, values):
  observer.observe(frame(), 2, N - 1)
  assert published(observer).valid
  observer.observe(frame(**values), 2, N)
  assert not published(observer).valid


@pytest.mark.parametrize("bus", [0, 1, 0x82, 0xC2])
def test_other_bus_and_tx_echo_are_not_vehicle_observations(observer, bus):
  observer.observe(frame(), bus, N)
  assert not published(observer).available


@pytest.mark.parametrize("sample", [0, N + 1, N - 500_000_001])
def test_bad_receive_time_never_becomes_fresh_on_publish(observer, sample):
  observer.observe(frame(), 2, sample)
  assert not published(observer).valid


def test_older_green_cannot_replace_red_and_same_stamp_conflict_is_invalid(observer):
  observer.observe(frame(), 2, N)
  observer.observe(frame(APP_tcControlLightState=2), 2, N - 1)
  assert published(observer).light == "red"
  observer.observe(frame(APP_tcControlLightState=2), 2, N)
  assert not published(observer).valid


@pytest.mark.parametrize("candidate", [CAR.TESLA_MODEL_3, CAR.TESLA_MODEL_Y, CAR.TESLA_MODEL_X])
@pytest.mark.parametrize("enabled", [False, True])
@pytest.mark.parametrize("longitudinal", [False, True])
def test_startup_switch_is_scoped_without_new_panda_permission(monkeypatch, candidate, enabled, longitudinal):
  def get_params():
    if candidate == CAR.TESLA_MODEL_X:
      return CarInterface._get_params(structs.CarParams(carFingerprint=candidate), candidate,
                                      gen_empty_fingerprint(), [], longitudinal, False, False)
    return CarInterface.get_params(candidate, gen_empty_fingerprint(), [], longitudinal, False, False)

  monkeypatch.setattr("opendbc.car.tesla.interface.Params", lambda: SimpleNamespace(
    get_bool=lambda key: enabled if key == "TeslaCanTrafficStop" else False))
  cp = get_params()
  assert bool(cp.flags & TeslaFlags.CAN_TRAFFIC_STOP) == (
    enabled and longitudinal and candidate in (CAR.TESLA_MODEL_3, CAR.TESLA_MODEL_Y))
  before = cp.safetyConfigs[0].safetyParam
  monkeypatch.setattr("opendbc.car.tesla.interface.Params", lambda: SimpleNamespace(get_bool=lambda key: False))
  original = get_params()
  assert original.safetyConfigs[0].safetyParam == before
  cs = CarState(cp)
  assert (cs.traffic_control is not None) == bool(cp.flags & TeslaFlags.CAN_TRAFFIC_STOP)


def test_production_interface_observes_app_without_modifying_other_parsers(monkeypatch):
  monkeypatch.setattr("opendbc.car.tesla.traffic_control.boot_time_ns", lambda: N)
  monkeypatch.setattr("opendbc.car.tesla.carstate.time.clock_gettime_ns", lambda clock: N)
  monkeypatch.setattr("opendbc.car.tesla.interface.Params", lambda: SimpleNamespace(
    get_bool=lambda key: key == "TeslaCanTrafficStop"))
  cp = CarInterface.get_params(CAR.TESLA_MODEL_Y, gen_empty_fingerprint(), [], True, False, False)
  interface = CarInterface(cp)
  original_cp = cp.as_reader().as_builder()
  original_cp.flags = int(cp.flags) & ~int(TeslaFlags.CAN_TRAFFIC_STOP)
  original = CarInterface(original_cp)
  before = original.update([(N, [(0x25D, frame(), 2)])])
  state = interface.update([(N, [(0x25D, frame(), 2)])])
  assert state.teslaTrafficControl.valid
  assert state.teslaTrafficControl.stopLineDistance == 80
  assert state.canValid == before.canValid
  assert {bus: set(parser.addresses) for bus, parser in interface.can_parsers.items()} == {
    bus: set(parser.addresses) for bus, parser in original.can_parsers.items()}
