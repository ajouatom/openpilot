import json
from types import SimpleNamespace

import pytest
from opendbc.can import CANPacker, CANParser

from openpilot.cereal import car, log
from openpilot.selfdrive.carrot.xiaoge_data import XiaogeDataBroadcaster


N = 10_000_000_000


def test_tesla_initialization_selects_the_reference_hw4_dbc_without_opening_real_sockets(monkeypatch):
  broadcaster = XiaogeDataBroadcaster.__new__(XiaogeDataBroadcaster)
  cp = car.CarParams.new_message(brand="tesla")
  broadcaster.params = SimpleNamespace(get=lambda key: cp.to_bytes())
  broadcaster.car_brand_checked = False
  sock = object()
  monkeypatch.setattr("openpilot.selfdrive.carrot.xiaoge_data.messaging.sub_sock", lambda service: sock)
  broadcaster.initialize_tesla_can()
  assert broadcaster.tesla_can_sock is sock
  assert broadcaster.tesla_can_parser.dbc_name == "tesla_modely_hw4_perception"
  assert broadcaster.tesla_can_parser.bus == 2
  assert 605 in broadcaster.tesla_can_parser.addresses


@pytest.fixture
def receiver(monkeypatch):
  broadcaster = XiaogeDataBroadcaster.__new__(XiaogeDataBroadcaster)
  broadcaster.car_brand_checked = True
  broadcaster.is_tesla = True
  broadcaster.tesla_can_sock = object()
  broadcaster.tesla_can_parser = CANParser("tesla_modely_hw4_perception", [("APP_trafficControl", float("nan"))], 2)
  broadcaster.tesla_das_road_updated_at = 0
  broadcaster.tesla_traffic_status = None
  state = SimpleNamespace(broadcaster=broadcaster, events=[], now=N)

  def drain(sock):
    events = state.events[:]
    state.events.clear()
    return events

  monkeypatch.setattr("openpilot.selfdrive.carrot.xiaoge_data.messaging.drain_sock", drain)
  monkeypatch.setattr("openpilot.selfdrive.carrot.xiaoge_data.time.clock_gettime_ns", lambda clock: state.now)
  return state


def traffic_event(values=None, *, bus=2, timestamp=N, dlc=8):
  fields = {"APP_tcControlType": 3, "APP_tcControlDistance": 80, "APP_tcControlLightState": 1}
  address, data, source = CANPacker("tesla_modely_hw4_perception").make_can_msg("APP_trafficControl", bus, fields | (values or {}))
  assert data[2] == (fields | (values or {}))["APP_tcControlDistance"]
  data = data[:dlc] + bytes(max(0, dlc - len(data)))
  event = log.Event.new_message(logMonoTime=timestamp)
  event.init("can", 1)
  event.can[0].address, event.can[0].dat, event.can[0].src = address, data, source
  return event


@pytest.mark.parametrize("light", [0, 1, 2, 3])
@pytest.mark.parametrize("dlc", [6, 7, 8])
def test_hw4_traffic_broadcast_preserves_metres_and_color(receiver, light, dlc):
  receiver.events.append(traffic_event({"APP_tcControlLightState": light}, dlc=dlc))
  assert receiver.broadcaster.collect_broadcast_data() == {"carState": {"stopLineDist": 80, "trafficLightColor": light}}


@pytest.mark.parametrize("values", [
  {"APP_tcControlType": 0}, {"APP_tcControlType": 2}, {"APP_tcControlType": 4},
  {"APP_tcControlLightState": 4}, {"APP_tcControlLightState": 5},
  {"APP_tcControlLightState": 6}, {"APP_tcControlLightState": 7},
  {"APP_tcControlDistance": 201}, {"APP_tcControlDistance": 255},
])
def test_invalid_traffic_observation_immediately_invalidates_cached_color(receiver, values):
  receiver.events.append(traffic_event())
  assert receiver.broadcaster.collect_broadcast_data() is not None
  receiver.events.append(traffic_event(values, timestamp=N + 1))
  receiver.now += 1
  assert receiver.broadcaster.collect_broadcast_data() is None
  assert receiver.broadcaster.tesla_traffic_status == "invalid_traffic_observation"


@pytest.mark.parametrize("dlc", [0, 1, 3, 5, 9, 12])
def test_short_or_unexpected_dlc_never_publishes_partial_traffic(receiver, dlc):
  receiver.events.append(traffic_event(dlc=dlc))
  assert receiver.broadcaster.collect_broadcast_data() is None
  assert receiver.broadcaster.tesla_traffic_status == "invalid_dlc"


@pytest.mark.parametrize("bus", [0, 1, 0x82, 0xC2])
def test_other_buses_and_tx_echoes_are_not_authoritative_traffic(receiver, bus):
  receiver.events.append(traffic_event(bus=bus))
  assert receiver.broadcaster.collect_broadcast_data() is None


@pytest.mark.parametrize("age", [-1, 1_000_000_001, N])
def test_queued_old_future_or_zero_timestamp_does_not_become_fresh_at_receipt(receiver, age):
  receiver.events.append(traffic_event(timestamp=N - age))
  assert receiver.broadcaster.collect_broadcast_data() is None


def test_cached_traffic_expires_and_older_green_cannot_replace_newer_red(receiver):
  receiver.events.append(traffic_event())
  assert receiver.broadcaster.collect_broadcast_data()["carState"]["trafficLightColor"] == 1
  receiver.events.append(traffic_event({"APP_tcControlLightState": 2}, timestamp=N - 1))
  assert receiver.broadcaster.collect_broadcast_data()["carState"]["trafficLightColor"] == 1
  receiver.now += 1_000_000_001
  assert receiver.broadcaster.collect_broadcast_data() is None


def test_last_frame_in_same_timestamp_batch_controls_validity(receiver):
  receiver.events.extend([traffic_event(), traffic_event({"APP_tcControlType": 0})])
  assert receiver.broadcaster.collect_broadcast_data() is None
  receiver.events.append(traffic_event({"APP_tcControlLightState": 2}))
  assert receiver.broadcaster.collect_broadcast_data()["carState"]["trafficLightColor"] == 2


def test_unavailable_traffic_logs_status_changes_without_polling_spam(receiver, monkeypatch):
  warnings = []
  monkeypatch.setattr("openpilot.selfdrive.carrot.xiaoge_data.carlog.warning", warnings.append)
  for _ in range(5):
    assert receiver.broadcaster.collect_broadcast_data() is None
  assert len(warnings) == 1 and "stale" in warnings[0]


def test_broadcast_data_contains_only_tesla_das_road_fields():
  broadcaster = XiaogeDataBroadcaster.__new__(XiaogeDataBroadcaster)
  tesla_data = {"stopLineDist": 42.5, "trafficLightColor": 2}
  broadcaster.collect_tesla_das_road = lambda: tesla_data

  data = broadcaster.collect_broadcast_data()

  assert data == {"carState": tesla_data}


def test_broadcast_data_is_absent_without_fresh_tesla_data():
  broadcaster = XiaogeDataBroadcaster.__new__(XiaogeDataBroadcaster)
  broadcaster.collect_tesla_das_road = lambda: None

  assert broadcaster.collect_broadcast_data() is None


def test_packet_keeps_protocol_envelope_and_only_contains_selected_data():
  broadcaster = XiaogeDataBroadcaster.__new__(XiaogeDataBroadcaster)
  broadcaster.sequence = 7
  broadcaster.device_ip = "127.0.0.1"

  packet = json.loads(broadcaster.create_packet({
    "carState": {"stopLineDist": 42.5, "trafficLightColor": 2},
  }))

  assert packet["version"] == 1
  assert packet["sequence"] == 7
  assert packet["ip"] == "127.0.0.1"
  assert packet["data"] == {
    "carState": {"stopLineDist": 42.5, "trafficLightColor": 2},
  }
