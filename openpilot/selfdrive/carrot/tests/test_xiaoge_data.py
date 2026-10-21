import json

from openpilot.selfdrive.carrot.xiaoge_data import XiaogeDataBroadcaster


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
