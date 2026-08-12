import asyncio
import errno
import json
import struct
from types import SimpleNamespace

import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from openpilot.selfdrive.carrot.carrot_navi import (
  BINARY_HEADER,
  CATALOG,
  CLUSTER_ENABLED_IMAGE_NAMES,
  CarrotNaviDiscoveryBeacon,
  ClusterNaviMapParamReader,
  CarrotNaviReceiver,
  DISCOVERY_PORT,
  PROTOCOL_VERSION,
  build_manifest,
  create_app,
  discovery_targets,
  parse_binary_packet,
  resolve_map_bitrate_kbps,
  resolve_map_hz,
  run_receiver_app,
  _safe_send_json,
)
from openpilot.selfdrive.carrot.carrot_navi_cereal import CarrotNaviCerealPublisher, build_carrot_navi_payload


def requirements_query() -> dict:
  return {
    "type": "requirements_query",
    "protocol_version": PROTOCOL_VERSION,
    "app_version": "test-app",
    "catalog_revision": 1,
    "streams": [
      {
        "kind": kind,
        "name": name,
        "schema_version": 1,
      }
      for kind, name in CATALOG
    ],
  }


def test_standalone_discovery_advertises_new_navi_endpoint(monkeypatch):
  sent = []

  class FakeSocket:
    def setsockopt(self, *args):
      pass

    def sendto(self, body, address):
      sent.append((body, address))

    def bind(self, address):
      assert address == ("192.168.0.10", 0)

    def close(self):
      pass

  monkeypatch.setattr("openpilot.selfdrive.carrot.carrot_navi.socket.socket", lambda *args: FakeSocket())

  CarrotNaviDiscoveryBeacon("192.168.0.10").broadcast_once()

  assert len(sent) == 1
  body, address = sent[0]
  assert json.loads(body) == {"ip": "192.168.0.10", "navi_debug": 1}
  assert address == ("255.255.255.255", DISCOVERY_PORT)


def test_discovery_advertises_each_ubuntu_network_interface(monkeypatch):
  monkeypatch.setattr(
    "openpilot.selfdrive.carrot.carrot_navi._interface_ipv4_addresses",
    lambda: (
      ("192.168.10.5", "255.255.255.0", "192.168.10.255"),
      ("10.42.0.1", "255.255.255.0", None),
    ),
  )

  assert discovery_targets() == (
    ("192.168.10.5", "192.168.10.255"),
    ("10.42.0.1", "10.42.0.255"),
  )


def test_receiver_app_retries_temporary_port_conflict(monkeypatch):
  calls = []

  def fake_run_app(app, **kwargs):
    calls.append((app, kwargs))
    if len(calls) == 1:
      raise OSError(errno.EADDRINUSE, "address already in use")

  monkeypatch.setattr("openpilot.selfdrive.carrot.carrot_navi.web.run_app", fake_run_app)
  run_receiver_app(CarrotNaviReceiver(), "0.0.0.0", 7714, retry_count=1, retry_interval_s=0.0)

  assert len(calls) == 2
  assert calls[0][0] is not calls[1][0]
  assert calls[1][1]["host"] == "0.0.0.0"
  assert calls[1][1]["port"] == 7714


def test_receiver_app_does_not_retry_other_start_errors(monkeypatch):
  def fake_run_app(app, **kwargs):
    raise OSError(errno.EACCES, "permission denied")

  monkeypatch.setattr("openpilot.selfdrive.carrot.carrot_navi.web.run_app", fake_run_app)
  with pytest.raises(OSError) as exc_info:
    run_receiver_app(CarrotNaviReceiver(), "0.0.0.0", 7714, retry_count=2, retry_interval_s=0.0)

  assert exc_info.value.errno == errno.EACCES


def test_manifest_disables_only_cluster_unused_images():
  manifest = build_manifest("12345678")

  assert manifest["type"] == "subscription_manifest"
  assert manifest["protocol_version"] == 2
  assert len(manifest["streams"]) == 28
  enabled = {
    (stream["kind"], stream["name"])
    for stream in manifest["streams"]
    if stream["enabled"]
  }
  assert enabled == set(CATALOG) - {("image", "lane_top")}
  assert CLUSTER_ENABLED_IMAGE_NAMES == {
    "tbt_current_compact", "tbt_current_full", "tbt_next",
    "traffic_signal", "lane_bottom",
    "safety_primary", "safety_secondary", "safety_section",
    "crossroad_minimized", "crossroad_expanded",
    "center_tbt_icon", "center_tbt_text", "center_tbt_fee",
  }
  assert [stream["stream_handle"] for stream in manifest["streams"]] == list(range(1, 29))
  map_stream = next(stream for stream in manifest["streams"] if stream["kind"] == "render")
  assert map_stream["params"]["map_theme"] == "auto"
  assert map_stream["params"]["screen_center_y_ratio"] == 0.8

  invalid_requirements = requirements_query()
  invalid_requirements["catalog_revision"] = 2
  with pytest.raises(ValueError, match="catalog revision"):
    CarrotNaviReceiver().negotiate(invalid_requirements, "test-app")


def test_manifest_requests_configured_map_appearance():
  manifest = build_manifest(
    "12345678",
    map_theme="dark",
    map_type="satellite",
    map_hz=60,
    map_bitrate_kbps=12000,
    screen_center_y_ratio=0.68,
  )
  map_stream = next(stream for stream in manifest["streams"] if stream["kind"] == "render")
  crossroad_streams = [
    stream for stream in manifest["streams"]
    if stream["kind"] == "image" and stream["name"].startswith("crossroad_")
  ]

  assert map_stream["params"]["map_theme"] == "dark"
  assert map_stream["params"]["map_type"] == "satellite"
  assert map_stream["params"]["fps"] == 60
  assert map_stream["params"]["h264_bitrate_kbps"] == 12000
  assert map_stream["params"]["screen_center_y_ratio"] == 0.68
  assert len(crossroad_streams) == 2
  assert all(stream["params"]["theme"] == "dark" for stream in crossroad_streams)

  with pytest.raises(ValueError, match="map theme"):
    build_manifest("12345678", map_theme="neon")
  with pytest.raises(ValueError, match="map type"):
    build_manifest("12345678", map_type="terrain")
  with pytest.raises(ValueError, match="refresh rate"):
    build_manifest("12345678", map_hz=61)
  with pytest.raises(ValueError, match="bitrate"):
    build_manifest("12345678", map_bitrate_kbps=12001)
  with pytest.raises(ValueError, match="screen center ratio"):
    build_manifest("12345678", screen_center_y_ratio=0.49)


def test_cluster_navi_map_params_and_receiver_updates_next_manifest():
  class FakeParams:
    values = {
      "ClusterNaviMapTheme": 2,
      "ClusterNaviMapType": 1,
      "ClusterNaviMapFps": 3,
      "CarrotNaviHudMapProfile": 1,
    }

    def get_int(self, key):
      return self.values[key]

  reader = ClusterNaviMapParamReader(FakeParams())
  receiver = CarrotNaviReceiver(
    map_theme="dark", map_type="normal", map_hz=10, map_bitrate_kbps=3000,
  )

  assert reader() == ("light", "satellite", 30, 6000, 0.68)
  assert receiver.set_map_config(*reader()) is True
  assert receiver.set_map_config(*reader()) is False

  manifest = receiver.negotiate(requirements_query(), "test-app")
  map_stream = next(stream for stream in manifest["streams"] if stream["kind"] == "render")
  assert map_stream["params"]["map_theme"] == "light"
  assert map_stream["params"]["map_type"] == "satellite"
  assert map_stream["params"]["fps"] == 30
  assert map_stream["params"]["h264_bitrate_kbps"] == 6000
  assert map_stream["params"]["screen_center_y_ratio"] == 0.68
  assert receiver.health()["map_theme"] == "light"
  assert receiver.health()["map_type"] == "satellite"
  assert receiver.health()["map_hz"] == 30
  assert receiver.health()["map_bitrate_kbps"] == 6000
  assert receiver.health()["screen_center_y_ratio"] == 0.68


def test_cluster_navi_map_fps_modes_and_automatic_bitrate_resolution():
  assert resolve_map_hz(0) == 5
  assert resolve_map_hz(1) == 10
  assert resolve_map_hz(2) == 20
  assert resolve_map_hz(3) == 30
  assert resolve_map_hz(6) == 10
  assert resolve_map_hz(None) == 10

  assert resolve_map_bitrate_kbps(960, 540, 5) == 1500
  assert resolve_map_bitrate_kbps(960, 540, 10) == 3000
  assert resolve_map_bitrate_kbps(960, 540, 20) == 3000
  assert resolve_map_bitrate_kbps(960, 540, 30) == 6000
  assert resolve_map_bitrate_kbps(480, 270, 30) == 1500
  assert resolve_map_bitrate_kbps(1920, 1080, 30) == 12000


def test_receiver_logs_changed_tbt_and_sdi_values(capsys):
  receiver = CarrotNaviReceiver()
  manifest = receiver.negotiate(requirements_query(), "test-app")
  streams = {stream["name"]: stream for stream in manifest["streams"] if stream["kind"] == "json"}

  def record(name, sequence, value):
    stream = streams[name]
    receiver.record_json(manifest["session_id"], name, {
      "type": "item_update",
      "protocol_version": 2,
      "session_id": manifest["session_id"],
      "manifest_revision": manifest["revision"],
      "schema_version": 1,
      "kind": "json",
      "name": name,
      "stream_handle": stream["stream_handle"],
      "sequence": sequence,
      "source_timestamp_ms": 1000 + sequence,
      "sent_at_ms": 2000 + sequence,
      "present": True,
      "value": value,
    }, "192.168.0.171")

  current = {"distance_m": 320, "turn_type": 12, "main_text": "Turn left"}
  record("guidance_current", 1, current)
  record("guidance_current", 2, current)
  record("guidance_next", 1, {"distance_m": 950, "turn_type": 6, "main_text": "Turn right"})
  record("speed", 1, {
    "current_kph": 48,
    "road_limit_kph": 50,
    "sdi": {"type": 1, "distance_m": 420, "speed_limit_kph": 50},
  })

  output = capsys.readouterr().out
  assert output.count("[carrot_navi][TBT current]") == 1
  assert "[carrot_navi][TBT next]" in output
  assert '"main_text":"Turn left"' in output
  assert "[carrot_navi][SDI]" in output
  assert '"distance_m":420' in output


def test_receiver_preserves_latest_speed_bump_and_secondary_sdi_fields():
  receiver = CarrotNaviReceiver()
  manifest = receiver.negotiate(requirements_query(), "test-app")
  speed_stream = next(
    stream for stream in manifest["streams"]
    if stream["kind"] == "json" and stream["name"] == "speed"
  )
  app_status_stream = next(
    stream for stream in manifest["streams"]
    if stream["kind"] == "json" and stream["name"] == "app_status"
  )
  value = {
    "current_kph": 0,
    "road_limit_kph": 30,
    "sdi": {"type": 22, "distance_m": 93},
    "sdi_secondary": {"type": 1, "distance_m": 420, "future_field": "kept"},
    "future_group_field": {"enabled": True},
  }
  receiver.record_json(manifest["session_id"], "speed", {
    "type": "item_update",
    "protocol_version": 2,
    "session_id": manifest["session_id"],
    "manifest_revision": manifest["revision"],
    "schema_version": 1,
    "kind": "json",
    "name": "speed",
    "stream_handle": speed_stream["stream_handle"],
    "sequence": 1,
    "source_timestamp_ms": 1234,
    "sent_at_ms": 1235,
    "present": True,
    "value": value,
  }, "127.0.0.1")
  receiver.record_json(manifest["session_id"], "app_status", {
    "type": "item_update",
    "protocol_version": 2,
    "session_id": manifest["session_id"],
    "manifest_revision": manifest["revision"],
    "schema_version": 1,
    "kind": "json",
    "name": "app_status",
    "stream_handle": app_status_stream["stream_handle"],
    "sequence": 1,
    "source_timestamp_ms": 1236,
    "sent_at_ms": 1237,
    "present": True,
    "value": {"foreground": False, "window_focused": False},
  }, "127.0.0.1")
  value["sdi_secondary"]["future_field"] = "changed-after-receive"

  received = receiver.latest()["items"]["json:speed"]

  assert received["sent_at_ms"] == 1235
  assert received["value"]["sdi"]["type"] == 22
  assert received["value"]["sdi_secondary"] == {
    "type": 1, "distance_m": 420, "future_field": "kept",
  }
  assert received["value"]["future_group_field"] == {"enabled": True}
  assert receiver.health()["app_foreground"] is False
  assert receiver.latest()["app_foreground"] is False


def test_receiver_validates_json_envelope_and_value_shapes():
  receiver = CarrotNaviReceiver()
  manifest = receiver.negotiate(requirements_query(), "test-app")
  streams = {
    stream["name"]: stream for stream in manifest["streams"]
    if stream["kind"] == "json"
  }

  def envelope(name, value, **changes):
    stream = streams[name]
    result = {
      "type": "item_update",
      "protocol_version": 2,
      "session_id": manifest["session_id"],
      "manifest_revision": manifest["revision"],
      "schema_version": 1,
      "kind": "json",
      "name": name,
      "stream_handle": stream["stream_handle"],
      "sequence": 1,
      "source_timestamp_ms": 1000,
      "sent_at_ms": 1001,
      "present": True,
      "value": value,
    }
    result.update(changes)
    return result

  with pytest.raises(ValueError, match="sent_at_ms"):
    invalid = envelope("speed", {})
    del invalid["sent_at_ms"]
    receiver.record_json(manifest["session_id"], "speed", invalid, "127.0.0.1")
  with pytest.raises(ValueError, match="must be an object"):
    receiver.record_json(
      manifest["session_id"], "speed", envelope("speed", []), "127.0.0.1",
    )
  with pytest.raises(ValueError, match="array of objects"):
    receiver.record_json(
      manifest["session_id"], "lane_ahead", envelope("lane_ahead", {}), "127.0.0.1",
    )
  with pytest.raises(ValueError, match="valid reason"):
    receiver.record_json(
      manifest["session_id"], "speed",
      envelope("speed", None, present=False, reason=""), "127.0.0.1",
    )


def test_safe_websocket_error_send_ignores_closing_transport():
  class ClosingWebSocket:
    closed = False

    async def send_json(self, _payload):
      raise ConnectionResetError("Cannot write to closing transport")

  assert asyncio.run(_safe_send_json(ClosingWebSocket(), {"error": True})) is False


def test_map_param_change_reconnects_websocket_with_new_manifest():
  map_config = ["dark", "normal", 10, 3000]

  async def scenario():
    receiver = CarrotNaviReceiver(
      map_theme=map_config[0], map_type=map_config[1],
      map_hz=map_config[2], map_bitrate_kbps=map_config[3],
    )
    client = TestClient(TestServer(create_app(receiver, lambda: tuple(map_config))))
    await client.start_server()
    first_control = None
    second_control = None
    try:
      first_control = await client.ws_connect("/api/navi/ws/v2/control/10.0.0")
      await first_control.send_json(requirements_query())
      first_manifest = await first_control.receive_json()
      first_map = next(stream for stream in first_manifest["streams"] if stream["kind"] == "render")
      assert first_map["params"]["map_type"] == "normal"

      map_config[:] = ["light", "satellite", 60, 12000]
      message = await first_control.receive(timeout=2.0)
      assert message.type in (WSMsgType.CLOSE, WSMsgType.CLOSED)

      second_control = await client.ws_connect("/api/navi/ws/v2/control/10.0.0")
      await second_control.send_json(requirements_query())
      second_manifest = await second_control.receive_json()
      second_map = next(stream for stream in second_manifest["streams"] if stream["kind"] == "render")
      assert second_map["params"]["map_theme"] == "light"
      assert second_map["params"]["map_type"] == "satellite"
      assert second_map["params"]["fps"] == 60
      assert second_map["params"]["h264_bitrate_kbps"] == 12000
    finally:
      if second_control is not None:
        await second_control.close()
      if first_control is not None:
        await first_control.close()
      await client.close()

  asyncio.run(scenario())


def test_parse_binary_packet_validates_cn_v2_payload():
  png = b"\x89PNG\r\n\x1a\ncontent"
  packet = BINARY_HEADER.pack(
    b"CNV2", 2, 1, 1, 1, 14, 1, 3, 1000, len(png), 32, 24,
  ) + png

  metadata, payload = parse_binary_packet(packet)

  assert metadata["stream_handle"] == 14
  assert metadata["sequence"] == 3
  assert metadata["width"] == 32
  assert payload == png

  invalid = bytearray(packet)
  struct.pack_into(">I", invalid, 32, len(png) + 1)
  with pytest.raises(ValueError, match="payload length"):
    parse_binary_packet(bytes(invalid))


def test_receiver_rejects_binary_formats_that_do_not_match_stream_kind():
  receiver = CarrotNaviReceiver()
  manifest = receiver.negotiate(requirements_query(), "test-app")
  image_stream = next(
    stream for stream in manifest["streams"]
    if stream["kind"] == "image" and stream["name"] == "tbt_next"
  )
  render_stream = next(
    stream for stream in manifest["streams"]
    if stream["kind"] == "render" and stream["name"] == "map_main"
  )

  def metadata(stream, format_code):
    return {
      "stream_handle": stream["stream_handle"],
      "manifest_revision": manifest["revision"],
      "sequence": 1,
      "source_timestamp_ms": 1000,
      "message_type": 1,
      "format_or_reason": format_code,
      "flags": 1,
      "width": 32,
      "height": 24,
    }

  with pytest.raises(ValueError, match="requires PNG"):
    receiver.record_binary(
      manifest["session_id"], "image", "tbt_next",
      metadata(image_stream, 2), b"jpeg", "127.0.0.1",
    )
  with pytest.raises(ValueError, match="requires JPEG"):
    receiver.record_binary(
      manifest["session_id"], "render", "map_main",
      metadata(render_stream, 1), b"png", "127.0.0.1",
    )


def test_dashboard_snapshot_retains_render_codec_config():
  receiver = CarrotNaviReceiver()
  manifest = receiver.negotiate(requirements_query(), "test-app")
  stream = next(
    item for item in manifest["streams"]
    if item["kind"] == "render" and item["name"] == "map_main"
  )
  identity = {
    "stream_handle": stream["stream_handle"],
    "manifest_revision": manifest["revision"],
    "source_timestamp_ms": 1000,
    "format_or_reason": 3,
    "flags": 0,
    "width": 960,
    "height": 540,
  }
  receiver.record_binary(
    manifest["session_id"], "render", "map_main",
    {**identity, "message_type": 2, "sequence": 1},
    b"\x00\x00\x00\x01\x67config", "127.0.0.1",
  )
  receiver.record_binary(
    manifest["session_id"], "render", "map_main",
    {**identity, "message_type": 3, "sequence": 2, "flags": 1},
    b"\x00\x00\x00\x01\x65keyframe", "127.0.0.1",
  )
  receiver.record_binary(
    manifest["session_id"], "render", "map_main",
    {**identity, "message_type": 3, "sequence": 3},
    b"\x00\x00\x00\x01\x41delta", "127.0.0.1",
  )

  snapshot = receiver.dashboard_snapshot()

  assert snapshot["media_generation"] == 3
  assert snapshot["records"]["render:map_main"].sequence == 3
  assert snapshot["binary_configs"]["render:map_main"].sequence == 1
  assert snapshot["binary_configs"]["render:map_main"].payload.endswith(b"config")

  updates = receiver.drain_media_updates()
  assert [record.sequence for record in updates] == [1, 2, 3]
  assert receiver.drain_media_updates() == []
  assert [record.sequence for record in receiver.media_bootstrap()] == [1, 2]

  receiver.record_binary(
    manifest["session_id"], "render", "map_main",
    {**identity, "message_type": 2, "sequence": 4},
    b"\x00\x00\x00\x01\x67new-config", "127.0.0.1",
  )
  assert [record.sequence for record in receiver.media_bootstrap()] == [4]


@pytest.mark.asyncio
async def test_websocket_negotiation_and_json_receive():
  receiver = CarrotNaviReceiver()
  client = TestClient(TestServer(create_app(receiver)))
  await client.start_server()
  control = None
  item = None
  try:
    control = await client.ws_connect("/api/navi/ws/v2/control/10.0.0")
    await control.send_json(requirements_query())
    manifest = await control.receive_json()

    assert manifest["type"] == "subscription_manifest"
    assert len(manifest["streams"]) == 28
    vehicle = next(
      stream for stream in manifest["streams"]
      if stream["kind"] == "json" and stream["name"] == "vehicle"
    )

    session_id = manifest["session_id"]
    item = await client.ws_connect(
      f"/api/navi/ws/v2/json/{session_id}/vehicle",
    )
    await item.send_json({
      "type": "item_update",
      "protocol_version": 2,
      "session_id": session_id,
      "manifest_revision": manifest["revision"],
      "schema_version": 1,
      "kind": "json",
      "name": "vehicle",
      "stream_handle": vehicle["stream_handle"],
      "sequence": 1,
      "source_timestamp_ms": 1234,
      "sent_at_ms": 1235,
      "present": True,
      "value": {"speed_kph": 42.0},
    })

    health = None
    for _ in range(50):
      response = await client.get("/health")
      health = await response.json()
      if health["session_received_count"] == 1:
        break
      await asyncio.sleep(0.01)

    assert health is not None
    assert health["port"] == 7714
    assert health["error"] is None
    assert health["items"]["json:vehicle"]["value"] == {"speed_kph": 42.0}
    assert health["state_generation"] >= 3
    assert next(
      stream for stream in manifest["streams"]
      if stream["kind"] == "image" and stream["name"] == "tbt_current_compact"
    )["enabled"] is True
    assert next(
      stream for stream in manifest["streams"]
      if stream["kind"] == "image" and stream["name"] == "lane_top"
    )["enabled"] is False
    with pytest.raises(ValueError, match="not enabled"):
      receiver.stream_config(session_id, "image", "lane_top")
  finally:
    if item is not None:
      await item.close()
    if control is not None:
      await control.close()
    await client.close()


@pytest.mark.asyncio
async def test_all_enabled_catalog_item_routes_receive_value_and_clear():
  receiver = CarrotNaviReceiver()
  client = TestClient(TestServer(create_app(receiver)))
  await client.start_server()
  control = None
  try:
    control = await client.ws_connect("/api/navi/ws/v2/control/10.0.0")
    await control.send_json(requirements_query())
    manifest = await control.receive_json()
    session_id = manifest["session_id"]

    enabled_streams = [stream for stream in manifest["streams"] if stream["enabled"]]
    for received_index, stream in enumerate(enabled_streams, start=1):
      kind = stream["kind"]
      name = stream["name"]
      item = await client.ws_connect(f"/api/navi/ws/v2/{kind}/{session_id}/{name}")
      try:
        if kind == "json":
          await item.send_json({
            "type": "item_update",
            "protocol_version": 2,
            "session_id": session_id,
            "manifest_revision": manifest["revision"],
            "schema_version": 1,
            "kind": kind,
            "name": name,
            "stream_handle": stream["stream_handle"],
            "sequence": 1,
            "source_timestamp_ms": 1000,
            "sent_at_ms": 1001,
            "present": True,
            "value": [] if name == "lane_ahead" else {"smoke": name},
          })
          await item.send_json({
            "type": "item_update",
            "protocol_version": 2,
            "session_id": session_id,
            "manifest_revision": manifest["revision"],
            "schema_version": 1,
            "kind": kind,
            "name": name,
            "stream_handle": stream["stream_handle"],
            "sequence": 2,
            "source_timestamp_ms": 1002,
            "sent_at_ms": 1003,
            "present": False,
            "value": None,
            "reason": "source_absent",
          })
        else:
          payload = b"\x89PNG\r\n\x1a\ncontent" if kind == "image" else b"\xff\xd8content\xff\xd9"
          await item.send_bytes(BINARY_HEADER.pack(
            b"CNV2", 2, 1, 1 if kind == "image" else 2, 0,
            stream["stream_handle"], manifest["revision"], 1, 1000,
            len(payload), 2, 2,
          ) + payload)
          await item.send_bytes(BINARY_HEADER.pack(
            b"CNV2", 2, 4, 1, 0,
            stream["stream_handle"], manifest["revision"], 2, 1002, 0, 0, 0,
          ))
        for _ in range(50):
          if receiver.health()["session_received_count"] >= received_index * 2:
            break
          await asyncio.sleep(0.002)
      finally:
        await item.close()

    health = receiver.health()
    assert health["error"] is None
    assert health["session_received_count"] == len(enabled_streams) * 2
    assert set(health["items"]) == {
      f"{stream['kind']}:{stream['name']}"
      for stream in enabled_streams
    }
    assert all(item["present"] is False for item in health["items"].values())
  finally:
    if control is not None:
      await control.close()
    await client.close()


def test_receiver_snapshot_builds_bounded_typed_payload():
  receiver = CarrotNaviReceiver()
  manifest = receiver.negotiate(requirements_query(), "test-app")
  receiver.control_connected()
  guidance = next(
    stream for stream in manifest["streams"]
    if stream["kind"] == "json" and stream["name"] == "guidance_current"
  )
  receiver.record_json(manifest["session_id"], "guidance_current", {
    "type": "item_update",
    "protocol_version": 2,
    "session_id": manifest["session_id"],
    "manifest_revision": manifest["revision"],
    "schema_version": 1,
    "kind": "json",
    "name": "guidance_current",
    "stream_handle": guidance["stream_handle"],
    "sequence": 7,
    "source_timestamp_ms": 1234,
    "sent_at_ms": 1235,
    "present": True,
    "value": {
      "distance_m": 320,
      "time_sec": 35,
      "turn_type": 12,
      "main_text": "Turn left",
    },
  }, "127.0.0.1")

  snapshot = receiver.cereal_snapshot()
  payload = build_carrot_navi_payload(snapshot, publish_mono_ns=999)

  assert snapshot["connected"] is True
  assert payload["schemaVersion"] == 1
  assert payload["publishMonoTimeNanos"] == 999
  assert payload["guidanceCurrent"]["meta"]["sequence"] == 7
  assert payload["guidanceCurrent"]["distanceM"] == 320
  assert payload["guidanceCurrent"]["mainText"] == "Turn left"
  assert payload["route"]["polyline"] == []


def test_payload_bounds_route_and_publisher_sends_dedicated_service():
  route_points = [
    {"lat": 37.0 + index * 0.00001, "lon": 127.0 + index * 0.00001}
    for index in range(300)
  ]
  snapshot = {
    "generation": 11,
    "session_id": "session",
    "connected": True,
    "items": {
      "route": {
        "present": True,
        "sequence": 3,
        "source_timestamp_ms": 1234,
        "received_mono_ns": 5678,
        "value": {"polyline": route_points},
      },
    },
  }

  class FakeMessage:
    carrotNavi = None
    carrotNaviMedia = None

  class FakePubMaster:
    def __init__(self, services):
      self.services = services
      self.sent = []

    def send(self, service, message):
      self.sent.append((service, message))

  class FakeMessaging:
    def __init__(self):
      self.pub_master = None

    def PubMaster(self, services):
      self.pub_master = FakePubMaster(services)
      return self.pub_master

    @staticmethod
    def new_message(service, valid):
      assert service in ("carrotNavi", "carrotNaviMedia")
      assert valid is True
      return FakeMessage()

  class FakeParams:
    @staticmethod
    def get_bool(_key):
      return False

  receiver = CarrotNaviReceiver()
  messaging = FakeMessaging()
  publisher = CarrotNaviCerealPublisher(receiver, messaging, params=FakeParams())

  assert publisher.publish_once(snapshot) == 11
  assert messaging.pub_master is not None
  assert messaging.pub_master.services == ["carrotNavi", "carrotNaviMedia"]
  assert len(messaging.pub_master.sent) == 1
  service, message = messaging.pub_master.sent[0]
  assert service == "carrotNavi"
  assert len(message.carrotNavi["route"]["polyline"]) == 256
  assert receiver.health()["cereal_publish_count"] == 1

  media_record = SimpleNamespace(
    kind="image",
    name="tbt_next",
    sequence=5,
    source_timestamp_ms=1234,
    received_mono_ns=5678,
    present=True,
    message_type=1,
    format_or_reason=1,
    flags=0,
    width=32,
    height=24,
    reason=None,
    payload=b"png-data",
  )
  publisher.publish_media(media_record, "session")
  service, message = messaging.pub_master.sent[-1]
  assert service == "carrotNaviMedia"
  assert message.carrotNaviMedia["sessionId"] == "session"
  assert message.carrotNaviMedia["name"] == "tbt_next"
  assert message.carrotNaviMedia["payload"] == b"png-data"

  publisher.publish_media(media_record, "session", kind_override="web_render")
  _, message = messaging.pub_master.sent[-1]
  assert message.carrotNaviMedia["kind"] == "web_render"

  publisher.publish_media(media_record, "session", kind_override="web_image")
  _, message = messaging.pub_master.sent[-1]
  assert message.carrotNaviMedia["kind"] == "web_image"


def test_payload_projects_primary_and_secondary_sdi_for_consumers():
  payload = build_carrot_navi_payload({
    "generation": 1,
    "session_id": "session",
    "connected": True,
    "items": {
      "speed": {
        "present": True,
        "sequence": 9,
        "source_timestamp_ms": 1234,
        "received_mono_ns": 5678,
        "value": {
          "road_limit_kph": 50,
          "sdi": {
            "type": 1,
            "distance_m": 420,
            "speed_limit_kph": 50,
            "section_type": 7,
            "block_type": 2,
            "block_speed_kph": 40,
            "block_distance_m": 390,
          },
          "sdi_secondary": {
            "type": 22,
            "distance_m": 93,
            "block_type": 3,
            "block_distance_m": 80,
          },
        },
      },
    },
  }, publish_mono_ns=999)

  speed = payload["speed"]
  assert speed["sdiPresent"] is True
  assert (speed["sdiSectionType"], speed["sdiBlockType"], speed["sdiBlockDistanceM"]) == (7, 2, 390)
  assert speed["secondarySdiPresent"] is True
  assert (speed["secondarySdiType"], speed["secondarySdiDistanceM"]) == (22, 93)
  assert (speed["secondarySdiBlockType"], speed["secondarySdiBlockDistanceM"]) == (3, 80)


def test_payload_rejects_encoded_or_invalid_road_limit():
  def speed_payload(road_limit):
    return build_carrot_navi_payload({
      "generation": 1,
      "session_id": "session",
      "connected": True,
      "items": {
        "speed": {
          "present": True,
          "sequence": 9,
          "source_timestamp_ms": 1234,
          "received_mono_ns": 5678,
          "value": {"road_limit_kph": road_limit},
        },
      },
    }, publish_mono_ns=999)["speed"]

  speed = speed_payload(60)
  assert speed["roadLimitValid"] is True
  assert speed["roadLimitKph"] == 60

  speed = speed_payload(320)
  assert speed["roadLimitValid"] is False
  assert speed["roadLimitKph"] == 0

  speed = speed_payload(520)
  assert speed["roadLimitValid"] is False
  assert speed["roadLimitKph"] == 0

  speed = speed_payload(1020)
  assert speed["roadLimitValid"] is False
  assert speed["roadLimitKph"] == 0

  speed = speed_payload(300)
  assert speed["roadLimitValid"] is False
  assert speed["roadLimitKph"] == 0
