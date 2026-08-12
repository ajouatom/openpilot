#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
from collections import deque
from collections.abc import Callable
from contextlib import suppress
import copy
import errno
import ipaddress
import json
import secrets
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Any

from aiohttp import WSMsgType, web


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 7714
DISCOVERY_PORT = 7705
DISCOVERY_INTERVAL_S = 1.0
PROTOCOL_VERSION = 2
CATALOG_REVISION = 1
MAX_MESSAGE_BYTES = 8 * 1024 * 1024
CONTROL_WEBSOCKET_HEARTBEAT_S = 5.0
STREAM_WEBSOCKET_HEARTBEAT_S = 10.0
BIND_RETRY_COUNT = 10
BIND_RETRY_INTERVAL_S = 0.5
MAP_THEMES = frozenset(("auto", "dark", "light"))
MAP_TYPES = frozenset(("normal", "satellite"))
MAP_RENDER_WIDTH = 960
MAP_RENDER_HEIGHT = 540
MAP_HZ_MIN = 1
MAP_HZ_MAX = 60
MAP_HZ_DEFAULT = 10
MAP_BITRATE_KBPS_MIN = 1
MAP_BITRATE_KBPS_MAX = 12_000
MAP_BITRATE_KBPS_DEFAULT = 3_000
MAP_SCREEN_CENTER_Y_RATIO_DEFAULT = 0.80
MAP_SCREEN_CENTER_Y_RATIO_HUD = 0.68
MAP_SCREEN_CENTER_Y_RATIO_MIN = 0.50
MAP_SCREEN_CENTER_Y_RATIO_MAX = 0.90
MAP_AUTO_BITRATE_REFERENCE_HZ = 10
MAP_AUTO_BITRATE_KBPS_BY_HZ = {
  5: 1_500,
  10: 3_000,
  20: 3_000,
  30: 6_000,
}
MAP_HZ_BY_MODE = {
  0: 5,
  1: 10,
  2: 20,
  3: 30,
}
BINARY_HEADER = struct.Struct(">4sBBBBIIQQIHH")

JSON_NAMES = (
  "vehicle", "guidance_current", "guidance_next", "lane_current",
  "lane_ahead", "speed", "traffic_signal", "crossroad", "route",
  "navigation_status", "app_status", "camera_state", "composition_state",
)
CLUSTER_JSON_NAMES = frozenset({
  "vehicle", "guidance_current", "guidance_next", "lane_current",
  "lane_ahead", "speed", "traffic_signal", "crossroad", "route",
  "navigation_status",
})
IMAGE_NAMES = (
  "tbt_current_compact", "tbt_current_full", "tbt_next",
  "traffic_signal", "lane_top", "lane_bottom",
  "safety_primary", "safety_secondary", "safety_section",
  "crossroad_minimized", "crossroad_expanded",
  "center_tbt_icon", "center_tbt_text", "center_tbt_fee",
)
CLUSTER_ENABLED_IMAGE_NAMES = frozenset(name for name in IMAGE_NAMES if name != "lane_top")
RENDER_NAMES = ("map_main",)
CATALOG = tuple(
  [("json", name) for name in JSON_NAMES]
  + [("image", name) for name in IMAGE_NAMES]
  + [("render", name) for name in RENDER_NAMES]
)
CATALOG_SET = frozenset(CATALOG)
JSON_ARRAY_NAMES = frozenset(("lane_ahead",))
CLEAR_REASONS = {
  1: "source_absent",
  2: "cleared",
  3: "expired",
  4: "passed",
  5: "invalid",
}


def detect_advertise_ip(bind_host: str) -> str:
  if bind_host not in ("", "0.0.0.0", "::"):
    return bind_host
  probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  try:
    probe.connect(("8.8.8.8", 80))
    return str(probe.getsockname()[0])
  except OSError:
    try:
      return socket.gethostbyname(socket.gethostname())
    except OSError:
      return "127.0.0.1"
  finally:
    probe.close()


def _interface_ipv4_addresses() -> tuple[tuple[str, str | None, str | None], ...]:
  try:
    import psutil
  except ImportError:
    return ()
  addresses = []
  for interface_addresses in psutil.net_if_addrs().values():
    for address in interface_addresses:
      if address.family != socket.AF_INET or address.address.startswith("127."):
        continue
      addresses.append((address.address, address.netmask, address.broadcast))
  return tuple(addresses)


def discovery_targets(advertise_ip: str | None = None) -> tuple[tuple[str, str], ...]:
  interfaces = _interface_ipv4_addresses()
  targets = []
  for address, netmask, broadcast in interfaces:
    if advertise_ip is not None and address != advertise_ip:
      continue
    if not broadcast and netmask:
      try:
        broadcast = str(ipaddress.ip_network(f"{address}/{netmask}", strict=False).broadcast_address)
      except ValueError:
        broadcast = None
    targets.append((address, broadcast or "255.255.255.255"))

  if not targets:
    address = advertise_ip or detect_advertise_ip(DEFAULT_HOST)
    if not address.startswith("127."):
      targets.append((address, "255.255.255.255"))
  return tuple(dict.fromkeys(targets))


class CarrotNaviDiscoveryBeacon:
  def __init__(self, advertise_ip: str | None = None, interval_s: float = DISCOVERY_INTERVAL_S) -> None:
    self.advertise_ip = advertise_ip
    self.interval_s = max(0.2, float(interval_s))
    self._stop = threading.Event()
    self._thread: threading.Thread | None = None

  def start(self) -> None:
    if self._thread is not None:
      return
    self._stop.clear()
    self._thread = threading.Thread(target=self._run, name="carrot_navi_discovery", daemon=True)
    self._thread.start()

  def stop(self) -> None:
    self._stop.set()
    if self._thread is not None:
      self._thread.join(timeout=1.0)
    self._thread = None

  def broadcast_once(self) -> None:
    for source_ip, broadcast_ip in discovery_targets(self.advertise_ip):
      body = json.dumps({"ip": source_ip, "navi_debug": 1}).encode("utf-8")
      sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind((source_ip, 0))
        sock.sendto(body, (broadcast_ip, DISCOVERY_PORT))
      except OSError:
        continue
      finally:
        sock.close()

  def _run(self) -> None:
    while not self._stop.is_set():
      try:
        self.broadcast_once()
      except OSError:
        pass
      self._stop.wait(self.interval_s)


def now_ms() -> int:
  return time.time_ns() // 1_000_000


def resolve_map_hz(map_fps_mode: object) -> int:
  try:
    map_mode = int(map_fps_mode)
  except (TypeError, ValueError):
    map_mode = 1
  return MAP_HZ_BY_MODE.get(map_mode, MAP_HZ_DEFAULT)


def resolve_map_bitrate_kbps(
  width: int,
  height: int,
  map_hz: int,
) -> int:
  pixels = max(1, int(width)) * max(1, int(height))
  effective_hz = max(MAP_HZ_MIN, min(MAP_HZ_MAX, int(map_hz)))
  reference_pixels = MAP_RENDER_WIDTH * MAP_RENDER_HEIGHT
  reference_bitrate = MAP_AUTO_BITRATE_KBPS_BY_HZ.get(effective_hz)
  if reference_bitrate is None:
    reference_bitrate = (
      MAP_BITRATE_KBPS_DEFAULT * effective_hz + MAP_AUTO_BITRATE_REFERENCE_HZ // 2
    ) // MAP_AUTO_BITRATE_REFERENCE_HZ
  numerator = reference_bitrate * pixels
  denominator = reference_pixels
  automatic = (numerator + denominator // 2) // denominator
  return max(MAP_BITRATE_KBPS_MIN, min(MAP_BITRATE_KBPS_MAX, automatic))


def parse_json_object(raw: str) -> dict[str, Any]:
  value = json.loads(raw)
  if not isinstance(value, dict):
    raise ValueError("message must be a JSON object")
  return value


def parse_binary_packet(packet: bytes) -> tuple[dict[str, int], bytes]:
  if not BINARY_HEADER.size <= len(packet) <= MAX_MESSAGE_BYTES:
    raise ValueError("invalid v2 binary packet size")

  (
    magic, protocol_version, message_type, format_or_reason, flags,
    stream_handle, manifest_revision, sequence, source_timestamp_ms,
    payload_length, width, height,
  ) = BINARY_HEADER.unpack_from(packet)
  payload = packet[BINARY_HEADER.size:]

  if magic != b"CNV2" or protocol_version != PROTOCOL_VERSION:
    raise ValueError("unsupported v2 binary header")
  if payload_length != len(payload):
    raise ValueError("v2 binary payload length mismatch")
  if stream_handle <= 0 or manifest_revision <= 0:
    raise ValueError("invalid v2 binary identity")

  if message_type == 4:
    if payload or width != 0 or height != 0 or format_or_reason not in CLEAR_REASONS:
      raise ValueError("invalid v2 CLEAR packet")
  elif message_type == 1:
    if format_or_reason == 1:
      if not payload.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError("invalid v2 PNG payload")
    elif format_or_reason == 2:
      if not payload.startswith(b"\xff\xd8") or not payload.endswith(b"\xff\xd9"):
        raise ValueError("invalid v2 JPEG payload")
    else:
      raise ValueError("invalid v2 image format")
    if width <= 0 or height <= 0:
      raise ValueError("invalid v2 image dimensions")
  elif message_type in (2, 3):
    if format_or_reason != 3 or not payload.startswith((
      b"\x00\x00\x00\x01",
      b"\x00\x00\x01",
    )):
      raise ValueError("invalid v2 Annex-B payload")
    if width <= 0 or height <= 0:
      raise ValueError("invalid v2 video dimensions")
  else:
    raise ValueError("invalid v2 binary message type")

  return {
    "protocol_version": protocol_version,
    "message_type": message_type,
    "format_or_reason": format_or_reason,
    "flags": flags,
    "stream_handle": stream_handle,
    "manifest_revision": manifest_revision,
    "sequence": sequence,
    "source_timestamp_ms": source_timestamp_ms,
    "payload_length": payload_length,
    "width": width,
    "height": height,
  }, payload


def _stream_params(
  kind: str,
  name: str,
  map_theme: str = "auto",
  map_type: str = "normal",
  map_hz: int = MAP_HZ_DEFAULT,
  map_bitrate_kbps: int = MAP_BITRATE_KBPS_DEFAULT,
  screen_center_y_ratio: float = MAP_SCREEN_CENTER_Y_RATIO_DEFAULT,
) -> dict[str, Any]:
  if kind == "json":
    return {
      "delivery_mode": "on_change",
      "interval_ms": 1000,
      "stale_timeout_ms": 10000,
    }
  if kind == "image":
    params: dict[str, Any] = {
      "format": "png",
      "max_fps": 5,
      "stale_timeout_ms": 15000,
    }
    if name.startswith("crossroad_"):
      params["theme"] = map_theme
    return params
  return {
    "composition": "map_route_vehicle",
    "width": MAP_RENDER_WIDTH,
    "height": MAP_RENDER_HEIGHT,
    "dpi": 360,
    "fps": map_hz,
    "jpeg_quality": 75,
    "codec": "h264",
    "h264_bitrate_kbps": map_bitrate_kbps,
    "h264_keyframe_interval_sec": 2,
    "camera_mode": "app_sync",
    "map_theme": map_theme,
    "map_type": map_type,
    "zoom": 11.0,
    "tilt": 50.0,
    "bearing": 0.0,
    "follow_vehicle_bearing": True,
    "fov": 40.0,
    "screen_center_y_ratio": screen_center_y_ratio,
    "follow_vehicle": True,
    "center_latitude": None,
    "center_longitude": None,
    "stale_timeout_ms": 5000,
  }


def build_manifest(
  session_id: str,
  revision: int = 1,
  map_theme: str = "auto",
  map_type: str = "normal",
  map_hz: int = MAP_HZ_DEFAULT,
  map_bitrate_kbps: int = MAP_BITRATE_KBPS_DEFAULT,
  screen_center_y_ratio: float = MAP_SCREEN_CENTER_Y_RATIO_DEFAULT,
) -> dict[str, Any]:
  normalized_map_theme = str(map_theme).strip().lower()
  if normalized_map_theme not in MAP_THEMES:
    raise ValueError(f"unsupported map theme: {map_theme}")
  normalized_map_type = str(map_type).strip().lower()
  if normalized_map_type not in MAP_TYPES:
    raise ValueError(f"unsupported map type: {map_type}")
  if not MAP_HZ_MIN <= int(map_hz) <= MAP_HZ_MAX:
    raise ValueError(f"unsupported map refresh rate: {map_hz}")
  if not MAP_BITRATE_KBPS_MIN <= int(map_bitrate_kbps) <= MAP_BITRATE_KBPS_MAX:
    raise ValueError(f"unsupported map bitrate: {map_bitrate_kbps}")
  normalized_screen_center_y_ratio = float(screen_center_y_ratio)
  if not MAP_SCREEN_CENTER_Y_RATIO_MIN <= normalized_screen_center_y_ratio <= MAP_SCREEN_CENTER_Y_RATIO_MAX:
    raise ValueError(f"unsupported map screen center ratio: {screen_center_y_ratio}")
  streams = []
  for handle, (kind, name) in enumerate(CATALOG, start=1):
    streams.append({
      "kind": kind,
      "name": name,
      "schema_version": 1,
      "stream_handle": handle,
      "enabled": kind != "image" or name in CLUSTER_ENABLED_IMAGE_NAMES,
      "params": _stream_params(
        kind, name, normalized_map_theme, normalized_map_type,
        int(map_hz), int(map_bitrate_kbps), normalized_screen_center_y_ratio,
      ),
    })
  return {
    "type": "subscription_manifest",
    "protocol_version": PROTOCOL_VERSION,
    "session_id": session_id,
    "revision": revision,
    "metrics_enabled": False,
    "limit_adjustments": [],
    "streams": streams,
  }


@dataclass(frozen=True)
class ItemRecord:
  kind: str
  name: str
  schema_version: int
  stream_handle: int
  manifest_revision: int
  sequence: int
  source_timestamp_ms: int
  sent_at_ms: int | None
  present: bool
  value: Any
  payload: bytes | None
  message_type: int | None
  format_or_reason: int | None
  flags: int
  width: int
  height: int
  reason: str | None
  peer: str
  received_at_ms: int
  received_mono_ns: int

  def summary(self) -> dict[str, Any]:
    result = {
      "kind": self.kind,
      "name": self.name,
      "schema_version": self.schema_version,
      "stream_handle": self.stream_handle,
      "manifest_revision": self.manifest_revision,
      "sequence": self.sequence,
      "source_timestamp_ms": self.source_timestamp_ms,
      "present": self.present,
      "reason": self.reason,
      "peer": self.peer,
      "received_at_ms": self.received_at_ms,
      "bytes": len(self.payload) if self.payload is not None else 0,
    }
    if self.kind == "json":
      result["sent_at_ms"] = self.sent_at_ms
      result["value"] = copy.deepcopy(self.value)
    else:
      result.update({
        "message_type": self.message_type,
        "format_or_reason": self.format_or_reason,
        "flags": self.flags,
        "width": self.width,
        "height": self.height,
      })
    return result


class CarrotNaviReceiver:
  def __init__(
    self,
    port: int = DEFAULT_PORT,
    map_theme: str = "auto",
    map_type: str = "normal",
    map_hz: int = MAP_HZ_DEFAULT,
    map_bitrate_kbps: int = MAP_BITRATE_KBPS_DEFAULT,
    screen_center_y_ratio: float = MAP_SCREEN_CENTER_Y_RATIO_DEFAULT,
  ) -> None:
    self._lock = threading.RLock()
    self._port = port
    self._map_theme = str(map_theme).strip().lower()
    if self._map_theme not in MAP_THEMES:
      raise ValueError(f"unsupported map theme: {map_theme}")
    self._map_type = str(map_type).strip().lower()
    if self._map_type not in MAP_TYPES:
      raise ValueError(f"unsupported map type: {map_type}")
    self._map_hz = int(map_hz)
    if not MAP_HZ_MIN <= self._map_hz <= MAP_HZ_MAX:
      raise ValueError(f"unsupported map refresh rate: {map_hz}")
    self._map_bitrate_kbps = int(map_bitrate_kbps)
    if not MAP_BITRATE_KBPS_MIN <= self._map_bitrate_kbps <= MAP_BITRATE_KBPS_MAX:
      raise ValueError(f"unsupported map bitrate: {map_bitrate_kbps}")
    self._screen_center_y_ratio = float(screen_center_y_ratio)
    if not MAP_SCREEN_CENTER_Y_RATIO_MIN <= self._screen_center_y_ratio <= MAP_SCREEN_CENTER_Y_RATIO_MAX:
      raise ValueError(f"unsupported map screen center ratio: {screen_center_y_ratio}")
    self._session_id: str | None = None
    self._app_version = ""
    self._manifest: dict[str, Any] | None = None
    self._manifest_by_key: dict[str, dict[str, Any]] = {}
    self._records: dict[str, ItemRecord] = {}
    self._binary_configs: dict[str, ItemRecord] = {}
    self._binary_keyframes: dict[str, ItemRecord] = {}
    self._received_count = 0
    self._session_received_count = 0
    self._last_received_at_ms = 0
    self._last_peer = "-"
    self._last_error: str | None = None
    self._control_connections = 0
    self._control_events: list[dict[str, Any]] = []
    self._state_generation = 0
    self._media_generation = 0
    self._media_updates: deque[ItemRecord] = deque(maxlen=256)
    self._state_changed = threading.Event()
    self._cereal_publish_count = 0
    self._last_cereal_publish_mono_ns = 0
    self._cereal_error: str | None = None
    self._navigation_log_values: dict[str, str] = {}

  def negotiate(self, requirements: dict[str, Any], app_version: str) -> dict[str, Any]:
    if requirements.get("type") != "requirements_query" \
        or requirements.get("protocol_version") != PROTOCOL_VERSION:
      raise ValueError("invalid v2 requirements query")
    catalog_revision = requirements.get("catalog_revision")
    if isinstance(catalog_revision, bool) or catalog_revision != CATALOG_REVISION:
      raise ValueError("unsupported v2 catalog revision")

    offered_streams = requirements.get("streams")
    if not isinstance(offered_streams, list) or len(offered_streams) != len(CATALOG):
      raise ValueError("app v2 catalog does not contain exactly 28 items")

    offered: list[tuple[str, str]] = []
    for stream in offered_streams:
      if not isinstance(stream, dict) or stream.get("schema_version") != 1:
        raise ValueError("invalid v2 catalog entry")
      offered.append((str(stream.get("kind")), str(stream.get("name"))))
    if len(set(offered)) != len(offered) or set(offered) != CATALOG_SET:
      raise ValueError("app v2 catalog does not match receiver catalog")

    session_id = secrets.token_hex(8)
    with self._lock:
      manifest = build_manifest(
        session_id,
        map_theme=self._map_theme,
        map_type=self._map_type,
        map_hz=self._map_hz,
        map_bitrate_kbps=self._map_bitrate_kbps,
        screen_center_y_ratio=self._screen_center_y_ratio,
      )
      self._session_id = session_id
      self._app_version = app_version
      self._manifest = manifest
      self._manifest_by_key = {
        f"{stream['kind']}:{stream['name']}": stream
        for stream in manifest["streams"]
      }
      self._records.clear()
      self._binary_configs.clear()
      self._binary_keyframes.clear()
      self._media_updates.clear()
      self._session_received_count = 0
      self._control_events.clear()
      self._navigation_log_values.clear()
      self._last_error = None
      self._mark_state_changed_locked()
    return copy.deepcopy(manifest)

  def set_map_config(
    self,
    map_theme: str,
    map_type: str,
    map_hz: int,
    map_bitrate_kbps: int,
    screen_center_y_ratio: float,
  ) -> bool:
    normalized_map_theme = str(map_theme).strip().lower()
    normalized_map_type = str(map_type).strip().lower()
    if normalized_map_theme not in MAP_THEMES:
      raise ValueError(f"unsupported map theme: {map_theme}")
    if normalized_map_type not in MAP_TYPES:
      raise ValueError(f"unsupported map type: {map_type}")
    normalized_map_hz = int(map_hz)
    if not MAP_HZ_MIN <= normalized_map_hz <= MAP_HZ_MAX:
      raise ValueError(f"unsupported map refresh rate: {map_hz}")
    normalized_map_bitrate_kbps = int(map_bitrate_kbps)
    if not MAP_BITRATE_KBPS_MIN <= normalized_map_bitrate_kbps <= MAP_BITRATE_KBPS_MAX:
      raise ValueError(f"unsupported map bitrate: {map_bitrate_kbps}")
    normalized_screen_center_y_ratio = float(screen_center_y_ratio)
    if not MAP_SCREEN_CENTER_Y_RATIO_MIN <= normalized_screen_center_y_ratio <= MAP_SCREEN_CENTER_Y_RATIO_MAX:
      raise ValueError(f"unsupported map screen center ratio: {screen_center_y_ratio}")
    with self._lock:
      if (
        normalized_map_theme == self._map_theme
        and normalized_map_type == self._map_type
        and normalized_map_hz == self._map_hz
        and normalized_map_bitrate_kbps == self._map_bitrate_kbps
        and normalized_screen_center_y_ratio == self._screen_center_y_ratio
      ):
        return False
      self._map_theme = normalized_map_theme
      self._map_type = normalized_map_type
      self._map_hz = normalized_map_hz
      self._map_bitrate_kbps = normalized_map_bitrate_kbps
      self._screen_center_y_ratio = normalized_screen_center_y_ratio
      self._mark_state_changed_locked()
      return True

  def control_connected(self) -> None:
    with self._lock:
      self._control_connections += 1
      self._mark_state_changed_locked()

  def control_disconnected(self) -> None:
    with self._lock:
      self._control_connections = max(0, self._control_connections - 1)
      self._mark_state_changed_locked()

  def record_control(self, payload: dict[str, Any], peer: str) -> None:
    if payload.get("protocol_version") != PROTOCOL_VERSION:
      raise ValueError("unsupported v2 control protocol")
    with self._lock:
      self._control_events.append(copy.deepcopy(payload))
      if len(self._control_events) > 256:
        del self._control_events[:-256]
      self._last_peer = peer
      if payload.get("type") == "protocol_error":
        self._last_error = str(payload.get("message", payload.get("code", "error")))

  def record_json(self, session_id: str, name: str,
                  envelope: dict[str, Any], peer: str) -> None:
    if envelope.get("type") != "item_update" \
        or envelope.get("protocol_version") != PROTOCOL_VERSION \
        or envelope.get("session_id") != session_id \
        or envelope.get("kind") != "json" \
        or envelope.get("name") != name:
      raise ValueError("v2 JSON envelope/path mismatch")

    with self._lock:
      stream = self._validate_stream_locked(session_id, "json", name, envelope)
      sequence = self._nonnegative_int(envelope, "sequence")
      source_timestamp_ms = self._nonnegative_int(envelope, "source_timestamp_ms")
      sent_at_ms = self._nonnegative_int(envelope, "sent_at_ms")
      present = envelope.get("present")
      if not isinstance(present, bool):
        raise ValueError("v2 JSON present must be boolean")
      if "value" not in envelope:
        raise ValueError("v2 JSON item must contain value")
      value = envelope.get("value")
      if present:
        self._validate_json_value(name, value)
      else:
        if value is not None:
          raise ValueError("absent JSON item must contain value=null")
        reason = envelope.get("reason")
        if not isinstance(reason, str) or not reason or len(reason) > 64:
          raise ValueError("absent JSON item must contain a valid reason")

      key = f"json:{name}"
      self._validate_sequence_locked(key, sequence)
      value = copy.deepcopy(value) if present else None
      self._records[key] = ItemRecord(
        kind="json",
        name=name,
        schema_version=int(stream["schema_version"]),
        stream_handle=int(stream["stream_handle"]),
        manifest_revision=int(self._manifest["revision"]),
        sequence=sequence,
        source_timestamp_ms=source_timestamp_ms,
        sent_at_ms=sent_at_ms,
        present=present,
        value=value,
        payload=None,
        message_type=None,
        format_or_reason=None,
        flags=0,
        width=0,
        height=0,
        reason=None if present else str(envelope.get("reason", "source_absent")),
        peer=peer,
        received_at_ms=now_ms(),
        received_mono_ns=time.monotonic_ns(),
      )
      self._mark_received_locked(peer)
      self._mark_state_changed_locked()
      self._log_navigation_update_locked(name, sequence, present, value, envelope.get("reason"))

  def _log_navigation_update_locked(
    self,
    name: str,
    sequence: int,
    present: bool,
    value: Any,
    reason: Any,
  ) -> None:
    labels = {
      "guidance_current": "TBT current",
      "guidance_next": "TBT next",
      "speed": "SDI",
    }
    label = labels.get(name)
    if label is None:
      return
    content: Any = value if present else {"present": False, "reason": str(reason or "source_absent")}
    try:
      text = json.dumps(content, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    except (TypeError, ValueError):
      text = repr(content)
    text = text[:1500]
    if self._navigation_log_values.get(name) == text:
      return
    self._navigation_log_values[name] = text
    print(f"[carrot_navi][{label}] seq={sequence} {text}", flush=True)

  def record_binary(self, session_id: str, kind: str, name: str,
                    metadata: dict[str, int], payload: bytes, peer: str) -> None:
    if kind not in ("image", "render"):
      raise ValueError("invalid v2 binary stream kind")

    message_type = int(metadata["message_type"])
    if kind == "image" and message_type not in (1, 4):
      raise ValueError("v2 image stream received non-image message")
    if kind == "render" and message_type not in (1, 2, 3, 4):
      raise ValueError("v2 render stream received invalid message")
    if kind == "image" and message_type == 1 and int(metadata["format_or_reason"]) != 1:
      raise ValueError("v2 image stream requires PNG frames")
    if kind == "render" and message_type == 1 and int(metadata["format_or_reason"]) != 2:
      raise ValueError("v2 render image frame requires JPEG")

    with self._lock:
      stream = self._validate_stream_locked(session_id, kind, name, metadata)
      sequence = int(metadata["sequence"])
      key = f"{kind}:{name}"
      self._validate_sequence_locked(key, sequence)
      clear = message_type == 4
      format_or_reason = int(metadata["format_or_reason"])
      record = ItemRecord(
        kind=kind,
        name=name,
        schema_version=int(stream["schema_version"]),
        stream_handle=int(stream["stream_handle"]),
        manifest_revision=int(self._manifest["revision"]),
        sequence=sequence,
        source_timestamp_ms=int(metadata["source_timestamp_ms"]),
        sent_at_ms=None,
        present=not clear,
        value=None,
        payload=None if clear else bytes(payload),
        message_type=message_type,
        format_or_reason=format_or_reason,
        flags=int(metadata["flags"]),
        width=int(metadata["width"]),
        height=int(metadata["height"]),
        reason=CLEAR_REASONS.get(format_or_reason) if clear else None,
        peer=peer,
        received_at_ms=now_ms(),
        received_mono_ns=time.monotonic_ns(),
      )
      self._records[key] = record
      if clear:
        self._binary_configs.pop(key, None)
        self._binary_keyframes.pop(key, None)
      elif kind == "render" and message_type == 2:
        self._binary_configs[key] = record
        self._binary_keyframes.pop(key, None)
      elif kind == "render" and message_type == 3 and record.flags & 1:
        self._binary_keyframes[key] = record
      self._media_generation += 1
      self._media_updates.append(record)
      self._mark_received_locked(peer)
      self._state_changed.set()

  def fail(self, message: str, peer: str = "-") -> None:
    with self._lock:
      self._last_error = str(message)
      if peer != "-":
        self._last_peer = peer

  def stream_config(self, session_id: str, kind: str, name: str) -> dict[str, Any]:
    with self._lock:
      return copy.deepcopy(self._validate_stream_locked(session_id, kind, name, {}))

  def health(self) -> dict[str, Any]:
    with self._lock:
      return {
        "ok": True,
        "service": "carrot_navi_receiver",
        "port": self._port,
        "protocol_version": PROTOCOL_VERSION,
        "map_theme": self._map_theme,
        "map_type": self._map_type,
        "map_hz": self._map_hz,
        "map_bitrate_kbps": self._map_bitrate_kbps,
        "screen_center_y_ratio": self._screen_center_y_ratio,
        "app_foreground": self._app_foreground_locked(),
        "control_connected": self._control_connections > 0,
        "control_connections": self._control_connections,
        "session_id": self._session_id,
        "app_version": self._app_version,
        "manifest_revision": int(self._manifest["revision"]) if self._manifest else 0,
        "received_count": self._received_count,
        "session_received_count": self._session_received_count,
        "last_received_at_ms": self._last_received_at_ms,
        "peer": self._last_peer,
        "error": self._last_error,
        "control_event_count": len(self._control_events),
        "state_generation": self._state_generation,
        "cereal_publish_count": self._cereal_publish_count,
        "last_cereal_publish_mono_ns": self._last_cereal_publish_mono_ns,
        "cereal_error": self._cereal_error,
        "items": {
          key: record.summary()
          for key, record in self._records.items()
        },
      }

  def cereal_snapshot(self) -> dict[str, Any]:
    with self._lock:
      items: dict[str, dict[str, Any]] = {}
      for name in CLUSTER_JSON_NAMES:
        record = self._records.get(f"json:{name}")
        if record is None:
          continue
        items[name] = {
          "present": record.present,
          "sequence": record.sequence,
          "source_timestamp_ms": record.source_timestamp_ms,
          "received_mono_ns": record.received_mono_ns,
          "value": copy.deepcopy(record.value) if record.present else None,
        }
      return {
        "generation": self._state_generation,
        "session_id": self._session_id or "",
        "connected": self._control_connections > 0 and self._session_id is not None,
        "items": items,
      }

  def wait_for_state_change(self, timeout: float) -> bool:
    changed = self._state_changed.wait(max(0.0, timeout))
    self._state_changed.clear()
    return changed

  def drain_media_updates(self) -> list[ItemRecord]:
    with self._lock:
      updates = list(self._media_updates)
      self._media_updates.clear()
      return updates

  def media_bootstrap(self) -> list[ItemRecord]:
    with self._lock:
      records = [self._binary_configs[key] for key in sorted(self._binary_configs)]
      records.extend(self._binary_keyframes[key] for key in sorted(self._binary_keyframes))
      for key in sorted(self._records):
        record = self._records[key]
        if not record.present:
          continue
        if record.kind == "image":
          records.append(record)
      return records

  def record_cereal_publish(self, error: str | None = None) -> None:
    with self._lock:
      if error is None:
        self._cereal_publish_count += 1
        self._last_cereal_publish_mono_ns = time.monotonic_ns()
        self._cereal_error = None
      else:
        self._cereal_error = str(error)[:256]

  def latest(self) -> dict[str, Any]:
    with self._lock:
      return {
        "session_id": self._session_id,
        "manifest_revision": int(self._manifest["revision"]) if self._manifest else 0,
        "received_count": self._received_count,
        "session_received_count": self._session_received_count,
        "last_received_at_ms": self._last_received_at_ms,
        "peer": self._last_peer,
        "app_foreground": self._app_foreground_locked(),
        "error": self._last_error,
        "items": {
          key: record.summary()
          for key, record in self._records.items()
        },
        "last_control_events": copy.deepcopy(self._control_events[-20:]),
      }

  def dashboard_snapshot(self) -> dict[str, Any]:
    """Return immutable record references for the in-process cluster UI.

    ItemRecord and payload bytes are immutable, so this shallow snapshot avoids
    copying large map/image frames while keeping receiver dictionaries private.
    """
    with self._lock:
      return {
        "connected": self._control_connections > 0 and self._session_id is not None,
        "session_id": self._session_id or "",
        "app_version": self._app_version,
        "manifest_revision": int(self._manifest["revision"]) if self._manifest else 0,
        "received_count": self._received_count,
        "last_received_at_ms": self._last_received_at_ms,
        "peer": self._last_peer,
        "error": self._last_error,
        "state_generation": self._state_generation,
        "media_generation": self._media_generation,
        "records": dict(self._records),
        "binary_configs": dict(self._binary_configs),
      }

  def _validate_stream_locked(self, session_id: str, kind: str, name: str,
                              identity: dict[str, Any]) -> dict[str, Any]:
    if session_id != self._session_id:
      raise ValueError("stale v2 session")
    stream = self._manifest_by_key.get(f"{kind}:{name}")
    if stream is None or not stream.get("enabled", False):
      raise ValueError("v2 stream is not enabled")
    if identity:
      if int(identity.get("manifest_revision", -1)) != int(self._manifest["revision"]):
        raise ValueError("stale v2 manifest revision")
      if int(identity.get("stream_handle", -1)) != int(stream["stream_handle"]):
        raise ValueError("v2 stream handle mismatch")
      if "schema_version" in identity \
          and int(identity["schema_version"]) != int(stream["schema_version"]):
        raise ValueError("v2 item schema mismatch")
    return stream

  def _validate_sequence_locked(self, key: str, sequence: int) -> None:
    if sequence < 0:
      raise ValueError("invalid v2 sequence")
    previous = self._records.get(key)
    if previous is not None and sequence <= previous.sequence:
      raise ValueError("stale v2 sequence")

  @staticmethod
  def _validate_json_value(name: str, value: Any) -> None:
    if name in JSON_ARRAY_NAMES:
      if not isinstance(value, list) or any(not isinstance(item, dict) for item in value):
        raise ValueError(f"v2 JSON {name} value must be an array of objects")
      return
    if not isinstance(value, dict):
      raise ValueError(f"v2 JSON {name} value must be an object")

  def _app_foreground_locked(self) -> bool | None:
    record = self._records.get("json:app_status")
    if record is None or not record.present or not isinstance(record.value, dict):
      return None
    foreground = record.value.get("foreground")
    return foreground if isinstance(foreground, bool) else None

  @staticmethod
  def _nonnegative_int(value: dict[str, Any], key: str) -> int:
    candidate = value.get(key)
    if isinstance(candidate, bool):
      raise ValueError(f"invalid v2 {key}")
    try:
      parsed = int(candidate)
    except (TypeError, ValueError) as exc:
      raise ValueError(f"invalid v2 {key}") from exc
    if parsed < 0:
      raise ValueError(f"invalid v2 {key}")
    return parsed

  def _mark_received_locked(self, peer: str) -> None:
    self._received_count += 1
    self._session_received_count += 1
    self._last_received_at_ms = now_ms()
    self._last_peer = peer
    self._last_error = None

  def _mark_state_changed_locked(self) -> None:
    self._state_generation += 1
    self._state_changed.set()


RECEIVER_KEY = web.AppKey("carrot_navi_receiver", CarrotNaviReceiver)
WEBSOCKETS_KEY = web.AppKey("carrot_navi_websockets", set)
MAP_CONFIG_READER_KEY = web.AppKey("carrot_navi_map_config_reader", Callable)


class ClusterNaviMapParamReader:
  THEMES = {0: "auto", 1: "dark", 2: "light"}
  TYPES = {0: "normal", 1: "satellite"}

  def __init__(self, params: Any | None = None) -> None:
    if params is None:
      from openpilot.common.params import Params

      params = Params()
    self.params = params

  def _read_int(self, key: str, default: int) -> int:
    try:
      return int(self.params.get_int(key))
    except Exception:
      pass

    # Source files can be updated before the native Params key table is
    # rebuilt. Read the backing file during that short compatibility window.
    try:
      with open(self.params.get_param_path(key), "rb") as f:
        return int(float(f.read().decode("utf-8", errors="replace")))
    except Exception:
      return default

  def read(self) -> tuple[str, str, int, int, float]:
    theme_value = self._read_int("ClusterNaviMapTheme", 1)
    type_value = self._read_int("ClusterNaviMapType", 0)
    map_hz = resolve_map_hz(self._read_int("ClusterNaviMapFps", 1))
    map_bitrate_kbps = resolve_map_bitrate_kbps(
      MAP_RENDER_WIDTH,
      MAP_RENDER_HEIGHT,
      map_hz,
    )
    hud_profile = self._read_int("CarrotNaviHudMapProfile", 0) == 1
    return (
      self.THEMES.get(theme_value, "dark"),
      self.TYPES.get(type_value, "normal"),
      map_hz,
      map_bitrate_kbps,
      MAP_SCREEN_CENTER_Y_RATIO_HUD if hud_profile else MAP_SCREEN_CENTER_Y_RATIO_DEFAULT,
    )

  def __call__(self) -> tuple[str, str, int, int, float]:
    return self.read()


def _track_websocket(request: web.Request, ws: web.WebSocketResponse) -> None:
  request.app[WEBSOCKETS_KEY].add(ws)


def _untrack_websocket(request: web.Request, ws: web.WebSocketResponse) -> None:
  request.app[WEBSOCKETS_KEY].discard(ws)


async def _watch_map_config(app: web.Application) -> None:
  receiver = app[RECEIVER_KEY]
  reader = app[MAP_CONFIG_READER_KEY]
  while True:
    map_theme, map_type, map_hz, map_bitrate_kbps, screen_center_y_ratio = reader()
    if receiver.set_map_config(
      map_theme, map_type, map_hz, map_bitrate_kbps, screen_center_y_ratio,
    ):
      sockets = tuple(app[WEBSOCKETS_KEY])
      if sockets:
        await asyncio.gather(*(
          ws.close(code=1012, message=b"map configuration changed")
          for ws in sockets
        ), return_exceptions=True)
    await asyncio.sleep(1.0)


async def _map_config_context(app: web.Application):
  task = asyncio.create_task(_watch_map_config(app))
  try:
    yield
  finally:
    task.cancel()
    with suppress(asyncio.CancelledError):
      await task


def _peer(request: web.Request) -> str:
  return request.headers.get("X-Forwarded-For", request.remote or "-")


def _protocol_error(code: str, message: str, kind: str | None = None,
                    name: str | None = None) -> dict[str, Any]:
  result: dict[str, Any] = {
    "type": "protocol_error",
    "protocol_version": PROTOCOL_VERSION,
    "code": code,
    "recoverable": True,
    "message": str(message)[:256],
  }
  if kind is not None:
    result["kind"] = kind
  if name is not None:
    result["name"] = name
  return result


async def _safe_send_json(ws: web.WebSocketResponse, payload: dict[str, Any]) -> bool:
  if ws.closed:
    return False
  try:
    await ws.send_json(payload)
    return True
  except (ConnectionError, RuntimeError):
    return False


async def _safe_close_websocket(ws: web.WebSocketResponse, code: int, message: bytes) -> None:
  if ws.closed:
    return
  try:
    await ws.close(code=code, message=message)
  except (ConnectionError, RuntimeError):
    pass


async def index(request: web.Request) -> web.Response:
  return web.json_response({
    "name": "Carrot Navi Receiver",
    "port": DEFAULT_PORT,
    "protocol_version": PROTOCOL_VERSION,
    "health": "/health",
    "latest": "/api/navi/latest",
    "ws_control": "/api/navi/ws/v2/control/{version}",
    "ws_json": "/api/navi/ws/v2/json/{session_id}/{name}",
    "ws_image": "/api/navi/ws/v2/image/{session_id}/{name}",
    "ws_render": "/api/navi/ws/v2/render/{session_id}/{name}",
  })


async def health(request: web.Request) -> web.Response:
  return web.json_response(request.app[RECEIVER_KEY].health())


async def latest(request: web.Request) -> web.Response:
  return web.json_response(request.app[RECEIVER_KEY].latest())


async def ws_control(request: web.Request) -> web.WebSocketResponse:
  receiver = request.app[RECEIVER_KEY]
  peer = _peer(request)
  app_version = request.match_info["version"]
  ws = web.WebSocketResponse(
    heartbeat=CONTROL_WEBSOCKET_HEARTBEAT_S,
    max_msg_size=MAX_MESSAGE_BYTES,
    compress=False,
  )
  await ws.prepare(request)
  _track_websocket(request, ws)
  receiver.control_connected()
  print(f"[carrot_navi][WS] control connected peer={peer} app={app_version}", flush=True)
  try:
    async for message in ws:
      if message.type != WSMsgType.TEXT:
        error = "v2 control accepts JSON text only"
        receiver.fail(error, peer)
        if not await _safe_send_json(ws, _protocol_error("invalid_control_message", error)):
          break
        continue
      try:
        payload = parse_json_object(message.data)
        if payload.get("type") == "requirements_query":
          manifest = receiver.negotiate(payload, app_version)
          if not await _safe_send_json(ws, manifest):
            break
        else:
          receiver.record_control(payload, peer)
      except (TypeError, ValueError) as exc:
        receiver.fail(str(exc), peer)
        print(f"[carrot_navi][WS] control rejected peer={peer}: {exc}", flush=True)
        if not await _safe_send_json(ws, _protocol_error("invalid_control_message", str(exc))):
          break
  finally:
    receiver.control_disconnected()
    _untrack_websocket(request, ws)
    print(f"[carrot_navi][WS] control disconnected peer={peer} app={app_version}", flush=True)
  return ws


async def ws_json(request: web.Request) -> web.WebSocketResponse:
  receiver = request.app[RECEIVER_KEY]
  peer = _peer(request)
  session_id = request.match_info["session_id"]
  name = request.match_info["name"]
  ws = web.WebSocketResponse(
    heartbeat=STREAM_WEBSOCKET_HEARTBEAT_S,
    max_msg_size=MAX_MESSAGE_BYTES,
    compress=False,
  )
  await ws.prepare(request)
  _track_websocket(request, ws)
  try:
    receiver.stream_config(session_id, "json", name)
    async for message in ws:
      if message.type != WSMsgType.TEXT:
        raise ValueError("v2 JSON item stream accepts text only")
      receiver.record_json(session_id, name, parse_json_object(message.data), peer)
  except (TypeError, ValueError) as exc:
    receiver.fail(str(exc), peer)
    print(f"[carrot_navi][WS] json:{name} rejected peer={peer}: {exc}", flush=True)
    await _safe_send_json(ws, _protocol_error("json_stream_error", str(exc), "json", name))
    await _safe_close_websocket(ws, 1008, b"invalid JSON stream")
  finally:
    _untrack_websocket(request, ws)
  return ws


async def _ws_binary(request: web.Request, kind: str) -> web.WebSocketResponse:
  receiver = request.app[RECEIVER_KEY]
  peer = _peer(request)
  session_id = request.match_info["session_id"]
  name = request.match_info["name"]
  ws = web.WebSocketResponse(
    heartbeat=STREAM_WEBSOCKET_HEARTBEAT_S,
    max_msg_size=MAX_MESSAGE_BYTES,
    compress=False,
  )
  await ws.prepare(request)
  _track_websocket(request, ws)
  try:
    receiver.stream_config(session_id, kind, name)
    async for message in ws:
      if message.type != WSMsgType.BINARY:
        raise ValueError("v2 binary item stream accepts binary messages only")
      metadata, payload = parse_binary_packet(message.data)
      receiver.record_binary(session_id, kind, name, metadata, payload, peer)
  except (KeyError, TypeError, ValueError) as exc:
    receiver.fail(str(exc), peer)
    print(f"[carrot_navi][WS] {kind}:{name} rejected peer={peer}: {exc}", flush=True)
    await _safe_send_json(ws, _protocol_error("binary_stream_error", str(exc), kind, name))
    await _safe_close_websocket(ws, 1008, b"invalid binary stream")
  finally:
    _untrack_websocket(request, ws)
  return ws


async def ws_image(request: web.Request) -> web.WebSocketResponse:
  return await _ws_binary(request, "image")


async def ws_render(request: web.Request) -> web.WebSocketResponse:
  return await _ws_binary(request, "render")


def create_app(
  receiver: CarrotNaviReceiver | None = None,
  map_config_reader: Callable[[], tuple[str, str, int, int, float]] | None = None,
) -> web.Application:
  app = web.Application(client_max_size=MAX_MESSAGE_BYTES)
  app[RECEIVER_KEY] = receiver or CarrotNaviReceiver()
  app[WEBSOCKETS_KEY] = set()
  if map_config_reader is not None:
    app[MAP_CONFIG_READER_KEY] = map_config_reader
    app.cleanup_ctx.append(_map_config_context)
  app.router.add_get("/", index)
  app.router.add_get("/health", health)
  app.router.add_get("/api/navi/latest", latest)
  app.router.add_get("/api/navi/ws/v2/control/{version}", ws_control)
  app.router.add_get("/api/navi/ws/v2/json/{session_id}/{name}", ws_json)
  app.router.add_get("/api/navi/ws/v2/image/{session_id}/{name}", ws_image)
  app.router.add_get("/api/navi/ws/v2/render/{session_id}/{name}", ws_render)
  return app


def run_receiver_app(
  receiver: CarrotNaviReceiver,
  host: str,
  port: int,
  *,
  retry_count: int = BIND_RETRY_COUNT,
  retry_interval_s: float = BIND_RETRY_INTERVAL_S,
  map_config_reader: Callable[[], tuple[str, str, int, int, float]] | None = None,
) -> None:
  retry_count = max(0, int(retry_count))
  retry_interval_s = max(0.0, float(retry_interval_s))
  for retry in range(retry_count + 1):
    try:
      web.run_app(
        create_app(receiver, map_config_reader),
        host=host,
        port=port,
        access_log=None,
      )
      return
    except OSError as exc:
      if exc.errno != errno.EADDRINUSE or retry >= retry_count:
        raise
      print(
        f"[carrot_navi] {host}:{port} still in use;",
        f"retrying in {retry_interval_s:.1f}s ({retry + 1}/{retry_count})",
        flush=True,
      )
      time.sleep(retry_interval_s)


def main() -> None:
  parser = argparse.ArgumentParser(description="Carrot Navi WebSocket receiver")
  parser.add_argument("--host", default=DEFAULT_HOST)
  parser.add_argument("--port", type=int, default=DEFAULT_PORT)
  parser.add_argument("--advertise-ip", default=None)
  parser.add_argument("--no-beacon", action="store_true")
  parser.add_argument("--map-theme", choices=tuple(sorted(MAP_THEMES)), default=None)
  parser.add_argument("--map-type", choices=tuple(sorted(MAP_TYPES)), default=None)
  parser.add_argument("--no-cereal", action="store_true")
  args = parser.parse_args()

  param_reader = ClusterNaviMapParamReader()

  def read_map_config() -> tuple[str, str, int, int, float]:
    param_map_theme, param_map_type, map_hz, map_bitrate_kbps, screen_center_y_ratio = param_reader()
    return (
      args.map_theme or param_map_theme,
      args.map_type or param_map_type,
      map_hz,
      map_bitrate_kbps,
      screen_center_y_ratio,
    )

  map_theme, map_type, map_hz, map_bitrate_kbps, screen_center_y_ratio = read_map_config()
  receiver = CarrotNaviReceiver(
    port=args.port,
    map_theme=map_theme,
    map_type=map_type,
    map_hz=map_hz,
    map_bitrate_kbps=map_bitrate_kbps,
    screen_center_y_ratio=screen_center_y_ratio,
  )
  advertise_ip = args.advertise_ip or (args.host if args.host not in ("", "0.0.0.0", "::") else None)
  beacon = None if args.no_beacon else CarrotNaviDiscoveryBeacon(advertise_ip)
  publisher = None
  if not args.no_cereal:
    try:
      from openpilot.selfdrive.carrot.carrot_navi_cereal import CarrotNaviCerealPublisher

      publisher = CarrotNaviCerealPublisher(receiver)
      publisher.start()
    except Exception as exc:
      receiver.record_cereal_publish(f"publisher unavailable: {exc}")
      print(f"[carrot_navi] cereal publisher unavailable: {exc}")

  if beacon is not None:
    beacon.start()
    advertised = ", ".join(address for address, _ in discovery_targets(advertise_ip)) or "unavailable"
    print(f"[carrot_navi] discovery advertising {advertised}:{args.port} via UDP {DISCOVERY_PORT}")
  print(
    f"[carrot_navi] starting receiver on {args.host}:{args.port}",
    f"map_theme={map_theme} map_type={map_type} map_hz={map_hz}",
    f"map_bitrate_kbps={map_bitrate_kbps}",
    f"screen_center_y_ratio={screen_center_y_ratio:.2f}",
  )
  try:
    run_receiver_app(receiver, args.host, args.port, map_config_reader=read_map_config)
  finally:
    if beacon is not None:
      beacon.stop()
    if publisher is not None:
      publisher.stop()


if __name__ == "__main__":
  main()
