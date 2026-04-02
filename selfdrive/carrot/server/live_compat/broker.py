from __future__ import annotations

import json
import time
from typing import Iterable

_MESSAGING_IMPORT_ERROR: Exception | None = None
try:
  from cereal import messaging
except ModuleNotFoundError as exc:
  messaging = None  # type: ignore[assignment]
  _MESSAGING_IMPORT_ERROR = exc

from .contract import DEFAULT_CAMERA_KIND, LIVE_ENCODING_MSGPACK, build_live_hello
from .services import build_service_list
from .snapshot import build_live_payload, empty_live_payload

try:
  from openpilot.common.params import Params  # type: ignore
except Exception:
  Params = None

try:
  import msgpack  # type: ignore
except Exception:
  msgpack = None


class RealtimeBroker:
  def __init__(
    self,
    *,
    repo_flavor: str,
    selected_camera: str = DEFAULT_CAMERA_KIND,
    include_optional: Iterable[str] | None = None,
    exclude_services: Iterable[str] | None = None,
  ) -> None:
    if messaging is None:
      raise RuntimeError(
        "realtime broker requires openpilot native messaging artifacts. "
        "Build openpilot first so msgq/ipc_pyx.so is available."
      ) from _MESSAGING_IMPORT_ERROR

    self.repo_flavor = repo_flavor
    self.selected_camera = selected_camera
    self.service_names = build_service_list(include_optional=include_optional, exclude=exclude_services)
    self.sm = messaging.SubMaster(self.service_names)
    self.params = Params() if Params is not None else None

    self.last_snapshot = empty_live_payload(repo_flavor=repo_flavor, selected_camera=selected_camera)
    self.last_snapshot_monotonic = 0.0
    self.last_payload_bytes = b""
    self.last_payload_encoding = LIVE_ENCODING_MSGPACK

  def poll(self, timeout: int = 0) -> dict:
    self.sm.update(timeout)
    params_snapshot = self._read_params()
    snapshot = build_live_payload(
      sm=self.sm,
      repo_flavor=self.repo_flavor,
      selected_camera=self.selected_camera,
      params_snapshot=params_snapshot,
      capability_flags=self.capability_flags(),
      previous_payload=self.last_snapshot,
    )
    self.last_snapshot = snapshot
    self.last_snapshot_monotonic = time.monotonic()
    self.last_payload_encoding, self.last_payload_bytes = self._encode_snapshot(snapshot)
    return snapshot

  def snapshot_age_ms(self) -> int | None:
    if self.last_snapshot_monotonic <= 0:
      return None
    return int((time.monotonic() - self.last_snapshot_monotonic) * 1000)

  def hello_meta(self) -> dict:
    return build_live_hello(
      repo_flavor=self.repo_flavor,
      selected_camera=self.selected_camera,
      capabilities=self.capability_flags(),
      encoding=self.last_payload_encoding,
    )

  def hello_payload_bytes(self) -> bytes:
    _, payload = self._encode_payload(self.hello_meta())
    return payload

  def capability_flags(self) -> list[str]:
    flags = ["live", "camera-road", "road-only-ui", "carrotlink-projection"]
    if "wideRoadCameraState" in self.service_names:
      flags.append("wide-meta")
    return flags

  def _read_params(self) -> dict:
    if self.params is None:
      return {}

    names = (
      "LongitudinalPersonality",
      "ShowDateTime",
      "ShowPathEnd",
      "ShowLaneInfo",
      "ShowPathMode",
      "ShowPathColor",
      "ShowPathModeLane",
      "ShowPathColorLane",
      "ShowPathColorCruiseOff",
      "ShowPathWidth",
      "ShowPlotMode",
      "ShowRadarInfo",
      "CustomSR",
    )
    snapshot: dict[str, object] = {}
    for name in names:
      try:
        value = self.params.get(name)
      except Exception:
        value = None
      if value is None:
        snapshot[name] = None
      elif isinstance(value, bytes):
        snapshot[name] = value.decode("utf-8", errors="replace")
      else:
        snapshot[name] = str(value)
    return snapshot

  def _encode_snapshot(self, snapshot: dict) -> tuple[str, bytes]:
    return self._encode_payload(snapshot)

  def _encode_payload(self, payload: dict) -> tuple[str, bytes]:
    if msgpack is not None:
      return LIVE_ENCODING_MSGPACK, msgpack.packb(payload, use_bin_type=True)
    return "json", json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")

  def debug_stats(self) -> dict[str, object]:
    runtime = self.last_snapshot.get("runtime", {}) if isinstance(self.last_snapshot, dict) else {}
    services = self.last_snapshot.get("services", {}) if isinstance(self.last_snapshot, dict) else {}
    model = services.get("modelV2") if isinstance(services, dict) else {}
    if not isinstance(model, dict):
      model = {}
    position = model.get("position") if isinstance(model.get("position"), dict) else {}
    lane_lines = model.get("laneLines") if isinstance(model.get("laneLines"), list) else []
    road_edges = model.get("roadEdges") if isinstance(model.get("roadEdges"), list) else []
    leads = model.get("leadsV3") if isinstance(model.get("leadsV3"), list) else []

    lane_points = sum(len(line.get("x", [])) for line in lane_lines if isinstance(line, dict))
    edge_points = sum(len(edge.get("x", [])) for edge in road_edges if isinstance(edge, dict))

    return {
      "payloadEncoding": self.last_payload_encoding,
      "payloadBytes": len(self.last_payload_bytes),
      "activeCoreServices": runtime.get("activeCoreServices"),
      "activeOptionalServices": runtime.get("activeOptionalServices"),
      "serviceCount": len(services) if isinstance(services, dict) else 0,
      "modelFrameId": model.get("frameId"),
      "modelPositionPoints": len(position.get("x", [])) if isinstance(position, dict) else 0,
      "modelLaneLines": len(lane_lines),
      "modelLanePoints": lane_points,
      "modelRoadEdges": len(road_edges),
      "modelRoadEdgePoints": edge_points,
      "modelLeads": len(leads),
    }
