from __future__ import annotations

from typing import Iterable


SCHEMA_VERSION = 2
LIVE_ENCODING_MSGPACK = "msgpack"
DEFAULT_CAMERA_KIND = "road"
PAYLOAD_KIND_SERVICE_RAW = "service-raw-v1"


def build_live_hello(
  *,
  repo_flavor: str,
  selected_camera: str = DEFAULT_CAMERA_KIND,
  capabilities: Iterable[str] | None = None,
  encoding: str = LIVE_ENCODING_MSGPACK,
) -> dict:
  return {
    "type": "hello",
    "schemaVersion": SCHEMA_VERSION,
    "repoFlavor": repo_flavor,
    "selectedCamera": selected_camera,
    "payloadKind": PAYLOAD_KIND_SERVICE_RAW,
    "encoding": encoding,
    "capabilities": list(capabilities or ()),
  }
