from __future__ import annotations

from typing import Any


RAW_PROTOCOL_VERSION = 1
RAW_WIRE_FORMAT = "cereal-event-capnp"
RAW_MODE = "raw-capnp-relay"


def build_raw_hello(*, service: str) -> dict[str, Any]:
  return {
    "type": "hello",
    "service": service,
    "protocolVersion": RAW_PROTOCOL_VERSION,
    "mode": RAW_MODE,
    "wireFormat": RAW_WIRE_FORMAT,
  }
