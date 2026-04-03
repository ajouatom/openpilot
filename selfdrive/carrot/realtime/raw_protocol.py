from __future__ import annotations

from typing import Any


RAW_PROTOCOL_VERSION = 1
RAW_WIRE_FORMAT = "cereal-event-capnp"
RAW_MODE = "raw-capnp-relay"
RAW_MULTIPLEX_WIRE_FORMAT = "service-name+capnp-frame"
RAW_MULTIPLEX_MODE = "raw-capnp-multiplex-relay"


def build_raw_hello(*, service: str) -> dict[str, Any]:
  return {
    "type": "hello",
    "service": service,
    "protocolVersion": RAW_PROTOCOL_VERSION,
    "mode": RAW_MODE,
    "wireFormat": RAW_WIRE_FORMAT,
  }


def build_raw_multiplex_hello(*, services: list[str]) -> dict[str, Any]:
  return {
    "type": "hello",
    "services": services,
    "protocolVersion": RAW_PROTOCOL_VERSION,
    "mode": RAW_MULTIPLEX_MODE,
    "wireFormat": RAW_MULTIPLEX_WIRE_FORMAT,
  }


def encode_raw_multiplex_frame(*, service: str, payload: bytes) -> bytes:
  service_bytes = service.encode("utf-8")
  if len(service_bytes) > 255:
    raise ValueError("service name too long for multiplex frame")
  return bytes([len(service_bytes)]) + service_bytes + payload
