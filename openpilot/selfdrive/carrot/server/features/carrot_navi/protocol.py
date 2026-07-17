from __future__ import annotations

import json
import struct
from typing import Any


MEDIA_WIRE_MAGIC = b"CNWB"
MEDIA_WIRE_VERSION = 1
MEDIA_HEADER = struct.Struct(">4sBI")


def encode_media_frame(metadata: dict[str, Any], payload: bytes) -> bytes:
  header = json.dumps(metadata, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
  return MEDIA_HEADER.pack(MEDIA_WIRE_MAGIC, MEDIA_WIRE_VERSION, len(header)) + header + payload
