from __future__ import annotations

import json
import struct
from typing import Any


MEDIA_WIRE_MAGIC = b"CNWB"
MEDIA_WIRE_VERSION = 1
MEDIA_HEADER = struct.Struct(">4sBI")
SESSION_WIRE_VERSION = 1
SESSION_CONTROL_TYPE = "carrotNaviSession"
SESSION_ACCEPTED_CODE = "carrot_navi_accepted"
SESSION_BUSY_CODE = "carrot_navi_busy"
SESSION_REPLACED_CODE = "carrot_navi_replaced"
SESSION_BUSY_CLOSE_CODE = 4409
SESSION_REPLACED_CLOSE_CODE = 4401


def encode_media_frame(metadata: dict[str, Any], payload: bytes) -> bytes:
  header = json.dumps(metadata, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
  return MEDIA_HEADER.pack(MEDIA_WIRE_MAGIC, MEDIA_WIRE_VERSION, len(header)) + header + payload


def encode_session_status(status: str, code: str) -> str:
  return json.dumps({
    "type": SESSION_CONTROL_TYPE,
    "version": SESSION_WIRE_VERSION,
    "status": str(status),
    "code": str(code),
  }, separators=(",", ":"))
