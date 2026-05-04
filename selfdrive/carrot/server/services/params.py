import base64
import hashlib
import importlib
import json
import os
import subprocess
import sys
import threading
import zlib
from typing import Any, Dict, List, Optional

try:
  import brotli
except Exception:
  brotli = None


# -----------------------
# Optional openpilot Params
# -----------------------
HAS_PARAMS = False
Params = None
ParamKeyType = None

try:
  from openpilot.common.params import Params as _Params
  Params = _Params
  HAS_PARAMS = True
except Exception:
  pass

# ParamKeyType는 fork/버전에 따라 위치가 다를 수 있어서 방어적으로 처리
if HAS_PARAMS:
  try:
    from openpilot.common.params import ParamKeyType as _ParamKeyType
    ParamKeyType = _ParamKeyType
  except Exception:
    ParamKeyType = None


# In-memory fallback store when Params is unavailable
_mem_store: Dict[str, str] = {}

QR_BACKUP_PREFIX_V1 = "CQR1"
QR_BACKUP_PREFIX_V2 = "CQR2"
QR_BACKUP_PREFIX_V3 = "CQR3"
QR_BACKUP_PREFIX_V4 = "CQR4"
QR_BACKUP_PREFIX = QR_BACKUP_PREFIX_V4
QR_BACKUP_TYPE = "params_backup"
QR_BACKUP_VERSION = 4
QR_BACKUP_ECC = "L"
QR_BACKUP_CODE_BYTES = 3
QR_BACKUP_CODE_BYTES_V3 = 2
QR_BACKUP_CHECKSUM_CHARS = 12
QR_BACKUP_SCHEMA_BYTES = 4
QR_BACKUP_BASE45_ALPHABET = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ $%*+-./:"
QR_BACKUP_DEPENDENCY = "brotli"
QR_BACKUP_PYDEPS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..", "pydeps"))
QR_BACKUP_WHEEL_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..", "third_party", "wheels"))
_qr_dependency_lock = threading.Lock()


# -----------------------
# Type inference / clamp
# -----------------------
def infer_type_from_setting(p: Optional[Dict[str, Any]]) -> str:
  """
  Fallback when get_type/ParamKeyType unavailable.
  returns one of: "bool","int","float","string","json","time"
  """
  if not p:
    return "string"
  mn, mx, d = p.get("min"), p.get("max"), p.get("default")

  # bool heuristic: min=0 max=1 and default is 0/1
  if mn in (0, 0.0) and mx in (1, 1.0) and d in (0, 1, 0.0, 1.0):
    return "bool"

  # int vs float
  if isinstance(mn, int) and isinstance(mx, int) and isinstance(d, int):
    return "int"

  if isinstance(mn, (int, float)) and isinstance(mx, (int, float)) and isinstance(d, (int, float)):
    if any(isinstance(x, float) for x in (mn, mx, d)):
      return "float"
    return "int"

  return "string"


def clamp_numeric(value: float, p: Optional[Dict[str, Any]]) -> float:
  if not p:
    return value
  mn = p.get("min")
  mx = p.get("max")
  try:
    if mn is not None:
      value = max(value, float(mn))
    if mx is not None:
      value = min(value, float(mx))
  except Exception:
    pass
  return value


def _read_param_value(params: "Params", name: str, default: Any) -> Any:
  try:
    t = params.get_type(name)

    if ParamKeyType is not None and t == ParamKeyType.BOOL:
      return bool(params.get_bool(name))

    if ParamKeyType is not None and t == ParamKeyType.INT:
      return int(params.get_int(name))

    if ParamKeyType is not None and t == ParamKeyType.FLOAT:
      return float(params.get_float(name))

    v = params.get(name)
    if v is None:
      return default if default is not None else ""
    if isinstance(v, (bytes, bytearray, memoryview)):
      return v.decode("utf-8", errors="replace")
    return str(v)

  except Exception:
    pass

  # fallback: raw get + minimal decode
  try:
    v = params.get(name)
    if v is None:
      return default if default is not None else ""
    return v.decode("utf-8", errors="replace")
  except Exception:
    return default if default is not None else ""


# -----------------------
# Single-key get/set (used by features/params and tools)
# -----------------------
def get_param_value(name: str, default: Any) -> Any:
  # 'GitPullTime' is a synthetic key backed by services/git_state.py.
  # Imported lazily to avoid a circular import at module load.
  from .git_state import read_custom_meta_value
  custom_value = read_custom_meta_value(name)
  if custom_value is not None:
    return custom_value

  if not HAS_PARAMS:
    s = _mem_store.get(name, None)
    return default if s is None else s

  return _read_param_value(Params(), name, default)


def get_param_values(names: list[str], defaults: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
  defaults = defaults or {}

  # Imported lazily to avoid a circular import at module load.
  from .git_state import read_custom_meta_value

  values: Dict[str, Any] = {}
  if not HAS_PARAMS:
    for name in names:
      custom_value = read_custom_meta_value(name)
      if custom_value is not None:
        values[name] = custom_value
      else:
        values[name] = _mem_store.get(name, defaults.get(name, 0))
    return values

  params = Params()
  for name in names:
    custom_value = read_custom_meta_value(name)
    if custom_value is not None:
      values[name] = custom_value
      continue
    values[name] = _read_param_value(params, name, defaults.get(name, 0))
  return values


def put_typed(params: "Params", key: str, value: Any) -> None:
  try:
      t = params.get_type(key)

      # BOOL
      if t == ParamKeyType.BOOL:
        v = value in ("1", "true", "True", "on", "yes") if isinstance(value, str) else bool(value)
        params.put_bool(key, v)
        return

      # INT
      if t == ParamKeyType.INT:
        params.put_int(key, int(float(value)))
        return

      # FLOAT
      if t == ParamKeyType.FLOAT:
        params.put_float(key, float(value))
        return

      # TIME (string ISO)
      if t == ParamKeyType.TIME:
        params.put(key, str(value))
        return

      # STRING
      if t == ParamKeyType.STRING:
        params.put(key, str(value))
        return

      # JSON
      if t == ParamKeyType.JSON:
        obj = json.loads(value) if isinstance(value, str) else value
        params.put(key, obj)

      # BYTES 등은 일단 스킵
      raise RuntimeError(f"Unsupported ParamKeyType for {key}: {t}")

  except Exception:
    # fall through to inference
    pass


def set_param_value(name: str, value: Any) -> None:
  if not HAS_PARAMS:
    _mem_store[name] = str(value)
    return
  params = Params()
  put_typed(params, name, value)


# -----------------------
# Bulk backup / restore
# -----------------------
def get_all_param_values_for_backup() -> Dict[str, str]:
  if not HAS_PARAMS or ParamKeyType is None:
    raise RuntimeError("Params/ParamKeyType not available")

  params = Params()
  out: Dict[str, str] = {}

  for k in params.all_keys():
    if isinstance(k, (bytes, bytearray, memoryview)):
      try:
        key = k.decode("utf-8")
      except Exception:
        continue
    else:
      key = str(k)

    try:
      t = params.get_type(key)
    except Exception:
      continue

    # skip heavy/unsupported
    if t in (ParamKeyType.BYTES, ParamKeyType.JSON):
      continue

    # default 없는 키 제외
    try:
      dv = params.get_default_value(key)
    except Exception:
      continue
    if dv is None:
      continue

    try:
      v = params.get(key, block=False, return_default=False)
    except Exception:
      v = None

    if v is None:
      v = dv

    if isinstance(v, (dict, list)):
      out[key] = json.dumps(v, ensure_ascii=False)
    else:
      out[key] = str(v)

  return out


def _backup_param_names() -> List[str]:
  if not HAS_PARAMS or ParamKeyType is None:
    raise RuntimeError("Params/ParamKeyType not available")

  params = Params()
  names: List[str] = []

  for k in params.all_keys():
    if isinstance(k, (bytes, bytearray, memoryview)):
      try:
        key = k.decode("utf-8")
      except Exception:
        continue
    else:
      key = str(k)

    try:
      t = params.get_type(key)
    except Exception:
      continue

    if t in (ParamKeyType.BYTES, ParamKeyType.JSON):
      continue

    try:
      if params.get_default_value(key) is None:
        continue
    except Exception:
      continue

    names.append(key)

  return names


def _backup_param_type_map(names: Optional[List[str]] = None) -> Dict[str, Any]:
  if not HAS_PARAMS or ParamKeyType is None:
    return {}

  params = Params()
  type_map: Dict[str, Any] = {}
  for key in names or _backup_param_names():
    try:
      type_map[key] = params.get_type(key)
    except Exception:
      continue
  return type_map


def restore_param_values_from_backup(values: Dict[str, Any]) -> Dict[str, Any]:
  if not HAS_PARAMS or ParamKeyType is None:
    raise RuntimeError("Params/ParamKeyType not available")

  params = Params()
  ok_cnt = 0
  fail_cnt = 0
  fails = []

  for key, value in values.items():
    try:
      t = params.get_type(key)

      if t == ParamKeyType.BOOL:
        v = value in ("1", "true", "True", "on", "yes") if isinstance(value, str) else bool(value)
        params.put_bool(key, v)

      elif t == ParamKeyType.INT:
        params.put_int(key, int(float(value)))

      elif t == ParamKeyType.FLOAT:
        params.put_float(key, float(value))

      elif t == ParamKeyType.TIME:
        params.put(key, str(value))

      elif t == ParamKeyType.STRING:
        params.put(key, str(value))

      else:
        # JSON/BYTES는 백업에서 제외했지만, 혹시 들어오면 skip
        continue

      ok_cnt += 1

    except Exception as e:
      fail_cnt += 1
      fails.append({"key": key, "err": str(e)})

  return {"ok_cnt": ok_cnt, "fail_cnt": fail_cnt, "fails": fails[:30]}


# -----------------------
# QR backup / restore
# -----------------------
def _ensure_pydeps_on_path() -> None:
  if QR_BACKUP_PYDEPS_DIR not in sys.path:
    sys.path.insert(0, QR_BACKUP_PYDEPS_DIR)


def _load_brotli_module() -> Any:
  global brotli
  if brotli is not None:
    return brotli
  _ensure_pydeps_on_path()
  brotli = importlib.import_module(QR_BACKUP_DEPENDENCY)
  return brotli


def get_qr_dependency_status() -> Dict[str, Any]:
  try:
    module = _load_brotli_module()
    return {
      "ok": True,
      "installed": True,
      "dependency": QR_BACKUP_DEPENDENCY,
      "format": QR_BACKUP_PREFIX_V3,
      "module_path": getattr(module, "__file__", ""),
      "target": QR_BACKUP_PYDEPS_DIR,
    }
  except Exception as exc:
    return {
      "ok": True,
      "installed": False,
      "dependency": QR_BACKUP_DEPENDENCY,
      "format": QR_BACKUP_PREFIX_V4,
      "module_path": "",
      "target": QR_BACKUP_PYDEPS_DIR,
      "error": str(exc),
    }


def ensure_qr_dependency() -> Dict[str, Any]:
  status = get_qr_dependency_status()
  if status.get("installed"):
    return {**status, "configured": False, "message": "already installed"}

  with _qr_dependency_lock:
    status = get_qr_dependency_status()
    if status.get("installed"):
      return {**status, "configured": False, "message": "already installed"}

    os.makedirs(QR_BACKUP_PYDEPS_DIR, exist_ok=True)
    _ensure_pydeps_on_path()
    cmd = [
      sys.executable or "python3",
      "-m", "pip", "install",
      "--target", QR_BACKUP_PYDEPS_DIR,
      "--upgrade",
      QR_BACKUP_DEPENDENCY,
    ]

    wheel_files = []
    try:
      wheel_files = [
        name for name in os.listdir(QR_BACKUP_WHEEL_DIR)
        if name.lower().startswith("brotli-") and name.endswith(".whl")
      ]
    except Exception:
      wheel_files = []

    if wheel_files:
      cmd = [
        sys.executable or "python3",
        "-m", "pip", "install",
        "--no-index",
        "--find-links", QR_BACKUP_WHEEL_DIR,
        "--target", QR_BACKUP_PYDEPS_DIR,
        "--upgrade",
        QR_BACKUP_DEPENDENCY,
      ]

    env = os.environ.copy()
    env["PYTHONPATH"] = QR_BACKUP_PYDEPS_DIR + os.pathsep + env.get("PYTHONPATH", "")
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=180, env=env, check=False)
    if proc.returncode != 0 and wheel_files:
      fallback_cmd = [
        sys.executable or "python3",
        "-m", "pip", "install",
        "--target", QR_BACKUP_PYDEPS_DIR,
        "--upgrade",
        QR_BACKUP_DEPENDENCY,
      ]
      proc = subprocess.run(fallback_cmd, capture_output=True, text=True, timeout=180, env=env, check=False)

    if proc.returncode != 0:
      return {
        "ok": False,
        "installed": False,
        "configured": False,
        "dependency": QR_BACKUP_DEPENDENCY,
        "format": QR_BACKUP_PREFIX_V4,
        "target": QR_BACKUP_PYDEPS_DIR,
        "error": (proc.stderr or proc.stdout or "pip install failed")[-1200:],
      }

    importlib.invalidate_caches()
    try:
      module = _load_brotli_module()
      return {
        "ok": True,
        "installed": True,
        "configured": True,
        "dependency": QR_BACKUP_DEPENDENCY,
        "format": QR_BACKUP_PREFIX_V3,
        "module_path": getattr(module, "__file__", ""),
        "target": QR_BACKUP_PYDEPS_DIR,
        "message": "installed",
      }
    except Exception as exc:
      return {
        "ok": False,
        "installed": False,
        "configured": False,
        "dependency": QR_BACKUP_DEPENDENCY,
        "format": QR_BACKUP_PREFIX_V4,
        "target": QR_BACKUP_PYDEPS_DIR,
        "error": str(exc),
      }


def _b64url_encode(data: bytes) -> str:
  return base64.urlsafe_b64encode(data).decode("ascii").rstrip("=")


def _b64url_decode(text: str) -> bytes:
  return base64.urlsafe_b64decode(text + ("=" * (-len(text) % 4)))


def _base45_encode(data: bytes) -> str:
  chars = QR_BACKUP_BASE45_ALPHABET
  out = []
  i = 0
  while i < len(data):
    if i + 1 < len(data):
      n = (data[i] << 8) + data[i + 1]
      out.append(chars[n % 45])
      out.append(chars[(n // 45) % 45])
      out.append(chars[n // (45 * 45)])
      i += 2
    else:
      n = data[i]
      out.append(chars[n % 45])
      out.append(chars[n // 45])
      i += 1
  return "".join(out)


def _base45_decode(text: str) -> bytes:
  chars = QR_BACKUP_BASE45_ALPHABET
  values = {ch: i for i, ch in enumerate(chars)}
  out = bytearray()
  i = 0
  while i < len(text):
    remaining = len(text) - i
    if remaining == 1:
      raise ValueError("bad base45 payload")
    if remaining >= 3:
      try:
        n = values[text[i]] + values[text[i + 1]] * 45 + values[text[i + 2]] * 45 * 45
      except KeyError:
        raise ValueError("bad base45 payload")
      if n > 0xFFFF:
        raise ValueError("bad base45 payload")
      out.append(n >> 8)
      out.append(n & 0xFF)
      i += 3
    else:
      try:
        n = values[text[i]] + values[text[i + 1]] * 45
      except KeyError:
        raise ValueError("bad base45 payload")
      if n > 0xFF:
        raise ValueError("bad base45 payload")
      out.append(n)
      i += 2
  return bytes(out)


def _write_varint(value: int) -> bytes:
  if value < 0:
    raise ValueError("negative varint")
  out = bytearray()
  while True:
    b = value & 0x7F
    value >>= 7
    if value:
      out.append(b | 0x80)
    else:
      out.append(b)
      return bytes(out)


def _read_varint(data: bytes, pos: int) -> tuple[int, int]:
  shift = 0
  value = 0
  while True:
    if pos >= len(data) or shift > 63:
      raise ValueError("bad varint")
    b = data[pos]
    pos += 1
    value |= (b & 0x7F) << shift
    if not (b & 0x80):
      return value, pos
    shift += 7


def _zigzag_encode(value: int) -> int:
  return (value << 1) ^ (value >> 63)


def _zigzag_decode(value: int) -> int:
  return (value >> 1) ^ -(value & 1)


def _param_short_code(key: str) -> str:
  digest = hashlib.sha256(key.encode("utf-8")).digest()[:QR_BACKUP_CODE_BYTES]
  return _b64url_encode(digest)


def _param_short_code_bytes(key: str, size: int = QR_BACKUP_CODE_BYTES_V3) -> bytes:
  return hashlib.sha256(key.encode("utf-8")).digest()[:size]


def _build_param_code_maps(names: List[str]) -> tuple[Dict[str, str], Dict[str, str]]:
  buckets: Dict[str, List[str]] = {}
  for name in sorted(set(str(n) for n in names)):
    buckets.setdefault(_param_short_code(name), []).append(name)

  code_to_key = {
    code: keys[0]
    for code, keys in buckets.items()
    if len(keys) == 1
  }
  key_to_code = {key: code for code, key in code_to_key.items()}
  return key_to_code, code_to_key


def _build_param_code_maps_bytes(names: List[str], size: int = QR_BACKUP_CODE_BYTES_V3) -> tuple[Dict[str, bytes], Dict[bytes, str]]:
  buckets: Dict[bytes, List[str]] = {}
  for name in sorted(set(str(n) for n in names)):
    buckets.setdefault(_param_short_code_bytes(name, size), []).append(name)

  code_to_key = {
    code: keys[0]
    for code, keys in buckets.items()
    if len(keys) == 1
  }
  key_to_code = {key: code for code, key in code_to_key.items()}
  return key_to_code, code_to_key


def _schema_fingerprint(names: List[str], size: int = QR_BACKUP_CODE_BYTES_V3) -> bytes:
  encoded_names = "\n".join(sorted(set(str(n) for n in names))).encode("utf-8")
  return hashlib.sha256(bytes([size]) + encoded_names).digest()[:QR_BACKUP_SCHEMA_BYTES]


def _is_int_string(value: str) -> bool:
  if not value:
    return False
  if value == "0":
    return True
  if value.startswith("-"):
    body = value[1:]
    return bool(body) and body.isdigit() and not (len(body) > 1 and body.startswith("0"))
  return value.isdigit() and not value.startswith("0")


def _encode_qr_value(value: Any, param_type: Any = None) -> bytes:
  if ParamKeyType is not None and param_type == ParamKeyType.BOOL:
    bool_value = value in ("1", "true", "True", "on", "yes") if isinstance(value, str) else bool(value)
    return b"\x02" if bool_value else b"\x01"

  text = str(value)
  if text == "":
    return b"\x00"
  if text == "0":
    return b"\x01"
  if text == "1":
    return b"\x02"

  if ParamKeyType is not None and param_type == ParamKeyType.INT:
    try:
      return b"\x03" + _write_varint(_zigzag_encode(int(float(value))))
    except Exception:
      pass

  if param_type is None and _is_int_string(text):
    return b"\x03" + _write_varint(_zigzag_encode(int(text)))

  raw = text.encode("utf-8")
  return b"\x04" + _write_varint(len(raw)) + raw


def _decode_qr_value(data: bytes, pos: int) -> tuple[str, int]:
  if pos >= len(data):
    raise ValueError("bad QR backup value")
  tag = data[pos]
  pos += 1
  if tag == 0:
    return "", pos
  if tag == 1:
    return "0", pos
  if tag == 2:
    return "1", pos
  if tag == 3:
    value, pos = _read_varint(data, pos)
    return str(_zigzag_decode(value)), pos
  if tag == 4:
    size, pos = _read_varint(data, pos)
    end = pos + size
    if end > len(data):
      raise ValueError("bad QR backup string")
    return data[pos:end].decode("utf-8"), end
  raise ValueError("unsupported QR backup value")


def _build_params_qr_payload_v2(values: Dict[str, Any]) -> Dict[str, Any]:
  try:
    code_names = _backup_param_names()
  except Exception:
    code_names = list(values.keys())
  key_to_code, _ = _build_param_code_maps(code_names)
  pairs = []
  fallback: Dict[str, Any] = {}
  for key in sorted(values.keys()):
    key_text = str(key)
    code = key_to_code.get(key_text)
    if code:
      pairs.append([code, values[key]])
    else:
      fallback[key_text] = values[key]

  envelope: List[Any] = [2, pairs]
  if fallback:
    envelope.append(fallback)
  raw = json.dumps(envelope, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
  compressed = zlib.compress(raw, 9)
  checksum = hashlib.sha256(compressed).hexdigest()[:QR_BACKUP_CHECKSUM_CHARS]
  payload = f"{QR_BACKUP_PREFIX_V2}.{_b64url_encode(compressed)}.{checksum}"
  return {
    "payload": payload,
    "format": QR_BACKUP_PREFIX_V2,
    "count": len(values),
    "json_bytes": len(raw),
    "compressed_bytes": len(compressed),
    "payload_chars": len(payload),
    "ecc": QR_BACKUP_ECC,
    "version": 2,
    "checksum": checksum,
  }


def _build_params_qr_binary(values: Dict[str, Any], version: int) -> bytes:
  try:
    code_names = _backup_param_names()
  except Exception:
    code_names = list(values.keys())

  type_map = _backup_param_type_map(code_names)
  key_to_code, _ = _build_param_code_maps_bytes(code_names)
  raw = bytearray()
  pairs = []
  fallback = []

  for key in sorted(values.keys()):
    key_text = str(key)
    encoded_value = _encode_qr_value(values[key], type_map.get(key_text))
    code = key_to_code.get(key_text)
    if code:
      pairs.append((code, encoded_value))
    else:
      fallback.append((key_text.encode("utf-8"), encoded_value))

  raw.append(version)
  raw.append(QR_BACKUP_CODE_BYTES_V3)
  raw.extend(_schema_fingerprint(code_names))
  raw.extend(_write_varint(len(pairs)))
  for code, encoded_value in pairs:
    raw.extend(code)
    raw.extend(encoded_value)

  raw.extend(_write_varint(len(fallback)))
  for key_bytes, encoded_value in fallback:
    raw.extend(_write_varint(len(key_bytes)))
    raw.extend(key_bytes)
    raw.extend(encoded_value)

  return bytes(raw)


def _build_params_qr_payload_v3(values: Dict[str, Any]) -> Dict[str, Any]:
  brotli_module = _load_brotli_module()

  raw_bytes = _build_params_qr_binary(values, 3)
  compressed = brotli_module.compress(raw_bytes, quality=11)
  checksum = hashlib.sha256(compressed).hexdigest()[:QR_BACKUP_CHECKSUM_CHARS].upper()
  payload = f"{QR_BACKUP_PREFIX_V3}:{_base45_encode(compressed)}:{checksum}"
  return {
    "payload": payload,
    "format": QR_BACKUP_PREFIX_V3,
    "count": len(values),
    "json_bytes": len(raw_bytes),
    "compressed_bytes": len(compressed),
    "payload_chars": len(payload),
    "ecc": QR_BACKUP_ECC,
    "version": 3,
    "checksum": checksum,
  }


def _build_params_qr_payload_v4(values: Dict[str, Any]) -> Dict[str, Any]:
  raw_bytes = _build_params_qr_binary(values, 4)
  compressed = zlib.compress(raw_bytes, 9)
  checksum = hashlib.sha256(compressed).hexdigest()[:QR_BACKUP_CHECKSUM_CHARS].upper()
  payload = f"{QR_BACKUP_PREFIX_V4}:{_base45_encode(compressed)}:{checksum}"
  return {
    "payload": payload,
    "format": QR_BACKUP_PREFIX_V4,
    "count": len(values),
    "json_bytes": len(raw_bytes),
    "compressed_bytes": len(compressed),
    "payload_chars": len(payload),
    "ecc": QR_BACKUP_ECC,
    "version": 4,
    "checksum": checksum,
  }


def build_params_qr_payload(values: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
  if values is None:
    values = get_all_param_values_for_backup()

  try:
    return _build_params_qr_payload_v3(values)
  except Exception:
    pass

  try:
    return _build_params_qr_payload_v4(values)
  except Exception:
    return _build_params_qr_payload_v2(values)


def _parse_params_qr_payload_v1(compressed: bytes, checksum_text: str) -> Dict[str, Any]:
  checksum = hashlib.sha256(compressed).hexdigest()[:16]
  if checksum != checksum_text:
    raise ValueError("QR payload checksum mismatch")

  raw = zlib.decompress(compressed)
  envelope = json.loads(raw.decode("utf-8"))
  if envelope.get("type") != QR_BACKUP_TYPE:
    raise ValueError("unsupported QR backup type")
  if int(envelope.get("version", 0)) > 1:
    raise ValueError("unsupported QR backup version")

  values = envelope.get("values")
  if not isinstance(values, dict):
    raise ValueError("bad QR backup values")
  return values


def _parse_params_qr_payload_v2(compressed: bytes, checksum_text: str) -> Dict[str, Any]:
  checksum = hashlib.sha256(compressed).hexdigest()[:QR_BACKUP_CHECKSUM_CHARS]
  if checksum != checksum_text:
    raise ValueError("QR payload checksum mismatch")

  raw = zlib.decompress(compressed)
  envelope = json.loads(raw.decode("utf-8"))

  if isinstance(envelope, list):
    if len(envelope) < 2:
      raise ValueError("bad QR backup format")
    version = int(envelope[0])
    pairs = envelope[1]
    fallback = envelope[2] if len(envelope) > 2 else {}
  elif isinstance(envelope, dict):
    version = int(envelope.get("v", 0))
    pairs = envelope.get("d")
    fallback = envelope.get("n", {})
  else:
    raise ValueError("bad QR backup format")

  if version > QR_BACKUP_VERSION:
    raise ValueError("unsupported QR backup version")
  if not isinstance(pairs, list):
    raise ValueError("bad QR backup values")
  if fallback is None:
    fallback = {}
  if not isinstance(fallback, dict):
    raise ValueError("bad QR backup fallback")

  _, code_to_key = _build_param_code_maps(_backup_param_names())
  values: Dict[str, Any] = {}
  for item in pairs:
    if not isinstance(item, list) or len(item) != 2:
      continue
    key = code_to_key.get(str(item[0]))
    if key:
      values[key] = item[1]

  for key, value in fallback.items():
    values[str(key)] = value

  return values


def _parse_params_qr_binary(raw: bytes) -> Dict[str, Any]:
  if len(raw) < 2 + QR_BACKUP_SCHEMA_BYTES:
    raise ValueError("bad QR backup format")

  version = raw[0]
  code_size = raw[1]
  pos = 2 + QR_BACKUP_SCHEMA_BYTES
  if version > QR_BACKUP_VERSION:
    raise ValueError("unsupported QR backup version")
  if code_size < 1 or code_size > 8:
    raise ValueError("bad QR backup code size")

  _, code_to_key = _build_param_code_maps_bytes(_backup_param_names(), code_size)
  values: Dict[str, Any] = {}
  pair_count, pos = _read_varint(raw, pos)
  for _ in range(pair_count):
    end = pos + code_size
    if end > len(raw):
      raise ValueError("bad QR backup code")
    code = raw[pos:end]
    pos = end
    value, pos = _decode_qr_value(raw, pos)
    key = code_to_key.get(code)
    if key:
      values[key] = value

  fallback_count, pos = _read_varint(raw, pos)
  for _ in range(fallback_count):
    key_size, pos = _read_varint(raw, pos)
    key_end = pos + key_size
    if key_end > len(raw):
      raise ValueError("bad QR backup key")
    key = raw[pos:key_end].decode("utf-8")
    pos = key_end
    value, pos = _decode_qr_value(raw, pos)
    values[key] = value

  if pos != len(raw):
    raise ValueError("bad QR backup trailing data")

  return values


def _parse_params_qr_payload_v3(payload: str) -> Dict[str, Any]:
  brotli_module = _load_brotli_module()
  prefix = f"{QR_BACKUP_PREFIX_V3}:"
  if not payload.startswith(prefix):
    raise ValueError("unsupported QR payload")

  try:
    encoded, checksum_text = payload[len(prefix):].rsplit(":", 1)
  except ValueError:
    raise ValueError("bad QR payload")

  compressed = _base45_decode(encoded)
  checksum = hashlib.sha256(compressed).hexdigest()[:QR_BACKUP_CHECKSUM_CHARS].upper()
  if checksum != checksum_text.upper():
    raise ValueError("QR payload checksum mismatch")

  return _parse_params_qr_binary(brotli_module.decompress(compressed))


def _parse_params_qr_payload_v4(payload: str) -> Dict[str, Any]:
  prefix = f"{QR_BACKUP_PREFIX_V4}:"
  if not payload.startswith(prefix):
    raise ValueError("unsupported QR payload")

  try:
    encoded, checksum_text = payload[len(prefix):].rsplit(":", 1)
  except ValueError:
    raise ValueError("bad QR payload")

  compressed = _base45_decode(encoded)
  checksum = hashlib.sha256(compressed).hexdigest()[:QR_BACKUP_CHECKSUM_CHARS].upper()
  if checksum != checksum_text.upper():
    raise ValueError("QR payload checksum mismatch")

  return _parse_params_qr_binary(zlib.decompress(compressed))


def parse_params_qr_payload(data: Any) -> Dict[str, Any]:
  if isinstance(data, dict):
    values = data.get("values") if isinstance(data.get("values"), dict) else data
    if not isinstance(values, dict):
      raise ValueError("bad payload format")
    return values

  payload = str(data or "").strip()
  if not payload:
    raise ValueError("empty payload")

  if payload.startswith("{"):
    j = json.loads(payload)
    return parse_params_qr_payload(j)

  if payload.startswith(f"{QR_BACKUP_PREFIX_V3}:"):
    return _parse_params_qr_payload_v3(payload)
  if payload.startswith(f"{QR_BACKUP_PREFIX_V4}:"):
    return _parse_params_qr_payload_v4(payload)

  parts = payload.split(".")
  if len(parts) != 3 or parts[0] not in (QR_BACKUP_PREFIX_V1, QR_BACKUP_PREFIX_V2):
    raise ValueError("unsupported QR payload")

  compressed = _b64url_decode(parts[1])
  if parts[0] == QR_BACKUP_PREFIX_V1:
    return _parse_params_qr_payload_v1(compressed, parts[2])
  return _parse_params_qr_payload_v2(compressed, parts[2])


def _param_type_name(t: Any) -> str:
  name = getattr(t, "name", None)
  return str(name or t)


def _is_unsupported_param_type(t: Any) -> bool:
  if ParamKeyType is None:
    return True
  return t in (ParamKeyType.BYTES, ParamKeyType.JSON)


def _normalize_param_value(t: Any, value: Any) -> Any:
  if ParamKeyType is not None and t == ParamKeyType.BOOL:
    if isinstance(value, str):
      v = value.strip().lower()
      if v in ("1", "true", "on", "yes"):
        return True
      if v in ("0", "false", "off", "no", ""):
        return False
    return bool(value)

  if ParamKeyType is not None and t == ParamKeyType.INT:
    return int(float(value))

  if ParamKeyType is not None and t == ParamKeyType.FLOAT:
    return float(value)

  return str(value)


def _values_equal(t: Any, left: Any, right: Any) -> bool:
  try:
    l_norm = _normalize_param_value(t, left)
    r_norm = _normalize_param_value(t, right)
    if ParamKeyType is not None and t == ParamKeyType.FLOAT:
      return abs(float(l_norm) - float(r_norm)) < 0.000001
    return l_norm == r_norm
  except Exception:
    return str(left) == str(right)


def preview_param_restore_values(values: Dict[str, Any], selected_keys: Optional[List[str]] = None) -> Dict[str, Any]:
  if not HAS_PARAMS or ParamKeyType is None:
    raise RuntimeError("Params/ParamKeyType not available")

  selected = set(selected_keys or [])
  params = Params()
  current_values = get_param_values(list(values.keys()), {})
  entries = []
  summary = {"changed": 0, "same": 0, "skipped": 0, "invalid": 0, "selected": 0}

  for key in sorted(values.keys()):
    raw_value = values[key]
    status = "changed"
    reason = ""
    can_apply = True
    type_name = "unknown"
    normalized_value: Any = raw_value

    try:
      t = params.get_type(key)
      type_name = _param_type_name(t)
      if _is_unsupported_param_type(t):
        status = "skipped"
        reason = "unsupported type"
        can_apply = False
      else:
        normalized_value = _normalize_param_value(t, raw_value)
        current_value = current_values.get(key, "")
        if _values_equal(t, current_value, normalized_value):
          status = "same"
          can_apply = False
    except Exception as e:
      current_value = current_values.get(key, "")
      status = "invalid"
      reason = str(e)
      can_apply = False

    is_selected = can_apply and (not selected or key in selected)
    if is_selected:
      summary["selected"] += 1
    summary[status] += 1
    entries.append({
      "key": key,
      "type": type_name,
      "current": current_values.get(key, ""),
      "value": normalized_value,
      "status": status,
      "reason": reason,
      "apply": is_selected,
    })

  return {
    "count": len(entries),
    "summary": summary,
    "entries": entries,
  }


def restore_param_values_validated(values: Dict[str, Any], selected_keys: Optional[List[str]] = None) -> Dict[str, Any]:
  preview = preview_param_restore_values(values, selected_keys)
  apply_values = {
    entry["key"]: entry["value"]
    for entry in preview["entries"]
    if entry.get("apply")
  }
  result = restore_param_values_from_backup(apply_values) if apply_values else {
    "ok_cnt": 0,
    "fail_cnt": 0,
    "fails": [],
  }
  return {
    "preview": preview,
    "result": result,
  }
