import json
from typing import Any, Dict, Optional


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

  params = Params()
  try:
    t = params.get_type(name)

    if t == ParamKeyType.BOOL:
      return bool(params.get_bool(name))

    if t == ParamKeyType.INT:
      return int(params.get_int(name))

    if t == ParamKeyType.FLOAT:
      return float(params.get_float(name))

    # STRING / TIME / 기타는 raw string
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
