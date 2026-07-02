from __future__ import annotations

import asyncio
import hashlib
import json
import os
import socket
import time
from typing import Any
from urllib.parse import urlparse, urlunparse

from aiohttp import ClientSession

from .params import HAS_PARAMS, Params, get_param_values, infer_type_from_setting
from .settings import get_settings_cached


DEFAULT_TIMEOUT_S = 4.0
DEFAULT_RETRY_COUNT = 5
DEFAULT_RETRY_DELAY_S = 15.0
_DEFAULT_BASE_URL_KEY = 41
_DEFAULT_BASE_URL_BYTES = (
  65, 93, 93, 89, 90, 19, 6, 6, 74, 89, 95, 7, 67, 70, 68, 64, 71, 66, 64, 26,
  28, 29, 7, 69, 64, 95, 76,
)
USER_AGENT = "openpilot-carrot-param-value/1"
# Cloudflare Access service-token (optional). Sent as CF-Access-Client-Id /
# CF-Access-Client-Secret so devices pass when the server sits behind Access.
# Resolution order: env (CARROT_PARAM_VALUE_CF_ID/_SECRET) -> Params
# (CarrotParamValueCfId/CarrotParamValueCfSecret) -> embedded obfuscated default.
_DEFAULT_CF_ACCESS_ID_KEY = 41
_DEFAULT_CF_ACCESS_ID_BYTES: tuple[int, ...] = (
  25, 31, 76, 77, 24, 30, 75, 79, 31, 72, 75, 29, 26, 16, 17, 79, 28, 30, 28, 79,
  25, 31, 72, 25, 16, 16, 27, 31, 72, 79, 29, 75, 7, 72, 74, 74, 76, 90, 90,
)
_DEFAULT_CF_ACCESS_SECRET_KEY = 41
_DEFAULT_CF_ACCESS_SECRET_BYTES: tuple[int, ...] = (
  74, 31, 25, 29, 17, 79, 16, 74, 24, 24, 31, 29, 16, 24, 25, 76, 29, 76, 29, 26,
  75, 16, 27, 29, 24, 75, 16, 31, 27, 24, 74, 76, 76, 28, 74, 27, 30, 29, 79, 79,
  75, 79, 74, 74, 75, 16, 79, 16, 25, 72, 30, 24, 28, 29, 16, 28, 79, 16, 75, 16,
  31, 75, 76, 31,
)
_popular_values_memory: dict[str, Any] | None = None


def _decode_default_base_url() -> str:
  return "".join(chr(value ^ _DEFAULT_BASE_URL_KEY) for value in _DEFAULT_BASE_URL_BYTES)


def _env_float(name: str, default: float) -> float:
  try:
    return float(str(os.environ.get(name, "")).strip() or default)
  except Exception:
    return default


def _env_int(name: str, default: int) -> int:
  try:
    return int(str(os.environ.get(name, "")).strip() or default)
  except Exception:
    return default


def _param_text(params: Params, key: str, default: str = "") -> str:
  try:
    value = params.get(key)
    if isinstance(value, bytes):
      value = value.decode("utf-8", errors="replace")
    text = str(value or "").strip()
    return text or default
  except Exception:
    return default


def _meaningful_device_id(value: str) -> str:
  text = str(value or "").strip()
  if not text:
    return ""
  if text.lower() in {"unknown", "none", "null", "unregistereddevice"}:
    return ""
  return text


def _device_id(params: Params) -> str:
  for value in (
    _param_text(params, "DongleId"),
    _param_text(params, "HardwareSerial"),
  ):
    value = _meaningful_device_id(value)
    if value:
      return value
  return socket.gethostname() or "comma"


def _repo_id_from_remote(remote: str) -> str:
  text = str(remote or "").strip()
  if not text:
    return ""
  if text.startswith("git@github.com:"):
    text = "https://github.com/" + text.split(":", 1)[1]
  parsed = urlparse(text)
  path = parsed.path.strip("/")
  if path.endswith(".git"):
    path = path[:-4]
  parts = [part for part in path.split("/") if part]
  if len(parts) >= 2:
    return f"{parts[-2]}/{parts[-1]}"
  return ""


def _snapshot_url(params: Params | None = None) -> str:
  url = str(os.environ.get("CARROT_PARAM_VALUE_SNAPSHOT_URL", "")).strip()
  if not url and params is not None:
    url = _param_text(params, "CarrotParamValueSnapshotUrl")
  if url:
    return url

  base_url = str(os.environ.get("CARROT_PARAM_VALUE_URL", "")).strip()
  if not base_url and params is not None:
    base_url = _param_text(params, "CarrotParamValueUrl")
  if not base_url:
    base_url = _decode_default_base_url()
  if not base_url:
    return ""
  return base_url.rstrip("/") + "/api/carrot-settings/snapshot"


def _popular_url(params: Params | None = None) -> str:
  url = str(os.environ.get("CARROT_PARAM_VALUE_POPULAR_URL", "")).strip()
  if not url and params is not None:
    url = _param_text(params, "CarrotParamValuePopularUrl")
  if url:
    return url

  base_url = str(os.environ.get("CARROT_PARAM_VALUE_URL", "")).strip()
  if not base_url and params is not None:
    base_url = _param_text(params, "CarrotParamValueUrl")
  if not base_url:
    base_url = _decode_default_base_url()
  if base_url:
    return base_url.rstrip("/") + "/api/carrot-settings/popular"

  snapshot_url = _snapshot_url(params)
  if snapshot_url.endswith("/api/carrot-settings/snapshot"):
    return snapshot_url[: -len("/snapshot")] + "/popular"
  parsed = urlparse(snapshot_url)
  if not parsed.scheme or not parsed.netloc:
    return ""
  return urlunparse((parsed.scheme, parsed.netloc, "/api/carrot-settings/popular", "", "", ""))


def _settings_hash(data: dict[str, Any], param_names: list[str]) -> str:
  params = data.get("params", [])
  by_name = {str(item.get("name") or ""): item for item in params if isinstance(item, dict)}
  catalog = []
  for name in param_names:
    item = by_name.get(name, {})
    catalog.append({
      "name": name,
      "min": item.get("min"),
      "max": item.get("max"),
      "default": item.get("default"),
      "unit": item.get("unit"),
    })
  payload = {
    "apilot": data.get("apilot"),
    "params": catalog,
  }
  raw = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
  return hashlib.sha256(raw.encode("utf-8")).hexdigest()


def _param_catalog_payload(by_name: dict[str, dict[str, Any]], param_names: list[str]) -> dict[str, dict[str, Any]]:
  catalog: dict[str, dict[str, Any]] = {}
  for name in param_names:
    item = by_name.get(name, {})
    catalog[name] = {
      "min": item.get("min"),
      "max": item.get("max"),
      "default": item.get("default"),
      "unit": item.get("unit"),
    }
  return catalog


def _coerce_value(value: Any, setting: dict[str, Any]) -> Any:
  kind = infer_type_from_setting(setting)
  if kind == "bool":
    if isinstance(value, str):
      return 1 if value.strip().lower() in {"1", "true", "on", "yes"} else 0
    return 1 if bool(value) else 0
  if kind == "int":
    try:
      return int(float(value))
    except Exception:
      return int(setting.get("default") or 0)
  if kind == "float":
    try:
      return float(value)
    except Exception:
      return float(setting.get("default") or 0.0)
  return value


def build_snapshot_payload() -> dict[str, Any] | None:
  if not HAS_PARAMS:
    return None

  data, _, by_name, _ = get_settings_cached()
  param_names = [name for name in by_name.keys() if name]
  if not param_names:
    return None

  params = Params()
  car_key = _param_text(params, "CarSelected3")
  if not car_key:
    return None
  device_id = _device_id(params)
  if not device_id:
    return None

  defaults = {name: by_name.get(name, {}).get("default", 0) for name in param_names}
  raw_values = get_param_values(param_names, defaults)
  values = {
    name: _coerce_value(raw_values.get(name, defaults.get(name, 0)), by_name.get(name, {}))
    for name in param_names
  }

  repo_remote = _param_text(params, "GitRemote")
  settings_version = data.get("apilot")
  try:
    settings_version = int(settings_version)
  except Exception:
    settings_version = None

  return {
    "schema_version": 1,
    "device_id": device_id,
    "repo_id": _repo_id_from_remote(repo_remote),
    "repo_remote": repo_remote,
    "car_key_type": "CarSelected3",
    "car_key": car_key,
    "settings_version": settings_version,
    "settings_hash": _settings_hash(data, param_names),
    "app_commit": _param_text(params, "GitCommit"),
    "param_catalog": _param_catalog_payload(by_name, param_names),
    "values": values,
  }


def _current_settings_hash() -> str:
  if not HAS_PARAMS:
    return ""
  try:
    data, _, by_name, _ = get_settings_cached()
    param_names = [name for name in by_name.keys() if name]
    return _settings_hash(data, param_names) if param_names else ""
  except Exception:
    return ""


def _current_car_key() -> str:
  if not HAS_PARAMS:
    return ""
  return _param_text(Params(), "CarSelected3")


def _empty_cache(car_key: str = "", settings_hash: str = "") -> dict[str, Any]:
  return {
    "ok": True,
    "source": "empty",
    "updated_at": 0,
    "fetched_at": 0,
    "car_key_type": "CarSelected3",
    "car_key": car_key,
    "settings_hash": settings_hash,
    "popular_values": {},
  }


def read_popular_values_memory() -> dict[str, Any]:
  data = _popular_values_memory
  if not isinstance(data, dict):
    return _empty_cache(_current_car_key(), _current_settings_hash())
  current_car_key = _current_car_key()
  current_settings_hash = _current_settings_hash()
  cached_car_key = str(data.get("car_key") or "")
  cached_settings_hash = str(data.get("settings_hash") or "")
  if current_car_key and cached_car_key and current_car_key != cached_car_key:
    return _empty_cache(current_car_key, current_settings_hash)
  if current_settings_hash and cached_settings_hash and current_settings_hash != cached_settings_hash:
    return _empty_cache(current_car_key, current_settings_hash)
  result = dict(data)
  result.setdefault("ok", True)
  result.setdefault("source", "memory")
  result.setdefault("settings_hash", current_settings_hash)
  result.setdefault("popular_values", {})
  return result


def store_popular_values_memory(data: dict[str, Any]) -> dict[str, Any]:
  global _popular_values_memory
  clean = {
    "ok": bool(data.get("ok", True)),
    "source": "remote",
    "fetched_at": int(time.time()),
    "car_key_type": str(data.get("car_key_type") or "CarSelected3"),
    "car_key": str(data.get("car_key") or ""),
    "settings_hash": str(data.get("settings_hash") or _current_settings_hash()),
    "popular_values": data.get("popular_values") if isinstance(data.get("popular_values"), dict) else {},
  }
  _popular_values_memory = clean
  return clean


def get_popular_value_detail(name: str) -> dict[str, Any]:
  cache = read_popular_values_memory()
  values = cache.get("popular_values")
  if not isinstance(values, dict):
    values = {}
  detail = values.get(str(name or ""))
  if not isinstance(detail, dict):
    return {"top_values": []}
  return detail


def _cf_access_token(params: Params | None = None) -> tuple[str, str]:
  cid = str(os.environ.get("CARROT_PARAM_VALUE_CF_ID", "")).strip()
  sec = str(os.environ.get("CARROT_PARAM_VALUE_CF_SECRET", "")).strip()
  if params is not None:
    if not cid:
      cid = _param_text(params, "CarrotParamValueCfId")
    if not sec:
      sec = _param_text(params, "CarrotParamValueCfSecret")
  if not cid:
    cid = "".join(chr(v ^ _DEFAULT_CF_ACCESS_ID_KEY) for v in _DEFAULT_CF_ACCESS_ID_BYTES)
  if not sec:
    sec = "".join(chr(v ^ _DEFAULT_CF_ACCESS_SECRET_KEY) for v in _DEFAULT_CF_ACCESS_SECRET_BYTES)
  return cid, sec


def _request_headers(params: Params | None = None) -> dict[str, str]:
  headers = {"User-Agent": USER_AGENT}
  cid, sec = _cf_access_token(params)
  if cid and sec:
    headers["CF-Access-Client-Id"] = cid
    headers["CF-Access-Client-Secret"] = sec
  return headers


async def _post_snapshot(session: ClientSession, url: str, payload: dict[str, Any], timeout_s: float, headers: dict[str, str]) -> tuple[bool, int, str]:
  try:
    async with session.post(url, json=payload, timeout=timeout_s, headers=headers) as resp:
      text = await resp.text()
      return 200 <= resp.status < 300, int(resp.status), text
  except Exception as exc:
    return False, 0, str(exc)


async def download_popular_values_once(session: ClientSession) -> dict[str, Any] | None:
  if not HAS_PARAMS:
    return None
  params = Params()
  url = _popular_url(params)
  car_key = _param_text(params, "CarSelected3")
  settings_hash = _current_settings_hash()
  if not url or not car_key:
    return None

  timeout_s = max(1.0, _env_float("CARROT_PARAM_VALUE_TIMEOUT_S", DEFAULT_TIMEOUT_S))
  try:
    async with session.get(
      url,
      params={"car_key_type": "CarSelected3", "car_key": car_key, "settings_hash": settings_hash},
      timeout=timeout_s,
      headers=_request_headers(params),
    ) as resp:
      data = await resp.json(content_type=None)
      if resp.status < 200 or resp.status >= 300 or not isinstance(data, dict):
        print(f"[carrot_param_value] popular download failed status={resp.status}", flush=True)
        return None
      saved = store_popular_values_memory(data)
      print(
        f"[carrot_param_value] popular loaded car_key={saved.get('car_key')} params={len(saved.get('popular_values') or {})}",
        flush=True,
      )
      return saved
  except Exception as exc:
    print(f"[carrot_param_value] popular download failed: {exc}", flush=True)
    return None


async def popular_value_upload_once(session: ClientSession) -> bool:
  if not HAS_PARAMS:
    return False
  params = Params()
  url = _snapshot_url(params)
  if not url:
    return False

  try:
    payload = build_snapshot_payload()
  except Exception as exc:
    print(f"[carrot_param_value] snapshot build failed: {exc}", flush=True)
    return False
  if not payload:
    return False

  timeout_s = max(1.0, _env_float("CARROT_PARAM_VALUE_TIMEOUT_S", DEFAULT_TIMEOUT_S))
  retry_count = max(1, _env_int("CARROT_PARAM_VALUE_RETRY_COUNT", DEFAULT_RETRY_COUNT))
  retry_delay_s = max(1.0, _env_float("CARROT_PARAM_VALUE_RETRY_DELAY_S", DEFAULT_RETRY_DELAY_S))

  headers = _request_headers(params)
  for attempt in range(1, retry_count + 1):
    ok, status, body = await _post_snapshot(session, url, payload, timeout_s, headers)
    if ok:
      print(
        f"[carrot_param_value] uploaded car_key={payload.get('car_key')} params={len(payload.get('values') or {})}",
        flush=True,
      )
      return True
    print(f"[carrot_param_value] upload failed attempt={attempt}/{retry_count} status={status} body={body[:160]}", flush=True)
    if attempt < retry_count:
      await asyncio.sleep(retry_delay_s)
  return False


async def refresh_popular_values_once(session: ClientSession, *, upload: bool = True) -> dict[str, Any]:
  uploaded = False
  if upload:
    uploaded = await popular_value_upload_once(session)
  cache = await download_popular_values_once(session)
  if cache is None:
    cache = read_popular_values_memory()
  cache["uploaded"] = uploaded
  return cache


def start_popular_value_upload(app: Any) -> asyncio.Task | None:
  session = app.get("http")
  if session is None:
    return None
  return asyncio.create_task(refresh_popular_values_once(session, upload=True))


_popular_refresh_last_at = 0.0
_popular_refresh_task: asyncio.Task | None = None


def schedule_popular_value_refresh(app: Any, min_interval: float | None = None) -> None:
  """Kick a throttled, download-only refresh from the central server.

  Non-blocking and single-flight: called on each settings read so the web
  reflects fleet changes within ~min_interval, without polling or websockets.
  Does NOT re-upload (that only happens once at boot)."""
  global _popular_refresh_last_at, _popular_refresh_task
  session = app.get("http")
  if session is None:
    return
  interval = (
    _env_float("CARROT_PARAM_VALUE_REFRESH_MIN_S", 60.0)
    if min_interval is None else min_interval
  )
  now = time.time()
  if now - _popular_refresh_last_at < interval:
    return
  if _popular_refresh_task is not None and not _popular_refresh_task.done():
    return
  _popular_refresh_last_at = now
  _popular_refresh_task = asyncio.create_task(download_popular_values_once(session))
