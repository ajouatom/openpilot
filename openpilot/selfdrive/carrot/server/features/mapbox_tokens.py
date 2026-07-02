from __future__ import annotations

import json
import asyncio
import urllib.error
import urllib.parse
import urllib.request

from aiohttp import web

from ..services.params import get_param_value, set_param_value


MAPBOX_PUBLIC_KEY_PARAM = "MapboxPublicKey"
MAPBOX_SECRET_KEY_PARAM = "MapboxSecretKey"

TOKEN_PARAMS = {
  "public": MAPBOX_PUBLIC_KEY_PARAM,
  "secret": MAPBOX_SECRET_KEY_PARAM,
}


def _read_token(key_type: str) -> str:
  param = TOKEN_PARAMS.get(key_type)
  if not param:
    return ""
  return str(get_param_value(param, "") or "").strip()


def _mask_token(value: str) -> str:
  token = str(value or "").strip()
  if not token:
    return ""
  if len(token) <= 12:
    return token[:3] + "…" if len(token) > 3 else "…"
  return f"{token[:8]}…{token[-4:]}"


def _token_status() -> dict:
  public_key = _read_token("public")
  secret_key = _read_token("secret")
  return {
    "public": {
      "configured": bool(public_key),
      "masked": _mask_token(public_key),
      "param": MAPBOX_PUBLIC_KEY_PARAM,
    },
    "secret": {
      "configured": bool(secret_key),
      "masked": _mask_token(secret_key),
      "param": MAPBOX_SECRET_KEY_PARAM,
    },
  }


def _format_result(key_type: str, token: str) -> dict:
  token = str(token or "").strip()
  label = "public" if key_type == "public" else "secret"
  prefix = "pk." if key_type == "public" else "sk."

  if not token:
    return {"ok": False, "format_ok": False, "reason": "required", "message": f"Mapbox {label} token is required."}
  if any(ch.isspace() for ch in token):
    return {"ok": False, "format_ok": False, "reason": "space", "message": "Mapbox token must not contain spaces."}
  if not token.startswith(prefix):
    return {"ok": False, "format_ok": False, "reason": "prefix", "message": f"Mapbox {label} token should start with {prefix}"}
  if len(token) < 20:
    return {"ok": False, "format_ok": False, "reason": "short", "message": "Mapbox token is too short."}
  return {"ok": True, "format_ok": True, "reason": "ok", "message": "Mapbox token format looks valid."}


def _validate_public_token_online(token: str) -> dict:
  # Match the real navd usage as closely as practical: Mapbox Directions API.
  coords = "126.9780,37.5665;126.9790,37.5675"
  query = urllib.parse.urlencode({
    "access_token": token,
    "overview": "false",
    "steps": "false",
    "geometries": "geojson",
  })
  url = f"https://api.mapbox.com/directions/v5/mapbox/driving-traffic/{coords}?{query}"
  request = urllib.request.Request(url, headers={"User-Agent": "CarrotPilot Mapbox token check"})

  try:
    with urllib.request.urlopen(request, timeout=8) as response:
      body = response.read(4096).decode("utf-8", errors="replace")
      data = json.loads(body) if body else {}
      code = str(data.get("code", "")).lower()
      ok = 200 <= response.status < 300 and (not code or code == "ok")
      return {
        "online_ok": ok,
        "http_status": response.status,
        "message": "Mapbox Directions API is reachable." if ok else (data.get("message") or data.get("code") or "Mapbox validation failed."),
      }
  except urllib.error.HTTPError as exc:
    message = f"HTTP {exc.code}"
    try:
      data = json.loads(exc.read(4096).decode("utf-8", errors="replace") or "{}")
      message = data.get("message") or data.get("code") or message
    except Exception:
      pass
    return {"online_ok": False, "http_status": exc.code, "message": message}
  except Exception as exc:
    return {"online_ok": False, "http_status": None, "message": str(exc)}


async def api_get_tokens(request: web.Request) -> web.Response:
  return web.json_response({"ok": True, **_token_status()})


async def api_set_token(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)

  key_type = str(body.get("key_type", body.get("type", "public")) or "public").strip().lower()
  token = str(body.get("token", body.get("value", "")) or "").strip()
  param = TOKEN_PARAMS.get(key_type)
  if not param:
    return web.json_response({"ok": False, "error": "invalid key_type"}, status=400)

  result = _format_result(key_type, token)
  if not result.get("format_ok"):
    return web.json_response({"ok": False, "error": result.get("message", "invalid token"), **result}, status=400)

  try:
    set_param_value(param, token)
  except Exception as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=500)

  return web.json_response({"ok": True, "key_type": key_type, **_token_status()})


async def api_clear_token(request: web.Request) -> web.Response:
  key_type = str(request.query.get("key_type", request.query.get("type", "public")) or "public").strip().lower()
  param = TOKEN_PARAMS.get(key_type)
  if not param:
    return web.json_response({"ok": False, "error": "invalid key_type"}, status=400)

  try:
    set_param_value(param, "")
  except Exception as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=500)

  return web.json_response({"ok": True, "key_type": key_type, **_token_status()})


async def api_validate_token(request: web.Request) -> web.Response:
  body = {}
  if request.can_read_body:
    try:
      body = await request.json()
    except Exception:
      return web.json_response({"ok": False, "error": "invalid json"}, status=400)

  key_type = str(body.get("key_type", body.get("type", "public")) if isinstance(body, dict) else "public").strip().lower()
  if key_type not in TOKEN_PARAMS:
    return web.json_response({"ok": False, "error": "invalid key_type"}, status=400)

  has_explicit_token = isinstance(body, dict) and ("token" in body or "value" in body)
  raw_token = body.get("token", body.get("value", "")) if has_explicit_token else ""
  token = str(raw_token or "").strip()
  if not has_explicit_token:
    token = _read_token(key_type)

  result = _format_result(key_type, token)
  if key_type == "public" and result.get("format_ok"):
    online = await asyncio.to_thread(_validate_public_token_online, token)
    result.update(online)
    result["ok"] = bool(result.get("format_ok") and result.get("online_ok"))
  elif key_type == "secret" and result.get("format_ok"):
    result.update({
      "online_ok": None,
      "message": "Secret key format looks valid. This build does not use MapboxSecretKey at runtime.",
    })

  return web.json_response(result, status=200 if result.get("ok") else 409)


def register(app: web.Application) -> None:
  app.router.add_get("/api/mapbox/tokens", api_get_tokens)
  app.router.add_post("/api/mapbox/token", api_set_token)
  app.router.add_delete("/api/mapbox/token", api_clear_token)
  app.router.add_post("/api/mapbox/token/validate", api_validate_token)
