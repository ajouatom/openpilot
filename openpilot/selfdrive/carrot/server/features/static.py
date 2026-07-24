import asyncio
from html.parser import HTMLParser
import json
import os
import posixpath
import re
from typing import Final
from urllib.parse import unquote, unquote_plus, urlsplit, urlunsplit

from aiohttp import web

from ..config import SOUND_ASSETS_DIR, TRAINING_ASSETS_DIR, WEB_DIR
from ..services.asset_manifest import (
  AssetManifest,
  AssetManifestError,
  AssetManifestLoader,
  inject_asset_manifest,
)
from ..services.params import get_param_values
from ..services.static_assets import fingerprint_static_asset
from ..services.web_capabilities import resolve_web_capabilities, web_capability_client_spec
from ..services.web_settings import read_web_settings, web_settings_client_spec
from .intro.state import intro_bootstrap


_LANGUAGES_JSON_PATH = os.path.join(
  os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
  "ui", "translations", "languages.json",
)
_HTML_AMPERSAND_RE: Final = re.compile(r"&(?:amp|#0*38|#x0*26);", re.IGNORECASE)
_EXCLUDED_ASSET_PREFIXES: Final = (
  "/support-terminal-assets/",
  "/js/vendor/",
  "/css/vendor/",
)
_ASSET_MANIFEST_LOADER = AssetManifestLoader()
_INDEX_RETRY_DELAYS: Final = (0.0, 0.05, 0.1, 0.2, 0.4)
_EMPTY_ASSET_MANIFEST: Final[AssetManifest] = {"schemaVersion": 1, "assets": []}
_ASSET_RECOVERY_HTML: Final = """<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta http-equiv="refresh" content="1">
  <title>Carrot Web</title>
  <style>
    :root { color-scheme: dark; }
    body {
      align-items: center;
      background: #0b1016;
      color: #e8edf3;
      display: flex;
      font: 600 16px/1.5 system-ui, sans-serif;
      justify-content: center;
      margin: 0;
      min-height: 100vh;
    }
    main { text-align: center; }
    i {
      animation: spin .8s linear infinite;
      border: 3px solid #34404d;
      border-radius: 50%;
      border-top-color: #ff9f5a;
      display: block;
      height: 28px;
      margin: 0 auto 16px;
      width: 28px;
    }
    small { color: #8d99a6; display: block; font-weight: 500; margin-top: 4px; }
    @keyframes spin { to { transform: rotate(360deg); } }
    @media (prefers-reduced-motion: reduce) { i { animation: none; } }
  </style>
</head>
<body>
  <main role="status" aria-live="polite">
    <i aria-hidden="true"></i>
    Carrot Web 업데이트 적용 중
    <small>Applying update…</small>
  </main>
</body>
</html>
"""


class _StartTagLocator(HTMLParser):
  def __init__(self, html: str) -> None:
    super().__init__(convert_charrefs=False)
    self._line_offsets = [0, *(index + 1 for index, char in enumerate(html) if char == "\n")]
    self.spans: list[tuple[int, int]] = []

  def _record_start_tag(self) -> None:
    raw_tag = self.get_starttag_text()
    if raw_tag is None:
      return
    line, column = self.getpos()
    start = self._line_offsets[line - 1] + column
    self.spans.append((start, start + len(raw_tag)))

  def handle_starttag(self, _tag: str, _attrs: list[tuple[str, str | None]]) -> None:
    self._record_start_tag()

  def handle_startendtag(self, _tag: str, _attrs: list[tuple[str, str | None]]) -> None:
    self._record_start_tag()


def _load_device_languages() -> list:
  """Read selfdrive/ui/translations/languages.json and return
  a list of ``{code, name}`` dicts that the web client can use directly."""
  try:
    with open(_LANGUAGES_JSON_PATH, "r", encoding="utf-8") as f:
      mapping = json.load(f)  # e.g. {"English": "main_en", ...}
    return [{"code": code, "name": name} for name, code in mapping.items()]
  except Exception:  # noqa: BROAD_EXCEPT_OK - device language discovery is best-effort
    return []


def _build_bootstrap_payload() -> dict:  # noqa: DICT_OK - serialized mixed-shape client bootstrap
  try:
    device_values = get_param_values(
      ["LanguageSetting", "SoundLanguageSetting"],
      {"LanguageSetting": "", "SoundLanguageSetting": "auto"},
    )
    device_language = device_values.get("LanguageSetting", "")
    sound_language = device_values.get("SoundLanguageSetting", "auto")
  except Exception:  # noqa: BROAD_EXCEPT_OK - optional native Params backend varies by device
    device_language = ""
    sound_language = "auto"
  # Injected (not fetched) so the client can gate on the very first render —
  # a separate request would let the home page paint before the intro takes over.
  try:
    intro = intro_bootstrap()
  except Exception:  # noqa: BROAD_EXCEPT_OK - intro failure must not block index delivery
    # Never let the intro decision keep the page from loading.
    intro = {"shouldShow": False, "reason": "bootstrap_error"}

  web_settings = read_web_settings()
  return {
    "webSettings": web_settings,
    "webSettingsSpec": web_settings_client_spec(),
    "webCapabilities": resolve_web_capabilities(web_settings),
    "webCapabilitiesSpec": web_capability_client_spec(),
    "deviceLanguage": device_language,
    "soundLanguage": sound_language,
    "deviceLanguages": _load_device_languages(),
    "intro": intro,
  }


def _inject_bootstrap(html: str) -> str:
  payload = json.dumps(_build_bootstrap_payload(), ensure_ascii=False).replace("</", "<\\/")
  script = f'<script id="carrotBootstrap">window.__CARROT_BOOTSTRAP__ = {payload};</script>\n'
  marker = "<head>"
  if marker in html:
    return html.replace(marker, marker + "\n  " + script, 1)
  return script + html


def _fingerprinted_asset_url(raw_url: str, web_root: str) -> str | None:
  decoded_url = _HTML_AMPERSAND_RE.sub("&", raw_url)
  if decoded_url.startswith("//"):
    return None
  try:
    parts = urlsplit(decoded_url)
  except ValueError:
    return None
  if parts.scheme or parts.netloc:
    return None

  decoded_path = unquote(parts.path)
  policy_path = posixpath.normpath("/" + decoded_path.lstrip("/").replace("\\", "/"))
  if any(policy_path.startswith(prefix) for prefix in _EXCLUDED_ASSET_PREFIXES):
    return None
  fingerprint = fingerprint_static_asset(web_root, decoded_path)
  if fingerprint is None:
    return None

  query_pairs = [
    pair for pair in parts.query.split("&")
    if unquote_plus(pair.partition("=")[0]) != "v"
  ] if parts.query else []
  query_pairs.append(f"v={fingerprint}")
  rewritten = urlunsplit(("", "", parts.path, "&".join(query_pairs), parts.fragment))
  return rewritten.replace("&", "&amp;")


def _rewrite_start_tag_asset_urls(raw_tag: str, web_root: str) -> str:
  replacements: list[tuple[int, int, str]] = []
  length = len(raw_tag)
  index = 1
  while index < length and not raw_tag[index].isspace() and raw_tag[index] not in "/>":
    index += 1

  while index < length:
    while index < length and raw_tag[index].isspace():
      index += 1
    if index >= length or raw_tag[index] in "/>":
      break

    name_start = index
    while index < length and not raw_tag[index].isspace() and raw_tag[index] not in "/>=":
      index += 1
    if name_start == index:
      index += 1
      continue
    attribute_name = raw_tag[name_start:index]
    while index < length and raw_tag[index].isspace():
      index += 1
    if index >= length or raw_tag[index] != "=":
      continue

    index += 1
    while index < length and raw_tag[index].isspace():
      index += 1
    if index >= length:
      break
    if raw_tag[index] in "'\"":
      quote = raw_tag[index]
      value_start = index + 1
      value_end = raw_tag.find(quote, value_start)
      if value_end < 0:
        value_end = length
        index = length
      else:
        index = value_end + 1
    else:
      value_start = index
      while index < length and not raw_tag[index].isspace() and raw_tag[index] != ">":
        index += 1
      value_end = index

    if attribute_name.casefold() not in ("src", "href"):
      continue
    rewritten = _fingerprinted_asset_url(raw_tag[value_start:value_end], web_root)
    if rewritten is not None:
      replacements.append((value_start, value_end, rewritten))

  if not replacements:
    return raw_tag
  pieces: list[str] = []
  previous_end = 0
  for value_start, value_end, rewritten in replacements:
    pieces.extend((raw_tag[previous_end:value_start], rewritten))
    previous_end = value_end
  pieces.append(raw_tag[previous_end:])
  return "".join(pieces)


def _rewrite_index_asset_urls(html: str, web_root: str) -> str:
  locator = _StartTagLocator(html)
  locator.feed(html)
  locator.close()
  pieces: list[str] = []
  previous_end = 0
  for tag_start, tag_end in locator.spans:
    pieces.append(html[previous_end:tag_start])
    pieces.append(_rewrite_start_tag_asset_urls(html[tag_start:tag_end], web_root))
    previous_end = tag_end
  pieces.append(html[previous_end:])
  return "".join(pieces)


def _render_index_html(manifest: AssetManifest) -> str:
  index_path = os.path.join(WEB_DIR, "index.html")
  with open(index_path, encoding="utf-8") as f:
    html = inject_asset_manifest(f.read(), manifest)
  return _rewrite_index_asset_urls(html, WEB_DIR)


def _load_index_html() -> str:
  return _render_index_html(_ASSET_MANIFEST_LOADER.load(WEB_DIR))


def _load_index_html_without_manifest() -> str:
  # The manifest is a cache-busting and dynamic-worker catalog, not a reason
  # to take the entire web UI offline. Direct index assets are still
  # fingerprinted from their actual content by _rewrite_index_asset_urls().
  return _render_index_html(_EMPTY_ASSET_MANIFEST)


async def _load_index_after_update() -> tuple[str, bool] | None:
  for delay in _INDEX_RETRY_DELAYS:
    if delay:
      await asyncio.sleep(delay)
    try:
      return await asyncio.to_thread(_load_index_html), False
    except (AssetManifestError, OSError, UnicodeError):
      continue
  try:
    return await asyncio.to_thread(_load_index_html_without_manifest), True
  except (AssetManifestError, OSError, UnicodeError):
    return None


async def handle_index(request: web.Request) -> web.Response:
  loaded = await _load_index_after_update()
  if loaded is None:
    response = web.Response(
      status=503,
      text=_ASSET_RECOVERY_HTML,
      content_type="text/html",
    )
    response.headers["Retry-After"] = "1"
    response.headers["Cache-Control"] = "no-store"
    response.headers["X-Carrot-Asset-Status"] = "recovering"
    return response
  html, manifest_degraded = loaded
  html = _inject_bootstrap(html)
  response = web.Response(text=html, content_type="text/html")
  response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
  response.headers["Pragma"] = "no-cache"
  response.headers["Expires"] = "0"
  response.headers["X-Carrot-Asset-Status"] = "degraded" if manifest_degraded else "ready"
  return response


def register(app: web.Application) -> None:
  app.router.add_get("/", handle_index)
  if os.path.isdir(TRAINING_ASSETS_DIR):
    app.router.add_static("/training/", TRAINING_ASSETS_DIR, show_index=False)
  if os.path.isdir(SOUND_ASSETS_DIR):
    app.router.add_static("/sound-assets/", SOUND_ASSETS_DIR, show_index=False)
