import json
import os

from aiohttp import web

from ..config import TRAINING_ASSETS_DIR, WEB_DIR
from ..services.params import get_param_values
from ..services.web_settings import read_web_settings


def _build_bootstrap_payload() -> dict:
  try:
    device_values = get_param_values(["LanguageSetting"], {"LanguageSetting": ""})
    device_language = device_values.get("LanguageSetting", "")
  except Exception:
    device_language = ""
  return {
    "webSettings": read_web_settings(),
    "deviceLanguage": device_language,
  }


def _inject_bootstrap(html: str) -> str:
  payload = json.dumps(_build_bootstrap_payload(), ensure_ascii=False).replace("</", "<\\/")
  script = f'<script id="carrotBootstrap">window.__CARROT_BOOTSTRAP__ = {payload};</script>\n'
  marker = "<head>"
  if marker in html:
    return html.replace(marker, marker + "\n  " + script, 1)
  return script + html


async def handle_index(request: web.Request) -> web.Response:
  index_path = os.path.join(WEB_DIR, "index.html")
  with open(index_path, "r", encoding="utf-8") as f:
    html = _inject_bootstrap(f.read())
  response = web.Response(text=html, content_type="text/html")
  response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
  response.headers["Pragma"] = "no-cache"
  response.headers["Expires"] = "0"
  return response


def register(app: web.Application) -> None:
  app.router.add_get("/", handle_index)
  if os.path.isdir(TRAINING_ASSETS_DIR):
    app.router.add_static("/training/", TRAINING_ASSETS_DIR, show_index=False)
