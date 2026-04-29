#!/usr/bin/env python3
import argparse
import logging
import os

from aiohttp import web
from openpilot.common.realtime import set_core_affinity

from .server.app import make_app
from .server.config import DEFAULT_SETTINGS_PATH, WEB_DIR
from .server.services.settings import settings_cache as _settings_cache


def main():
  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    print("[carrot_server] failed to set core affinity")

  parser = argparse.ArgumentParser()
  parser.add_argument("--host", type=str, default="0.0.0.0")
  parser.add_argument("--port", type=int, default=7000)
  parser.add_argument("--settings", type=str, default=DEFAULT_SETTINGS_PATH,
                      help="path to carrot_settings.json")
  args = parser.parse_args()

  _settings_cache["path"] = args.settings

  if not os.path.isdir(WEB_DIR):
    raise RuntimeError(f"web dir not found: {WEB_DIR}")
  if not os.path.exists(_settings_cache["path"]):
    print(f"[WARN] settings file not found: {_settings_cache['path']}")

  logging.getLogger("aiohttp.access").setLevel(logging.WARNING)
  print(f"[carrot_server] serving {WEB_DIR} on {args.host}:{args.port}")
  web.run_app(make_app(), host=args.host, port=args.port)


if __name__ == "__main__":
  main()
