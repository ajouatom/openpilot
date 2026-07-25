import argparse
import asyncio
import logging
import os

from aiohttp import web

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.webrtc import webrtcd
from openpilot.system.webrtc.carrot_session import get_stream, on_shutdown, stream_session_cleanup_context


def carrot_webrtcd_thread(host: str, port: int, debug: bool) -> None:
  webrtcd._set_carrot_vision_active(False)
  logging.basicConfig(level=logging.CRITICAL, handlers=[logging.StreamHandler()])
  logging_level = logging.DEBUG if debug else logging.INFO
  logging.getLogger("WebRTCStream").setLevel(logging_level)
  logging.getLogger("webrtcd").setLevel(logging_level)

  app = web.Application(middlewares=[webrtcd.cors_middleware])
  app["streams"] = {}
  app["stream_lock"] = asyncio.Lock()
  app["debug"] = debug
  app.cleanup_ctx.append(stream_session_cleanup_context)
  app.on_shutdown.append(on_shutdown)
  app.router.add_post("/stream", get_stream)
  app.router.add_post("/notify", webrtcd.post_notify)
  app.router.add_get("/schema", webrtcd.get_schema)
  app.router.add_route("OPTIONS", "/{tail:.*}", webrtcd.handle_cors_preflight)
  web.run_app(app, host=host, port=port)


def main() -> None:
  parser = argparse.ArgumentParser(description="Carrot Vision WebRTC daemon")
  parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to listen on")
  parser.add_argument("--port", type=int, default=5001, help="Port to listen on")
  parser.add_argument("--debug", action="store_true", help="Enable debug mode")
  args = parser.parse_args()

  # This process is the only owner of Carrot Vision mode. The standard
  # openpilot WebRTC daemon keeps its original session lifecycle.
  webrtcd._carrot_vision_mode = True
  webrtcd._carrot_vision_params = Params()

  try:
    from openpilot.common.realtime import set_core_affinity
    cores = [int(core) for core in os.getenv("CARROT_VISION_WEBRTC_CORES", "0,1,2,3").split(",") if core.strip()]
    if cores:
      set_core_affinity(cores)
  except Exception:
    cloudlog.exception("[webrtcd] failed setting Carrot Vision core affinity")

  carrot_webrtcd_thread(args.host, args.port, args.debug)


if __name__ == "__main__":
  main()
