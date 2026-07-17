import asyncio
import json
import os
import re
import sys

from aiohttp import web, WSMsgType

from ..config import TMUX_WEB_SESSION
from ..services import tmux
from ..services.terminal_pty import PTY_SESSION
from ..terminal_commands import translate_meta_command
from ..terminal_commands.custom_commands import load_commands
from ..terminal_commands.registry import get_command, iter_commands


TMUX_ATTACH_RE = re.compile(r"^\s*tmux\s+(?:a|attach|attach-session)(?:\s*)$", re.IGNORECASE)
TMUX_ATTACH_TARGET_RE = re.compile(r"^\s*tmux\s+(?:a|attach|attach-session)\s+-t\s+\S+\s*$", re.IGNORECASE)


def _translate_terminal_line(line: str, *, nested_tmux: bool = False) -> str:
  translated = translate_meta_command(line)
  if translated:
    return translated
  if not nested_tmux:
    return str(line or "")
  text = str(line or "")
  if TMUX_ATTACH_RE.match(text):
    return "TMUX= tmux a -t comma"
  if TMUX_ATTACH_TARGET_RE.match(text):
    return f"TMUX= {text.strip()}"
  return text


async def ws_terminal(request: web.Request) -> web.WebSocketResponse:
  ws = web.WebSocketResponse(heartbeat=20, compress=False)
  await ws.prepare(request)

  session = (request.query.get("session") or TMUX_WEB_SESSION).strip() or TMUX_WEB_SESSION
  last_screen = None

  try:
    created = await asyncio.to_thread(tmux.ensure_session, session)
    await ws.send_str(json.dumps({
      "type": "meta",
      "session": session,
      "created": created,
      "user": "comma",
    }))
  except Exception as e:
    await ws.send_str(json.dumps({
      "type": "error",
      "error": str(e),
      "session": session,
    }))
    await ws.close()
    return ws

  async def push_screen(force: bool = False, delay: float = 0.0) -> None:
    nonlocal last_screen
    if delay > 0:
      await asyncio.sleep(delay)
    screen = await asyncio.to_thread(tmux.capture, session)
    if force or screen != last_screen:
      last_screen = screen
      await ws.send_str(json.dumps({
        "type": "screen",
        "session": session,
        "text": screen,
      }))

  async def pump_screen():
    while not ws.closed:
      try:
        await push_screen(force=False)
      except asyncio.CancelledError:
        raise
      except Exception as e:
        await ws.send_str(json.dumps({
          "type": "error",
          "error": str(e),
          "session": session,
        }))
        break
      await asyncio.sleep(0.18)

  pump_task = asyncio.create_task(pump_screen())

  try:
    await push_screen(force=True, delay=0.02)
    async for msg in ws:
      if msg.type == WSMsgType.TEXT:
        try:
          data = json.loads(msg.data)
        except Exception:
          continue

        typ = data.get("type")
        try:
          if typ == "input":
            line = str(data.get("data") or "")
            await asyncio.to_thread(tmux.send_line, session, _translate_terminal_line(line, nested_tmux=True))
            await push_screen(force=True, delay=0.03)
          elif typ == "control":
            action = (data.get("action") or "").strip()
            if action == "ctrl_c":
              await asyncio.to_thread(tmux.ctrl_c, session)
              await push_screen(force=True, delay=0.03)
            elif action == "clear":
              await asyncio.to_thread(tmux.clear, session)
              await push_screen(force=True, delay=0.05)
            elif action == "refresh":
              await push_screen(force=True)
            elif action == "new_session":
              created = await asyncio.to_thread(tmux.ensure_session, session)
              await ws.send_str(json.dumps({
                "type": "meta",
                "session": session,
                "created": created,
                "user": "comma",
              }))
              await push_screen(force=True, delay=0.08)
        except Exception as e:
          await ws.send_str(json.dumps({
            "type": "error",
            "error": str(e),
            "session": session,
          }))
      elif msg.type in (WSMsgType.ERROR, WSMsgType.CLOSE, WSMsgType.CLOSING):
        break
  finally:
    pump_task.cancel()
    try:
      await pump_task
    except Exception:
      pass
    try:
      await ws.close()
    except Exception:
      pass
  return ws


async def ws_terminal_pty(request: web.Request) -> web.WebSocketResponse:
  ws = web.WebSocketResponse(heartbeat=20, compress=False)
  await ws.prepare(request)

  rows = int(request.query.get("rows") or 28)
  cols = int(request.query.get("cols") or 100)
  reset = request.query.get("reset") in ("1", "true", "yes")

  try:
    if reset:
      await PTY_SESSION.terminate()
    await PTY_SESSION.attach(ws, rows, cols)
  except Exception as e:
    await ws.send_str(json.dumps({
      "type": "error",
      "error": str(e),
      "session": PTY_SESSION.session,
    }))
    await ws.close()
    return ws

  try:
    async for msg in ws:
      if msg.type == WSMsgType.TEXT:
        try:
          data = json.loads(msg.data)
        except Exception:
          continue
        typ = data.get("type")
        try:
          if typ == "input":
            line = _translate_terminal_line(str(data.get("data") or ""))
            await PTY_SESSION.write_text(line + "\r")
          elif typ == "raw":
            text = str(data.get("data") or "")
            if text:
              await PTY_SESSION.write_text(text)
          elif typ == "resize":
            await PTY_SESSION.resize(ws, int(data.get("rows") or rows), int(data.get("cols") or cols))
          elif typ == "control":
            action = (data.get("action") or "").strip()
            if action == "ctrl_c":
              await PTY_SESSION.write(b"\x03")
            elif action == "clear":
              await PTY_SESSION.clear_history()
              await PTY_SESSION.write(b"clear\r")
            elif action == "refresh":
              await PTY_SESSION.write(b"\x0c")
            elif action == "detach":
              # AGNOS tmux prefix is backtick, not Ctrl-B, so detach = ` then d.
              await PTY_SESSION.write(b"\x60d")
        except Exception as e:
          await ws.send_str(json.dumps({
            "type": "error",
            "error": str(e),
            "session": PTY_SESSION.session,
          }))
      elif msg.type in (WSMsgType.ERROR, WSMsgType.CLOSE, WSMsgType.CLOSING):
        break
  finally:
    await PTY_SESSION.detach(ws)
    try:
      await ws.close()
    except Exception:
      pass
  return ws


async def handle_terminal_pty_status(request: web.Request) -> web.Response:
  return web.json_response({"ok": True, **await PTY_SESSION.snapshot()})


async def handle_terminal_commands(_request: web.Request) -> web.Response:
  load_commands()
  commands = [
    {
      "name": command.name,
      "summary": command.summary,
      "usage": command.usage,
    }
    for command in iter_commands()
  ]
  return web.json_response({"ok": True, "command": "carrot", "commands": commands})


async def handle_terminal_command_run(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)

  name = str(body.get("command") or "").strip().lower()
  raw_args = body.get("args", [])
  if not isinstance(raw_args, list):
    return web.json_response({"ok": False, "error": "args must be a list"}, status=400)
  args = [str(arg) for arg in raw_args]
  if not name or len(args) > 32 or any(len(arg) > 256 for arg in args):
    return web.json_response({"ok": False, "error": "invalid command arguments"}, status=400)

  load_commands()
  command = get_command(name)
  if command is None:
    return web.json_response({"ok": False, "error": f"unknown command: {name}"}, status=404)

  env = os.environ.copy()
  package_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))
  env["PYTHONPATH"] = os.pathsep.join(filter(None, (package_root, env.get("PYTHONPATH", ""))))
  proc = await asyncio.create_subprocess_exec(
    sys.executable,
    "-m",
    "selfdrive.carrot.server.terminal_commands.cli",
    name,
    *args,
    cwd=package_root,
    env=env,
    stdout=asyncio.subprocess.PIPE,
    stderr=asyncio.subprocess.PIPE,
  )
  try:
    stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=45.0)
  except asyncio.TimeoutError:
    proc.kill()
    await proc.wait()
    return web.json_response({"ok": False, "error": "command timed out"}, status=504)

  output = stdout.decode("utf-8", errors="replace").strip()
  error = stderr.decode("utf-8", errors="replace").strip()
  return web.json_response({
    "ok": proc.returncode == 0,
    "rc": int(proc.returncode or 0),
    "out": output,
    "error": error,
  }, status=200 if proc.returncode == 0 else 400)


async def handle_download_tmux(request: web.Request) -> web.Response:
  path = "/data/media/tmux.log"
  if not os.path.exists(path):
    return web.json_response({"ok": False, "error": "file not found"}, status=404)

  return web.FileResponse(
    path,
    headers={
      "Content-Disposition": "attachment; filename=tmux.log"
    }
  )


def register(app: web.Application) -> None:
  app.router.add_get("/api/terminal_pty/status", handle_terminal_pty_status)
  app.router.add_get("/api/terminal_commands", handle_terminal_commands)
  app.router.add_post("/api/terminal_commands/run", handle_terminal_command_run)
  app.router.add_get("/ws/terminal", ws_terminal)
  app.router.add_get("/ws/terminal_pty", ws_terminal_pty)
  app.router.add_get("/download/tmux.log", handle_download_tmux)
