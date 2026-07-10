import asyncio
import base64
import json
import time

import pytest
from aiohttp import web, WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from openpilot.selfdrive.carrot.server.services import support_terminal as support_module
from openpilot.selfdrive.carrot.server.services.support_terminal import SupportSession, SupportTerminalManager


class FakePtySession:
  session = "login-shell"

  def __init__(self) -> None:
    self.clients = set()
    self.text_writes = []
    self.byte_writes = []
    self.cleared = 0

  async def ensure(self, rows=30, cols=100):
    return False

  async def attach(self, ws, rows=30, cols=100, *, primary_eligible=True):
    self.clients.add(ws)
    await ws.send_json({"type": "meta", "mode": "pty", "session": self.session, "user": "comma", "rows": 30, "cols": 100})
    await ws.send_json({"type": "pty_output", "b64": base64.b64encode(b"ready\r\n").decode("ascii"), "replay": True})
    return False

  async def detach(self, ws):
    self.clients.discard(ws)

  async def write_text(self, text):
    self.text_writes.append(text)

  async def write(self, data):
    self.byte_writes.append(data)

  async def clear_history(self):
    self.cleared += 1


def make_session(permission_mode="approve_each") -> SupportSession:
  return SupportSession(
    id="support-test",
    pin="123456",
    note="",
    created_at=time.time(),
    expires_at=0,
    ttl_seconds=1800,
    command_timeout_seconds=30,
    state="sharing",
    permission_mode=permission_mode,
  )


async def receive_type(ws, expected, limit=16):
  for _ in range(limit):
    message = await ws.receive(timeout=2)
    assert message.type == WSMsgType.TEXT
    payload = json.loads(message.data)
    if payload.get("type") == expected:
      return payload
  raise AssertionError(f"message type not received: {expected}")


async def make_client(manager: SupportTerminalManager) -> TestClient:
  app = web.Application()
  app.router.add_get("/owner", manager.handle_owner_ws)
  app.router.add_get("/guest/{session_id}", manager.handle_guest_ws)
  app.router.add_get("/support/terminal/{session_id}", manager.handle_guest_page)
  app.router.add_get("/support-terminal-assets/{asset_name}", manager.handle_guest_asset)
  client = TestClient(TestServer(app))
  await client.start_server()
  return client


@pytest.mark.asyncio
async def test_allow_all_uses_one_shared_pty_controller(monkeypatch):
  fake_pty = FakePtySession()
  monkeypatch.setattr(support_module, "PTY_SESSION", fake_pty)
  manager = SupportTerminalManager()
  manager._session = make_session("allow_all")
  client = await make_client(manager)

  try:
    owner = await client.ws_connect("/owner")
    await receive_type(owner, "session_status")

    first = await client.ws_connect("/guest/support-test")
    await first.send_json({"type": "auth", "pin": "123456"})
    assert (await receive_type(first, "auth_ok"))["control_granted"] is True
    await receive_type(first, "pty_output")
    assert (await receive_type(first, "control_role"))["granted"] is True

    second = await client.ws_connect("/guest/support-test")
    await second.send_json({"type": "auth", "pin": "123456"})
    assert (await receive_type(second, "auth_ok"))["control_granted"] is False
    await receive_type(second, "pty_output")
    assert (await receive_type(second, "control_role"))["granted"] is False

    await second.send_json({"type": "raw", "data": "blocked"})
    await receive_type(second, "input_denied")
    assert fake_pty.text_writes == []

    await first.send_json({"type": "raw", "data": "whoami\r"})
    for _ in range(20):
      if fake_pty.text_writes:
        break
      await asyncio.sleep(0.01)
    assert fake_pty.text_writes == ["whoami\r"]

    await first.send_json({"type": "disconnect"})
    assert (await receive_type(second, "control_role"))["granted"] is True
    await second.send_json({"type": "raw", "data": "pwd\r"})
    for _ in range(20):
      if len(fake_pty.text_writes) >= 2:
        break
      await asyncio.sleep(0.01)
    assert fake_pty.text_writes[-1] == "pwd\r"

    await second.close()
    await owner.close()
  finally:
    await client.close()


@pytest.mark.asyncio
async def test_approve_each_writes_only_after_owner_approval(monkeypatch):
  fake_pty = FakePtySession()
  monkeypatch.setattr(support_module, "PTY_SESSION", fake_pty)
  manager = SupportTerminalManager()
  manager._session = make_session("approve_each")
  client = await make_client(manager)

  try:
    owner = await client.ws_connect("/owner")
    await receive_type(owner, "session_status")
    guest = await client.ws_connect("/guest/support-test")
    await guest.send_json({"type": "auth", "pin": "123456"})
    await receive_type(guest, "auth_ok")
    await receive_type(guest, "pty_output")

    await guest.send_json({"type": "input", "data": "pwd"})
    request = await receive_type(owner, "command_request")
    await receive_type(guest, "command_waiting_approval")
    assert fake_pty.text_writes == []

    result = await manager.approve_command(request["id"])
    assert result["ok"] is True
    assert fake_pty.text_writes == ["pwd\r"]

    await guest.close()
    await owner.close()
  finally:
    await client.close()


@pytest.mark.asyncio
async def test_guest_page_serves_external_xterm_assets():
  manager = SupportTerminalManager()
  manager._session = make_session()
  client = await make_client(manager)
  try:
    response = await client.get("/support/terminal/support-test")
    html = await response.text()
    assert response.status == 200
    assert 'data-session-id="support-test"' in html
    assert "/support-terminal-assets/xterm.js" in html
    assert "script-src 'self'" in response.headers["Content-Security-Policy"]

    asset = await client.get("/support-terminal-assets/guest.js")
    assert asset.status == 200
    assert "new window.Terminal" in await asset.text()
  finally:
    await client.close()
