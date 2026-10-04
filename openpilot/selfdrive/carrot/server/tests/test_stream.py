import asyncio
import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

import pytest

@pytest.fixture
def stream(monkeypatch):
  # Load only this HTTP feature; unrelated Bluetooth/device routes require Linux.
  name = "openpilot.selfdrive.carrot.server.features.stream"
  diagnostics = ModuleType("openpilot.selfdrive.carrot.server.services.vision_diag")
  diagnostics.record_stream_proxy_event = lambda event: None
  monkeypatch.setitem(sys.modules, diagnostics.__name__, diagnostics)
  spec = importlib.util.spec_from_file_location(name, Path(__file__).resolve().parents[1] / "features/stream.py")
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


@pytest.mark.parametrize("cluster_hud", [0, 1])
@pytest.mark.parametrize("status", [200, 409])
def test_stream_with_hud_preserves_upstream_response(monkeypatch, stream, cluster_hud, status):
  body = b'{"cameras":["road"],"client_id":"browser"}'
  answer = json.dumps({"sdp": "answer"} if status == 200 else {"code": "carrot_vision_busy"}).encode()
  response = SimpleNamespace(status=status, headers={"Content-Type": "application/json"}, read=AsyncMock(return_value=answer))
  session = MagicMock()
  session.post.return_value.__aenter__ = AsyncMock(return_value=response)
  session.post.return_value.__aexit__ = AsyncMock(return_value=False)
  params = MagicMock()
  params.get_int.return_value = cluster_hud
  request = SimpleNamespace(
    app={"params": params, "http": session}, remote="127.0.0.1",
    headers={"Content-Type": "application/json"}, read=AsyncMock(return_value=body),
  )
  events = []
  monkeypatch.setattr(stream, "record_stream_proxy_event", events.append)

  result = asyncio.run(stream.proxy_stream(request))

  assert result.status == status
  assert result.body == answer
  assert result.headers["Content-Type"] == "application/json"
  assert session.post.call_args.args == (stream.WEBRTCD_URL,)
  assert session.post.call_args.kwargs["data"] == body
  assert events[-1]["ok"] == (status == 200)
