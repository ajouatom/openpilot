import asyncio
import gzip
import json

from aiohttp.test_utils import TestClient, TestServer
from aiohttp import web
import pytest

from ..server import create_app
from ..radar import RadarJobs
from .test_viewer import DIRECTORY, ROUTE, OTHER_ROUTE, create_share, seed_route, viewer_config


def test_radar_route_scope_and_revocation(tmp_path, monkeypatch):
  calls = []

  async def response(_self, source, sensor, sensitivity):
    calls.append((source, sensor, sensitivity))
    return web.json_response({"status": "preparing"}, status=202)

  monkeypatch.setattr(RadarJobs, "response", response)

  async def run():
    seed_route(tmp_path, ROUTE, (0, 1))
    seed_route(tmp_path, OTHER_ROUTE, (0,))
    async with TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False))) as client:
      base = f"/routes/{DIRECTORY}/{ROUTE}--0"
      assert (await client.get(base + "/radar/1")).status == 404
      assert not calls
      assert (await client.get(base + "/radar/0?sensor=front&sensitivity=5")).status == 202
      assert calls[-1] == (tmp_path / "uploads/routes" / DIRECTORY / f"{ROUTE}--0/rlog.zst", "front", 5)
      assert (await client.get(base + "/radar/0?sensitivity=bad")).status == 400
      share = await create_share(client)
      from urllib.parse import urlsplit
      shared_base = urlsplit(share["shareUrl"]).path
      assert (await client.get(shared_base + "/radar/1")).status == 202
      from .test_viewer import admin_headers
      assert (await client.post(f"/api/admin/shares/{share['id']}/revoke", headers=admin_headers(), json={})).status == 200
      count = len(calls)
      assert (await client.get(shared_base + "/radar/1")).status in (403, 404, 410)
      assert len(calls) == count
      page = await (await client.get(base)).text()
      assert "attachRadarReview(video)" in page
      assert (await client.get('/assets/radar_view.js')).status == 200

  asyncio.run(run())


def test_job_deduplicates_and_publishes_cache(tmp_path, monkeypatch):
  from openpilot.selfdrive.carrot.radar.tools import radar_web_export
  monkeypatch.setattr(radar_web_export, "source_version", lambda: "test-version")

  async def run():
    manager = RadarJobs(tmp_path / "cache")
    source = tmp_path / "rlog.zst"
    source.write_bytes(b"test")
    calls = []
    release = asyncio.Event()

    async def prepare(src, output, sensor, sensitivity):
      calls.append(src)
      await release.wait()
      output.parent.mkdir(exist_ok=True)
      output.write_bytes(gzip.compress(b'{"frames":[{}]}'))

    monkeypatch.setattr(manager, "prepare", prepare)
    assert (await manager.response(source, "auto", 3)).status == 202
    assert (await manager.response(source, "auto", 3)).status == 202
    await asyncio.sleep(0)
    assert len(calls) == 1
    release.set()
    await asyncio.gather(*manager.jobs.values())
    assert (await manager.response(source, "auto", 3)).status == 200
    assert not manager.jobs
    with pytest.raises(web.HTTPBadRequest):
      await manager.response(source, "invalid", 3)
    source.write_bytes(b"replacement log")
    assert (await manager.response(source, "auto", 3)).status == 202
    await manager.close(None)

  asyncio.run(run())


def test_export_matches_desktop_controller():
  from openpilot.selfdrive.carrot.radar.tools import radar_web_export as exporter
  from openpilot.selfdrive.carrot.tests.test_radar_lead_simulator import frame, point
  frames = [frame((point(41, 25.0, 0.0),), time_s=i * .05) for i in range(20)]
  payload = exporter.export_frames(frames)
  desktop = exporter.replay.ProductionDPathSelector(frames)
  from dataclasses import asdict
  for i, item in enumerate(payload['frames']):
    selection = desktop.select(frames[i], i)
    assert item['selection']['lead_one'] == exporter.finite_json(asdict(selection.lead_one)) if selection.lead_one else item['selection']['lead_one'] is None
  assert payload['sensitivity'] == exporter.replay.VALIDATION_DEFAULT_SENSITIVITY
  json.dumps(payload, allow_nan=False)
