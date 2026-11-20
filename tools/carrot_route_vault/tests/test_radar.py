import asyncio
import gzip
import json
from pathlib import Path

from aiohttp.test_utils import TestClient, TestServer
from aiohttp import web
import pytest

from ..server import create_app
from ..radar import RadarJobs
from .test_viewer import DIRECTORY, ROUTE, OTHER_ROUTE, create_share, seed_route, viewer_config


def test_radar_route_scope_and_revocation(tmp_path, monkeypatch):
  calls = []

  async def response(_self, source, sensor, sensitivity, radar_track_flip="recorded"):
    calls.append((source, sensor, sensitivity, radar_track_flip))
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
      assert calls[-1] == (tmp_path / "uploads/routes" / DIRECTORY / f"{ROUTE}--0/rlog.zst", "front", 3, "recorded")
      assert (await client.get(base + "/radar/0?radar_track_flip=flipped")).status == 202
      assert calls[-1][3] == "flipped"
      assert (await client.get(base + "/radar/0?sensitivity=bad")).status == 202
      assert calls[-1][2] == 3
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

    async def prepare(src, output, sensor, sensitivity, radar_track_flip="recorded"):
      calls.append((src, radar_track_flip))
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
    assert (await manager.response(source, "auto", 3, "flipped")).status == 202
    await asyncio.gather(*manager.jobs.values())
    assert calls[-1] == (source, "flipped")
    assert (await manager.response(source, "auto", 3, "flipped")).status == 200
    assert (await manager.response(source, "auto", 3, "normal")).status == 202
    await asyncio.gather(*manager.jobs.values())
    assert calls[-1] == (source, "normal")
    with pytest.raises(web.HTTPBadRequest):
      await manager.response(source, "auto", 3, "invalid")
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


def test_radar_asset_defaults_to_compact_track_ids():
  asset = (Path(__file__).parents[1] / "radar_view.js").read_text(encoding="utf-8")
  assert '<option value="track" selected>Track ID</option>' in asset
  assert 'data-show-live checked' in asset
  assert 'data-show-classic checked' in asset
  assert 'data-hide-scc' in asset
  assert '`T${point.track_id??point.slot}`' in asset
  assert 'C${p.slot}:S${p.status}' not in asset


def test_export_adds_classic_238_objects_without_changing_selection():
  from openpilot.selfdrive.carrot.radar.tools import radar_web_export as exporter
  from openpilot.selfdrive.carrot.tests.test_radar_lead_simulator import frame, point
  frames = [frame((point(41, 25.0, 0.0),), time_s=i * .05) for i in range(3)]
  snapshot = (frames[1].mono_time_s, [{
    'slot': 3, 'status': 2, 'd_rel': 30.0, 'y_rel': -1.0, 'v_lead': 20.0,
    'object_sequence': 7, 'rolling_counter': 1, 'counter_consistent': True,
    'confirmed_candidate': True, 'first_raw_hex': '0123456789abcdef',
    'second_raw_hex': '4000000000000000',
    'third_raw_hex': '4000000700000000',
  }])
  changed = (frames[2].mono_time_s, [{
    **snapshot[1][0], 'object_sequence': 8,
    'second_raw_hex': '8000000000000001', 'third_raw_hex': '8000000800000002',
  }])
  payload = exporter.export_frames(frames, classic_238_bus=1, classic_238_snapshots=[snapshot, changed])
  assert payload['schemaVersion'] == 4
  assert payload['classic238'] == {
    'available': True, 'bus': 1, 'controlConnected': False, 'confirmedStatus': 2,
  }
  assert payload['frames'][0]['classic_238_objects'] == []
  assert payload['frames'][1]['classic_238_objects'][0]['slot'] == 3
  assert payload['frames'][1]['classic_238_objects'][0]['track_id'] == 0
  assert payload['frames'][2]['classic_238_objects'][0]['confirmed_candidate'] is True
  assert payload['frames'][2]['classic_238_objects'][0]['second_unknown_change_mask'] == '0000000000000001'
  assert payload['frames'][2]['classic_238_objects'][0]['third_unknown_change_mask'] == '0000000000000002'
  assert payload['frames'][2]['classic_238_objects'][0]['second_unknown_changed_bits'] == 1
  assert payload['frames'][2]['classic_238_objects'][0]['third_unknown_changed_bits'] == 1
  assert payload['frames'][2]['classic_238_objects'][0]['object_sequence_delta'] == 1
  assert payload['frames'][2]['classic_238_objects'][0]['first_raw_hex'] == '0123456789abcdef'
  assert payload['frames'][1]['selection']['lead_one']['track_id'] == 41


def test_graph_preserves_missing_scc_samples_and_desktop_series():
  from dataclasses import replace
  from openpilot.selfdrive.carrot.radar.tools import radar_web_export as exporter
  from openpilot.selfdrive.carrot.tests.test_radar_lead_simulator import frame, point
  frames = [replace(frame((point(41, 25.0, 0.0),), time_s=i * .05),
                    video_time_s=10 + i * .05, scc_a_req_raw=value, carrot_a_target=-.5)
            for i, value in enumerate((-1.0, None, -2.0))]
  payload = exporter.export_frames(frames)
  assert payload['graphs']['sccAccel'] == [
    {'color': '#f75ea0', 'samples': [[0.0, -1.0]]},
    {'color': '#f75ea0', 'samples': [[.1, -2.0]]},
  ]
  assert payload['graphs']['carrotAccel'][0]['samples'] == [[0.0, -.5], [.05, -.5], [.1, -.5]]
  assert payload['frames'][0]['video_time_s'] == 10
  assert set(payload['graphs']) == {'leadOne', 'leadTwo', 'vision', 'leadSpeed', 'sccDistance', 'sccAccel', 'carrotAccel'}


@pytest.mark.parametrize("mode", (-2, -1, 0, 1, 2, 3, None))
def test_export_uses_recorded_source_policy_including_zero(mode):
  from dataclasses import asdict, replace
  from openpilot.selfdrive.carrot.radar.tools import radar_web_export as exporter
  from openpilot.selfdrive.carrot.tests.test_radar_lead_simulator import frame, point

  frames = [replace(frame((replace(point(0, 25.0, 4.0), source="scc"),), time_s=i * .05),
                    recorded_radar_track_mode=mode) for i in range(20)]
  effective = 2 if mode is None else mode
  payload = exporter.export_frames(frames)
  desktop = exporter.replay.ProductionDPathSelector(frames, enable_radar_tracks=effective)
  assert payload["enableRadarTracks"] == effective
  for i, item in enumerate(payload["frames"]):
    lead = desktop.select(frames[i], i).lead_one
    assert item["selection"]["lead_one"] == (exporter.finite_json(asdict(lead)) if lead else None)
