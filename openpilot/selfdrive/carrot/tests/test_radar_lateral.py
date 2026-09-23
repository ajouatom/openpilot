from copy import deepcopy

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.carrot.radar.lateral import set_radar_track_flip
from openpilot.selfdrive.carrot.radar.tools.radar_validation_replay import load_frames


def tracks_message(flipped=False):
  event = log.Event.new_message(logMonoTime=2_000_000_000)
  data = event.init("liveTracks")
  data.points = [dict(trackId=track_id, radarSource=source, dRel=20., yRel=-2.7,
                      yvRel=.6, vRel=-3., vLead=10., aRel=-.2, aLead=-.3,
                      jLead=.1, measured=True, trackState=3)
                 for track_id, source in [(36, "frontRadar"), (52, "frontRadar"), (0, "scc"),
                                          (200, "corner235"), (240, "corner180"), (300, "corner430")]]
  set_radar_track_flip(data, flipped)
  return event


def test_publication_copy_is_idempotent_and_preserves_other_sources_and_fields():
  raw = tracks_message()
  expected = raw.liveTracks.to_dict()
  for _ in range(3):
    # Same native decoder result can be reused; publication must not mutate it.
    published = log.Event.new_message()
    published.liveTracks = raw.liveTracks
    set_radar_track_flip(published.liveTracks, True)
    set_radar_track_flip(published.liveTracks, True)
    actual = published.liveTracks.to_dict()
    corrected = deepcopy(expected)
    corrected["radarTrackFlipped"] = True
    for point in corrected["points"][:2]:
      point["yRel"] *= -1
      point["yvRel"] *= -1
    assert actual == corrected
    assert raw.liveTracks.to_dict() == expected
    set_radar_track_flip(published.liveTracks, False)
    assert published.liveTracks.to_dict() == expected


@pytest.mark.parametrize("recorded", [False, True])
@pytest.mark.parametrize("override", [None, False, True])
def test_replay_uses_recorded_orientation_or_explicit_override_once(tmp_path, recorded, override):
  tracks = tracks_message(recorded)
  model = log.Event.new_message(logMonoTime=2_020_000_000)
  model.init("modelV2").timestampEof = 2_000_000_000
  path = tmp_path / "rlog"
  path.write_bytes(tracks.to_bytes() + model.to_bytes())
  frames = load_frames(path, radar_track_flip=override)
  effective = recorded if override is None else override
  assert len(frames) == 1
  assert frames[0].radar_track_flipped == effective
  for point in frames[0].points:
    flipped = effective and point.source == "frontRadar"
    assert point.y_rel == pytest.approx(2.7 if flipped else -2.7)
    assert point.yv_rel == pytest.approx(-.6 if flipped else .6)
    assert point.d_rel == 20. and point.v_rel == -3.


def test_group3_matching_receives_native_geometry_before_orientation(tmp_path, monkeypatch):
  from openpilot.selfdrive.carrot.radar.tools import radar_group3_replay
  params = log.Event.new_message(logMonoTime=1_000_000_000)
  cp = params.init("carParams")
  cp.brand, cp.carFingerprint, cp.extFlags = "hyundai", "KIA_EV5", 2048
  can = log.Event.new_message(logMonoTime=1_990_000_000)
  can.init("can", 1)[0] = dict(address=0x400, src=1, dat=bytes(24))
  tracks = tracks_message(True)
  model = log.Event.new_message(logMonoTime=2_020_000_000)
  model.init("modelV2").timestampEof = 2_000_000_000
  called = []

  def correct(self, t, points):
    called.append(t)
    assert points[0].yRel == pytest.approx(-2.7)
    assert points[0].yvRel == pytest.approx(.6)
    return points

  monkeypatch.setattr(radar_group3_replay.Group3Replay, "correct", correct)
  path = tmp_path / "rlog"
  path.write_bytes(b"".join(e.to_bytes() for e in [params, can, tracks, model]))
  frames = load_frames(path)
  assert called and frames[0].points[0].y_rel == pytest.approx(2.7)
