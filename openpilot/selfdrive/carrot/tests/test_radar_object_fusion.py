from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import RadarPoint
from openpilot.selfdrive.carrot.radar.radar_object_fusion import RadarObjectFusion


def point(track_id: int, d_rel: float, y_rel: float, v_rel: float, source: str) -> RadarPoint:
  return RadarPoint(track_id, d_rel, y_rel, v_rel, 0.0, 0.0, 20.0 + v_rel, True, source)


def test_confirmed_pair_uses_corner_lateral_and_range_weighted_distance() -> None:
  fusion = RadarObjectFusion()
  outputs = ()
  for index in range(4):
    outputs = fusion.update(index * 0.05, (
      point(40, 8.0 + (0.5 if index == 3 else 0.0), 1.1, -1.0, "frontRadar"),
      point(1000, 7.0, 2.4, -1.0, "corner180"),
    ))

  fused = next(obj for obj in outputs if obj.front_track_id == 40 and obj.corner_track_id == 1000)
  assert fused.y_rel == 2.4
  assert fused.lateral_source == "corner180"
  assert fused.distance_source == "corner-weighted"
  assert fused.d_rel < 8.5


def test_far_pair_prefers_front_distance() -> None:
  fusion = RadarObjectFusion(confirm_frames=1)
  outputs = fusion.update(0.0, (
    point(40, 90.0, 0.4, -1.0, "frontRadar"),
    point(1000, 88.8, 0.5, -1.0, "corner235"),
  ))

  fused = next(obj for obj in outputs if obj.corner_track_id == 1000)
  assert fused.distance_source == "front-weighted"
  assert abs(fused.d_rel - 90.0) < abs(fused.d_rel - 88.8)


def test_scc_is_excluded_by_default_and_only_standalone_without_front_tracks() -> None:
  scc = point(0, 30.0, 0.0, -2.0, "scc")
  assert RadarObjectFusion().update(0.0, (scc,)) == ()

  outputs = RadarObjectFusion(include_scc=True).update(0.0, (scc,))
  assert len(outputs) == 1
  assert outputs[0].scc_track_id == 0


def test_enabled_scc_does_not_duplicate_matching_front_object() -> None:
  front = point(40, 30.2, 1.8, -2.1, "frontRadar")
  scc = point(0, 30.0, 0.0, -2.0, "scc")

  outputs = RadarObjectFusion(include_scc=True).update(0.0, (front, scc))

  assert len(outputs) == 1
  assert outputs[0].front_track_id == 40


def test_low_quality_geometric_pair_is_not_fused() -> None:
  fusion = RadarObjectFusion()
  outputs = ()
  for index in range(5):
    outputs = fusion.update(index * 0.05, (
      point(40, 20.0, 1.0, -1.0, "frontRadar"),
      point(1000, 14.1, 3.3, 0.4, "corner180"),
    ))

  assert not any(obj.front_track_id == 40 and obj.corner_track_id == 1000 for obj in outputs)


def test_confirmed_pair_is_not_stolen_by_a_new_nearby_corner() -> None:
  fusion = RadarObjectFusion()
  for index in range(4):
    fusion.update(index * 0.05, (
      point(40, 20.0, 1.0, -1.0, "frontRadar"),
      point(1000, 19.0, 1.2, -1.0, "corner180"),
    ))
  outputs = fusion.update(0.25, (
    point(40, 20.0, 1.0, -1.0, "frontRadar"),
    point(1000, 19.0, 1.2, -1.0, "corner180"),
    point(1001, 19.8, 1.0, -1.0, "corner180"),
  ))

  assert any(obj.front_track_id == 40 and obj.corner_track_id == 1000 for obj in outputs)
  assert not any(obj.front_track_id == 40 and obj.corner_track_id == 1001 for obj in outputs)


def test_pair_requires_consecutive_quality_frames_after_bad_history() -> None:
  fusion = RadarObjectFusion()
  for index in range(8):
    fusion.update(index * 0.05, (
      point(40, 20.0, 1.0, -1.0, "frontRadar"),
      point(1000, 14.1, 3.3, 0.4, "corner180"),
    ))

  outputs = fusion.update(0.45, (
    point(40, 20.0, 1.0, -1.0, "frontRadar"),
    point(1000, 19.0, 1.2, -1.0, "corner180"),
  ))

  assert not any(obj.front_track_id == 40 and obj.corner_track_id == 1000 for obj in outputs)
