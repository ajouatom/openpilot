from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar.radar_lead_model import RadarLeadFeatures, RadarLeadPrediction
from openpilot.selfdrive.carrot.radar.radar_sensor_objects import (
  independent_radar_objects,
  match_corner_to_front_identity,
  post_match_corner_to_front,
)


def point(track_id: int, d_rel: float, y_rel: float, v_rel: float, source: str, a_lead: float = 0.0):
  return SimpleNamespace(
    track_id=track_id, d_rel=d_rel, y_rel=y_rel, v_rel=v_rel, a_rel=0.0, yv_rel=0.0,
    v_lead=20.0 + v_rel, a_lead=a_lead, j_lead=0.0, measured=True, source=source,
  )


def prediction(obj) -> RadarLeadPrediction:
  features = RadarLeadFeatures(obj.object_id, (obj.object_id,), obj, (), 10, obj.y_rel, obj.y_rel, 0.0)
  return RadarLeadPrediction(features, 0.0, 1.0, 1.0)


def front_sources(enable_radar_tracks: int, include_scc: bool = True) -> tuple[str, ...]:
  objects = independent_radar_objects((
    point(40, 20.0, 0.1, -1.0, "frontRadar"),
    point(0, 19.0, 0.1, -1.0, "scc"),
    point(1, 8.0, 0.1, -18.0, "scc"),
    point(1000, 18.0, 1.2, -1.0, "corner180"),
  ), include_scc=include_scc, enable_radar_tracks=enable_radar_tracks)
  return tuple(obj.distance_source for obj in objects.front)


def test_sensor_objects_remain_independent_before_inference() -> None:
  objects = independent_radar_objects((
    point(40, 20.0, 1.0, -1.0, "frontRadar", 0.8),
    point(1000, 19.0, 1.2, -1.0, "corner180", -0.4),
  ))

  assert len(objects.front) == 1
  assert len(objects.corner) == 1
  assert objects.front[0].corner_track_id is None
  assert objects.corner[0].front_track_id is None


def test_front_model_sources_follow_enable_radar_tracks() -> None:
  assert front_sources(-2) == ()
  assert front_sources(-1) == ("scc", "scc")
  assert front_sources(0) == ("scc", "scc")
  assert front_sources(1) == ("frontRadar",)
  assert front_sources(2) == ("frontRadar", "scc")
  assert front_sources(3) == ("frontRadar", "scc")


def test_front_model_can_disable_scc_even_in_scc_modes() -> None:
  assert front_sources(0, include_scc=False) == ()
  assert front_sources(2, include_scc=False) == ("frontRadar",)


def test_selected_corner_can_post_match_front_control_values() -> None:
  objects = independent_radar_objects((
    point(40, 20.0, 1.0, -1.0, "frontRadar", 0.8),
    point(1000, 19.0, 1.2, -1.0, "corner180", -0.4),
  ))

  matched = post_match_corner_to_front(prediction(objects.corner[0]), (prediction(objects.front[0]),))

  assert matched is not None
  assert matched.features.radar_object.front_track_id == 40
  assert matched.features.radar_object.a_lead == 0.8


def test_selected_near_side_corner_does_not_match_noisy_front() -> None:
  objects = independent_radar_objects((
    point(40, 4.2, 1.0, -1.0, "frontRadar", 0.8),
    point(1000, 3.5, 1.8, -1.0, "corner180", -0.4),
  ))

  matched = post_match_corner_to_front(prediction(objects.corner[0]), (prediction(objects.front[0]),))

  assert matched is None


def test_near_side_corner_still_matches_front_identity() -> None:
  objects = independent_radar_objects((
    point(43, 5.05, -1.00, 0.30, "frontRadar", 0.8),
    point(1013, 4.75, -1.54, -0.11, "corner235", -0.4),
  ))

  matched = match_corner_to_front_identity(
    prediction(objects.corner[0]), (prediction(objects.front[0]),),
  )

  assert matched is not None
  assert matched.features.radar_object.front_track_id == 43


def test_adjacent_corner_does_not_confirm_distant_front_identity() -> None:
  objects = independent_radar_objects((
    point(48, 34.4, 5.8, -0.3, "frontRadar"),
    point(1002, 30.0, 4.0, -1.4, "corner180"),
  ))

  matched = match_corner_to_front_identity(
    prediction(objects.corner[0]), (prediction(objects.front[0]),),
  )

  assert matched is None
