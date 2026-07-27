from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import (
  Candidate,
  Selection,
)
from openpilot.selfdrive.carrot.radar.tools.radar_lead_video_review import (
  ReviewEvent,
  _merge_related_events,
  _priority,
  _signals_for_frame,
)
from openpilot.selfdrive.carrot.tests.test_radar_lead_simulator import frame, point


def test_video_review_reports_predictor_cutin() -> None:
  prediction = Candidate(
    10,
    0.8,
    "physical front dPath predictor",
    d_rel=25.0,
    y_rel=2.5,
  )
  selection = Selection(
    None,
    None,
    decision_cutin_candidates=(prediction,),
  )

  assert _signals_for_frame(selection) == [
    ("predictor-cutin", prediction),
  ]


def test_video_review_has_no_signal_without_predictor_decision() -> None:
  assert _signals_for_frame(Selection(None, None)) == []


def test_related_events_merge_physical_track_id_change() -> None:
  frames = [
    frame((
      point(10, 25.0, 2.5),
      point(11, 25.2, 2.4),
    ), time_s=0.3),
    frame((point(11, 25.2, 2.4),), time_s=0.5),
  ]
  events = [
    ReviewEvent(
      "a", "log", "shadow-only", 10, 0.0, 0.2, 0.1, 0.7,
      25.0, 2.5, "frontRadar", 100.0,
    ),
    ReviewEvent(
      "b", "log", "shadow-only", 11, 0.4, 0.6, 0.5, 0.8,
      25.2, 2.4, "frontRadar", 101.0,
    ),
  ]

  merged = _merge_related_events(events, frames, 1.0)

  assert len(merged) == 1
  assert merged[0].start_s == 0.0
  assert merged[0].end_s == 0.6
  assert merged[0].track_id == 11


def test_shadow_only_priority_is_explicitly_supported() -> None:
  assert _priority("predictor-cutin", 0.8, 20.0) > _priority("uncertain", 0.8, 20.0)
