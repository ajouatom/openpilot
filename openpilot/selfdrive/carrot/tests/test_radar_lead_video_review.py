import json
from pathlib import Path
from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import Candidate, Selection
from openpilot.selfdrive.carrot.radar.tools.radar_lead_fused_dataset import candidate_cutin_overrides
from openpilot.selfdrive.carrot.radar.tools.radar_lead_video_review import (
  ReviewEvent,
  _inventory_logs,
  _merge_related_events,
  _queue_log_paths,
  _reviewed_event,
  _segment_key,
  _same_object,
  _signals_for_frame,
  export_annotations,
  mark_event,
)

from openpilot.selfdrive.carrot.tests.test_radar_lead_simulator import frame, point


def test_inventory_logs_filter_usable_vehicle_profiles(tmp_path: Path) -> None:
  inventory = tmp_path / "inventory.jsonl"
  profiles = [
    {"profile": {"log": "C:/routes/HYUNDAI/rlog.zst", "vehicle": "HYUNDAI IONIQ",
                 "has_qcamera": True, "live_tracks_frames": 10, "model_frames": 10,
                 "recorded_cutin_frames": 2}},
    {"profile": {"log": "C:/routes/KIA/rlog.zst", "vehicle": "KIA K8",
                 "has_qcamera": False, "live_tracks_frames": 10, "model_frames": 10}},
    {"profile": {"log": "C:/routes/VW/rlog.zst", "vehicle": "VOLKSWAGEN ID4",
                 "has_qcamera": True, "live_tracks_frames": 10, "model_frames": 10}},
    {"error": "broken"},
  ]
  inventory.write_text("\n".join(json.dumps(item) for item in profiles), encoding="utf-8")

  assert _inventory_logs(inventory, "HYUNDAI|KIA") == [Path("C:/routes/HYUNDAI/rlog.zst")]


def test_inventory_logs_can_limit_each_vehicle_folder(tmp_path: Path) -> None:
  inventory = tmp_path / "inventory.jsonl"
  profiles = [
    {"profile": {"log": f"C:/routes/{vehicle}/{index}/rlog.zst", "vehicle": vehicle,
                 "has_qcamera": True, "live_tracks_frames": 10, "model_frames": 10,
                 "side_motion_frames": 10 - index}}
    for vehicle in ("HYUNDAI IONIQ", "KIA K8") for index in range(3)
  ]
  inventory.write_text("\n".join(json.dumps(item) for item in profiles), encoding="utf-8")

  logs = _inventory_logs(inventory, "HYUNDAI|KIA", max_logs_per_vehicle=1)

  assert len(logs) == 2
  assert {path.parts[-3] for path in logs} == {"HYUNDAI IONIQ", "KIA K8"}


def test_inventory_logs_deduplicates_rlog_variants_in_one_segment(tmp_path: Path) -> None:
  inventory = tmp_path / "inventory.jsonl"
  profiles = [
    {"profile": {"log": f"C:/routes/HYUNDAI/segment/{name}", "vehicle": "HYUNDAI IONIQ",
                 "has_qcamera": True, "live_tracks_frames": 10, "model_frames": 10}}
    for name in ("rlog.2.zst", "rlog.zst", "rlog.1.zst")
  ]
  inventory.write_text("\n".join(json.dumps(item) for item in profiles), encoding="utf-8")

  assert _inventory_logs(inventory, "HYUNDAI") == [Path("C:/routes/HYUNDAI/segment/rlog.zst")]


def test_queue_log_paths_includes_scanned_logs_and_legacy_events(tmp_path: Path) -> None:
  queue = tmp_path / "queue.json"
  queue.write_text(json.dumps({
    "scanned_logs": ["C:/routes/scanned/rlog.zst"],
    "events": [{"log": "C:/routes/event/rlog.zst"}],
  }), encoding="utf-8")

  assert _queue_log_paths(queue) == {
    Path("C:/routes/scanned/rlog.zst"),
    Path("C:/routes/event/rlog.zst"),
  }


def test_segment_key_treats_rlog_variants_as_one_segment() -> None:
  assert _segment_key(Path("C:/routes/HYUNDAI/segment/rlog.zst")) == \
         _segment_key(Path("C:/routes/HYUNDAI/segment/rlog.2.zst"))


def test_same_object_accepts_fused_sensor_aliases() -> None:
  radar_frame = frame((
    point(50, 22.0, 1.1, 14.0),
    point(1050, 23.2, 1.5, 14.5, "corner235"),
  ))

  assert _same_object(radar_frame, 50, 1050)


def test_signals_separate_model_and_radard_disagreement() -> None:
  radar_frame = frame((
    point(50, 22.0, 1.1, 14.0),
    point(60, 35.0, -2.0, 15.0),
  ))
  model = Candidate(50, 0.94, "MLP active cutin", 0.9)
  selection = Selection(None, model, active_cutin_candidates=(model,))

  signals = _signals_for_frame(radar_frame, selection, {60})

  assert [(kind, candidate.track_id) for kind, candidate in signals] == [
    ("model-only", 50),
    ("radard-only", 60),
  ]


def test_signals_treat_spatial_sensor_alias_as_agreement() -> None:
  radar_frame = frame((
    point(50, 22.0, 1.1, 14.0),
    point(1050, 23.0, 1.4, 14.2, "corner235"),
  ))
  model = Candidate(50, 0.94, "MLP active cutin", 0.9)
  selection = Selection(None, model, active_cutin_candidates=(model,))

  signals = _signals_for_frame(radar_frame, selection, {1050})

  assert [(kind, candidate.track_id) for kind, candidate in signals] == [("agreement", 50)]


def test_merge_related_events_combines_signal_transitions_and_sensor_aliases() -> None:
  frames = [frame((
    point(50, 22.0, 1.1, 14.0),
    point(1050, 23.0, 1.4, 14.2, "corner235"),
  ))]
  first = ReviewEvent("a", "log", "model-only", 50, 0.0, 0.2, 0.1, 0.91, 22.0, 1.1, "frontRadar", 109.0)
  second = ReviewEvent("b", "log", "agreement", 1050, 0.15, 0.4, 0.3, 0.96, 23.0, 1.4, "corner235", 59.0)

  merged = _merge_related_events([first, second], frames, 0.75)

  assert len(merged) == 1
  assert merged[0].signal_kinds == ["model-only", "agreement"]
  assert merged[0].start_s == 0.0
  assert merged[0].end_s == 0.4


def test_reviewed_event_matches_candidate_and_overlapping_time() -> None:
  event = ReviewEvent(
    "a", "C:/routes/segment/rlog.zst", "model-only", 50,
    2.0, 3.0, 2.5, 0.9, 20.0, 1.0, "frontRadar", 100.0,
  )
  annotations = {"logs": {"segment/rlog.zst": [
    {"start_s": 1.8, "end_s": 2.2, "cutin_candidate": 50, "cutin_label": False},
    {"start_s": 2.0, "end_s": 3.0, "cutin_candidate": 60, "cutin_label": True},
  ]}}

  assert _reviewed_event(event, annotations)
  assert not _reviewed_event(
    ReviewEvent("b", event.log, event.kind, 60, 4.0, 5.0, 4.5, 0.9, 20.0, 1.0, event.source, 100.0),
    annotations,
  )


def test_export_excludes_ambiguous_and_low_confidence(tmp_path: Path) -> None:
  queue = tmp_path / "queue.json"
  output = tmp_path / "annotations.json"
  queue.write_text("""{
    "events": [
      {"id":"yes","log":"C:/routes/seg/rlog.zst","status":"reviewed","verdict":"positive","confidence":"high","track_id":50,"start_s":2.0,"end_s":3.0},
      {"id":"no","log":"C:/routes/seg/rlog.zst","status":"reviewed","verdict":"negative","confidence":"medium","track_id":60,"start_s":5.0,"end_s":6.0},
      {"id":"maybe","log":"C:/routes/seg/rlog.zst","status":"reviewed","verdict":"ambiguous","confidence":"high","track_id":70,"start_s":7.0,"end_s":8.0},
      {"id":"low","log":"C:/routes/seg/rlog.zst","status":"reviewed","verdict":"positive","confidence":"low","track_id":80,"start_s":9.0,"end_s":10.0}
    ]
  }""", encoding="utf-8")
  args = SimpleNamespace(queue=queue, output=output, confidence=["high", "medium"], padding=0.2)

  assert export_annotations(args) == 0
  text = output.read_text(encoding="utf-8")
  assert '"cutin_candidate": 50' in text
  assert '"cutin_label": true' in text
  assert '"cutin_candidate": 60' in text
  assert '"cutin_label": false' in text
  assert 'maybe' not in text
  assert 'low' not in text


def test_candidate_cutin_overrides_only_label_reviewed_track() -> None:
  frames = [frame((point(50, 20.0, 1.0, 10.0), point(60, 25.0, -1.0, 11.0)))]
  entries = [{"start_s": 0.0, "end_s": 0.1, "cutin_candidate": 50, "cutin_label": False}]

  overrides, count = candidate_cutin_overrides(frames, entries)

  assert overrides == [{50: False}]
  assert count == 1


def test_mark_event_accepts_visual_time_correction(tmp_path: Path) -> None:
  queue = tmp_path / "queue.json"
  queue.write_text('{"events":[{"id":"event","start_s":2.0,"end_s":3.0,"track_id":50}]}', encoding="utf-8")
  args = SimpleNamespace(
    queue=queue, id="event", verdict="positive", confidence="high", note="video onset",
    track_id=None, start_s=1.4, end_s=3.2,
  )

  assert mark_event(args) == 0
  event = json.loads(queue.read_text(encoding="utf-8"))["events"][0]
  assert event["start_s"] == 1.4
  assert event["end_s"] == 3.2


def test_export_preserves_base_annotations(tmp_path: Path) -> None:
  queue = tmp_path / "queue.json"
  base = tmp_path / "base.json"
  output = tmp_path / "annotations.json"
  queue.write_text('{"events":[]}', encoding="utf-8")
  base.write_text('{"version":1,"logs":{"old/rlog.zst":[{"start_s":1,"end_s":2,"cutin":7}]}}', encoding="utf-8")
  args = SimpleNamespace(
    queue=queue, output=output, base=base, confidence=["high", "medium"], padding=0.2,
  )

  assert export_annotations(args) == 0
  assert json.loads(output.read_text(encoding="utf-8"))["logs"]["old/rlog.zst"][0]["cutin"] == 7
