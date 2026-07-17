from contextlib import contextmanager
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

from openpilot.selfdrive.carrot.server.features.dashcam import replay
from openpilot.selfdrive.carrot.server.features.dashcam import routes as dashcam_routes
from openpilot.selfdrive.carrot.server.features.dashcam.replay_query import (
  QueryCancelled,
  query_field_events,
  query_snapshot_events,
)
from openpilot.selfdrive.carrot.server.features.dashcam.replay_semantics import (
  field_semantics,
  registry_provenance,
  registry_versions,
  service_semantics,
  statistics_policy,
  validate_registry,
)


@contextmanager
def replace_attr(target, name, value):
  original = getattr(target, name)
  setattr(target, name, value)
  try:
    yield
  finally:
    setattr(target, name, original)


class FakeEvent:
  def __init__(self, service, mono_ns, *, valid=True, payload=None):
    self._service = service
    self.logMonoTime = mono_ns
    self.valid = valid
    setattr(self, service, payload if payload is not None else SimpleNamespace())

  def which(self):
    return self._service


def init_event(mono_ns=1):
  return FakeEvent("initData", mono_ns, payload=SimpleNamespace(
    gitCommit="abc123",
    gitBranch="test-branch",
    gitSrcCommit="source123",
    version="0.0-test",
    deviceType="tici",
    dirty=True,
    passive=False,
  ))


def sentinel_event(kind, mono_ns):
  return FakeEvent("sentinel", mono_ns, payload=SimpleNamespace(type=kind))


def test_client_replay_source_description_only_exposes_raw_files():
  segment = "00000001--clienttest--0"
  with TemporaryDirectory() as directory:
    root = Path(directory)
    (root / "rlog.zst").write_bytes(b"zstd-source")
    (root / "qcamera.ts").write_bytes(b"mpegts-source")
    with replace_attr(dashcam_routes, "segment_dir", lambda value: str(root)):
      payload = dashcam_routes.client_replay_source_description(segment)

  assert payload["mode"] == "client"
  assert payload["segmentIndex"] == 0
  assert payload["rlog"]["compression"] == "zstd"
  assert payload["rlog"]["size"] == len(b"zstd-source")
  assert payload["video"]["container"] == "mpegts"
  assert payload["video"]["size"] == len(b"mpegts-source")
  assert payload["rlog"]["url"].endswith("/rlog")
  assert payload["video"]["url"].endswith("/video")


def test_build_timeline_collects_raw_stats_without_expanding_compact():
  start_ns = 10_000_000_000
  events = [
    init_event(),
    sentinel_event("startOfRoute", start_ns),
    FakeEvent("carState", start_ns + 10_000_000, payload=SimpleNamespace(frameId=1)),
    FakeEvent("carState", start_ns + 20_000_000, valid=False, payload=SimpleNamespace(frameId=2)),
    FakeEvent("bookmarkButton", start_ns + 30_000_000, valid=False),
    sentinel_event("endOfSegment", start_ns + 1_000_000_000),
  ]

  with replace_attr(replay, "_compact_frame", lambda event, service, sequence: bytes((sequence,))):
    timeline, metadata = replay.build_replay_timeline(events, expected_segment=0)

  magic, timeline_version, _flags, _duration_ms, _record_count = replay.REPLAY_TIMELINE_HEADER.unpack_from(timeline)
  assert magic == replay.REPLAY_TIMELINE_MAGIC
  assert timeline_version == replay.REPLAY_TIMELINE_VERSION == 3
  assert replay.REPLAY_CACHE_VERSION > replay.REPLAY_TIMELINE_VERSION
  assert metadata["sampleCount"] == 1
  assert metadata["rawEventCount"] == len(events)
  assert metadata["rawInvalidEventCount"] == 2
  assert metadata["rawDurationMs"] == 1000
  assert metadata["rawDurationBasis"] == "sentinel"
  assert metadata["recordedSchema"]["gitCommit"] == "abc123"
  assert metadata["recordedSchema"]["deviceType"] == "tici"
  assert metadata["rawServiceStats"]["carState"]["count"] == 2
  assert metadata["rawServiceStats"]["carState"]["invalidCount"] == 1
  assert metadata["rawServiceStats"]["bookmarkButton"]["count"] == 1
  assert metadata["rawServiceStats"]["bookmarkButton"]["invalidCount"] == 1


def test_build_timeline_keeps_latest_valid_event_in_compact_bucket():
  start_ns = 20_000_000_000
  events = [
    sentinel_event("startOfSegment", start_ns),
    FakeEvent("carState", start_ns + 1_000_000, payload=SimpleNamespace(marker=1)),
    FakeEvent("carState", start_ns + 2_000_000, payload=SimpleNamespace(marker=2)),
    sentinel_event("endOfSegment", start_ns + 100_000_000),
  ]

  with replace_attr(replay, "_compact_frame", lambda event, service, sequence: bytes((event.carState.marker,))):
    _timeline, metadata = replay.build_replay_timeline(events, expected_segment=1)

  assert metadata["sampleCount"] == 1
  assert metadata["services"]["carState"] == 1
  assert metadata["rawServiceStats"]["carState"]["count"] == 2


def test_build_timeline_returns_partial_stats_when_log_tail_is_truncated():
  start_ns = 30_000_000_000

  class BrokenIterator:
    def __init__(self):
      self.events = iter([
        sentinel_event("startOfSegment", start_ns),
        FakeEvent("carState", start_ns + 1_000_000),
      ])

    def __iter__(self):
      return self

    def __next__(self):
      try:
        return next(self.events)
      except StopIteration as exc:
        raise RuntimeError("truncated message") from exc

  with replace_attr(replay, "_compact_frame", lambda event, service, sequence: b"x"):
    _timeline, metadata = replay.build_replay_timeline(BrokenIterator(), expected_segment=2)

  assert metadata["sampleCount"] == 1
  assert metadata["rawParseStatus"] == "partial"
  assert "truncated message" in metadata["rawParseError"]
  assert metadata["rawServiceStats"]["carState"]["firstMonoTimeNanos"] == str(start_ns + 1_000_000)


def test_raw_numeric_query_keeps_spike_and_invalid_statistics():
  base_ns = 40_000_000_000
  events = []
  for index in range(201):
    value = 80.0 if index == 100 else float(index % 7)
    events.append(FakeEvent(
      "carState",
      base_ns + index * 10_000_000,
      valid=index != 150,
      payload=SimpleNamespace(vEgo=value),
    ))

  result = query_field_events(
    events,
    service="carState",
    path="carState.vEgo",
    base_mono_ns=base_ns,
    field_meta={"path": "carState.vEgo", "type": "float32", "privacy": "driving"},
    max_points=50,
  )

  assert result["stats"]["numeric"]["count"] == 201
  assert result["stats"]["numeric"]["max"] == 80.0
  assert result["stats"]["numeric"]["invalidCount"] == 1
  assert result["stats"]["numeric"]["rms"] > 0
  assert result["stats"]["numeric"]["weightedDurationMs"] == 2000.0
  assert sum(item["count"] for item in result["stats"]["numeric"]["histogram"]["bins"]) == 201
  assert result["downsample"] == "minmax-envelope"
  assert len(result["series"]) <= 50
  assert max(point["value"] for point in result["series"]) == 80.0


def test_raw_query_extracts_nested_lists_and_state_transitions():
  base_ns = 50_000_000_000
  events = [
    FakeEvent("modelV2", base_ns, payload=SimpleNamespace(leads=[SimpleNamespace(prob=0.1), SimpleNamespace(prob=0.9)])),
    FakeEvent("modelV2", base_ns + 10_000_000, payload=SimpleNamespace(leads=[SimpleNamespace(prob=0.3)])),
  ]
  list_result = query_field_events(
    events,
    service="modelV2",
    path="modelV2.leads[].prob",
    base_mono_ns=base_ns,
    field_meta={"path": "modelV2.leads[].prob", "type": "float32", "privacy": "driving"},
  )

  assert list_result["kind"] == "number-list"
  assert list_result["stats"]["numeric"]["count"] == 3
  assert list_result["stats"]["listLength"] == {"count": 2, "min": 1, "max": 2, "mean": 1.5}

  states = [
    FakeEvent("carState", base_ns, payload=SimpleNamespace(cruiseState=SimpleNamespace(enabled=False))),
    FakeEvent("carState", base_ns + 10_000_000, payload=SimpleNamespace(cruiseState=SimpleNamespace(enabled=False))),
    FakeEvent("carState", base_ns + 20_000_000, payload=SimpleNamespace(cruiseState=SimpleNamespace(enabled=True))),
  ]
  state_result = query_field_events(
    states,
    service="carState",
    path="carState.cruiseState.enabled",
    base_mono_ns=base_ns,
    field_meta={"path": "carState.cruiseState.enabled", "type": "bool", "privacy": "driving"},
  )
  assert [sample["value"] for sample in state_result["series"]] == [False, True]
  assert state_result["stats"]["distinctValueCount"] == 2
  assert state_result["stats"]["stateTransitions"][0]["count"] == 1


def test_raw_query_honors_cancellation_before_scanning():
  try:
    query_field_events(
      [],
      service="carState",
      path="carState.vEgo",
      base_mono_ns=0,
      field_meta={"path": "carState.vEgo", "type": "float32", "privacy": "driving"},
      should_cancel=lambda: True,
    )
  except QueryCancelled:
    pass
  else:
    raise AssertionError("query cancellation was ignored")


def test_snapshot_uses_nearest_event_and_redacts_sensitive_values():
  base_ns = 60_000_000_000
  events = [
    FakeEvent("gpsLocation", base_ns, payload={"latitude": 37.1, "longitude": 127.1, "speed": 1.0}),
    FakeEvent("gpsLocation", base_ns + 2_000_000_000, valid=False, payload={"latitude": 37.2, "longitude": 127.2, "speed": 2.0}),
  ]
  result = query_snapshot_events(
    events,
    service="gpsLocation",
    target_ms=1700,
    base_mono_ns=base_ns,
    fields=[
      {"path": "gpsLocation.latitude", "type": "float64", "privacy": "route-location"},
      {"path": "gpsLocation.longitude", "type": "float64", "privacy": "route-location"},
      {"path": "gpsLocation.speed", "type": "float32", "privacy": "driving"},
    ],
  )
  assert result["found"] is True
  assert result["timeMs"] == 2000.0
  assert result["valid"] is False
  assert result["snapshot"]["latitude"]["redacted"] is True
  assert result["snapshot"]["longitude"]["redacted"] is True
  assert result["snapshot"]["speed"] == 2.0


def test_uint64_query_preserves_decimal_precision_and_sequence_quality():
  base_ns = 70_000_000_000
  start = 9_007_199_254_740_993
  events = [
    FakeEvent("roadCameraState", base_ns, payload=SimpleNamespace(timestampSof=start)),
    FakeEvent("roadCameraState", base_ns + 10_000_000, payload=SimpleNamespace(timestampSof=start + 2)),
    FakeEvent("roadCameraState", base_ns + 20_000_000, payload=SimpleNamespace(timestampSof=start + 1)),
  ]
  result = query_field_events(
    events,
    service="roadCameraState",
    path="roadCameraState.timestampSof",
    base_mono_ns=base_ns,
    field_meta={"path": "roadCameraState.timestampSof", "type": "uint64", "privacy": "driving"},
  )
  stats = result["stats"]["integer64"]
  assert result["kind"] == "integer64"
  assert stats["min"] == str(start)
  assert stats["max"] == str(start + 2)
  assert stats["sequenceQuality"]["backwardsCount"] == 1
  assert [sample["value"] for sample in result["series"]] == [str(start), str(start + 2), str(start + 1)]


def test_semantic_registry_is_the_authoritative_field_contract():
  speed = field_semantics("carState", "carState.vEgo", "float32")
  assert speed.unit == "m/s"
  assert speed.unit_status == "verified-source"
  assert speed.group == "vehicle"
  assert speed.statistics_policy == "continuous-numeric"
  assert speed.recommended_view == "graph"

  unknown = field_semantics("carState", "carState.futureNumeric", "float32")
  assert unknown.unit_status == "unresolved-numeric"
  assert unknown.statistics_policy == "unresolved-numeric"
  assert unknown.recommended_view == "numeric-audit"

  boolean_with_unit_suffix = field_semantics(
    "androidGnssDEPRECATED",
    "androidGnssDEPRECATED.measurements.clock.hasBiasNanos",
    "bool",
  )
  assert boolean_with_unit_suffix.unit == "boolean"
  assert boolean_with_unit_suffix.unit_status == "not-applicable"


def test_semantic_registry_versions_services_and_policies_are_valid():
  assert registry_versions() == {
    "catalogVersion": 2,
    "semanticVersion": 1,
    "statisticsVersion": 1,
  }
  provenance = registry_provenance()
  assert provenance["fieldOverrideCount"] == 823
  assert provenance["serviceOverrideCount"] == 88
  assert provenance["schemaCommit"]
  assert service_semantics("roadCameraState").group == "camera"
  numeric = statistics_policy("continuous-numeric")
  assert numeric.series_mode == "minmax-envelope"
  assert "time-weighted-mean" in numeric.metrics
  assert validate_registry() == []
