from openpilot.selfdrive.carrot.radar.tools.radar_lead_regression_audit import compare_payloads


def event(start_s: float, end_s: float, track_id: int) -> dict:
  return {
    "log": "route/rlog.zst",
    "start_s": start_s,
    "end_s": end_s,
    "track_ids": [track_id],
  }


def test_compare_classifies_internal_decision_as_suppressed_output() -> None:
  baseline = {"events": [event(57.3, 58.2, 32)]}
  current_decision = event(58.0, 58.2, 1024)
  current_decision["review"] = "known_detect"
  current = {
    "events": [],
    "decision_events": [current_decision],
  }

  result = compare_payloads(baseline, current)

  assert result["summary"] == {
    "matched": 0,
    "removed_known_clear": 0,
    "retained_known_clear": 0,
    "suppressed_current": 1,
    "decision_only_current": 0,
    "missing_current": 0,
    "new_current": 0,
    "later_current": 0,
  }
  assert result["comparisons"][0]["current_decision"]["track_ids"] == [1024]


def test_compare_keeps_missing_when_no_output_or_decision_exists() -> None:
  result = compare_payloads({"events": [event(10.0, 10.5, 35)]}, {"events": []})

  assert result["summary"]["missing_current"] == 1
  assert result["summary"]["suppressed_current"] == 0


def test_compare_counts_known_clear_removal_even_if_internal_decision_remains() -> None:
  old_false = event(13.5, 14.8, 48)
  old_false["review"] = "known_clear"
  current_decision = event(15.3, 17.5, 48)
  current_decision["review"] = "unreviewed"

  result = compare_payloads(
    {"events": [old_false]},
    {"events": [], "decision_events": [current_decision]},
  )

  assert result["summary"]["removed_known_clear"] == 1
  assert result["summary"]["suppressed_current"] == 0
  assert result["comparisons"][0]["current_decision"]["track_ids"] == [48]


def test_compare_allows_sensor_duplicates_to_match_one_current_episode() -> None:
  baseline = {
    "events": [
      event(10.0, 10.4, 35),
      event(10.1, 10.5, 1007),
    ],
  }
  current = {"events": [event(9.9, 10.6, 1007)]}

  result = compare_payloads(baseline, current)

  assert result["summary"]["matched"] == 2
  assert result["summary"]["missing_current"] == 0
  assert result["summary"]["new_current"] == 0
