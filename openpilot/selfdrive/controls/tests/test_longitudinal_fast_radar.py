import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.controls.lib.longitudinal_fast_radar import (
  LEAD_ONE_MASK,
  LEAD_TWO_MASK,
  FastRadarOverlay,
  RadarStateOverride,
)


def make_radar_state(
  lead_one_id: int = 35,
  *,
  lead_one_d_rel: float = 20.0,
  lead_one_v_rel: float = -2.0,
  lead_two_id: int = -1,
):
  state = log.RadarState.new_message()
  lead_one = state.leadOne
  lead_one.status = lead_one_id >= 0
  lead_one.radar = lead_one_id >= 0
  lead_one.radarTrackId = lead_one_id
  lead_one.dRel = lead_one_d_rel
  lead_one.yRel = 0.3
  lead_one.dPath = 0.2
  lead_one.vRel = lead_one_v_rel
  lead_one.vLead = 8.0
  lead_one.vLeadK = 8.0
  lead_one.aLead = 0.0
  lead_one.aLeadK = 0.0
  lead_one.aLeadTau = 1.5
  lead_one.modelProb = 0.95

  lead_two = state.leadTwo
  lead_two.status = lead_two_id >= 0
  lead_two.radar = lead_two_id >= 0
  lead_two.radarTrackId = lead_two_id
  lead_two.dRel = 30.0
  lead_two.vRel = -1.0
  lead_two.vLead = 9.0
  lead_two.vLeadK = 9.0
  lead_two.aLeadTau = 1.5
  return state


def make_radar_data(*points):
  data = car.RadarData.new_message()
  builders = data.init("points", len(points))
  for builder, values in zip(builders, points, strict=True):
    builder.trackId = values.get("track_id", 35)
    builder.measured = values.get("measured", True)
    builder.radarSource = values.get("source", "frontRadar")
    builder.dRel = values.get("d_rel", 19.0)
    builder.yRel = values.get("y_rel", 0.3)
    builder.vRel = values.get("v_rel", -2.0)
    builder.aRel = values.get("a_rel", -0.5)
    builder.aLead = values.get("a_lead", -1.0)
    builder.jLead = values.get("j_lead", -2.0)
  return data


def confirm_selection(overlay, state, start_ns=1_000_000_000):
  overlay.observe_radar_state(state, start_ns, True)
  overlay.observe_radar_state(state, start_ns + 50_000_000, True)
  return start_ns + 50_000_000


def build(overlay, state, data, radar_mono_ns, *, live_offset_ns=50_000_000, **kwargs):
  return overlay.build(
    state,
    data,
    v_ego=10.0,
    radar_state_mono_ns=radar_mono_ns,
    live_tracks_mono_ns=radar_mono_ns + live_offset_ns,
    radar_state_valid=kwargs.get("radar_state_valid", True),
    live_tracks_valid=kwargs.get("live_tracks_valid", True),
  )


def test_requires_two_distinct_full_radard_confirmations():
  state = make_radar_state()
  data = make_radar_data({})
  overlay = FastRadarOverlay(front_radar_delay_s=0.05)
  overlay.observe_radar_state(state, 1_000_000_000, True)

  result = build(overlay, state, data, 1_000_000_000)

  assert not overlay.lead_one_ready(state)
  assert result.lead_mask == 0
  assert result.lead_one_reason == "selectionUnstable"

  overlay.observe_radar_state(state, 1_050_000_000, True)
  assert overlay.lead_one_ready(state)


def test_matching_stable_track_refreshes_only_longitudinal_kinematics():
  state = make_radar_state()
  data = make_radar_data({"d_rel": 19.0, "v_rel": -2.0, "a_lead": -1.0, "j_lead": -2.0})
  overlay = FastRadarOverlay(front_radar_delay_s=0.05)
  radar_mono_ns = confirm_selection(overlay, state)

  result = build(overlay, state, data, radar_mono_ns)
  lead = result.radar_state.leadOne

  assert result.lead_mask == LEAD_ONE_MASK
  assert result.lead_one_track_id == 35
  assert result.lead_one_reason == "active"
  assert lead.dRel == pytest.approx(18.9)
  assert lead.vRel == pytest.approx(-2.0)
  assert lead.vLead == pytest.approx(8.0)
  assert lead.aLead == pytest.approx(-1.0)
  assert lead.aLeadK == pytest.approx(-1.0)
  assert lead.jLead == pytest.approx(-2.0)
  assert lead.aLeadTau == pytest.approx(1.35)
  assert lead.yRel == pytest.approx(0.3)
  assert lead.dPath == pytest.approx(0.2)
  assert lead.modelProb == pytest.approx(0.95)
  assert state.leadOne.dRel == pytest.approx(20.0)


def test_large_declared_front_radar_delay_is_supported():
  state = make_radar_state(
    lead_one_d_rel=19.2,
    lead_one_v_rel=-1.0,
  )
  data = make_radar_data({
    "d_rel": 20.0,
    "v_rel": -1.0,
    "a_lead": 0.0,
    "j_lead": 0.0,
  })
  overlay = FastRadarOverlay(front_radar_delay_s=0.8)
  radar_mono_ns = confirm_selection(overlay, state)

  result = build(overlay, state, data, radar_mono_ns)

  assert result.lead_mask == LEAD_ONE_MASK
  assert result.lead_one_reason == "active"
  assert result.radar_state.leadOne.dRel == pytest.approx(19.2)


@pytest.mark.parametrize(
  ("data", "reason"),
  [
    (make_radar_data({"track_id": 36}), "trackMissing"),
    (make_radar_data({"d_rel": 100.0}), "distanceDiscontinuity"),
    (make_radar_data({"v_rel": -10.0}), "velocityDiscontinuity"),
    (make_radar_data({"measured": False}), "trackUnmeasured"),
    (make_radar_data({"d_rel": float("nan")}), "nonFinite"),
  ],
)
def test_unsafe_or_mismatched_track_falls_back_to_validated_state(data, reason):
  state = make_radar_state()
  overlay = FastRadarOverlay(front_radar_delay_s=0.05)
  radar_mono_ns = confirm_selection(overlay, state)

  result = build(overlay, state, data, radar_mono_ns)

  assert result.lead_mask == 0
  assert result.lead_one_reason == reason
  assert result.radar_state.leadOne.dRel == pytest.approx(20.0)


def test_duplicate_track_id_is_ambiguous_and_rejected():
  state = make_radar_state()
  data = make_radar_data({"track_id": 35}, {"track_id": 35})
  overlay = FastRadarOverlay(front_radar_delay_s=0.05)
  radar_mono_ns = confirm_selection(overlay, state)

  result = build(overlay, state, data, radar_mono_ns)

  assert result.lead_mask == 0
  assert result.lead_one_reason == "trackMissing"


def test_stale_or_invalid_inputs_never_activate_fast_path():
  state = make_radar_state()
  data = make_radar_data({})
  overlay = FastRadarOverlay(front_radar_delay_s=0.05)
  radar_mono_ns = confirm_selection(overlay, state)

  stale = build(overlay, state, data, radar_mono_ns, live_offset_ns=151_000_000)
  invalid = build(overlay, state, data, radar_mono_ns, live_tracks_valid=False)

  assert stale.lead_mask == 0
  assert stale.lead_one_reason == "selectionStale"
  assert invalid.lead_mask == 0
  assert invalid.lead_one_reason == "liveTracksInvalid"


def test_lead_two_can_refresh_independently_after_validation():
  state = make_radar_state(lead_one_id=-1, lead_two_id=42)
  data = make_radar_data({"track_id": 42, "d_rel": 29.0, "v_rel": -1.0})
  overlay = FastRadarOverlay(front_radar_delay_s=0.05)
  radar_mono_ns = confirm_selection(overlay, state)

  result = build(overlay, state, data, radar_mono_ns)

  assert result.lead_mask == LEAD_TWO_MASK
  assert result.lead_one_track_id == -1
  assert result.radar_state.leadTwo.dRel == pytest.approx(28.95)


def test_radar_state_override_delegates_everything_except_radar_state():
  class FakeSubMaster:
    def __init__(self):
      self.logMonoTime = {"radarState": 1}

    def __getitem__(self, service):
      return original

  original = object()
  replacement = object()
  sm = FakeSubMaster()
  view = RadarStateOverride(sm, replacement)

  assert view["radarState"] is replacement
  assert view["carState"] is original
  assert view.logMonoTime == {"radarState": 1}


def test_debug_reason_names_match_longitudinal_plan_schema():
  reasons = (
    "inactive", "active", "notRadarLead", "selectionPending",
    "selectionUnstable", "trackMissing", "trackUnmeasured", "nonFinite",
    "invalidDistance", "distanceDiscontinuity", "velocityDiscontinuity",
    "radarStateInvalid", "liveTracksInvalid", "selectionStale",
  )
  for reason in reasons:
    plan = log.LongitudinalPlan.new_message()
    plan.fastLeadReason = reason
    assert str(plan.fastLeadReason) == reason


def test_radar_state_timestamp_uses_deprecated_schema_group():
  plan = log.LongitudinalPlan.new_message()
  plan.deprecated.radarStateMonoTime = 123
  assert plan.deprecated.radarStateMonoTime == 123
