from openpilot.selfdrive.carrot.radar.radar_lead_tau import RadarLeadTauTracker
from openpilot.selfdrive.carrot.tests.test_radar_lead_controller import prediction


def test_reaction_factor_sets_quiet_lead_tau() -> None:
  lead = prediction(40, 0.2, 0.9, 0.1, a_lead=0.1, j_lead=0.1)
  tracker = RadarLeadTauTracker(0.5)
  tracker.update(0.0, (lead,))
  assert tracker.value(lead) == 0.75


def test_acceleration_change_decays_tau_like_legacy_track() -> None:
  lead = prediction(40, 0.2, 0.9, 0.1, a_lead=-1.0, j_lead=-0.8)
  tracker = RadarLeadTauTracker(1.0)
  tracker.update(0.0, (lead,))
  assert 0.0 < tracker.value(lead) < 1.5
