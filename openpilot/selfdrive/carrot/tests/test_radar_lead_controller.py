from openpilot.selfdrive.carrot.radar_lead_controller import RadarLeadModelController
from openpilot.selfdrive.carrot.radar_lead_model import RadarLeadDecision, RadarLeadFeatures, RadarLeadPrediction
from openpilot.selfdrive.carrot.radar_lead_runtime import RadarLeadRuntimeResult
from openpilot.selfdrive.carrot.radar_object_fusion import FusedRadarObject


def prediction(track_id: int, y_rel: float, lead_prob: float, cutin_prob: float) -> RadarLeadPrediction:
  obj = FusedRadarObject(
    object_id=f"front:{track_id}", d_rel=12.0 + track_id / 10.0, y_rel=y_rel,
    v_rel=-1.0, a_rel=0.0, yv_rel=-0.4, v_lead=19.0,
    front_track_id=track_id, corner_track_id=None, scc_track_id=None,
    front_d_rel=12.0, corner_d_rel=None, front_y_rel=y_rel, corner_y_rel=None,
    front_v_rel=-1.0, corner_v_rel=None, distance_source="frontRadar", lateral_source="frontRadar",
    match_confidence=0.35, pair_age=8,
  )
  features = RadarLeadFeatures(f"front:{track_id}", (f"front:{track_id}",), obj, (), 8, y_rel, y_rel - 0.4, 0.4)
  return RadarLeadPrediction(features, lead_prob, cutin_prob, max(lead_prob, cutin_prob))


class Runtime:
  def __init__(self, available: bool = True) -> None:
    self.available = available

  def update(self, *_args):
    lead = prediction(40, 0.2, 0.92, 0.1)
    cutin = prediction(41, 2.0, 0.3, 0.91)
    return RadarLeadRuntimeResult(
      self.available,
      RadarLeadDecision((lead,), (cutin,)) if self.available else RadarLeadDecision((), ()),
      (lead, cutin) if self.available else (),
      0.1,
      "" if self.available else "missing model",
    )


def test_model_path_selects_lead_one_and_independent_cutin_lead_two() -> None:
  controller = RadarLeadModelController()
  controller.runtime = Runtime()
  output = controller.update(0.0, 20.0, (), None)
  assert output.available
  assert output.lead_one is not None and output.lead_one["radarTrackId"] == 40
  assert output.lead_two is not None and output.lead_two["radarTrackId"] == 41
  assert output.leads_cutin[0]["modelProb"] == 0.91


def test_model_failure_is_exposed_without_legacy_fallback() -> None:
  controller = RadarLeadModelController()
  controller.runtime = Runtime(available=False)
  output = controller.update(0.0, 20.0, (), None)
  assert not output.available
  assert output.error == "missing model"
  assert output.lead_one is None
  assert output.lead_two is None
