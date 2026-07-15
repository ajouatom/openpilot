from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar_lead_controller import RadarLeadModelController
from openpilot.selfdrive.carrot.radar_lead_model import RadarLeadDecision, RadarLeadFeatures, RadarLeadPrediction
from openpilot.selfdrive.carrot.radar_lead_runtime import RadarLeadRuntimeResult
from openpilot.selfdrive.carrot.radar_object_fusion import FusedRadarObject


def prediction(
  track_id: int, y_rel: float, lead_prob: float, cutin_prob: float, external_prob: float = 0.0,
  *, front: bool = True, scc: bool = False, v_lead: float = 19.0, a_lead: float = 0.0, j_lead: float = 0.0,
) -> RadarLeadPrediction:
  source = "scc" if scc else "front" if front else "corner"
  obj = FusedRadarObject(
    object_id=f"{source}:{track_id}", d_rel=12.0 + track_id / 10.0, y_rel=y_rel,
    v_rel=-1.0, a_rel=0.0, yv_rel=-0.4, v_lead=v_lead,
    front_track_id=track_id if front and not scc else None,
    corner_track_id=None if front or scc else track_id,
    scc_track_id=track_id if scc else None,
    front_d_rel=12.0 if front and not scc else None,
    corner_d_rel=None if front or scc else 12.0,
    front_y_rel=y_rel if front and not scc else None,
    corner_y_rel=None if front or scc else y_rel,
    front_v_rel=-1.0 if front and not scc else None,
    corner_v_rel=None if front or scc else -1.0,
    distance_source="scc" if scc else "frontRadar" if front else "corner235",
    lateral_source="scc" if scc else "frontRadar" if front else "corner235",
    match_confidence=0.35, pair_age=8, a_lead=a_lead, j_lead=j_lead,
  )
  features = RadarLeadFeatures(f"{source}:{track_id}", (f"{source}:{track_id}",), obj, (), 8, y_rel, y_rel - 0.4, 0.4)
  return RadarLeadPrediction(
    features, lead_prob, cutin_prob, max(lead_prob, cutin_prob, external_prob), external_prob,
  )


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


def test_corner_only_lead_is_not_used_without_cutin_probability() -> None:
  corner = prediction(204, 0.4, 0.95, 0.1, front=False, a_lead=-0.7, j_lead=0.3)

  class CornerRuntime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((corner,), ()), (corner,), 0.1)

  controller = RadarLeadModelController()
  controller.runtime = CornerRuntime()
  output = controller.update(0.0, 20.0, (), None)
  assert output.lead_one is None
  assert output.lead_two is None


def test_corner_cutin_probability_can_fill_lead_two() -> None:
  corner = prediction(204, 0.4, 0.1, 0.95, front=False, a_lead=-0.7, j_lead=0.3)

  class CornerRuntime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((), (corner,)), (corner,), 0.1)

  controller = RadarLeadModelController()
  controller.runtime = CornerRuntime()
  output = controller.update(0.0, 20.0, (), None)
  assert output.lead_one is None
  assert output.lead_two is not None and output.lead_two["radarTrackId"] == 204
  assert output.lead_two["aLead"] == -0.7
  assert output.lead_two["jLead"] == 0.3


def test_stationary_corner_only_lead_is_not_used_for_control() -> None:
  corner = prediction(204, 0.4, 0.95, 0.9, front=False, v_lead=0.0)

  class CornerRuntime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((corner,), (corner,)), (corner,), 0.1)

  controller = RadarLeadModelController()
  controller.runtime = CornerRuntime()
  output = controller.update(0.0, 20.0, (), None)
  assert output.lead_one is None
  assert output.lead_two is None


def test_front_prediction_does_not_bypass_temporal_decision() -> None:
  lead = prediction(40, 0.2, 0.01, 0.1)

  class WarmupRuntime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((), ()), (lead,), 0.1)

  controller = RadarLeadModelController()
  controller.runtime = WarmupRuntime()
  output = controller.update(0.0, 20.0, (), None)
  assert output.lead_one is None


def test_largest_active_front_probability_selects_lead_one() -> None:
  weak = prediction(40, 0.2, 0.20, 0.1)
  strong = prediction(41, 0.2, 0.55, 0.1)

  class WarmupRuntime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((weak, strong), ()), (weak, strong), 0.1)

  controller = RadarLeadModelController()
  controller.runtime = WarmupRuntime()
  output = controller.update(0.0, 20.0, (), None)
  assert output.lead_one is not None and output.lead_one["radarTrackId"] == 41


def test_scc_prediction_can_fill_lead_one_when_front_tracks_are_missing() -> None:
  lead = prediction(0, 0.1, 0.6, 0.0, front=False, scc=True)

  class SccRuntime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((lead,), ()), (lead,), 0.1)

  controller = RadarLeadModelController()
  controller.runtime = SccRuntime()
  output = controller.update(0.0, 20.0, (), None)
  assert output.lead_one is not None and output.lead_one["radarTrackId"] == 0


def test_vision_lead_fills_lead_one_when_model_runtime_fails() -> None:
  model = SimpleNamespace(
    leadsV3=(SimpleNamespace(prob=0.82, x=(21.52,), y=(0.2,), v=(18.0,), a=(-0.1,)),),
    velocity=SimpleNamespace(x=(20.0,)),
    position=SimpleNamespace(x=(0.0, 30.0), y=(0.0, 0.0)),
  )

  controller = RadarLeadModelController()
  controller.runtime = Runtime(available=False)
  output = controller.update(0.0, 20.0, (), model)
  assert not output.available
  assert output.lead_one is not None
  assert output.lead_one["status"]
  assert not output.lead_one["radar"]


def test_vision_fallback_uses_only_first_model_lead() -> None:
  model = SimpleNamespace(
    leadsV3=(
      SimpleNamespace(prob=0.62, x=(11.52,), y=(0.0,), v=(19.0,), a=(0.0,)),
      SimpleNamespace(prob=0.49, x=(31.52,), y=(-0.3,), v=(18.0,), a=(-0.1,)),
      SimpleNamespace(prob=0.75, x=(21.52,), y=(0.2,), v=(17.0,), a=(-0.2,)),
    ),
    velocity=SimpleNamespace(x=(20.0,)),
    position=SimpleNamespace(x=(0.0, 40.0), y=(0.0, 0.0)),
  )

  controller = RadarLeadModelController()
  controller.runtime = Runtime(available=False)
  output = controller.update(0.0, 20.0, (), model)
  assert output.lead_one is not None
  assert output.lead_one["dRel"] == 10.0
  assert output.lead_one["modelProb"] == 0.62


def test_external_probability_fills_lead_two_after_cutin() -> None:
  lead = prediction(40, 0.2, 0.8, 0.0)
  external = prediction(41, 0.6, 0.0, 0.0, 0.9)

  class ExternalRuntime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((lead,), (), (external,)), (lead, external), 0.1)

  controller = RadarLeadModelController()
  controller.runtime = ExternalRuntime()
  output = controller.update(0.0, 20.0, (), None)
  assert output.lead_one is not None and output.lead_one["radarTrackId"] == 40
  assert output.lead_two is not None and output.lead_two["radarTrackId"] == 41
  assert output.lead_external is not None and output.lead_external["modelProb"] == 0.9


def test_cutin_uses_temporal_decision_only() -> None:
  lead = prediction(40, 0.2, 0.2, 0.1)
  low_cutin = prediction(41, 2.0, 0.1, 0.69)
  high_cutin = prediction(42, 2.0, 0.1, 0.71)

  class CutinRuntime:
    def update(self, *_args):
      return RadarLeadRuntimeResult(True, RadarLeadDecision((lead,), (high_cutin,)), (lead, low_cutin, high_cutin), 0.1)

  controller = RadarLeadModelController()
  controller.runtime = CutinRuntime()
  output = controller.update(0.0, 20.0, (), None)
  assert output.lead_one is not None and output.lead_one["radarTrackId"] == 40
  assert output.lead_two is not None and output.lead_two["radarTrackId"] == 42
