from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar.radar_lead_runtime import (
  DEFAULT_CORNER_MODEL_PATH,
  DEFAULT_FRONT_MODEL_PATH,
  RadarLeadRuntime,
  runtime_context,
  runtime_points,
)
from openpilot.selfdrive.carrot.radar.radar_lead_model import RadarLeadDecision


class IntegerIndexOnlyList:
  def __init__(self, values):
    self.values = values

  def __len__(self):
    return len(self.values)

  def __getitem__(self, index):
    if not isinstance(index, int):
      raise TypeError("an integer is required")
    return self.values[index]


def test_runtime_context_accepts_capnp_style_lead_list() -> None:
  lead = SimpleNamespace(
    prob=0.9,
    x=(21.52,),
    y=(0.2,),
    v=(18.0,),
    a=(-0.4,),
    xStd=(0.5,),
    yStd=(0.2,),
    vStd=(0.7,),
  )
  model = SimpleNamespace(
    leadsV3=IntegerIndexOnlyList([lead]),
    position=SimpleNamespace(x=(0.0, 20.0), y=(0.0, 0.0)),
    laneLines=(),
    laneLineProbs=(),
  )

  context = runtime_context(1.0, 15.0, model)

  assert len(context.model_leads) == 1
  assert context.model_leads[0].d_rel == 20.0
  assert context.model_leads[0].y_rel == -0.2


def test_runtime_recovers_legacy_corner430_track_source() -> None:
  point = SimpleNamespace(
    trackId=300,
    dRel=24.0,
    yRel=1.6,
    vRel=-1.0,
    aRel=0.0,
    yvRel=-0.3,
    vLead=19.0,
    aLead=0.0,
    jLead=0.0,
    measured=True,
    radarSource="frontRadar",
  )

  assert runtime_points((point,))[0].source == "corner430"


def test_deployed_source_models_load_with_independent_state() -> None:
  assert DEFAULT_FRONT_MODEL_PATH.is_file()
  assert DEFAULT_CORNER_MODEL_PATH.is_file()

  runtime = RadarLeadRuntime(corner_radar_enabled=True)

  assert runtime._load()
  assert runtime.front_model is not None
  assert runtime.corner_model is not None
  assert runtime.front_model.sensor_mode == "front"
  assert runtime.corner_model.sensor_mode == "corner"
  assert runtime.front_features is not runtime.corner_features
  assert runtime.front_decisions is not runtime.corner_decisions


class _StubFeatures:
  def update(self, _context, _objects):
    return ()


class _StubModel:
  def __init__(self, prediction):
    self.prediction = prediction

  def predict(self, _features):
    return (self.prediction,)


class _StubDecisionFilter:
  def __init__(self, prediction):
    self.decision = RadarLeadDecision((), (prediction,))

  def update(self, _time_s, _predictions):
    return self.decision


def _runtime_with_stubbed_source_decisions(corner_radar_enabled: bool, monkeypatch) -> RadarLeadRuntime:
  runtime = RadarLeadRuntime(corner_radar_enabled=corner_radar_enabled)
  runtime.front_model = _StubModel("front")
  runtime.corner_model = _StubModel("corner")
  runtime.model = runtime.front_model
  runtime.front_features = _StubFeatures()
  runtime.corner_features = _StubFeatures()
  runtime.front_decisions = _StubDecisionFilter("front")
  runtime.corner_decisions = _StubDecisionFilter("corner")
  monkeypatch.setattr(runtime, "_load", lambda: True)
  return runtime


def _empty_model():
  return SimpleNamespace(
    leadsV3=(),
    position=SimpleNamespace(x=(), y=()),
    laneLines=(),
    laneLineProbs=(),
  )


def test_corner_vehicle_uses_only_corner_secondary_decision(monkeypatch) -> None:
  result = _runtime_with_stubbed_source_decisions(True, monkeypatch).update(0.0, 0.0, (), _empty_model())

  assert result.front_decision.cutin_candidates == ("front",)
  assert result.corner_decision.cutin_candidates == ("corner",)
  assert result.decision is result.corner_decision


def test_no_corner_vehicle_uses_front_secondary_decision(monkeypatch) -> None:
  result = _runtime_with_stubbed_source_decisions(False, monkeypatch).update(0.0, 0.0, (), _empty_model())

  assert result.decision is result.front_decision
