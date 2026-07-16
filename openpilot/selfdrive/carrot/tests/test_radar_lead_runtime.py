from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar_lead_runtime import runtime_context


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
