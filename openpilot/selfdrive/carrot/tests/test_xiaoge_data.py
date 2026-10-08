from types import SimpleNamespace

from openpilot.selfdrive.carrot.xiaoge_data import XiaogeDataBroadcaster, capnp_items


class NoSliceList(list):
  def __getitem__(self, index):
    if isinstance(index, slice):
      raise TypeError("an integer is required")
    return super().__getitem__(index)


def _line(*values):
  return SimpleNamespace(
    x=NoSliceList(values),
    y=NoSliceList(values),
    z=NoSliceList(values),
  )


def test_capnp_items_uses_only_integer_indexes():
  assert capnp_items(NoSliceList(range(6)), 4) == [0, 1, 2, 3]


def test_collect_model_data_accepts_non_sliceable_capnp_lists():
  model = SimpleNamespace(
    leadsV3=NoSliceList(),
    laneLineProbs=NoSliceList([0.1, 0.2, 0.3, 0.4, 0.5]),
    laneLineStds=NoSliceList([1.0, 2.0, 3.0, 4.0, 5.0]),
    laneLines=NoSliceList([_line(1.0, 2.0), _line(3.0)] * 3),
    roadEdgeStds=NoSliceList([0.5, 0.7, 0.9]),
    roadEdges=NoSliceList([_line(4.0, 5.0), _line(6.0)] * 2),
    meta=SimpleNamespace(distanceToRoadEdgeLeft=1.5, distanceToRoadEdgeRight=2.5),
    orientationRate=SimpleNamespace(z=NoSliceList([0.1, -0.4, 0.2])),
  )

  data = XiaogeDataBroadcaster.collect_model_data(model)

  assert data["laneLineProbs"] == [0.1, 0.2, 0.3, 0.4]
  assert data["laneLineStds"] == [1.0, 2.0, 3.0, 4.0]
  assert len(data["laneLines"]) == 4
  assert data["laneLines"][0]["x"] == [1.0, 2.0]
  assert len(data["roadEdges"]) == 2
  assert data["roadEdgeStds"] == [0.5, 0.7]


def test_collect_controls_state_uses_selfdrive_state_for_engagement_fields():
  controls_state = SimpleNamespace(curvature=0.0123)
  selfdrive_state = SimpleNamespace(
    enabled=True,
    active=True,
    vCruise=100.0,
    state="enabled",
    experimentalMode=True,
  )

  data = XiaogeDataBroadcaster.collect_controls_state(controls_state, selfdrive_state)

  assert data == {
    "enabled": True,
    "active": True,
    "vCruise": 100.0,
    "curvature": 0.0123,
    "state": "enabled",
    "experimentalMode": True,
  }
