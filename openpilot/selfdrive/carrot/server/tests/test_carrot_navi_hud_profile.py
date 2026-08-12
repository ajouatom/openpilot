from openpilot.selfdrive.carrot.server.features.carrot_navi.bridge import CarrotNaviWebBridge


class FakeParams:
  def __init__(self) -> None:
    self.writes: list[tuple[str, int]] = []

  def put_int_nonblocking(self, key: str, value: int) -> None:
    self.writes.append((key, value))


def test_hud_map_profile_is_reference_counted() -> None:
  bridge = CarrotNaviWebBridge(messaging_module=object())
  params = FakeParams()
  bridge._params = params

  bridge.set_hud_map_profile(True)
  bridge.set_hud_map_profile(True)
  bridge.set_hud_map_profile(False)
  bridge.set_hud_map_profile(False)

  assert params.writes == [
    ("CarrotNaviHudMapProfile", 1),
    ("CarrotNaviHudMapProfile", 1),
    ("CarrotNaviHudMapProfile", 1),
    ("CarrotNaviHudMapProfile", 0),
  ]
