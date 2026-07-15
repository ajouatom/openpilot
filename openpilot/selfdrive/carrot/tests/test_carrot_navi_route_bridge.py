from openpilot.selfdrive.carrot.carrot_man import CarrotMan
from openpilot.selfdrive.carrot.carrot_navi_control import parse_carrot_navi_control


class _Params:
  def __init__(self):
    self.onroad = True
    self.values = {}

  def get_bool(self, key):
    assert key == "IsOnroad"
    return self.onroad

  def put(self, key, value):
    self.values[key] = value


def _control(sequence=1, present=True):
  return parse_carrot_navi_control({
    "schemaVersion": 1,
    "connected": True,
    "sessionId": "session",
    "speed": {"meta": {"present": False, "sequence": 0}},
    "guidanceCurrent": {"meta": {"present": False, "sequence": 0}},
    "guidanceNext": {"meta": {"present": False, "sequence": 0}},
    "laneCurrent": {"meta": {"present": False, "sequence": 0}},
    "navigationStatus": {"meta": {"present": True, "sequence": 1}, "guidanceActive": True},
    "route": {
      "meta": {"present": present, "sequence": sequence},
      "polyline": [
        {"latitude": 37.5, "longitude": 127.1},
        {"latitude": 37.6, "longitude": 127.2},
      ] if present else [],
    },
  })


def _man():
  man = CarrotMan.__new__(CarrotMan)
  man.params = _Params()
  man.carrot_navi_route_session_id = ""
  man.carrot_navi_route_sequence = -1
  man.carrot_navi_route_owned = False
  man.navi_points = []
  man.navi_points_start_index = 0
  man.navi_points_active = False
  man.navd_active = False
  man.sent_routes = []
  man.send_routes = lambda coords: man.sent_routes.append(coords)
  return man


def test_7714_route_updates_existing_route_consumer_and_clears_on_tombstone():
  man = _man()
  control = _control()
  assert control is not None

  man._update_carrot_navi_route(control)
  assert man.navi_points == [(127.1, 37.5), (127.2, 37.6)]
  assert man.navi_points_active and man.navd_active
  assert man.carrot_navi_route_owned
  assert man.sent_routes[-1][0] == {"latitude": 37.5, "longitude": 127.1}

  man._update_carrot_navi_route(control)
  assert len(man.sent_routes) == 1

  man.navi_points = []
  man.navi_points_active = False
  man._update_carrot_navi_route(control)
  assert len(man.sent_routes) == 2
  assert man.navi_points_active

  tombstone = _control(sequence=2, present=False)
  assert tombstone is not None
  man._update_carrot_navi_route(tombstone)
  assert not man.navi_points_active
  assert not man.navd_active
  assert not man.carrot_navi_route_owned
  assert man.sent_routes[-1] == []


def test_7714_absent_route_does_not_clear_native_nav_route():
  man = _man()
  tombstone = _control(sequence=1, present=False)
  assert tombstone is not None

  man._update_carrot_navi_route(tombstone, force=True)
  assert man.sent_routes == []
  assert not man.carrot_navi_route_owned


def test_native_route_update_wins_when_owned_7714_route_is_no_longer_available():
  man = _man()
  control = _control()
  assert control is not None
  man._update_carrot_navi_route(control)
  assert man.carrot_navi_route_owned

  native_points = [(126.9, 37.4)]
  man.navi_points = native_points.copy()
  man.navi_points_active = True
  man.navd_active = True
  sent_count = len(man.sent_routes)

  tombstone = _control(sequence=2, present=False)
  assert tombstone is not None
  man._update_carrot_navi_route(tombstone, force=True)

  assert man.navi_points == native_points
  assert man.navi_points_active and man.navd_active
  assert not man.carrot_navi_route_owned
  assert len(man.sent_routes) == sent_count


def test_native_route_update_wins_when_7714_disconnects():
  man = _man()
  control = _control()
  assert control is not None
  man._update_carrot_navi_route(control)

  native_points = [(126.9, 37.4)]
  man.navi_points = native_points.copy()
  man.navi_points_active = True
  man.navd_active = True
  sent_count = len(man.sent_routes)

  man._update_carrot_navi_route(None, force=True)

  assert man.navi_points == native_points
  assert man.navi_points_active and man.navd_active
  assert not man.carrot_navi_route_owned
  assert len(man.sent_routes) == sent_count


def test_legacy_7713_route_path_remains_operational():
  man = _man()
  man.handle_route([
    {"x": 126.9, "y": 37.4, "valid": True},
    {"x": 127.0, "y": 37.5, "valid": True},
  ])

  assert man.navi_points == [(126.9, 37.4), (127.0, 37.5)]
  assert man.navi_points_active and man.navd_active
  assert man.sent_routes[-1][0] == {"latitude": 37.4, "longitude": 126.9}
  assert "NavDestination" in man.params.values
