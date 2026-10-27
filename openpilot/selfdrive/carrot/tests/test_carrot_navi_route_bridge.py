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
  from types import SimpleNamespace
  from openpilot.selfdrive.navigation.source import RouteSource
  man = CarrotMan.__new__(CarrotMan)
  man.params = _Params()
  man.route_source = RouteSource()
  man.external_active = False
  man.carrot_serv = SimpleNamespace(_external_navigation_connected=lambda: man.external_active)
  man.sent_routes = []
  man.pm = SimpleNamespace(send=lambda name, message: man.sent_routes.append(message.navRoute.to_dict()['coordinates']))
  return man


def test_7714_overrides_mapbox_and_tombstone_does_not_mix_sources():
  man = _man()
  man.send_routes([{'latitude': 36., 'longitude': 126.}], from_navd=True)
  man._select_navigation_route(_control())
  assert man.navi_points == [(127.1, 37.5), (127.2, 37.6)]
  assert len(man.sent_routes) == 1
  man._select_navigation_route(_control())
  assert len(man.sent_routes) == 1
  man._select_navigation_route(_control(sequence=2, present=False))
  assert man.sent_routes[-1] == []
  man._select_navigation_route(None)
  assert man.navi_points == [(126., 36.)]


def test_legacy_route_does_not_overwrite_saved_destination_and_clears():
  man = _man()
  man.params.values['NavDestination'] = 'saved mapbox destination'
  man.handle_route([{'x': 126.9, 'y': 37.4}, {'x': 127., 'y': 37.5}])
  man.external_active = True
  man._select_navigation_route(None)
  assert man.navi_points == [(126.9, 37.4), (127., 37.5)]
  assert man.params.values['NavDestination'] == 'saved mapbox destination'
  man.handle_route([])
  man._select_navigation_route(None)
  assert man.sent_routes[-1] == []


def test_late_mapbox_route_does_not_replace_external_route():
  man = _man()
  man._select_navigation_route(_control())
  man.send_routes([{'latitude': 36., 'longitude': 126.}], from_navd=True)
  man._select_navigation_route(_control())
  assert len(man.sent_routes) == 1
  assert man.navi_points == [(127.1, 37.5), (127.2, 37.6)]


def test_heartbeat_retains_route_identity_and_offroad_keeps_geometry(monkeypatch):
  man = _man()
  man._select_navigation_route(_control())
  identity = man.nav_route_id
  man.nav_route_next = 0.
  man._select_navigation_route(_control())
  assert man.nav_route_id == identity and len(man.sent_routes) == 2
  man.params.onroad = False
  man.carrot_serv.active_carrot = 2
  assert man.carrot_navi_route() == ([], [], 300)
  assert man.navi_points == [(127.1, 37.5), (127.2, 37.6)]
