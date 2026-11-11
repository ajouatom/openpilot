from dataclasses import replace

import pyray as rl

from openpilot.selfdrive.carrot.cluster.cluster_models import NaviDashboardState, NaviMediaFrame
from openpilot.selfdrive.ui.onroad.carrot_navi_map import CarrotNaviMapView, map_frame_from_dashboard


def test_map_frame_requires_connected_fresh_dashboard():
  frame = NaviMediaFrame("render:map_main", 7, True, "image/rgba", 1, 1, b"rgba")
  connected = NaviDashboardState(True, "ipc://carrotNaviMedia", media=(frame,))

  assert map_frame_from_dashboard(connected) is frame
  assert map_frame_from_dashboard(replace(connected, connected=False)) is None
  assert map_frame_from_dashboard(replace(connected, map_stream_stalled=True)) is None


def test_map_view_decodes_only_when_setting_is_enabled():
  frame = NaviMediaFrame("render:map_main", 8, True, "image/rgba", 1, 1, b"rgba")
  dashboard = NaviDashboardState(True, "ipc://carrotNaviMedia", media=(frame,))

  class FakeSource:
    def __init__(self, *, decode_media):
      self.decode_media = decode_media
      self.transitions = []
      self.closed = False

    def set_decode_media(self, enabled):
      self.decode_media = enabled
      self.transitions.append(enabled)

    def update(self, _navi):
      return dashboard

    def close(self):
      self.closed = True

  class FakeRenderer:
    def __init__(self):
      self.frames = []
      self.closed = False

    def draw(self, drawn_frame, rect):
      self.frames.append((drawn_frame, rect))
      return True

    def close(self):
      self.closed = True

  source = None
  renderer = None

  def source_factory(**kwargs):
    nonlocal source
    source = FakeSource(**kwargs)
    return source

  def renderer_factory():
    nonlocal renderer
    renderer = FakeRenderer()
    return renderer

  bootstrap_requests = []
  view = CarrotNaviMapView(source_factory, renderer_factory, lambda: bootstrap_requests.append(True))
  rect = rl.Rectangle(0, 0, 2160, 1080)

  assert view.render(rect, False) is False
  assert source is None
  assert renderer.frames == []
  assert view.render(rect, True) is True
  assert source.transitions == [True]
  assert bootstrap_requests == [True]
  assert renderer.frames == [(frame, rect)]
  assert view.render(rect, False) is False
  assert source.transitions == [True, False]
  assert source.closed is True

  view.close()
  assert renderer.closed is True
