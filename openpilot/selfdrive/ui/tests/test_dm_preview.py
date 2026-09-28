import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS

import pytest

from openpilot.selfdrive.ui.dm_preview import face_crop, preview_rect, preview_state, sample_fresh


def state(**changes):
  values = {'monitoring_fresh': True, 'camera_unavailable': False, 'driver_fresh': True,
                'frame_fresh': True, 'face_detected': True, 'distracted': False, 'lockout': False}
  return preview_state(**(values | changes))


@pytest.mark.parametrize(('changes', 'kind'), [
  ({}, 'tracking'), ({'monitoring_fresh': False, 'camera_unavailable': True}, 'waiting'),
  ({'camera_unavailable': True, 'driver_fresh': False}, 'wheel'),
  ({'frame_fresh': False}, 'waiting'), ({'driver_fresh': False}, 'waiting'),
  ({'face_detected': False}, 'searching'), ({'distracted': True}, 'warning'),
  ({'lockout': True, 'camera_unavailable': True}, 'warning'), ({'wheel_policy': True}, 'wheel'),
])
def test_status_does_not_claim_valid_camera_or_attention(changes, kind):
  assert state(**changes).kind == kind


@pytest.mark.parametrize(('stamp', 'fresh'), [(10, True), (9.51, True), (9.5, False), (0, False), (11, False), (float('nan'), False)])
def test_freshness_boundaries(stamp, fresh):
  assert sample_fresh(10, stamp) == fresh


@pytest.mark.parametrize(('width', 'height'), [(1928, 1208), (1344, 760)])
@pytest.mark.parametrize('face', [(-0.45, 0), (0.45, 0), (0, 0.4), (0, -0.1)])
@pytest.mark.parametrize('aspect', [256 / 144, 52 / 46])
def test_face_crop_fits_both_camera_sizes_and_driving_sides(width, height, face, aspect):
  x, y, w, h = face_crop(width, height, face, aspect)
  assert 0 <= x <= width - w and 0 <= y <= height - h
  assert w / h == pytest.approx(aspect)
  mirror = face_crop(width, height, (-face[0], face[1]), aspect)
  assert x + mirror[0] + w == pytest.approx(width)


@pytest.mark.parametrize('face', [[], [0], [float('nan'), 0], [0, float('inf')], [10, 10]])
def test_invalid_face_position_uses_full_image(face):
  assert face_crop(1928, 1208, face, 1) is None


def test_layout_respects_existing_panels_and_parent_offsets():
  # C3 time ends at y+190, plot starts at x+350, speed panel occupies the bottom.
  x, y, w, h = preview_rect(300, 30, 1830, 1020, False)
  assert x + w < 300 + 350 and y > 30 + 190 and y + h < 30 + 600
  # C4 sits right of D (ending at x+373), before the side strip (starting x+476).
  x, y, w, h = preview_rect(536, 0, 536, 240, True)
  assert (w, h) == (84, 84)
  assert x - (536 + 373) == 9 and (536 + 476) - (x + w) == 10
  assert y == 144 and y + h == 228


@pytest.fixture
def preview(monkeypatch):
  import pyray as rl
  for name in ('draw_rectangle_rec', 'draw_rectangle_lines_ex', 'draw_text_ex', 'draw_circle_lines', 'draw_line_ex'):
    monkeypatch.setattr(rl, name, lambda *a: None)
  labels, rendered, polls = [], [], []
  monkeypatch.setattr(rl, 'draw_text_ex', lambda font, text, *a: labels.append(text))
  now = [10.0]
  frame = NS(width=1928, height=1208)
  clients = []

  class Client:
    timestamp_sof = 10_000_000_000
    def __init__(self, *a, **kw):
      self.pending = frame
      clients.append(self)
    def recv(self, timeout_ms):
      polls.append(timeout_ms)
      pending, self.pending = self.pending, None
      return pending

  class Camera:
    def __init__(self, name, stream):
      self._name, self._stream_type = name, stream
      self.client = Client()
      self.frame = None
      self.closed = 0
    def close(self):
      self.closed += 1
    def set_enabled(self, enabled):
      assert not enabled
    def _clear_textures(self):
      pass
    def _ensure_connection(self):
      return True
    def render(self, rect):
      self._render(rect)
    def _render_textures(self, source, target):
      rendered.append((source, target))
    _render_egl = _render_textures

  dm = NS(cameraUnavailable=False, isRHD=False, lockout=False, alwaysOnLockout=False, activePolicy='vision',
          visionPolicyState=NS(faceDetected=True, isDistracted=False))
  driver = NS(leftDriverData=NS(facePosition=[-0.35, 0]), rightDriverData=NS(facePosition=[0.35, 0]))

  class SM(dict):
    valid = {'driverMonitoringState': True, 'driverStateV2': True}
    alive = valid.copy()
    recv_frame = {'driverMonitoringState': 11, 'driverStateV2': 11}
    logMonoTime = {'driverMonitoringState': 10_000_000_000, 'driverStateV2': 10_000_000_000}

  sm = SM(driverMonitoringState=dm, driverStateV2=driver, selfdriveState=NS(enabled=True))
  ui = NS(sm=sm, started=True, started_frame=1, always_on_dm=False)  # No writable Params API.
  for name, attrs in {
    'msgq.visionipc': {'VisionIpcClient': Client, 'VisionStreamType': NS(VISION_STREAM_DRIVER=1)},
    'openpilot.selfdrive.ui.onroad.cameraview': {'CameraView': Camera},
    'openpilot.selfdrive.ui.ui_state': {'ui_state': ui},
    'openpilot.system.hardware': {'TICI': False},
    'openpilot.system.ui.lib.application': {'FontWeight': NS(BOLD=1), 'gui_app': NS(font=lambda _: None)},
  }.items():
    module = ModuleType(name)
    module.__dict__.update(attrs)
    monkeypatch.setitem(sys.modules, name, module)
  spec = importlib.util.spec_from_file_location('_test_driver_preview', Path(__file__).parents[1] / 'onroad/driver_preview.py')
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
  inset = module.DriverPreview()
  return NS(inset=inset, ui=ui, sm=sm, dm=dm, frame=frame, now=now, labels=labels, rendered=rendered,
            polls=polls, clients=clients, rect=rl.Rectangle(0, 0, 2160, 1080))


def test_live_preview_is_passive_and_nonblocking(preview):
  p = preview
  assert p.inset.draw_onroad(p.rect, False)
  assert p.polls == [0] and len(p.rendered) == 1 and p.labels[-1] == 'DM FACE'
  assert p.rendered[0][0].width < 0  # A single mirrored crop.
  p.inset.draw_onroad(p.rect, False)
  assert p.polls == [0]  # No polling twice inside the 100 ms budget.


def test_camera_stall_drops_cached_face_even_with_fresh_dm(preview):
  p = preview
  p.inset.draw_onroad(p.rect, False)
  p.now[0] = 10.6
  p.sm.logMonoTime = {'driverMonitoringState': 10_600_000_000, 'driverStateV2': 10_600_000_000}
  p.inset.draw_onroad(p.rect, False)
  assert len(p.rendered) == 1 and p.inset.frame is None and p.labels[-1] == 'DM CHECK'


def test_camera_failure_and_recovery(preview):
  p = preview
  p.dm.cameraUnavailable = True
  p.inset.draw_onroad(p.rect, False)
  assert p.polls == [] and p.labels[-1] == 'DM HANDS'
  p.dm.cameraUnavailable = False
  p.inset.draw_onroad(p.rect, False)
  assert len(p.rendered) == 1 and p.labels[-1] == 'DM FACE'


def test_rhd_selection_and_lost_face_fall_back_to_full_view(preview):
  p = preview
  p.inset.draw_onroad(p.rect, False)
  left_x = p.rendered[-1][0].x
  p.dm.isRHD = True
  p.inset.draw_onroad(p.rect, False)
  assert p.rendered[-1][0].x > left_x
  p.dm.visionPolicyState.faceDetected = False
  p.inset.draw_onroad(p.rect, False)
  assert p.rendered[-1][0].x == 0 and p.rendered[-1][0].width == -p.frame.width
  assert p.labels[-1] == 'DM FACE?'


@pytest.mark.parametrize('timestamp', [0, 9_000_000_000, 11_000_000_000])
def test_received_but_old_or_future_frame_is_not_displayed(preview, timestamp):
  p = preview
  p.inset.client.timestamp_sof = timestamp
  p.inset.draw_onroad(p.rect, False)
  assert not p.rendered and p.labels[-1] == 'DM CHECK'


def test_offroad_transition_reconnects_and_discards_old_buffers(preview):
  p = preview
  p.inset.draw_onroad(p.rect, False)
  old = p.inset.client
  p.inset._offroad_transition()
  assert p.inset.client is not old and p.inset.frame is None


@pytest.mark.parametrize('invalid', ['valid', 'alive', 'recv_frame', 'logMonoTime'])
def test_bad_dm_never_shows_face_or_claims_touch_fallback(preview, invalid):
  p = preview
  getattr(p.sm, invalid)['driverMonitoringState'] = 0
  p.inset.draw_onroad(p.rect, False)
  assert not p.rendered and not p.polls and p.labels[-1] == 'DM CHECK'


def test_alert_offroad_and_disengagement_hide_and_drop_frame(preview):
  p = preview
  p.inset.draw_onroad(p.rect, False)
  assert not p.inset.draw_onroad(p.rect, True)
  assert p.inset.frame is None
  p.sm['selfdriveState'].enabled = False
  assert not p.inset.draw_onroad(p.rect, False)
  p.ui.always_on_dm = True
  assert p.inset.draw_onroad(p.rect, False)
  p.ui.started = False
  assert not p.inset.draw_onroad(p.rect, False)
  p.inset.close()
  p.inset.close()
  assert p.inset.closed == 1
