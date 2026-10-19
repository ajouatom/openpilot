import importlib.util
import math
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS

import pytest

from openpilot.selfdrive.ui.dm_preview import eye_expression, preview_rect, preview_state, sample_fresh, smooth_expression


def driver(**changes):
  return NS(**({'faceOrientation': [0.0, 0.0, 0.0], 'faceOrientationStd': [0.1, 0.1, 0.1],
               'leftEyeProb': 1.0, 'rightEyeProb': 1.0, 'leftBlinkProb': 0.0, 'rightBlinkProb': 0.0,
               'sunglassesProb': 0.0} | changes))


@pytest.mark.parametrize(('changes', 'kind'), [
  ({}, 'tracking'), ({'monitoring_fresh': False, 'camera_unavailable': True}, 'waiting'),
  ({'camera_unavailable': True, 'driver_fresh': False}, 'wheel'), ({'driver_fresh': False}, 'waiting'),
  ({'face_detected': False}, 'searching'), ({'distracted': True}, 'warning'),
  ({'lockout': True, 'camera_unavailable': True}, 'warning'), ({'wheel_policy': True}, 'wheel'),
  ({'distracted': True, 'wheel_policy': True}, 'warning'),
])
def test_status_does_not_claim_valid_camera_or_attention(changes, kind):
  values = {'monitoring_fresh': True, 'camera_unavailable': False, 'driver_fresh': True,
            'face_detected': True, 'distracted': False, 'lockout': False}
  assert preview_state(**(values | changes)).kind == kind


@pytest.mark.parametrize(('stamp', 'fresh'), [(10, True), (9.51, True), (9.5, False), (0, False), (11, False), (float('nan'), False)])
def test_freshness_boundaries(stamp, fresh):
  assert sample_fresh(10, stamp) == fresh


def test_layout_respects_existing_panels_and_parent_offsets():
  x, y, w, h = preview_rect(300, 30, 1830, 1020, False)
  assert x + w < 300 + 350 and y > 30 + 190 and y + h < 30 + 600
  x, y, w, h = preview_rect(536, 0, 536, 240, True)
  assert (w, h) == (84, 84)
  assert x - (536 + 373) == 9 and (536 + 476) - (x + w) == 10
  assert y == 144 and y + h == 228


def test_pose_is_bounded_mirrored_head_direction_and_blinks_are_independent():
  pose = eye_expression(driver(faceOrientation=[1.0, 1.0, 0.0], leftBlinkProb=1.0))
  assert (pose.x, pose.y) == (-1.0, 1.0)
  assert pose.left_open == 0.08 and pose.right_open == 1.0
  assert pose.left_known and pose.right_known
  pose = eye_expression(driver(faceOrientation=[-1.0, -1.0, 0.0], rightBlinkProb=1.0))
  assert (pose.x, pose.y) == (1.0, -1.0)
  assert pose.left_open == 1.0 and pose.right_open == 0.08


def test_expression_accepts_real_cereal_list_readers():
  from openpilot.cereal import log
  model = log.DriverStateV2.new_message()
  model.leftDriverData.faceOrientation = [0.1, -0.2, 0.0]
  model.leftDriverData.faceOrientationStd = [0.1, 0.1, 0.1]
  model.leftDriverData.leftEyeProb = model.leftDriverData.rightEyeProb = 1.0
  pose = eye_expression(model.as_reader().leftDriverData)
  assert pose.x == pytest.approx(1 / 3) and pose.y == pytest.approx(0.2)
  assert pose.left_known and pose.right_known


@pytest.mark.parametrize('changes', [
  {'faceOrientation': []}, {'faceOrientation': [0.0]}, {'faceOrientation': [math.nan, 0, 0]},
  {'faceOrientation': [0, math.inf, 0]}, {'faceOrientationStd': []}, {'faceOrientationStd': [0.3, 0.1, 0.1]},
  {'faceOrientationStd': [-0.1, 0.1, 0.1]}, {'faceOrientationStd': [0.1, math.nan, 0.1]},
])
def test_uncertain_pose_cannot_move_pupils(changes):
  pose = eye_expression(driver(**changes))
  assert pose.x == pose.y == 0.0


@pytest.mark.parametrize('changes', [
  {'leftEyeProb': 0.65}, {'leftEyeProb': math.nan}, {'leftEyeProb': 1.1},
  {'leftBlinkProb': math.nan}, {'leftBlinkProb': -0.1}, {'leftBlinkProb': 1.1},
  {'sunglassesProb': 0.9}, {'sunglassesProb': math.nan}, {'sunglassesProb': -0.1},
])
def test_unreliable_eyes_do_not_invent_a_blink(changes):
  pose = eye_expression(driver(**changes))
  assert not pose.left_known
  assert pose.left_open == 1.0


def test_smoothing_keeps_real_blinks_and_immediately_invalidates_unknown_eyes():
  opened = eye_expression(driver())
  closed = eye_expression(driver(leftBlinkProb=1.0, rightBlinkProb=1.0))
  blink = smooth_expression(opened, closed, 0.05)
  assert blink.left_open < 0.15 and blink.right_open < 0.15
  reopening = smooth_expression(blink, opened, 0.05)
  assert blink.left_open < reopening.left_open < 1.0
  unknown = smooth_expression(opened, eye_expression(driver(leftEyeProb=0)), 0.001)
  assert not unknown.left_known and unknown.right_known
  assert smooth_expression(None, closed, 0) == closed
  assert smooth_expression(opened, opened, 100) == opened  # No decorative blinks.


@pytest.fixture(params=[False, True], ids=['c3', 'c4'])
def preview(monkeypatch, request):
  import pyray as rl
  labels, shapes = [], []
  for name in ('draw_rectangle_rounded', 'draw_circle_v', 'draw_circle_lines', 'draw_line_ex'):
    monkeypatch.setattr(rl, name, lambda *a, name=name: shapes.append((name, a)))
  monkeypatch.setattr(rl, 'draw_text_ex', lambda font, text, *a: labels.append(text))
  for name in ('draw_rectangle_rec', 'draw_rectangle_lines_ex'):
    monkeypatch.setattr(rl, name, lambda *a: pytest.fail('Camera/card box must not be drawn'))
  now = [10.0]

  class Widget:
    def set_enabled(self, enabled):
      assert not enabled
    def render(self, rect):
      self._render(rect)

  dm = NS(cameraUnavailable=False, isRHD=False, lockout=False, alwaysOnLockout=False, activePolicy='vision',
          visionPolicyState=NS(faceDetected=True, isDistracted=False))
  model = NS(frameId=100, leftDriverData=driver(faceOrientation=[0, -0.3, 0]),
             rightDriverData=driver(faceOrientation=[0, 0.3, 0]))

  class SM(dict):
    def __init__(self, **values):
      super().__init__(**values)
      self.valid = {'driverMonitoringState': True, 'driverStateV2': True}
      self.alive = self.valid.copy()
      self.recv_frame = {'driverMonitoringState': 11, 'driverStateV2': 11}
      self.logMonoTime = {'driverMonitoringState': 10_000_000_000, 'driverStateV2': 10_000_000_000}

  sm = SM(driverMonitoringState=dm, driverStateV2=model, selfdriveState=NS(enabled=True))
  ui = NS(sm=sm, started=True, started_frame=1, always_on_dm=False)  # No writable Params API.
  for name, attrs in {
    'msgq.visionipc': {}, 'openpilot.selfdrive.ui.onroad.cameraview': {},  # Camera imports would fail.
    'openpilot.selfdrive.ui.ui_state': {'ui_state': ui},
    'openpilot.system.ui.widgets': {'Widget': Widget},
    'openpilot.system.ui.lib.application': {'FontWeight': NS(BOLD=1), 'gui_app': NS(font=lambda _: None)},
    'openpilot.system.ui.lib.text_measure': {'measure_text_cached': lambda font, text, size: NS(x=len(text) * size * 0.6)},
  }.items():
    module = ModuleType(name)
    module.__dict__.update(attrs)
    monkeypatch.setitem(sys.modules, name, module)
  spec = importlib.util.spec_from_file_location('_test_driver_preview', Path(__file__).parents[1] / 'onroad/driver_preview.py')
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
  inset = module.DriverPreview(compact=request.param)
  return NS(inset=inset, ui=ui, sm=sm, dm=dm, model=model, now=now, labels=labels, shapes=shapes,
            module=module, rect=rl.Rectangle(0, 0, 536, 240) if request.param else rl.Rectangle(0, 0, 2160, 1080))


def test_pearl_eyes_are_passive_without_camera_resources_or_box(preview):
  p = preview
  assert p.inset.draw_onroad(p.rect, False)
  assert p.labels[-1] == 'DM FACE' and p.inset._expression.left_known
  assert not hasattr(p.inset, 'client') and not hasattr(p.inset, 'shader')
  rounded = [args for name, args in p.shapes if name == 'draw_rectangle_rounded']
  assert len(rounded) == 6  # two silhouettes, two eyes, two pupils
  color = rounded[1][-1]
  assert (color.r, color.g, color.b, color.a) == (244, 237, 223, 255)
  x, y, w, h = preview_rect(p.rect.x, p.rect.y, p.rect.width, p.rect.height, p.inset.compact)
  for rect, *_ in rounded:
    assert x <= rect.x < rect.x + rect.width <= x + w
    assert y <= rect.y < rect.y + rect.height <= y + h


def test_stalled_model_frame_drops_pose_even_with_fresh_publications(preview):
  p = preview
  p.inset.draw_onroad(p.rect, False)
  p.now[0] += 0.5
  p.sm.logMonoTime = dict.fromkeys(p.sm.logMonoTime, int(p.now[0] * 1e9))
  p.inset.draw_onroad(p.rect, False)
  assert p.inset._expression is None and p.labels[-1] == 'DM CHECK'
  p.model.frameId += 1
  p.inset.draw_onroad(p.rect, False)
  assert p.inset._expression is not None and p.labels[-1] == 'DM FACE'


def test_camera_failure_wheel_policy_and_recovery(preview):
  p = preview
  p.dm.cameraUnavailable = True
  p.inset.draw_onroad(p.rect, False)
  assert p.inset._expression is None and p.labels[-1] == 'DM HANDS'
  p.dm.cameraUnavailable = False
  p.dm.activePolicy = 'wheeltouch'
  p.inset.draw_onroad(p.rect, False)
  assert p.inset._expression is None and p.labels[-1] == 'DM HANDS'
  p.dm.activePolicy = 'vision'
  p.inset.draw_onroad(p.rect, False)
  assert p.inset._expression is not None and p.labels[-1] == 'DM FACE'


def test_rhd_selection_lost_face_and_eye_confidence(preview):
  p = preview
  p.inset.draw_onroad(p.rect, False)
  assert p.inset._expression.x > 0
  p.dm.isRHD = True
  p.inset.draw_onroad(p.rect, False)
  assert p.inset._expression.x < 0
  p.dm.visionPolicyState.faceDetected = False
  p.inset.draw_onroad(p.rect, False)
  assert p.inset._expression is None and p.labels[-1] == 'DM FACE?'
  p.dm.visionPolicyState.faceDetected = True
  p.model.rightDriverData.sunglassesProb = 1.0
  p.inset.draw_onroad(p.rect, False)
  assert not p.inset._expression.left_known and p.labels[-1] == 'DM EYES?'


@pytest.mark.parametrize('service', ['driverMonitoringState', 'driverStateV2'])
@pytest.mark.parametrize(('field', 'value'), [('valid', False), ('alive', False), ('recv_frame', 1),
                                           ('logMonoTime', 0), ('logMonoTime', 9_500_000_000), ('logMonoTime', 11_000_000_000)])
def test_bad_data_never_retains_live_eyes(preview, service, field, value):
  p = preview
  p.inset.draw_onroad(p.rect, False)
  getattr(p.sm, field)[service] = value
  p.inset.draw_onroad(p.rect, False)
  assert p.inset._expression is None and p.labels[-1] == 'DM CHECK'


def test_warning_and_lockout_take_precedence(preview):
  p = preview
  p.dm.visionPolicyState.isDistracted = True
  p.inset.draw_onroad(p.rect, False)
  assert p.labels[-1] == 'DM ALERT'
  p.dm.alwaysOnLockout = True
  p.dm.cameraUnavailable = True
  p.inset.draw_onroad(p.rect, False)
  assert p.labels[-1] == 'DM LIMIT' and p.inset._expression is None


def test_alert_offroad_disengagement_and_close_discard_expression(preview):
  p = preview
  p.inset.draw_onroad(p.rect, False)
  assert not p.inset.draw_onroad(p.rect, True)
  assert p.inset._expression is None
  p.sm['selfdriveState'].enabled = False
  assert not p.inset.draw_onroad(p.rect, False)
  p.ui.always_on_dm = True
  assert p.inset.draw_onroad(p.rect, False)
  p.ui.started = False
  assert not p.inset.draw_onroad(p.rect, False)
  assert p.inset._expression is None
  p.inset.close()
  p.inset.close()
  p.ui.started = True
  assert not p.inset.draw_onroad(p.rect, False)


def test_new_session_resets_pose_without_an_offroad_draw(preview):
  p = preview
  p.inset.draw_onroad(p.rect, False)
  assert p.inset._expression.x > 0
  p.ui.started_frame = 5
  p.model.leftDriverData.faceOrientation = [0, 0.3, 0]
  p.inset.draw_onroad(p.rect, False)
  assert p.inset._expression.x < 0  # No blend with the previous session.


def test_c3_and_c4_use_the_same_renderer():
  root = Path(__file__).parents[1]
  assert 'DriverPreview()' in (root / 'onroad/augmented_road_view.py').read_text(encoding='utf-8')
  assert 'DriverPreview(compact=True)' in (root / 'mici/onroad/augmented_road_view.py').read_text(encoding='utf-8')
