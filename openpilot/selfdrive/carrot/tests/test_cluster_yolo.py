from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "cluster"))
from cluster_yolo import build_yolo_display, camera_boxes, yolo_text


def display(**kwargs):
  message = {'state': 'run', 'runs': 42, 'executionTime': .007,
             'detections': [{'label': 'car', 'confidence': .8}, {'label': 'car', 'confidence': .6}]}
  return build_yolo_display(**{'message': message, 'enabled': True, 'valid': True,
                               'transport_age': .03, 'image_age': .1, **kwargs})


def test_current_detections_group_counts_and_best_confidence():
  result = display()
  assert result.objects == (('car', 2, 80),)
  assert yolo_text(result, 'ko') == ('YOLO 실행 7.0ms · #42', '자동차 2 (80%)')
  assert yolo_text(result, 'en') == ('YOLO RUN 7.0ms · #42', 'car 2 (80%)')


@pytest.mark.parametrize('state', ['paused', 'error', 'overrun'])
def test_invalid_status_messages_clear_old_objects_but_preserve_state(state):
  assert display(message={'state': state, 'runs': 43}, valid=False).state == state
  assert display(message={'state': state, 'runs': 43}, valid=False).objects == ()


@pytest.mark.parametrize('args', [{'image_age': .36}, {'transport_age': 2.01}, {'image_age': float('nan')}, {'image_age': -.1}])
def test_stale_or_bad_image_time_never_displays_previous_detections(args):
  result = display(**args)
  assert result.state == 'stale' and not result.objects
  assert result.execution_ms is None


def test_absent_disabled_invalid_and_empty_are_distinct():
  assert display(message=None, enabled=False) is None
  assert display(message=None).state == 'waiting'
  assert display(enabled=False).state == 'off'
  assert display(valid=False).state == 'invalid'
  assert yolo_text(replace(display(), objects=()), 'en')[1] == 'No detections'


def test_cpu_signal_summary_bridges_frames_without_relaxing_box_alignment():
  message = {'modelId': 'signal-v33-observe-int8-s260911', 'state': 'run', 'runs': 3,
             'camera': 'road', 'timestampEof': 10_000_000_000,
             'detections': [{'label': 'red_visible', 'confidence': .8, 'cameraPoints': [.1, .1, .2, .1, .2, .2, .1, .2]}]}
  result = display(message=message, image_age=.5)
  assert result.state == 'run'
  assert '빨강 감지' in yolo_text(result, 'ko')[1]
  assert not camera_boxes(result, camera='road', timestamp_eof=10_300_000_000, video_rect=(0, 0, 512, 256))
  assert display(message=message, image_age=.61).state == 'stale'


def test_live_source_uses_publication_age_and_clears_paused_content():
  from cluster_live import OpenpilotLiveSource
  source = object.__new__(OpenpilotLiveSource)
  source._egpu_active = True
  source.sm = SimpleNamespace(seen={'carrotYolo': True}, recv_time={'carrotYolo': 100.},
                              logMonoTime={'carrotYolo': 900_000_000_000}, valid={'carrotYolo': True})
  source._service_data = lambda service: SimpleNamespace(state='run', runs=12, executionTime=.006,
      timestampEof=899_900_000_000, detections=[SimpleNamespace(label='person', confidence=.9)])
  assert source._yolo_display(100.1).objects == (('person', 1, 90),)
  assert source._yolo_display(100.3).state == 'stale'


def test_all_hud_layouts_receive_yolo_summary(monkeypatch):
  import cluster_renderer
  from cluster_live import standby_state
  renderer = object.__new__(cluster_renderer.ClusterUiRenderer)
  calls = []
  renderer._draw_text = lambda text, *args, **kwargs: calls.append(text)
  renderer._ellipsize_text = lambda text, *args: text
  monkeypatch.setattr(cluster_renderer.rl, 'draw_rectangle_rounded', lambda *args: None)
  monkeypatch.setattr(cluster_renderer.rl, 'draw_rectangle_rounded_lines_ex', lambda *args: None)
  for mode in [0, 1, 2, cluster_renderer.CLUSTER_SCREEN_MODE_FULLSCREEN_3D]:
    renderer._draw_yolo_status(replace(standby_state(), yolo=display()), mode)
  assert calls.count('YOLO 실행 7.0ms · #42') == 4


def boxed_display(**kwargs):
  return display(message={'state': 'run', 'camera': 'road', 'timestampEof': 10_000_000_000,
                          'detections': [{'label': 'car', 'confidence': .8,
                                          'cameraPoints': [.1, .2, .4, .21, .39, .6, .11, .59]}]}, **kwargs)


def test_camera_corners_follow_video_crop_zoom_and_swapped_panel_offset():
  result = boxed_display()
  box, points = camera_boxes(result, camera='road', timestamp_eof=10_100_000_000,
                             video_rect=(800., -100., 1200., 700.))[0]
  assert box.label == 'car' and box.confidence == 80
  assert points[0] == pytest.approx((920., 40.))
  assert points[2] == pytest.approx((1268., 320.))
  assert points[0][1] != points[1][1]  # retain calibrated quadrilateral, not model-space rectangle


@pytest.mark.parametrize('camera,stamp', [('wideRoad', 10_000_000_000), ('road', 0),
                                         ('road', 10_201_000_000), ('road', 9_799_000_000)])
def test_wrong_camera_or_mismatched_video_frame_never_draws_boxes(camera, stamp):
  assert not camera_boxes(boxed_display(), camera=camera, timestamp_eof=stamp, video_rect=(0, 0, 100, 100))


@pytest.mark.parametrize('kwargs', [{'valid': False}, {'image_age': .36}, {'enabled': False}])
def test_invalid_stale_disabled_clears_boxes(kwargs):
  assert not boxed_display(**kwargs).boxes


def test_malformed_camera_points_never_fall_back_to_wrong_model_coordinates():
  result = display(message={'state': 'run', 'detections': [
    {'label': 'car', 'confidence': .8, 'cameraPoints': [float('nan')]*8},
    {'label': 'car', 'confidence': .8, 'cameraPoints': [0., 1.]},
    {'label': 'car', 'confidence': .8, 'x1': .1, 'y1': .1, 'x2': .2, 'y2': .2},
  ]})
  assert not result.boxes and result.objects == (('car', 3, 80),)


def test_live_renderer_draws_four_camera_edges_and_localized_label(monkeypatch):
  import cluster_renderer as module
  from cluster_live import standby_state
  renderer = object.__new__(module.ClusterUiRenderer)
  renderer.height = 480
  renderer.language = 'ko'
  renderer._ellipsize_text = lambda text, *args: text
  labels, lines = [], []
  renderer._draw_text_with_stroke = lambda text, *args: labels.append(text)
  monkeypatch.setattr(module.rl, 'draw_line_ex', lambda *args: lines.append(args))
  rect = SimpleNamespace(x=0., y=0., width=1128., height=480.)
  projection = SimpleNamespace(dest=rect, video_dest=rect, wide_camera=False)
  renderer._draw_yolo_camera_boxes(replace(standby_state(), yolo=boxed_display()), projection, 10_050_000_000)
  assert len(lines) == 4 and labels == ['자동차 80%']
  renderer._draw_yolo_camera_boxes(replace(standby_state(), yolo=boxed_display()), projection, 11_000_000_000)
  assert len(lines) == 4


def test_budget_wait_has_specific_status_and_no_boxes():
  result = display(message={'state': 'no_budget', 'runs': 4018}, valid=False)
  assert yolo_text(result, 'ko')[0] == 'YOLO 여유시간대기 · #4018'
  assert not result.boxes
