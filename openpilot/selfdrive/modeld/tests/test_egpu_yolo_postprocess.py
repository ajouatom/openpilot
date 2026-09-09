from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.modeld.egpu_yolo_postprocess import OutputWorker, decode_packet, project_detections


def test_full_cpu_output_socket_skips_without_waiting():
  import threading
  worker = OutputWorker.__new__(OutputWorker)
  worker.lock = threading.Lock()
  worker.recover = False
  worker.process = SimpleNamespace(poll=lambda: None)

  def full(data):
    raise BlockingIOError

  worker.socket = SimpleNamespace(send=full)
  assert not worker.send({'frame': 1})
  worker.process = SimpleNamespace(poll=lambda: 1)
  with pytest.raises(RuntimeError, match='exited'):
    worker.send({'frame': 2})


@pytest.mark.parametrize('dtype', [np.float32, np.float16])
def test_cpu_decoder_keeps_originating_frame_and_maps_only_its_transform(monkeypatch, dtype):
  from openpilot.selfdrive.modeld import egpu_yolo
  timestamps = iter([100., 100.001, 100.002])
  monkeypatch.setattr(egpu_yolo, 'camera_time', lambda: next(timestamps))
  metadata = {'frameId': 321, 'state': 'run', 'detections': []}
  values = np.zeros((1, 6, 2688), dtype=dtype)
  values[0, :, 0] = [256, 128, 100, 60, .8, 1]
  result = decode_packet((metadata, values, np.eye(3)*2, (1024, 512), ['person', 'bicycle'], 0.))
  assert result['frameId'] == 321
  assert result['detections'][0]['label'] == 'bicycle'
  assert result['detections'][0]['cameraPoints'][0] == pytest.approx((256-50)/1024)
  assert result['postprocessTime'] == pytest.approx(.001)


def test_cpu_decoder_rejects_wrong_compact_shape_and_preserves_paused_status():
  with pytest.raises(ValueError, match='compact'):
    decode_packet(({}, np.zeros((1, 84, 2688)), np.eye(3), (1344, 760), [], 0.))
  metadata = {'frameId': 456, 'state': 'paused', 'detections': []}
  assert decode_packet((metadata, None, None, None, None, None)) == metadata


def test_signal_model_keeps_two_class_names_and_its_frozen_threshold():
  values = np.zeros((1, 6, 2688), dtype=np.float32)
  values[0, :, 0] = [256, 100, 6, 6, .3, 1]
  packet = ({'modelId': 'signal-v33-observe-s260911'}, values, np.eye(3), (512, 256),
            ['red_visible', 'green_visible'], 0.)
  assert decode_packet(packet)['detections'][0]['label'] == 'green_visible'
  packet[0]['modelId'] = 'regular-yolo'
  assert decode_packet(packet)['detections'] == []


@pytest.mark.parametrize('count', [0, 1, 2, 10, 40])
@pytest.mark.parametrize('transform', [np.eye(3), np.array([[1.1, .2, 7], [-.03, .8, 19], [.01, -.01, 1]]),
                                    np.array([[1, 0, 0], [0, 1, 0], [0, 0, -1]]), np.zeros((3, 3)), np.full((3, 3), np.nan)])
def test_batch_projection_matches_original_for_dense_frames_and_invalid_depth(count, transform):
  from openpilot.selfdrive.modeld.egpu_yolo import camera_detections
  rng = np.random.default_rng(5)
  detections = []
  for index in range(count):
    x, y = rng.uniform(-.1, 1.1, 2)
    detections.append({'classId': index % 8, 'confidence': .8, 'x1': x, 'y1': y, 'x2': x+.2, 'y2': y+.3})
  expected = camera_detections(detections, transform, (512, 256), (1344, 760))
  actual = project_detections(detections, transform, (512, 256), (1344, 760))
  assert len(actual) == len(expected)
  for a, b in zip(actual, expected, strict=True):
    np.testing.assert_allclose(a.pop('cameraPoints'), b.pop('cameraPoints'), rtol=0, atol=1e-14)
    assert a == b
