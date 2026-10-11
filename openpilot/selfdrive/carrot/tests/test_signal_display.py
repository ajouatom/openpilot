import json
import pytest

from openpilot.selfdrive.carrot.signal_display import SignalDisplayReader, publish_display, visible_tracks, project_box


def record():
  return dict(version=1, stream='road', size=[1344, 760], frame_id=20, timestamp=10., session='worker',
              tracks=[dict(id=3, box=[600, 200, 640, 215], state='red', raw='red', age=0.)])


CAMERA = (22, 10.1, 1344, 760, False)


def test_red_green_unknown_are_observations_not_permissions():
  r = record()
  assert visible_tracks(r, 10.2, CAMERA)[0]['state'] == 'red'
  r['tracks'][0].update(raw='green', state='green')
  assert visible_tracks(r, 10.2, CAMERA)[0]['state'] == 'green'
  r['tracks'][0]['raw'] = 'unknown'
  assert visible_tracks(r, 10.2, CAMERA)[0]['state'] == 'unknown'


@pytest.mark.parametrize('change', [dict(timestamp=9.8), dict(timestamp=11), dict(timestamp=float('nan')),
                                  dict(stream='wide'), dict(size=[1928, 1208]), dict(frame_id=5),
                                  dict(version=2), dict(tracks=None), dict(session=None)])
def test_stale_wrong_camera_or_invalid_records_are_hidden(change):
  r = record(); r.update(change)
  assert visible_tracks(r, 10.2, CAMERA) == ()


@pytest.mark.parametrize('camera', [None, (22, 9.8, 1344, 760, False), (22, 11., 1344, 760, False),
                                  (22, 10.1, 1344, 760, True), (22, 10.1, 1928, 1208, False)])
def test_wrong_or_stale_displayed_image_hides_boxes(camera):
  assert visible_tracks(record(), 10.2, camera) == ()


@pytest.mark.parametrize('change', [dict(age=.05), dict(box=[-1, 0, 30, 10]), dict(box=[0, 0, 10, 800]),
                                  dict(box=[2, 0, 1, 10]), dict(box=[0, 0, float('nan'), 10]), dict(id='oops')])
def test_lost_invalid_or_out_of_frame_boxes_are_hidden(change):
  r = record(); r['tracks'][0].update(change)
  assert visible_tracks(r, 10.2, CAMERA) == ()


def test_mapping_uses_actual_camera_rectangle_with_crop_offsets():
  assert project_box([0, 0, 1344, 760], (-50, -20, 672, 380)) == (-50, -20, 672, 380)
  assert project_box([672, 380, 1344, 760], (-50, -20, 672, 380)) == (286, 170, 336, 190)


def test_reader_expires_even_when_file_does_not_change_and_handles_invalid_data(tmp_path):
  flag, path = tmp_path/'display_enabled', tmp_path/'display.json'
  path.write_text(json.dumps(record()))
  reader = SignalDisplayReader(flag, path)
  assert reader.read(CAMERA, 10.2) == () and not reader.enabled
  flag.write_text('1')
  reader = SignalDisplayReader(flag, path)
  assert len(reader.read(CAMERA, 10.2)) == 1
  assert reader.read(CAMERA, 10.4) == ()
  for bad in ['x'*9000, '[]', 'null', '{']:
    path.write_text(bad)
    reader.next_read = 0
    assert reader.read(CAMERA, 10.2) == ()
  flag.write_text('0')
  assert reader.read(CAMERA, 11.) == () and not reader.enabled


def test_display_has_separate_endpoint_and_no_control_fields(tmp_path):
  r = record(); t = r['tracks'][0]
  source = dict(tracks=[dict(t, evidence={'raw':'red'})], frame_id=20, timestamp_eof=10_000_000_000,
                session='worker', revalidated=True, daytime_comparison=True)
  path = tmp_path/'display.json'
  publish_display(source, path=path)
  out = json.loads(path.read_text())
  assert out['engine'] == 'trial' and len(visible_tracks(out, 10.2, CAMERA)) == 1
  assert not any(k in out for k in ['hold','red_sign','released','control_permission'])
  assert not (tmp_path/'carrot_signal_observation.json').exists()


def test_renderer_status_is_bounded_and_display_only(tmp_path):
  reader = SignalDisplayReader(tmp_path/'flag', tmp_path/'data')
  path = tmp_path/'status'
  reader.record_draw(CAMERA, 2, path=path, now=10.)
  assert not path.exists()
  reader.enabled = True; reader.record = record()
  reader.record_draw(CAMERA, 2, path=path, now=10.)
  before = path.read_bytes()
  reader.record_draw(CAMERA, 0, path=path, now=10.1)
  assert path.read_bytes() == before
  reader.record_draw(CAMERA, 0, path=path, now=11.)
  assert json.loads(path.read_text())['boxes_drawn'] == 0
